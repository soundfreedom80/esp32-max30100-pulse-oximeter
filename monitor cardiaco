#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define DEBUG

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

PulseOximeter pox;
uint32_t tsLastReport = 0;

constexpr float ALPHA = 0.60;
constexpr float MIN_HR = 50.0;
constexpr float MAX_HR = 180.0;
constexpr float MIN_SPO2 = 80.0;
constexpr float MAX_SPO2 = 100.0;
constexpr float MAX_SALTO_HR = 25.0;
constexpr float MAX_SALTO_SPO2 = 4.0;
constexpr uint16_t TIMEOUT_SENSOR = 3000;
constexpr uint16_t REPORTING_PERIOD_MS = 500;

constexpr uint8_t ECG_TOP    = 44;
constexpr uint8_t ECG_BOTTOM = 63;
constexpr uint8_t ECG_HEIGHT = ECG_BOTTOM - ECG_TOP;
constexpr uint8_t CENTRO_Y   = ECG_TOP + (ECG_HEIGHT / 2);  // 53

float hr = 0.0, spo2 = 0.0;
float hrFiltrado = 0.0, spo2Filtrado = 0.0;
int16_t ultimoHR = -1, ultimoSpO2 = -1;
uint32_t tsLastGoodHR = 0, tsLastGoodSpO2 = 0;

// --- Buffer para el ECG (desplazamiento continuo) ---
uint8_t ecgBuffer[SCREEN_WIDTH];          // valores de desplazamiento vertical (0 = centro, positivo = arriba)
uint32_t tsUltimoLatido = 0;              // momento del último latido
bool hayLatidoPendiente = false;          // para generar el pico

// --- Animación de desplazamiento ---
uint32_t tsUltimoDesplazamiento = 0;
const uint32_t INTERVALO_DESPLAZAMIENTO = 25; // ms entre cada desplazamiento

// Icono del corazón
const unsigned char heart_bmp[] PROGMEM = {
  0b00000000, 0b01100110, 0b11111111, 0b11111111,
  0b11111111, 0b01111110, 0b00111100, 0b00011000
};
bool mostrarCorazon = false;
uint32_t tsUltimoLatidoCorazon = 0;

// --- Callback al detectar latido ---
void onBeatDetected() {
#ifdef DEBUG
  Serial.println(F("Beat!"));
#endif
  // Registramos el latido para generar el pico en el buffer
  tsUltimoLatido = millis();
  hayLatidoPendiente = true;

  // Parpadeo del corazón
  mostrarCorazon = true;
  tsUltimoLatidoCorazon = millis();
}

// --- Función que genera el valor de amplitud según el tiempo desde el último latido ---
int8_t obtenerAmplitud(uint32_t tiempoDesdeLatido) {
  if (tiempoDesdeLatido > 400) return 0; // línea base después de 400 ms

  // Simulamos un complejo QRS:
  // - Pico rápido (R) a los 20-30 ms
  // - Onda S (pequeña depresión) a los 60 ms
  // - Onda T (elevación suave) a los 150-200 ms
  // Simplificamos con una función que da un pico y decae con una pequeña cola

  if (tiempoDesdeLatido < 30) {
    // Subida rápida hasta el pico
    return (int8_t)(tiempoDesdeLatido * 8 / 30); // de 0 a 8
  } else if (tiempoDesdeLatido < 50) {
    // Pico y caída rápida (onda R)
    return (int8_t)(8 - (tiempoDesdeLatido - 30) * 8 / 20); // de 8 a 0
  } else if (tiempoDesdeLatido < 80) {
    // Onda S (depresión negativa)
    return (int8_t)(-(tiempoDesdeLatido - 50) * 2 / 30); // de 0 a -2
  } else if (tiempoDesdeLatido < 120) {
    // Retorno a la línea base
    return (int8_t)(-2 + (tiempoDesdeLatido - 80) * 2 / 40); // de -2 a 0
  } else if (tiempoDesdeLatido < 200) {
    // Onda T (elevación suave)
    return (int8_t)( (tiempoDesdeLatido - 120) * 4 / 80 ); // de 0 a 4
  } else if (tiempoDesdeLatido < 260) {
    // Descenso de la onda T
    return (int8_t)(4 - (tiempoDesdeLatido - 200) * 4 / 60 ); // de 4 a 0
  } else {
    return 0;
  }
}

// --- Desplazar el buffer y añadir nuevo valor ---
void actualizarBufferECG() {
  // Desplazar todo a la izquierda (sobrescribir el primer elemento)
  for (uint8_t i = 0; i < SCREEN_WIDTH - 1; i++) {
    ecgBuffer[i] = ecgBuffer[i + 1];
  }

  // Calcular nuevo valor para la última columna (derecha)
  uint32_t ahora = millis();
  uint32_t delta = ahora - tsUltimoLatido;
  int8_t nuevoValor = obtenerAmplitud(delta);

  // Si hay un latido pendiente, forzamos el pico (por si acaso)
  if (hayLatidoPendiente && delta < 50) {
    // Asegurar que el pico se vea alto
    if (nuevoValor < 8) nuevoValor = 8;
    hayLatidoPendiente = false;
  }

  // Ajustar al rango de la pantalla (centro +- 12 píxeles para no salirse)
  if (nuevoValor > 12) nuevoValor = 12;
  if (nuevoValor < -12) nuevoValor = -12;

  // Guardar en el buffer (desplazamiento vertical desde el centro)
  ecgBuffer[SCREEN_WIDTH - 1] = (uint8_t)(nuevoValor + 12); // almacenamos como offset positivo de 0-24
}

// --- Dibujar todo el trazado ECG en la pantalla ---
void dibujarECG() {
  // Borrar área del ECG
  display.fillRect(0, ECG_TOP, SCREEN_WIDTH, ECG_HEIGHT + 1, SSD1306_BLACK);

  // Dibujar línea base (opcional, pero da contexto)
  display.drawLine(0, CENTRO_Y, SCREEN_WIDTH - 1, CENTRO_Y, SSD1306_WHITE);

  // Dibujar el trazado conectando puntos consecutivos
  for (uint8_t i = 0; i < SCREEN_WIDTH - 1; i++) {
    int16_t y1 = CENTRO_Y - (int16_t)(ecgBuffer[i] - 12); // convertir a desplazamiento real
    int16_t y2 = CENTRO_Y - (int16_t)(ecgBuffer[i + 1] - 12);
    display.drawLine(i, y1, i + 1, y2, SSD1306_WHITE);
  }

  display.display();
}

// --- Configuración inicial ---
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (true);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  // Título
  display.setTextSize(1);
  display.setCursor(20, 0);
  display.print(F("Pulse Oximeter"));

  // Etiquetas fijas
  display.setTextSize(1.5);
  display.setCursor(2, 16);
  display.print(F("BPM"));
  display.setCursor(70, 16);
  display.print(F("SpO2"));

  // Corazón
  display.drawBitmap(110, 1, heart_bmp, 8, 8, SSD1306_WHITE);
  display.display();

  if (!pox.begin()) while (true);
  pox.setIRLedCurrent(MAX30100_LED_CURR_14_2MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);

  // Inicializar buffer con valores 0 (centro)
  for (uint8_t i = 0; i < SCREEN_WIDTH; i++) {
    ecgBuffer[i] = 12; // valor 12 significa desplazamiento 0 (centro)
  }
}

void loop() {
  pox.update();

  // Lectura y actualización de números cada REPORTING_PERIOD_MS
  if (millis() - tsLastReport >= REPORTING_PERIOD_MS) {
    leerSensor();
    filtrarDatos();
    imprimirSerial();
    actualizarPantalla();
    tsLastReport = millis();
  }

  // Gestión del trazado ECG (desplazamiento y dibujo) cada INTERVALO_DESPLAZAMIENTO
  if (millis() - tsUltimoDesplazamiento >= INTERVALO_DESPLAZAMIENTO) {
    actualizarBufferECG();
    dibujarECG();
    tsUltimoDesplazamiento = millis();
  }

  // Apagar el corazón tras 200 ms
  if (mostrarCorazon && (millis() - tsUltimoLatidoCorazon > 200)) {
    mostrarCorazon = false;
    display.fillRect(110, 1, 8, 8, SSD1306_BLACK);
    display.drawBitmap(110, 1, heart_bmp, 8, 8, SSD1306_WHITE);
    display.display();
  }
}

void leerSensor() {
  hr = pox.getHeartRate();
  spo2 = pox.getSpO2();
}

void filtrarDatos() {
  hrFiltrado = filtroEMA(hr, hrFiltrado, MIN_HR, MAX_HR, MAX_SALTO_HR, tsLastGoodHR);
  spo2Filtrado = filtroEMA(spo2, spo2Filtrado, MIN_SPO2, MAX_SPO2, MAX_SALTO_SPO2, tsLastGoodSpO2);
}

float filtroEMA(float dato, float &valorFiltrado, float minimo, float maximo, float saltoMax, uint32_t &ultimoValido) {
  if (dato < 40.0 || (dato > 220.0 && maximo == MAX_HR)) {
    if (millis() - ultimoValido > TIMEOUT_SENSOR) valorFiltrado = 0.0;
    return valorFiltrado;
  }
  if (dato >= minimo && dato <= maximo) {
    if (valorFiltrado > 0 && abs(dato - valorFiltrado) > saltoMax) dato = valorFiltrado;
    if (valorFiltrado == 0) valorFiltrado = dato;
    else valorFiltrado = (ALPHA * valorFiltrado) + ((1.0 - ALPHA) * dato);
    ultimoValido = millis();
  }
  else if (dato == 0 && (millis() - ultimoValido > TIMEOUT_SENSOR)) valorFiltrado = 0.0;
  return valorFiltrado;
}

void imprimirSerial() {
#ifdef DEBUG
  Serial.print(F("HR: "));
  Serial.print(hrFiltrado);
  Serial.print(F(" bpm / SpO2: "));
  Serial.print(spo2Filtrado);
  Serial.println(F(" %"));
#endif
}

void actualizarPantalla() {
  int16_t hrEntero = (int16_t)hrFiltrado;
  int16_t spo2Entero = (int16_t)spo2Filtrado;
  bool requiereDisplay = false;

  if (hrEntero != ultimoHR) {
    display.fillRect(2, 26, 58, 18, SSD1306_BLACK);
    display.setTextSize(2);
    display.setCursor(2, 26);
    if (hrEntero == 0) display.print(F("--"));
    else display.print(hrEntero);
    ultimoHR = hrEntero;
    requiereDisplay = true;
  }

  if (spo2Entero != ultimoSpO2) {
    display.fillRect(68, 26, 50, 18, SSD1306_BLACK);
    display.setTextSize(2);
    display.setCursor(68, 26);
    if (spo2Entero == 0) display.print(F("--"));
    else display.print(spo2Entero);
    ultimoSpO2 = spo2Entero;

    display.setTextSize(1.8);
    display.setCursor(106, 26);
    display.print(F("%"));
    requiereDisplay = true;
  }

  if (requiereDisplay) display.display();

}
