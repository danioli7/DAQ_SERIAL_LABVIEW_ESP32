# DAQ_SERIAL_LABVIEW_ESP32
Sistema con ESP32 que envía datos analógicos y digitales por UDP a LabVIEW y recibe comandos para controlar un DAC y salidas digitales. Incluye muestreo a 1 kHz, reconexión WiFi y calibración ADC.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

// Pines CONFIGURADOS PARA EVITAR CONFLICTOS
#define POT1 33 // ADC1_CH5
#define POT2 32 // ADC1_CH4
#define DIG1 13
#define DIG2 16
#define DIG3 17
#define DIG4 18

#define DAC_PIN 25
#define OUT1 26
#define OUT2 27
#define OUT3 14
#define OUT4 12

// Configuración WiFi
const char *ssid = "Livebox6-952C";
const char *pwd = "ZXefk6CN5oho";

// UDP
const char *udpAddress = "192.168.1.26";
const int udpPort = 8000;
const int localPort = 8001;

// ADC calibrado
#define DEFAULT_VREF 1100
#define NO_OF_SAMPLES 64
esp_adc_cal_characteristics_t *adc_chars;

// Muestreo
const int tamañoBloque = 10;
const int intervaloMuestreo = 1000; // 1 ms
uint16_t buffer1[tamañoBloque];
uint16_t buffer2[tamañoBloque];
int indice = 0;
unsigned long tiempoAnterior = 0;

WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nIniciando sistema ESP32...");

  configurarPinesSeguros();
  configurarADC();
  inicializarWiFiSinConflictos();
  delay(2000);
  conectarWiFiRobusto();

  if (WiFi.status() == WL_CONNECTED) {
    iniciarUDP();
  }
}

void loop() {
  manejoConexionAutomatico();
  procesarDatosRecibidos();
}

void procesarDatosRecibidos() {
  int packetSize = udp.parsePacket();
  if (packetSize == 2) {
    uint8_t datosRecibidos[2];
    udp.read(datosRecibidos, 2);

    uint8_t valorDAC = datosRecibidos[0];
    uint8_t controlBits = datosRecibidos[1];

    analogWrite(DAC_PIN, valorDAC);

    digitalWrite(OUT1, (controlBits >> 0) & 0x01);
    digitalWrite(OUT2, (controlBits >> 1) & 0x01);
    digitalWrite(OUT3, (controlBits >> 2) & 0x01);
    digitalWrite(OUT4, (controlBits >> 3) & 0x01);

    Serial.print("RECIBIDO -> DAC: ");
    Serial.print(valorDAC);
    Serial.print(" | Bits: ");
    Serial.println(controlBits, BIN);
  }
}

void configurarPinesSeguros() {
  pinMode(POT1, INPUT);
  pinMode(POT2, INPUT);
  pinMode(DIG1, INPUT_PULLUP);
  pinMode(DIG2, INPUT_PULLUP);
  pinMode(DIG3, INPUT_PULLUP);
  pinMode(DIG4, INPUT_PULLUP);

  pinMode(DAC_PIN, OUTPUT);
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(OUT3, OUTPUT);
  pinMode(OUT4, OUTPUT);

  Serial.println("Pines configurados en modo seguro");
}

void configurarADC() {
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11); // GPIO 33
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11); // GPIO 32
  adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  Serial.println("ADC configurado con calibración");
}

void inicializarWiFiSinConflictos() {
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
}

void conectarWiFiRobusto() {
  Serial.println("\nIniciando conexión WiFi robusta...");
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  Serial.println("Escaneando redes...");
  int n = WiFi.scanNetworks();
  bool redEncontrada = false;

  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == ssid) {
      redEncontrada = true;
      Serial.print("Red encontrada. Canal: ");
      Serial.println(WiFi.channel(i));
      break;
    }
  }

  if (!redEncontrada) {
    Serial.println("¡Error! Red no encontrada en el escaneo");
    return;
  }

  WiFi.begin(ssid, pwd);
  unsigned long inicioConexion = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicioConexion < 20000) {
    delay(500);
    Serial.print(".");
    if (millis() - inicioConexion > 10000 && WiFi.status() != WL_CONNECTED) {
      Serial.println("\nReiniciando configuración WiFi...");
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(ssid, pwd);
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n¡Conexión WiFi exitosa!");
    mostrarInfoWiFiCompleta();
  } else {
    Serial.println("\nFallo en conexión WiFi");
    diagnosticarErrorWiFi();
  }
}

void mostrarInfoWiFiCompleta() {
  Serial.println("\nInformación de conexión:");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
}

void diagnosticarErrorWiFi() {
  wl_status_t status = WiFi.status();
  Serial.print("Código de error: ");
  Serial.println(status);

  switch (status) {
    case WL_NO_SSID_AVAIL:
      Serial.println("Red no encontrada. Verifica SSID.");
      break;
    case WL_CONNECT_FAILED:
      Serial.println("Contraseña incorrecta.");
      break;
    case WL_CONNECTION_LOST:
      Serial.println("Conexión perdida.");
      break;
    case WL_DISCONNECTED:
      Serial.println("WiFi desconectado.");
      break;
    default:
      Serial.println("Error desconocido.");
  }
}

void iniciarUDP() {
  if (udp.begin(localPort)) {
    Serial.println("\nUDP iniciado correctamente");
  } else {
    Serial.println("Error al iniciar UDP");
  }
}

void manejoConexionAutomatico() {
  static unsigned long ultimaVerificacion = 0;
  const unsigned long intervalo = 30000;

  if (millis() - ultimaVerificacion >= intervalo) {
    ultimaVerificacion = millis();

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\nConexión perdida. Reconectando...");
      inicializarWiFiSinConflictos();
      conectarWiFiRobusto();
      if (WiFi.status() == WL_CONNECTED) {
        udp.stop();
        iniciarUDP();
      }
    }
  }
}
