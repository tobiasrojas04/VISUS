#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// ===================== Pines / I2C =====================
#define SDA_PIN 0
#define SCL_PIN 1
#define XSHUT_IZQ 6
#define XSHUT_DER 3

// ---- MOTORES ----
#define MOTOR_IZQ 5
#define MOTOR_DER 4

// ---- BUZZER ----
#define BUZZER_PIN 7
const int FREQ_BUZZ = 3000;
const int RES_BUZZ = 10;
bool buzzerState = false;
unsigned long lastBeepTime = 0;

// ===================== Direcciones I2C =====================
#define ADDR_IZQ 0x30
#define ADDR_DER 0x29

// ===================== BLE =====================
#define DEVICE_NAME "VISUS"
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
bool deviceConnected = false;

// ===================== Sensores ToF =====================
VL53L0X sensorIzq;
VL53L0X sensorDer;
int distanciaIzq = 0;
int distanciaDer = 0;
int distanciaMin = 2000;

int distanciaFiltrada(int izq, int der) {
  bool izqValida = (izq > 0 && izq <= 2000);
  bool derValida = (der > 0 && der <= 2000);

  if (!izqValida && !derValida) return 0;     // los dos fallaron
  if (izqValida && !derValida) return izq;    // solo izq es válida
  if (!izqValida && derValida) return der;    // solo der es válida

  return min(izq, der);                       // ambas válidas
}


// ===================== Flags =====================
bool motorOn = true;
bool buzzerOn = true;
bool motorFuncEnabled = true;
bool buzzerFuncEnabled = true;

// ===================== Tiempos =====================
unsigned long lastTOFRead = 0;
const unsigned long intervaloTOF = 1000;
unsigned long lastBLETime = 0;
const unsigned long bleInterval = 3000;

// PWM CONFIG
const int PWM_FREQ = 1000;
const int PWM_RES = 12;
const int PWM_POWER = 3800;

// =======================================================
// BLE CALLBACKS
// =======================================================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) {
    deviceConnected = true;
    Serial.println("Conectado");
    pServer->updateConnParams(param->connect.remote_bda, 6, 32, 0, 1000);
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Desconectado. Reiniciando anuncio...");
    BLEDevice::startAdvertising();
  }
};

// =======================================================
// BLE WRITE CALLBACK
// =======================================================
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    if (!deviceConnected) return;

    String rxValue = pCharacteristic->getValue().c_str();
    if (rxValue.length() == 0) return;

    Serial.print("BLE <- ");
    Serial.println(rxValue.c_str());

    if (rxValue == "CMO") {
      motorFuncEnabled = false;
      Serial.println("Función MOTOR deshabilitada.");
    } else if (rxValue == "CMI") {
      motorFuncEnabled = true;
      Serial.println("Función MOTOR habilitada.");
    } else if (rxValue == "CBO") {
      buzzerFuncEnabled = false;
      ledcWrite(BUZZER_PIN, 0);
      Serial.println("Función BUZZER deshabilitada.");
    } else if (rxValue == "CBI") {
      buzzerFuncEnabled = true;
      Serial.println("Función BUZZER habilitada.");
    } else {
      Serial.println("Comando BLE no reconocido.");
    }
  }
};

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Iniciando VISUS...");

  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(XSHUT_IZQ, OUTPUT);
  pinMode(XSHUT_DER, OUTPUT);

  // Motores
  ledcAttach(MOTOR_IZQ, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_DER, PWM_FREQ, PWM_RES);
  ledcWrite(MOTOR_IZQ, 0);
  ledcWrite(MOTOR_DER, 0);

  // Buzzer
  ledcAttach(BUZZER_PIN, FREQ_BUZZ, RES_BUZZ);
  ledcWrite(BUZZER_PIN, 800);

  // Inicializar ToF
  digitalWrite(XSHUT_IZQ, LOW);
  digitalWrite(XSHUT_DER, LOW);
  delay(10);

  digitalWrite(XSHUT_IZQ, HIGH);
  delay(10);
  sensorIzq.init(true);
  sensorIzq.setAddress(ADDR_IZQ);

  digitalWrite(XSHUT_DER, HIGH);
  delay(10);
  sensorDer.init(true);

  sensorIzq.setTimeout(1000);
  sensorDer.setTimeout(1000);

  // BLE
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setPower(ESP_PWR_LVL_P9);
  BLEDevice::setMTU(256);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_WRITE
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinInterval(32);
  pAdvertising->setMaxInterval(48);
  pAdvertising->setMinPreferred(6);
  pAdvertising->setMaxPreferred(32);

  BLEDevice::startAdvertising();
  Serial.println("Advertising iniciado...");
}

// =======================================================
// Lectura sensores
// =======================================================
void leerSensores() {
  distanciaIzq = sensorIzq.readRangeSingleMillimeters();
  distanciaDer = sensorDer.readRangeSingleMillimeters();

  distanciaMin = distanciaFiltrada(distanciaIzq, distanciaDer);

  Serial.print("Izq: "); Serial.print(distanciaIzq);
  Serial.print(" | Der: "); Serial.print(distanciaDer);
  Serial.print(" | MinFiltrada: "); Serial.println(distanciaMin);
}


// =======================================================
// Motores independientes
// =======================================================
void voidMotor(unsigned long currentMillis) {
  static unsigned long lastToggleIzq = 0;
  static unsigned long lastToggleDer = 0;
  static bool motorIzqActivo = false;
  static bool motorDerActivo = false;

  int pwm = PWM_POWER;

  // Motor Izquierdo
  if (!motorOn || !motorFuncEnabled || distanciaIzq <= 0 || distanciaIzq > 2000) {
    ledcWrite(MOTOR_IZQ, 0);
  } else {
    int vibrateIntervalIzq;

    if      (distanciaIzq > 1500) vibrateIntervalIzq = 900;
    else if (distanciaIzq > 1000) vibrateIntervalIzq = 650;
    else if (distanciaIzq > 750)  vibrateIntervalIzq = 450;
    else if (distanciaIzq > 400)  vibrateIntervalIzq = 300;
    else                          vibrateIntervalIzq = 150;

    if (currentMillis - lastToggleIzq >= vibrateIntervalIzq) {
      motorIzqActivo = !motorIzqActivo;
      ledcWrite(MOTOR_IZQ, motorIzqActivo ? pwm : 0);
      lastToggleIzq = currentMillis;
    }
  }

  // Motor Derecho
  if (!motorOn || !motorFuncEnabled || distanciaDer <= 0 || distanciaDer > 2000) {
    ledcWrite(MOTOR_DER, 0);
  } else {
    int vibrateIntervalDer;

    if      (distanciaDer > 1500) vibrateIntervalDer = 900;
    else if (distanciaDer > 1000) vibrateIntervalDer = 650;
    else if (distanciaDer > 750)  vibrateIntervalDer = 450;
    else if (distanciaDer > 400)  vibrateIntervalDer = 300;
    else                          vibrateIntervalDer = 150;

    if (currentMillis - lastToggleDer >= vibrateIntervalDer) {
      motorDerActivo = !motorDerActivo;
      ledcWrite(MOTOR_DER, motorDerActivo ? pwm : 0);
      lastToggleDer = currentMillis;
    }
  }
}

// =======================================================
// Buzzer
// =======================================================
void voidBuzzer(unsigned long currentMillis, int distancia) {
  if (!buzzerFuncEnabled || !buzzerOn || distancia == 0) {
    ledcWrite(BUZZER_PIN, 0);
    buzzerState = false;
    return;
  }

  int beepInterval;

  if      (distancia > 1500) beepInterval = 900;
  else if (distancia > 1000) beepInterval = 650;
  else if (distancia > 750)  beepInterval = 450;
  else if (distancia > 400)  beepInterval = 300;
  else                       beepInterval = 100;

  if (currentMillis - lastBeepTime >= (unsigned long)beepInterval) {
    lastBeepTime = currentMillis;
    buzzerState = !buzzerState;

    if (buzzerState)
      ledcWrite(BUZZER_PIN, 800);
    else
      ledcWrite(BUZZER_PIN, 0);
  }
}

// =======================================================
// BLE envío periódico
// =======================================================
void enviarBLE() {
  String data = "MI,BI";
  pCharacteristic->setValue(data.c_str());
  pCharacteristic->notify();

  Serial.print("BLE -> ");
  Serial.println(data);
}

// =======================================================
// LOOP
// =======================================================
void loop() {
  unsigned long now = millis();

  if (now - lastTOFRead >= intervaloTOF) {
    lastTOFRead = now;
    leerSensores();
  }

  voidMotor(now);
  voidBuzzer(now, distanciaMin);

  if (deviceConnected && now - lastBLETime >= bleInterval) {
    lastBLETime = now;
    enviarBLE();
  }
}