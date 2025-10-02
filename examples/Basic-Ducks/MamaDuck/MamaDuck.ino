#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <SparkFun_GridEYE_Arduino_Library.h>
#include <CDP.h>
#include <arduino-timer.h>

// --- Pin & Timing Definitions ---
#define SDA_PIN      21
#define SCL_PIN      22
#define SMOOTH_N     2
#define INTERVAL_MS  300

// --- Topic IDs (Safe Zone) ---
#define TOPIC_SENSOR1 0x31
#define TOPIC_SENSOR2 0x32
#define TOPIC_SENSOR3 0x33
#define TOPIC_SENSOR4 0x34

// --- Globals ---
GridEYE grideye;

MamaDuck duck;
auto timer = timer_create_default();

// --- Send CSV String to CDP ---
void sendPacket(byte topic, const String& payload) {
  std::vector<byte> data(payload.begin(), payload.end());
  Serial.printf("[MAMA] Sending topic 0x%02X: %s\n", topic, payload.c_str());
  int result = duck.sendData(topic, data);
  Serial.printf("[MAMA] Topic 0x%02X send: %s\n", topic, result == 0 ? "✅ SUCCESS" : "❌ FAIL");
}

// --- Grid-EYE Data Collection and Transmission ---
bool sendGridEyeData(void*) {

  std::string message = "";

  for (int i = 0; i < 64; i++) {
    message += std::to_string(static_cast<int>(grideye.getPixelTemperature(i)));
    if(i!=63){
      message += ",";
    }
    
  }

  Serial.print("[MAMA]: GridEye Data: ");
  Serial.println(message.c_str());
  sendPacket(topics::sensor, String(message.c_str()));

  delay(100);
  return true;
}

// --- Setup ---
void setup() {
  WiFi.mode(WIFI_OFF);
  btStop();

  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  grideye.begin();


  duck.setupSerial();
  duck.setupRadio();
  std::array<byte, 8> deviceId = {'G','R','I','D','0','0','0','1'};
  duck.setDeviceId(deviceId);
  duck.setChannel(1);

  timer.every(INTERVAL_MS, sendGridEyeData);
}

// --- Loop ---
void loop() {
  timer.tick();
  duck.run();
}