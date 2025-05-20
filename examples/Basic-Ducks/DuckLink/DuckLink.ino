
/**
 * @file DuckLink.ino
 * @author
 * @brief This example creates a duck link that sends a counter message periodically.
 *
 * It's using a pre-built DuckLink available from the ClusterDuck SDK
 */

#include <arduino-timer.h>
#include <string>
#include <vector>
#include <CDP.h>


#ifdef SERIAL_PORT_USBVIRTUAL
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

// create a built-in duck link
DuckLink duck;
bool sendSensorData();
bool runSensor(void *);
bool sendBurstPacket(void *);
void scheduleNextPacket();
// create a timer with default settings
auto timer = timer_create_default();

// for sending the counter message
const int INTERVAL_MS = 5000;
int counter = 1;
int burstCount = 0;
int burstSent = 0;  

bool setupOK = false;

void setup() {
  // We are using a hardcoded device id here, but it should be retrieved or
  // given during the device provisioning then converted to a byte vector to
  // setup the duck NOTE: The Device ID must be exactly 8 bytes otherwise it
  // will get rejected
  std::string deviceId("BURSTLNK");
  std::array<byte,8> devId;
  int rc;
  std::copy(deviceId.begin(), deviceId.end(), devId.begin());
  rc = duck.setupWithDefaults(devId);
  if (rc != DUCK_ERR_NONE) {
    Serial.print("[LINK] Failed to setup ducklink: rc = ");
    Serial.println(rc);
    return;
  }

  setupOK = true;
  // Initialize the timer. The timer thread runs separately from the main loop
  // and will trigger sending a counter message.
  Serial.println("[LINK] Setup OK!");

  if (sendSensorData()) {
    timer.every(INTERVAL_MS, runSensor);
  } else {
    Serial.println("[LINK] ERROR - Failed to send sensor data");
  }
}

void loop() {
  if (!setupOK) {
    return; 
  }
  
  timer.tick();
  duck.run();
}

std::vector<byte> stringToByteVector(const std::string& str) {
    std::vector<byte> byteVec;
    byteVec.reserve(str.length());

    for (unsigned int i = 0; i < str.length(); ++i) {
        byteVec.push_back(static_cast<byte>(str[i]));
    }

    return byteVec;
}

bool sendSensorData() {
  bool result = false;
  const byte* buffer;

  std::string message = std::string("Counter:") + std::to_string(counter);
  Serial.print("[LINK] sensor data: ");
  Serial.println(message.c_str());
  std::vector<byte> data = stringToByteVector(message);
  int err = duck.sendData(topics::status, data.data(),data.size());
  if (err == DUCK_ERR_NONE) {
     result = true;
     counter++;
  } else {
    std::string errMessage = "[LINK] Failed to send data. error = " + std::to_string(err);
    Serial.println(errMessage.c_str());
    return false;
  }
  return result;
}

bool runSensor(void *) {
  burstCount = random(2, 6);
  burstSent = 0;
  Serial.print("[LINK] Starting burst of ");
  Serial.print(burstCount);
  Serial.println(" packets");

  // Schedule first packet immediately
  scheduleNextPacket();
  return true;
}

void scheduleNextPacket() {
  if (burstSent < burstCount) {
    int delayMs = random(300, 1200);  // random delay between 100 and 1000 ms
    timer.in(delayMs, sendBurstPacket);
  }
}

// Timer callback to send one packet and schedule the next one
bool sendBurstPacket(void *) {
  std::array<byte,8> dduid = { 'M', 'A', 'M', 'A', '0', '0', '0', '1' };
  std::string message = "Sending Burst Packet " + std::to_string(burstSent) + "|" + std::to_string(burstCount);
  int err = duck.sendData(topics::status, message, dduid);
  burstSent++;
  scheduleNextPacket();
  return false; // one-shot timer, so return false
}
