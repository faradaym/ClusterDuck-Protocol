#ifndef DETECTORDUCK_H
#define DETECTORDUCK_H

#include "Duck.h"
#include "../wifi/DuckWifi.h"

template <typename WifiCapability = DuckWifi, typename RadioType = DuckLoRa>
class DetectorDuck : public Duck<DuckWifi, RadioType> {
public:
  using Duck<WifiCapability, RadioType>::Duck;

  DetectorDuck(std::string name = "DETECTOR") : Duck<DuckWifi, RadioType>(std::move(name)) {}
  ~DetectorDuck() {}

  /// callback definition for receiving RSSI value
  using rxDoneCallback = void (*)(CdpPacket data);
  
  /**
   * @brief Regsiter a callback for receiving and handling RSSI value
   *
   * @param rssiCb a call back defined with the following signature: `void (*)(const int)`
   */
  void onReceiveDuckData(rxDoneCallback cb) { this->recvDataCallback = cb; }

  /**
   * @brief Get the DuckType
   *
   * @returns the duck type defined as DuckType
   */
  DuckType getType() { return DuckType::DETECTOR; }

private:
  rxDoneCallback recvDataCallback;

  void handleReceivedPacket(CdpPacket rxPacket) {
    loginfo_ln("====> handleReceivedPacket: START");

    if (rxPacket.topic == reservedTopic::pong) {
      logdbg("run() - got ping response!");
      CdpPacket signalDataPacket = rxPacket;

      float tRssi = this->duckRadio.getRSSI();
      float tSnr = this->duckRadio.getSNR();
      JsonDocument doc;
      doc["rssi"] = tRssi;
      doc["snr"] = tSnr;

      int signalScore; //1-10
      float normalizedRssi = (tRssi - RSSI_MIN)/(RSSI_MAX-RSSI_MIN);
      float normalizedSnr = (tSnr - SNR_MIN)/(SNR_MAX-SNR_MIN);
      signalScore = ((normalizedRssi + normalizedSnr) / 2.0f) * 10;

      doc["signalScore"] = signalScore;

      std::string jsonString;
      serializeJson(doc, jsonString);

      signalDataPacket.data = std::vector<byte>(jsonString.begin(), jsonString.end());

      if (recvDataCallback) recvDataCallback(signalDataPacket);
    } 
  }
};
#endif