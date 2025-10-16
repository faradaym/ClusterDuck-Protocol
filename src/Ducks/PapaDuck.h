#ifndef PAPADUCK_H
#define PAPADUCK_H

#include "Duck.h"
#include "../wifi/DuckWifi.h"

template <typename WifiCapability = DuckWifi, typename RadioType = DuckLoRa>
class PapaDuck : public Duck<WifiCapability, RadioType> {
public:
  using Duck<WifiCapability, RadioType>::Duck;
  
  PapaDuck(std::string name = "PAPADUCK") : Duck<WifiCapability, RadioType>(std::move(name)) {}
  ~PapaDuck() {}

  /// Papa Duck callback functions signature.
  using rxDoneCallback = void (*)(CdpPacket data);
  using txDoneCallback = void (*)(void);
  /**
   * @brief Register callback for handling data received from duck devices
   * 
   * The callback will be invoked if the packet needs to be relayed (i.e not seen before)
   * @param cb a callback to handle data received by the papa duck
   */
  void onReceiveDuckData(rxDoneCallback cb) { this->recvDataCallback = cb; }

  /**
   * @brief Get the DuckType
   *
   * @returns the duck type defined as DuckType
   */
  DuckType getType() { return DuckType::PAPA; }

  /**
   * @brief Send duck command to entire network
   *
   * The PapaDuck can send command messages to remote ducks. There are
   * two parts for each command. Only PapaDucks are able to send 
   * commands to remote ducks. See `MamaDuck.cpp` for available commands.
   * All ducks will execute this command and relay.
   *
   * @param cmd byte enum for command to be executed.
   * @param value contextual data to be used in executed command.
   */
  void sendCommand(byte cmd, std::vector<byte> value) {
    loginfo_ln("Initiate sending command");
    std::vector<byte> dataPayload;
    dataPayload.push_back(cmd);
    dataPayload.insert(dataPayload.end(), value.begin(), value.end());
    
    int err = this->sendReservedTopicData(this->dduid, reservedTopic::cmd, dataPayload);
  
    if (err != DUCK_ERR_NONE) {
      logerr_ln("ERROR handleReceivedPacket. Failed to send cmd. Error: %d",err);
    } 
  };

  /**
   * @brief Send duck command to specific duck
   *
   * The PapaDuck can send command messages to remote ducks. There are
   * two parts for each command. Only PapaDucks are able to send 
   * commands to remote ducks. See `MamaDuck.cpp` for available commands. 
   *
   * @param cmd byte enum for command to be executed.
   * @param value contextual data to be used in executed command.
   * @param dduid destination duck ID for command to be executed.
   */
  void sendCommand(byte cmd, std::vector<byte> value, std::array<byte,8> dduid);

  //remove this when mqtt quack pack is added
  bool isWifiConnected(){
    return this->duckWifi.connected();
  }

private:
  rxDoneCallback recvDataCallback;
  
  void handleReceivedPacket() override{
    if (this->duckRadio.getReceiveFlag()){
        bool relay = false;
        
        loginfo_ln("====> handleReceivedPacket: START");

        int err;
        std::optional<std::vector<uint8_t>> rxData = this->duckRadio.readReceivedData();
        if (!rxData) {
        logerr_ln("ERROR failed to get data from DuckRadio.");
        return;
        }
        CdpPacket rxPacket(rxData.value());

        this->duckRadio.getSignalScore();
        // Update routing table with signal info from last received packet
        this->router.insertIntoRoutingTable(duckutils::toString(rxPacket.sduid),
                                            this->duckRadio.signalInfo.signalScore,
                                            this->duckRadio.signalInfo.snr,
                                            this->duckRadio.signalInfo.rssi,
                                            millis());
        logdbg_ln("Got data from radio, prepare for relay. size: %d",rxPacket.size());

        // recvDataCallback(rxPacket.asBytes());
        loginfo_ln("handleReceivedPacket: packet RELAY START");
        
        //Check if Duck is desitination for this packet before relaying
        if (duckutils::isEqual(BROADCAST_DUID, rxPacket.dduid)) {
            ifBroadcast(rxPacket, err);
        } else if(duckutils::isEqual(this->duid, rxPacket.dduid)) { //Target device check
            ifNotBroadcast(rxPacket, err);
          /*
           * There needs to be a case for handling packets not addressed to this duck
           * but also not broadcast. This method may look very different once finished
           */
        } else { //Source device check
            ifNotBroadcast(rxPacket, err, true);
        }
    }
    }

    void ifBroadcast(CdpPacket rxPacket, int err) {
        switch(rxPacket.topic) {
            case reservedTopic::rreq: {
                loginfo_ln("RREQ received from %s. Updating RREQ!", rxPacket.sduid.data());
                RouteJSON rreqDoc = RouteJSON(rxPacket.asBytes());
                std::string rreq = rreqDoc.addToPath(this->deviceId);
                this->sendRouteResponse(rxPacket.sduid, rreq);
            }
            case reservedTopic::ping:
                loginfo_ln("PING received. Sending PONG!");
                err = this->sendPong();
                if (err != DUCK_ERR_NONE) {
                    logerr_ln("ERROR failed to send pong message. rc = %d",err);
                }
                return;
            case reservedTopic::pong:
                loginfo_ln("PONG received. Ignoring!");
                break;
            default:
                err = this->relayPacket(rxPacket);
                if (err != DUCK_ERR_NONE) {
                    logerr_ln("====> ERROR handleReceivedPacket failed to relay. rc = %d",err);
                } else {
                    loginfo_ln("handleReceivedPacket: packet RELAY DONE");
                }
        }
    }

    void ifNotBroadcast(CdpPacket rxPacket, int err, bool relay = false) {
        std::vector<uint8_t> dataPayload;
        uint8_t num = 1;

        switch(rxPacket.topic) {
            case reservedTopic::rreq: {
                RouteJSON rreqDoc = RouteJSON(rxPacket.asBytes());
                if(!relay) {
                    loginfo_ln("RREQ received. Updating RREQ!");

                    loginfo_ln("handleReceivedPacket: Sending RREP");
                    //add current duck to path
                    //update the rreq to make it into a rrep
                    //Serialize the updated RREQ packet
                    std::string strRREP = rreqDoc.addToPath(this->deviceId);
                    this->sendRouteResponse(PAPADUCK_DUID,
                                            dataPayload); //was this meant to be prepareforsending an rxPacket instead txPacket?
                    return;
                } else {
                    loginfo_ln("RREQ received for relay. Relaying!");
                    std::string packet = rreqDoc.addToPath(this->deviceId);
                    rxPacket.data = duckutils::stringToByteVector(packet);
                    err = this->relayPacket(rxPacket);
                    if (err != DUCK_ERR_NONE) {
                        logerr_ln("====> ERROR handleReceivedPacket failed to relay RREQ. rc = %d",err);
                    } else {
                        loginfo_ln("handleReceivedPacket: RREQ packet RELAY DONE");
                    }

                }
            }
            break;
            case reservedTopic::rrep: {
                loginfo_ln("Received Route Response from DUID: %s", duckutils::convertToHex(rxPacket.sduid.data(), rxPacket.sduid.size()).c_str());
                //extract path from rrep and update routing table
                RouteJSON rrepDoc = RouteJSON(rxPacket.asBytes());
                //if duck is not in a network already
                this->setNetworkState(NetworkState::PUBLIC);
                //send rrep to next hop in path
                std::string rrep = rrepDoc.removeFromPath(this->duid);;
                this->sendRouteResponse(rrepDoc.getlastInPath(),rrep);
                return;
            }
                break;
            default:
                err = this->relayPacket(rxPacket);
                if (err != DUCK_ERR_NONE) {
                    logerr_ln("====> ERROR handleReceivedPacket failed to relay. rc = %d",err);
                } else {
                    loginfo_ln("handleReceivedPacket: packet RELAY DONE");
                }
        }
    }
  };

#endif
