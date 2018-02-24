#pragma once
//--------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "Core.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreLogic
{
  public:

    CoreLogic();
    void begin(const uint8_t* defaultConfig, size_t configSize);
    void update();

    bool onUnknownCommand(const String& command, Stream* outStream);
    void onCoreStarted();
    void onIncomingCall(const String& phoneNumber, bool isKnownNumber, bool& shouldHangUp);
    void onIncomingSMS(const String& phoneNumber,const String& message, bool isKnownNumber);
    void onIncomingRS485Data(Stream* stream, uint16_t dataToRead);
    void onIncomingLoRaData(uint8_t* packet, int16_t packetSize);

  private:

    void doBegin();
    void doUpdate();
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreLogic Logic;
//--------------------------------------------------------------------------------------------------------------------------------------
