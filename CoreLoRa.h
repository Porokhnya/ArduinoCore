// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <SPI.h>
#include "CoreConfig.h"
#include "Core.h"

#ifdef CORE_LORA_TRANSPORT_ENABLED

#define LORA_DEFAULT_SS_PIN    10
#define LORA_DEFAULT_RESET_PIN 9
#define LORA_DEFAULT_DIO0_PIN  2

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1

//--------------------------------------------------------------------------------------------------------------------------------------
class LoRaClass : public Stream {
public:
  LoRaClass();

  int begin(long frequency);
  void end();

  int beginPacket(int implicitHeader = false);
  int endPacket();

  int parsePacket(int size = 0);
  int packetRssi();
  float packetSnr();

  // from Print
  virtual size_t write(uint8_t byte);
  virtual size_t write(const uint8_t *buffer, size_t size);

  // from Stream
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  void onReceive(void(*callback)(int));

  void receive(int size = 0);
  void idle();
  void sleep();

  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void enableCrc();
  void disableCrc();

  // deprecated
  void crc() { enableCrc(); }
  void noCrc() { disableCrc(); }

  byte random();

  void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN);
  void setSPIFrequency(uint32_t frequency);

  void dumpRegisters(Stream& out);

private:
  void explicitHeaderMode();
  void implicitHeaderMode();

  void handleDio0Rise();

  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

  static void onDio0Rise();

private:
  SPISettings _spiSettings;
  int _ss;
  int _reset;
  int _dio0;
  int _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  void (*_onReceive)(int);
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern LoRaClass LoRa;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  bool enabled;
  uint8_t frequency;
  uint8_t ss;
  uint8_t reset;
  uint8_t dio;
  uint8_t txPower;
  uint8_t bandwidth;
  bool useCrc;
  bool isMasterMode;
  uint8_t retransmitCount;
  unsigned long sendDuration;
  
} LoRaSettingsStruct;
//--------------------------------------------------------------------------------------------------------------------------------------
extern LoRaSettingsStruct LoRaSettings;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  lssSendData, // отсылаем данные
  lssWaitReceipt, // ждём квитанции
  lssSleep, // ожидаем между циклами отправки
  
} LoRaSlaveState;
//--------------------------------------------------------------------------------------------------------------------------------------
class LoraDispatcherClass
{
  public:

    LoraDispatcherClass();
  
    void begin();
    void update();
    void reset();


   private:

    static void coreLoraReceive(int packetSize);

    #ifndef CORE_LORA_DISABLE_CORE_LOGIC
    
      void updateMasterMode();
      void updateSlaveMode();
  
      bool parsePacket(uint8_t* packet, int16_t packetSize);
      bool checkHeaders(uint8_t* packet);
  
      // MASTER UTILS
      void parseSensorDataPacket(CoreTransportPacket* packet);
      void sendDataReceipt(CoreTransportPacket* packet, const String& sensorName); // отсылаем квитанцию о получении данных вызвавшей стороне
      
  
      // SLAVE UTILS
      uint32_t getDefaultSendWaitTime();
      uint32_t sendWaitTime; // сколько ждём до очередной итерации работы в режиме слейва
      uint32_t slaveTimer; // таймер для расчёта времени на всякие операции слейва
      LoRaSlaveState slaveState; // состояние конечного автомата слейва
      uint8_t slaveSensorNumber; // номер текущего датчика для отсыла данных
      bool slaveReceiptReceived; // флаг, что мы получили квитанцию
      uint8_t slaveFailTransmits; // кол-во неуспешных попыток отсыла пакетов
      void parseDataReceiptPacket(CoreTransportPacket* packet);
      void sendSensorDataPacket();

    #endif

  
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern LoraDispatcherClass LoraDispatcher;
//--------------------------------------------------------------------------------------------------------------------------------------

#endif // CORE_LORA_TRANSPORT_ENABLED

#endif
