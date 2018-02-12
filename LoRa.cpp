// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "LoRa.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_LORA_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
LoRaSettingsStruct LoRaSettings;
//--------------------------------------------------------------------------------------------------------------------------------------
// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255
//--------------------------------------------------------------------------------------------------------------------------------------
LoRaClass::LoRaClass() :
  _spiSettings(8E6, MSBFIRST, SPI_MODE0),
  _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
  _frequency(0),
  _packetIndex(0),
  _implicitHeaderMode(0),
  _onReceive(NULL)
{
  // overide Stream timeout value
  setTimeout(0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int LoRaClass::begin(long frequency)
{
  // setup pins
  pinMode(_ss, OUTPUT);
  // set SS high
  digitalWrite(_ss, HIGH);

  if (_reset != -1) {
    pinMode(_reset, OUTPUT);

    // perform reset
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    delay(10);
  }

  // start SPI
  SPI.begin();

  // check version
  uint8_t version = readRegister(REG_VERSION);
  if (version != 0x12) {
    return 0;
  }

  // put in sleep mode
  sleep();

  // set frequency
  setFrequency(frequency);

  // set base addresses
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

  // set LNA boost
  writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

  // set auto AGC
  writeRegister(REG_MODEM_CONFIG_3, 0x04);

  // set output power to 17 dBm
  setTxPower(17);

  // put in standby mode
  idle();

  return 1;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::end()
{
  // put in sleep mode
  sleep();

  // stop SPI
  SPI.end();
}
//--------------------------------------------------------------------------------------------------------------------------------------
int LoRaClass::beginPacket(int implicitHeader)
{
  // put in standby mode
  idle();

  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  // reset FIFO address and paload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);

  return 1;
}
//--------------------------------------------------------------------------------------------------------------------------------------
int LoRaClass::endPacket()
{
  // put in TX mode
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  // wait for TX done
  while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
    yield();
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

  return 1;
}
//--------------------------------------------------------------------------------------------------------------------------------------
int LoRaClass::parsePacket(int size)
{
  int packetLength = 0;
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    idle();
  } else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);

    // put in single RX mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

  return packetLength;
}
//--------------------------------------------------------------------------------------------------------------------------------------
int LoRaClass::packetRssi()
{
  return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}
//--------------------------------------------------------------------------------------------------------------------------------------
float LoRaClass::packetSnr()
{
  return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}
//--------------------------------------------------------------------------------------------------------------------------------------
size_t LoRaClass::write(uint8_t byte)
{
  return write(&byte, sizeof(byte));
}
//--------------------------------------------------------------------------------------------------------------------------------------
size_t LoRaClass::write(const uint8_t *buffer, size_t size)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}
//--------------------------------------------------------------------------------------------------------------------------------------
int LoRaClass::available()
{
  return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int LoRaClass::read()
{
  if (!available()) {
    return -1;
  }

  _packetIndex++;

  return readRegister(REG_FIFO);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int LoRaClass::peek()
{
  if (!available()) {
    return -1;
  }

  // store current FIFO address
  int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

  // read
  uint8_t b = readRegister(REG_FIFO);

  // restore FIFO address
  writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

  return b;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::flush()
{
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::onReceive(void(*callback)(int))
{
  _onReceive = callback;

  if (callback) {
    writeRegister(REG_DIO_MAPPING_1, 0x00);

    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::receive(int size)
{
  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::idle()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::sleep()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::setTxPower(int level, int outputPin)
{
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }

    writeRegister(REG_PA_CONFIG, 0x70 | level);
  } else {
    // PA BOOST
    if (level < 2) {
      level = 2;
    } else if (level > 17) {
      level = 17;
    }

    writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::setFrequency(long frequency)
{
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::setSpreadingFactor(int sf)
{
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }

  writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::setSignalBandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    bw = 9;
  }

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::setPreambleLength(long length)
{
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::setSyncWord(int sw)
{
  writeRegister(REG_SYNC_WORD, sw);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::enableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::disableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}
//--------------------------------------------------------------------------------------------------------------------------------------
byte LoRaClass::random()
{
  return readRegister(REG_RSSI_WIDEBAND);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::setPins(int ss, int reset, int dio0)
{
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::setSPIFrequency(uint32_t frequency)
{
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::dumpRegisters(Stream& out)
{
  for (int i = 0; i < 128; i++) {
    out.print("0x");
    out.print(i, HEX);
    out.print(": 0x");
    out.println(readRegister(i), HEX);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::explicitHeaderMode()
{
  _implicitHeaderMode = 0;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::implicitHeaderMode()
{
  _implicitHeaderMode = 1;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::handleDio0Rise()
{
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    if (_onReceive) {
      _onReceive(packetLength);
    }

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t LoRaClass::readRegister(uint8_t address)
{
  return singleTransfer(address & 0x7f, 0x00);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::writeRegister(uint8_t address, uint8_t value)
{
  singleTransfer(address | 0x80, value);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value)
{
  uint8_t response;

  digitalWrite(_ss, LOW);

  SPI.beginTransaction(_spiSettings);
  SPI.transfer(address);
  response = SPI.transfer(value);
  SPI.endTransaction();

  digitalWrite(_ss, HIGH);

  return response;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoRaClass::onDio0Rise()
{
  LoRa.handleDio0Rise();
}
//--------------------------------------------------------------------------------------------------------------------------------------
LoRaClass LoRa;
//--------------------------------------------------------------------------------------------------------------------------------------
// LoraDispatcherClass
//--------------------------------------------------------------------------------------------------------------------------------------
LoraDispatcherClass LoraDispatcher;
//--------------------------------------------------------------------------------------------------------------------------------------
LoraDispatcherClass::LoraDispatcherClass()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoraDispatcherClass::begin()
{
    if(!LoRaSettings.enabled)
      return;
  
    int8_t reset = LoRaSettings.reset == 0xFF ? -1 : LoRaSettings.reset;

    LoRa.setPins(LoRaSettings.ss, reset, LoRaSettings.dio);

    long frequency = 915E6;
    switch(LoRaSettings.frequency)
    {
      case 1:
        frequency = 433E6;
      break;

      case 2:
        frequency = 866E6;
      break;

      case 3:
        frequency = 915E6;
      break;
    }
    
    LoRa.begin(frequency);

    LoRa.setTxPower(LoRaSettings.txPower);

    long signalBandwidth = 125E3;
    
    switch(LoRaSettings.bandwidth)
    {
      case 1:
        signalBandwidth = 7.8E3;
      break;
      
      case 2:
        signalBandwidth = 10.4E3;
      break;
      
      case 3:
        signalBandwidth = 15.6E3;
      break;
      
      case 4:
        signalBandwidth = 20.8E3;
      break;
      
      case 5:
        signalBandwidth = 31.25E3;
      break;
      
      case 6:
        signalBandwidth = 41.7E3;
      break;
      
      case 7:
        signalBandwidth = 62.5E3;
      break;
      
      case 8:
        signalBandwidth = 125E3;
      break;
      
      case 9:
        signalBandwidth = 250E3;
      break;
    }

    
    LoRa.setSignalBandwidth(signalBandwidth);

    if(LoRaSettings.useCrc)
      LoRa.enableCrc();
    else
      LoRa.disableCrc();
  
    LoRa.onReceive(coreLoraReceive);
    LoRa.receive(); // переключаемся на приём

  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoraDispatcherClass::update()
{
    if(!LoRaSettings.enabled)
      return;

  
  #ifndef CORE_LORA_DISABLE_CORE_LOGIC
  
    if(LoRaSettings.isMasterMode)
    {
      updateMasterMode();
    }
    else
    {
      updateSlaveMode();
    }
    
  #endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoraDispatcherClass::reset()
{
  if(!LoRaSettings.enabled)
    return;

  LoRaSettings.retransmitCount = CORE_LORA_RETRANSMIT_COUNT;
  LoRaSettings.sendDuration = CORE_LORA_SEND_DURATION;
  LoRaSettings.sendDuration *= 1000;

#ifndef CORE_LORA_DISABLE_CORE_LOGIC

  sendWaitTime = getDefaultSendWaitTime();

  slaveState = lssSendData; // отсылаем данные
  slaveTimer = millis();

  slaveSensorNumber = 0;
  slaveReceiptReceived = false;
  slaveFailTransmits = 0;
  
#endif  
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifndef CORE_LORA_DISABLE_CORE_LOGIC
//--------------------------------------------------------------------------------------------------------------------------------------
unsigned long LoraDispatcherClass::getDefaultSendWaitTime()
{
  unsigned long swt = CORE_LORA_TIME_SHIFT;
  swt *= Core.DeviceID;
  
  return swt;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoraDispatcherClass::updateMasterMode()
{
  //TODO: тут работа в режиме мастера !!!

}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoraDispatcherClass::updateSlaveMode()
{
  // работа в режиме слейва

  if(!CoreDataStore.size()) // собственно, у нас нет датчиков, нечего отсылать
    return;

  /* 
      алгоритм работы слейва: отсылаем следующий пакет данных, запоминаем время начала отсыла.
      если пакет данных отослан успешно (получена квитанция от мастера) - то
      отсылаем следующий. Если пакет не обработан мастером - то повторяем отправку через CORE_LORA_DATA_SHIFT*Core.DeviceID.
      После N неуспешных попыток отправки - считаем, что мастер вообще не отвечает, и засыпаем на N секунд.
   */
   unsigned long now = millis();

   switch(slaveState)
   {
      case lssSendData:
      {
          // режим отсылки данных, проверяем, можем ли мы отослать пакет?
          if(now - slaveTimer > sendWaitTime)
          {
              DBGLN(F("LoRa: want to send next data packet!"));

              slaveReceiptReceived = false; // считаем, что квитанция не получена
              
              // переходим в ожидание квитанции
              slaveState = lssWaitReceipt;

              // отсылаем пакет с показаниями следующего датчика
              sendSensorDataPacket();

              DBGLN(F("LoRa: data packet was sent."));

              // и запоминаем время, когда мы отослали пакет
              slaveTimer = millis();
          } // if
      }
      break; // lssSendData

      case lssWaitReceipt:
      {
          // ждём получения квитанции от мастера
          if(slaveReceiptReceived)
          {
            
            // квитанция получена, можно отсылать следующие данные

              slaveFailTransmits = 0; // обнуляем счётчик неудачных попыток отсыла
            
              slaveSensorNumber++; // на следующий датчик
              slaveTimer = millis();

              if(slaveSensorNumber >= CoreDataStore.size())
              {
                DBGLN(F("LoRa: receipt received, cycle done, go to sleep..."));
                
                // дошли до конца списка, надо чуть-чуть поспать
                slaveSensorNumber = 0;
                slaveState = lssSleep;
              }
              else
              {
                DBGLN(F("LoRa: receipt received, switch to next sensor..."));
                
                sendWaitTime = 0; // обнулили таймер ожидания, т.к. мы успешно послали пакет, и следующий можно посылать сразу же,
                // в надежде на то, что мастер будет свободен
                
                slaveState = lssSendData; // переключаемся на передачу опять
              }
          } // if(slaveReceiptReceived)
          else
          {
            // квитанция всё ещё не получена, проверяем на таймаут
            if(now - slaveTimer > CORE_LORA_RECEIPT_TIMEOUT)
            {
              // таймаут, проверяем, как у нас дела с попытками перепосылки
              slaveFailTransmits++;
              
              if(slaveFailTransmits < LoRaSettings.retransmitCount)
              {
              
                DBGLN(F("LoRa: No receipt received, try to retransmit..."));
                
                sendWaitTime = getDefaultSendWaitTime(); // сдвигаем время следующего отсыла данных на маленькую величину
                slaveTimer = millis();
  
                slaveState = lssSendData; // переключаемся на отсыл
              } // if
              else
              {
                  DBGLN(F("LoRa: Master not answering, sleep for some time..."));
                  
                  slaveFailTransmits = 0;
                  slaveState = lssSleep;
                  slaveTimer = millis();
                
              } // else
              
            } // if
          } // else
      }
      break; // lssWaitReceipt

      case lssSleep:
      {
        // тут спим определённое время
        if(now - slaveTimer > LoRaSettings.sendDuration)
        {
          DBGLN(F("LoRa: Sleep done, switch to send mode..."));
          slaveState = lssSendData;
          slaveTimer = millis();
        }
      }
      break; // lssSleep
      
   } // switch


   
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool LoraDispatcherClass::checkHeaders(uint8_t* packet)
{
  return (packet[0] ==  CORE_HEADER1 && packet[1] ==  CORE_HEADER2 && packet[2] ==  CORE_HEADER3);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoraDispatcherClass::sendSensorDataPacket()
{
  DBG(F("LoRa: Send data - sensor #"));
  DBGLN(slaveSensorNumber);
  
  
  CoreTransportPacket packet;
  memset(&packet,0,sizeof(CoreTransportPacket));
  
  packet.header1 = CORE_HEADER1;
  packet.header2 = CORE_HEADER2;
  packet.header3 = CORE_HEADER3;

  packet.clusterID = Core.ClusterID;
  packet.deviceID = Core.DeviceID;
  packet.packetType = CoreSensorData;

  CoreSensorDataPacket* sdp = (CoreSensorDataPacket*) &(packet.packetData);
  
  CoreStoredData storedData = CoreDataStore.get(slaveSensorNumber);
  
  if(storedData.sensor)
  {
      // есть такой датчик
      String sensorName = storedData.sensor->getName();
      strcpy(sdp->sensorName, sensorName.c_str());
      sdp->dataType = (uint8_t) CoreSensor::getDataType(storedData.sensor->getType());

      if(storedData.hasData())
      {
          sdp->hasData = 1;
          sdp->dataLen = storedData.dataSize;
          memcpy(sdp->data,storedData.data,storedData.dataSize);
      }
      else
      {
        sdp->hasData = 0;
      }
    
  } // if
  else
    sdp->hasData = 0xFF; // датчик не найден нахрен

  packet.crc = Core.crc8((uint8_t*)&packet,sizeof(CoreTransportPacket)- 1);

  // пакет сформирован, отсылаем
  LoRa.beginPacket(); 
  LoRa.write((uint8_t*)&packet,sizeof(CoreTransportPacket)); 
  LoRa.endPacket();

  LoRa.receive();  // переключаемся на приём   
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoraDispatcherClass::parseSensorDataPacket(CoreTransportPacket* packet)
{
  DBGLN(F("LoRa: Sensor data packet detected, parse..."));

  CoreSensorDataPacket* sensorData = (CoreSensorDataPacket*) packet->packetData;

  if(sensorData->hasData == 0xFF)
  {
    DBGLN(F("LoRa: Sensor not found on slave, can't add local!"));

    // отправляем пустую квитанцию, что пакет обработан
    sendDataReceipt(packet,"");
    return;
  }
  
  String sensorName;
  
  char* namePtr = sensorData->sensorName;
  while(*namePtr)
  {
    sensorName += *namePtr++;
  }

  #ifdef CORE_USERDATA_SENSOR_ENABLED
  
  CoreUserDataSensor* newSensor = (CoreUserDataSensor*) Core.Sensors()->get(sensorName);
  
  if(!newSensor)
  {
    newSensor = (CoreUserDataSensor*) CoreSensorsFactory::createSensor(UserDataSensor);
   // добавляем в список датчиков
    Core.Sensors()->add(newSensor);
  }

  if(newSensor)
  {
    newSensor->setName(sensorName); // даём датчику имя

    // назначаем данные
    if(sensorData->hasData == 1) // только если они есть                           
      newSensor->setData(sensorData->data,sensorData->dataLen);
    else // говорим, что датчик поломатый, и с него не стало данных
      newSensor->setData(NULL,0);

    // назначаем тип данных
    newSensor->setUserDataType((CoreDataType)sensorData->dataType);


    // также помещаем его показания в хранилище
    Core.pushToStorage(newSensor);
    
    DBGLN(F("LoRa: Userdata sensor added!"));                          
    
  } // if(newSensor)
  
  #endif // CORE_USERDATA_SENSOR_ENABLED

   //тут необходимо посылать клиенту квитанцию, что данные получены.
   // если клиент не получит квитанцию об обработке данных мастером - 
   // он попытается переслать эти данные позже.
   sendDataReceipt(packet,sensorName);
   
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoraDispatcherClass::sendDataReceipt(CoreTransportPacket* packet, const String& sensorName)
{
  uint8_t destDeviceID = packet->deviceID;
  packet->deviceID = Core.DeviceID; // говорим, что пакет от нас

  DBG(F("LoRa: Send receipt to device #"));
  DBGLN(destDeviceID);

  CoreDataReceiptPacket* receipt = (CoreDataReceiptPacket*) packet->packetData;
  memset(receipt,0,sizeof(CoreDataReceiptPacket));

  receipt->toDeviceID = destDeviceID;
  strcpy(receipt->sensorName,sensorName.c_str());

  packet->packetType = CoreDataReceipt;

  packet->crc = Core.crc8((uint8_t*)packet,sizeof(CoreTransportPacket)- 1);

  // пакет сформирован, отсылаем
  LoRa.beginPacket();
  LoRa.write((uint8_t*)packet,sizeof(CoreTransportPacket));
  LoRa.endPacket();

  LoRa.receive();  // переключаемся на приём 

  DBGLN(F("LoRa: Receipt was sent."));
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LoraDispatcherClass::parseDataReceiptPacket(CoreTransportPacket* packet)
{
  // разбираем пакет с квитанцией о получении данных мастером
  DBGLN(F("LoRa: Receipt received, parse..."));

  CoreDataReceiptPacket* receipt = (CoreDataReceiptPacket*) packet->packetData;

  // проверяем, нам ли это квитанция?
  if(receipt->toDeviceID != Core.DeviceID)
  {
    DBGLN(F("LoRa: Receipt not for me, ignore."));
    return;
  }

  // квитанция пришла нам, следовательно, мы выставляем флаг успешного получения квитанции, чтобы работать дальше
  slaveReceiptReceived = true;
  
  DBGLN(F("LoRa: Receipt accepted."));
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool LoraDispatcherClass::parsePacket(uint8_t* bPacket, int packetSize)
{
  if(packetSize != sizeof(CoreTransportPacket))
    return false;

   // длина данных совпадает, проверяем заголовок
   if(!checkHeaders(bPacket))
    return false;

   CoreTransportPacket* packet = (CoreTransportPacket*) bPacket;

    DBGLN(F("LoRa: packet header OK!"));
    // пакет точно наш, проверяем CRC
    uint8_t crc = CoreClass::crc8(bPacket,packetSize-1);
    if(crc == packet->crc)
    {
       // контрольная сумма совпадает, продолжаем
       DBGLN(F("LoRa: CRC OK"));

       if(packet->clusterID == Core.ClusterID)
       {
          DBGLN(F("LoRa: Our cluster, continue..."));

          if(packet->deviceID != Core.DeviceID)
          {
            DBGLN(F("LoRa: Remote device detected, continue..."));

            switch(packet->packetType)
            {
              case CoreSensorData: // пакет с датчиками
              { 
                // пакет с датчиками мы обрабатываем только в режиме мастера, т.к. только мастер собирает данные от слейвов
                if(!LoRaSettings.isMasterMode)
                  return true;

                parseSensorDataPacket(packet);
                
              }
              return true; // CoreSensorData
              
              case CoreDataReceipt:
              {
                // пришла квитанция о получении данных мастером
                
                if(LoRaSettings.isMasterMode) // в режиме мастера мы игнорируем этот пакет
                  return true;

                 parseDataReceiptPacket(packet);
              }
              return true; // CoreDataReceipt


              //TODO: Тут другие типы пакетов !!!

              
            } // switch
          }
          else
          {
            DBGLN(F("LoRa: Same device ID, ignore!"));
          }
       }
       else
       {
          DBGLN(F("LoRa: Unknown cluster!"));
       }
       
    }
    else
    {
      DBGLN(F("LoRa: CRC Fail!"));
    }

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_LORA_DISABLE_CORE_LOGIC
//--------------------------------------------------------------------------------------------------------------------------------------
void LoraDispatcherClass::coreLoraReceive(int packetSize)
{
 

  // тут пришёл пакет от LoRa, и в зависимости от режима работы (мастер/слейв) - мы должны делать какие-либо действия
  
  if(packetSize > 0)
  {
    uint8_t* bPacket  = new uint8_t[packetSize];
    uint8_t cntr = 0;
    while(LoRa.available())
    {
      bPacket[cntr] = LoRa.read();
      cntr++;
    }
    #ifndef CORE_LORA_DISABLE_CORE_LOGIC
    if(!LoraDispatcher.parsePacket(bPacket,packetSize))
    {
      // не распарсили пакет, отправляем в событие
      ON_LORA_RECEIVE(bPacket,packetSize);
    }
  #else
    // логика ядра выключена, просто вызываем событие
    ON_LORA_RECEIVE(bPacket,packetSize);
  #endif
    delete [] bPacket;
    
  } // if


}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_LORA_TRANSPORT_ENABLED
