#include "CoreTransport.h"
#include "Core.h"
//--------------------------------------------------------------------------------------------------------------------------------------
extern "C" {
static void __nolora(byte* b, int dummy){}
static void __noclientconnect(CoreTransportClient& client) {}
static void __noclientdatareceived(CoreTransportClient& client) {}
static void __noclientwritedone(CoreTransportClient& client, bool isWriteSucceeded) {}
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_LORA_RECEIVE(byte*, int) __attribute__ ((weak, alias("__nolora")));
//--------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_CLIENT_CONNECT(CoreTransportClient& client) __attribute__ ((weak, alias("__noclientconnect")));
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_CLIENT_DATA_RECEIVED(CoreTransportClient& client) __attribute__ ((weak, alias("__noclientdatareceived")));
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_CLIENT_WRITE_DONE(CoreTransportClient& client, bool isWriteSucceeded) __attribute__ ((weak, alias("__noclientwritedone")));
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_RS485_TRANSPORT_ENABLED
CoreRS485Settings RS485Settings;
CoreRS485 RS485;
#endif // CORE_RS485_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_ESP_TRANSPORT_ENABLED
ESPTransportSettingsClass ESPTransportSettings;
#endif // CORE_ESP_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransport::CoreTransport()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransport::~CoreTransport()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::setClientBusy(CoreTransportClient& client,bool busy)
{
  client.setBusy(busy);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::setClientID(CoreTransportClient& client, uint8_t id)
{
    client.setID(id);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::setClientConnected(CoreTransportClient& client, bool connected)
{
  client.setConnected(connected);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::setClientBuffer(CoreTransportClient& client,const uint8_t* buff, size_t sz)
{
  client.setBuffer(buff,sz);
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_RS485_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreRS485::CoreRS485()
{
  memset(&RS485Settings,0,sizeof(RS485Settings));
  RS485Settings.isMasterMode = true; // по умолчанию - в режиме мастера
  workStream = NULL;

  rsPacketPtr = (byte*)&rs485Packet;
  rs485WritePtr = 0;

  
/*  
  dataBuffer = NULL;
  dataBufferLen = 0;
  writeIterator = 0;
  
  machineState = rs485WaitingHeader;
*/  
}
//--------------------------------------------------------------------------------------------------------------------------------------
HardwareSerial* CoreRS485::getMyStream(byte SerialNumber)
{
#if defined(__AVR_ATmega2560__) || (defined (__arm__) && defined (__SAM3X8E__))  
  switch(SerialNumber)
  {
    case 0:
      return &Serial;
 
    case 1:
      return &Serial1;

    case 2:
      return &Serial2;

    case 3:
      return &Serial3;

    default:
      return NULL;
  }
#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
  return &Serial;
#elif defined(ESP8266)
  #error "NOT IMPLEMENTED!!!"
#else
  #error "Unknown target board!"
#endif        
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::begin()
{
  if(workStream) // надо закончить работу на старом порту
    workStream->end();

  workStream = getMyStream(RS485Settings.SerialNumber);
  workStream->end();
  
  if(RS485Settings.UARTSpeed > 0)
  {
    unsigned long uspeed = RS485Settings.UARTSpeed;
    uspeed *= 9600;
    workStream->begin(uspeed);
  }

  if(RS485Settings.DEPin > 0)
    pinMode(RS485Settings.DEPin,OUTPUT);

  if(RS485Settings.isMasterMode)
  {
    // работаем в режиме мастера
    switchToSend();
  }
  else
    switchToReceive();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::switchToReceive()
{
  if(!RS485Settings.DEPin)
    return;
    
  digitalWrite(RS485Settings.DEPin,LOW); // переводим контроллер RS-485 на приём
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::switchToSend()
{
  if(!RS485Settings.DEPin)
    return;

  digitalWrite(RS485Settings.DEPin,HIGH); // переводим контроллер RS-485 на передачу
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreRS485::gotRS485Packet()
{
  // проверяем, есть ли у нас валидный RS-485 пакет
  return rs485WritePtr > ( sizeof(CoreTransportPacket)-1 );
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::processRS485Packet()
{

  // обрабатываем входящий пакет. Тут могут возникнуть проблемы с синхронизацией
  // начала пакета, поэтому мы сначала ищем заголовок и убеждаемся, что он валидный. 
  // если мы нашли заголовок и он не в начале пакета - значит, с синхронизацией проблемы,
  // и мы должны сдвинуть заголовок в начало пакета, чтобы потом дочитать остаток.
  if(!(rs485Packet.header1 == CORE_HEADER1 && rs485Packet.header2 == CORE_HEADER2 && rs485Packet.header3 == CORE_HEADER3))
  {
     // заголовок неправильный, ищем возможное начало пакета
     byte readPtr = 0;
     bool startPacketFound = false;
     while(readPtr < sizeof(CoreTransportPacket))
     {
       if(rsPacketPtr[readPtr] == CORE_HEADER1)
       {
        startPacketFound = true;
        break;
       }
        readPtr++;
     } // while

     if(!startPacketFound) // не нашли начало пакета
     {
        rs485WritePtr = 0; // сбрасываем указатель чтения и выходим
        return;
     }

     if(readPtr == 0)
     {
      // стартовый байт заголовка найден, но он в нулевой позиции, следовательно - что-то пошло не так
        rs485WritePtr = 0; // сбрасываем указатель чтения и выходим
        return;       
     } // if

     // начало пакета найдено, копируем всё, что после него, перемещая в начало буфера
     byte writePtr = 0;
     byte bytesWritten = 0;
     while(readPtr < sizeof(CoreTransportPacket) )
     {
      rsPacketPtr[writePtr++] = rsPacketPtr[readPtr++];
      bytesWritten++;
     }

     rs485WritePtr = bytesWritten; // запоминаем, куда писать следующий байт


     return;
         
  } // if
  else
  {
    // заголовок правильный 
       
        // отвечаем на пакет
         byte crc = Core.crc8((byte*)&rs485Packet,sizeof(CoreTransportPacket)- 1);
      
         if(crc != rs485Packet.crc)
         {
          String str = F("BAD CRC!");
          sendData((byte*)str.c_str(),str.length());
          rs485WritePtr = 0;
          return;
         }
      
        if(!(rs485Packet.header1 == CORE_HEADER1 && rs485Packet.header2 == CORE_HEADER2 && rs485Packet.header3 == CORE_HEADER3))
        {
          String str = F("BAD HEADERS!");
          sendData((byte*)str.c_str(),str.length());
          rs485WritePtr = 0;
          return;
        }
      
        if(rs485Packet.clusterID != Core.ClusterID)
        {
          String str = F("BAD CLUSTER!");
          sendData((byte*)str.c_str(),str.length());
          rs485WritePtr = 0;
          return;
        }
      
      
        if(!(rs485Packet.packetType == CoreDataRequest || rs485Packet.packetType == CoreSensorData))
        {
          String str = F("BAD PACKET TYPE!");
          sendData((byte*)str.c_str(),str.length());
          rs485WritePtr = 0;
          return;
        }
      
        // заполняем пакет данными
        switch(rs485Packet.packetType)
        {
          case CoreDataRequest:
          {
            // попросили отдать, скольку у нас датчиков, но сначала проверим - нам ли пакет?
            CoreDataRequestPacket* drp = (CoreDataRequestPacket*) rs485Packet.packetData;
            
            if(drp->toDeviceID != Core.DeviceID) // пакет не нам
            {
              String str = F("NOT FOT MY DEVICE!");
              sendData((byte*)str.c_str(),str.length());
              rs485WritePtr = 0;
              return;
            }
      
            // говорим, что у нас N датчиков
            drp->dataCount = CoreDataStore.size();

            drp->toDeviceID = rs485Packet.deviceID;
            rs485Packet.deviceID = Core.DeviceID;
      
            // пересчитываем CRC
            rs485Packet.crc = Core.crc8((byte*)&rs485Packet,sizeof(CoreTransportPacket)- 1);
      
            // пишем в поток
            sendData((byte*)&rs485Packet,sizeof(CoreTransportPacket));
            
          }
          break; // CoreDataRequest
      
          case CoreSensorData:
          {
             // попросили отдать данные с датчика, в первом байте данных пакета - лежит номер датчика, в нулевом байте - номер устройства на шине
             byte requestID = rs485Packet.packetData[0];
             byte sensorNumber = rs485Packet.packetData[1];
      
             if(requestID != Core.DeviceID) // пакет не нам
             {
              String str = F("NOT FOT MY DEVICE!");
              sendData((byte*)str.c_str(),str.length());
              rs485WritePtr = 0;
              return;
             }

              rs485Packet.deviceID = Core.DeviceID;
      
              CoreSensorDataPacket* sdp = (CoreSensorDataPacket*) rs485Packet.packetData;
              memset(sdp->sensorName,0,sizeof(sdp->sensorName));

              // тут получаем данные с запрошенного датчика
              CoreStoredData storedData = CoreDataStore.get(sensorNumber);

              if(storedData.sensor) // есть датчик в хранилище
              {
                String sensorName = storedData.sensor->getName();
                strcpy(sdp->sensorName, sensorName.c_str());
                if(storedData.hasData())
                {
                  sdp->hasData = 1;
                  sdp->dataType = (byte) CoreSensor::getDataType(storedData.sensor->getType());
                  sdp->dataLen = storedData.dataSize;
                  memcpy(sdp->data,storedData.data,storedData.dataSize);
                }
                else
                {
                  sdp->hasData = 0;
                }
              } // if(storedData.sensor)
              else
              {
                sdp->hasData = 0xFF; // датчик не найден
              }
      
              // отсылаем данные взад
              rs485Packet.crc = Core.crc8((byte*)&rs485Packet,sizeof(CoreTransportPacket)- 1);
              sendData((byte*)&rs485Packet,sizeof(CoreTransportPacket));
          }
          break; // CoreSensorData
          
        } // switch

    rs485WritePtr = 0; // обнуляем указатель записи, т.к. всё уже приняли и обработали

  } // правильный заголовок
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::processIncomingRS485Packets() // обрабатываем входящие пакеты по RS-485
{
  if(!workStream)
    return;
    
  while(workStream->available())
  {
    rsPacketPtr[rs485WritePtr++] = (byte) workStream->read();
   
    if(gotRS485Packet())
    {
      processRS485Packet();
    }
  } // while
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::updateSlaveMode()
{

  processIncomingRS485Packets();
  
/*
 if(!workStream->available()) // нет данных с потока
    return;

  while(workStream->available())
  {

    switch(machineState)
    {

      case rs485WaitingHeader:
      {
    
        byte bIncomingByte = (byte) workStream->read();
        dataBuffer[writeIterator] = bIncomingByte;
        writeIterator++;

        #ifdef _CORE_DEBUG
          Serial.print(F("RS-485: waiting header, receive: "));
          Serial.println(Core.byteToHexString(bIncomingByte));
        #endif
    
        if(writeIterator > dataBufferLen)
        {
          #ifdef _CORE_DEBUG
            Serial.println(F("RS-485: MALFORMED 1"));
          #endif
          writeIterator = 0;
          continue;
        }
    
        // тут смотрим - есть ли хоть один пакет с заголовком, соответствующим пакетам в списке.
        // для этого проходимся по принятым данным, и ищем там заголовок одного из известных пакетов
    
        bool knownPacketFound = false;
        
        for(byte i=0;i<writeIterator;i++)
        {
          for(size_t k=0;k<knownHeaders.size();k++)
          {
              if(!memcmp(&(dataBuffer[i]),knownHeaders[k].header,knownHeaders[k].headerLen))
              {
                  #ifdef _CORE_DEBUG
                    Serial.print(F("RS-485: FOUND KNOWN HEADER - "));
                    for(byte q=0;q<knownHeaders[k].headerLen;q++)
                    {
                      Serial.print(Core.byteToHexString(knownHeaders[k].header[q]));
                      Serial.print(' ');
                    }
                    Serial.println();
                  #endif
                          
                currentHeader = &(knownHeaders[k]); // запомнили текущий известный заголовок
                knownPacketFound = true; // нашли известный пакет, начиная с позиции i, перемещаем его в голову приёмного буфера
                for(byte in=i, out=0; in <  writeIterator; in++, out++)
                {
                  dataBuffer[out] = dataBuffer[in];
                }
    
                writeIterator -= i; // уменьшаем позицию записи, поскольку мы с позиции записи i переместили всё в голову приёмного буфера

                // переключаемся на чтение собственно данных пакета
                machineState = rs485WaitingData;
              }
          } // for
    
          if(knownPacketFound)
            break;
            
        } // for
    
        if(!knownPacketFound && writeIterator >= dataBufferLen)
        {
          // не найдено ни одного известного пакета, сдвигаем всё в голову на 1 байт, но только в том случае, если мы забили весь приёмный буфер!!!
          for(byte in=1,out=0; in < writeIterator;in++,out++)
          {
             dataBuffer[out] = dataBuffer[in];
          }
          writeIterator--;
        }

      } // case rs485WaitingHeader
      break;

      case rs485WaitingData:
      {

        byte bIncomingByte = (byte) workStream->read();
        dataBuffer[writeIterator] = bIncomingByte;
        writeIterator++;

        if(writeIterator >= (currentHeader->headerLen + currentHeader->packetDataLen))
        {
          // получили пакет полностью, надо формировать событие
            ON_RS485_RECEIVE(currentHeader->packetID,writeIterator, dataBuffer);
          
          #ifdef _CORE_DEBUG
            Serial.print(F("RECEIVE RS-485 PACKET #"));
            Serial.println(currentHeader->packetID);

            // просто печатаем пакет побайтово
            for(byte p=0;p<writeIterator;p++)
            {
              Serial.print(Core.byteToHexString(dataBuffer[p]));
              Serial.print(' ');
            }
            Serial.println();
            
          #endif

          machineState = rs485WaitingHeader;
          writeIterator = 0;
          // очищаем буфер в 0
          memset(dataBuffer,0,dataBufferLen);
        }
        
      }
      break; // rs485WaitingData

    } // switch(machineState)
   
  } // workStream->available()
*/  
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::updateMasterMode()
{

  if(!workStream)
    return;
    
  // тут обновляем данные, посылая в линию запросы на получение информации с датчиков

   static unsigned long past = 0;
   unsigned long now = millis();

   static int currentClientNumber = 0;

   unsigned long pollInterval = 0;//CORE_RS485_POLL_INTERVAL;

   // когда мало клиентов - минимальное время опроса между ними надо увеличивать, т.к. незачем дёргать два несчастных клиента несколько раз в секунду.
   // для этого мы смотрим - если живых клиентов мало, то мы пересчитываем интервал обновления.
   byte offlineClients = getOfflineModulesCount();
   byte aliveClients = CORE_RS485_MAX_ADDRESS - offlineClients + 1;

   // мы должны опросить aliveClients за CORE_RS485_ROUNDTRIP секунд
   unsigned long roundtrip = CORE_RS485_ROUNDTRIP;
   roundtrip *= 1000;

   // вычисляем, как часто опрашивать
   pollInterval = roundtrip/aliveClients;
     
   // проверяем, чтобы всё равно был минимальный интервал
   if(pollInterval < CORE_RS485_POLL_INTERVAL)
    pollInterval = CORE_RS485_POLL_INTERVAL;

  if(now - past > pollInterval)
  {

    // смотрим, не находится ли модуль в списке исключений, т.е. долго не отвечал
    while(inExcludedList(currentClientNumber))
    {
      currentClientNumber++;
      if(currentClientNumber > CORE_RS485_MAX_ADDRESS)
      {
        currentClientNumber = 0; // всё, нечего опрашивать, на шине никого нет
        return;
      }
    }

     unsigned long tmoDivider = RS485Settings.UARTSpeed;
     tmoDivider *= 9600;
     unsigned long readTimeout  = (10000000ul/tmoDivider)*10; // кол-во микросекунд, необходимое для вычитки десяти байт

/*     
     #ifdef _CORE_DEBUG
      Serial.print(F("RS485: Poll client #"));
      Serial.println(currentClientNumber);
     #endif
*/
     // тут формируем пакет, который запрашивает очередное устройство на шине
     CoreTransportPacket packet;
     packet.header1 = CORE_HEADER1;
     packet.header2 = CORE_HEADER2;
     packet.header3 = CORE_HEADER3;

     packet.clusterID = Core.ClusterID;
     packet.deviceID = Core.DeviceID;

     packet.packetType = CoreDataRequest; // это пакет с запросом информации о кол-ве данных на шине у устройства

     CoreDataRequestPacket drp;
     drp.toDeviceID = currentClientNumber;
     drp.dataCount = 0;

     memcpy(packet.packetData,&drp,sizeof(CoreDataRequestPacket));

     // считаем CRC
     packet.crc = Core.crc8((byte*) &packet, sizeof(CoreTransportPacket)-1);

     // теперь пишем в шину запрос
     sendData((byte*)&packet,sizeof(CoreTransportPacket));
/*
     #ifdef _CORE_DEBUG
      Serial.print(F("RS485: Data for client #"));
      Serial.print(currentClientNumber);
      Serial.println(F(" was sent, waiting answer..."));
     #endif
*/
     // обнуляем пакет
     memset(&packet,0,sizeof(CoreTransportPacket));
     byte bytesReaded = 0;
     bool received = false;
     byte* writePtr = (byte*)&packet;
     
     // принимаем данные
     
     // запоминаем время начала чтения
     unsigned long startReadingTime = micros();

  // начинаем читать данные
      while(1)
      {
        if( micros() - startReadingTime > readTimeout)
        {
          /*
          #ifdef _CORE_DEBUG
            Serial.print(F("RS485: client #"));
            Serial.print(currentClientNumber);
            Serial.println(F(" timeout!"));
          #endif
          */
          break;
        } // if

        if(workStream->available())
        {
          startReadingTime = micros(); // сбрасываем таймаут
          byte bIncoming = (byte) workStream->read();
          *writePtr++ = bIncoming;
          bytesReaded++;
        } // if available

        if(bytesReaded == sizeof(CoreTransportPacket)) // прочитали весь пакет
        {
          #ifdef _CORE_DEBUG
            Serial.println(F("RS485: Packet received from client!"));
          #endif

          received = true;
          break;
        }
    
     } // while

     if(received)
     {
        // получили полный пакет, проверяем CRC
        byte crc = Core.crc8((byte*)&packet,sizeof(CoreTransportPacket)-1);

        if(crc == packet.crc)
        {
          #ifdef _CORE_DEBUG
            Serial.println(F("RS485: CRC OK, continue..."));
          #endif

          // контрольная сумма совпала, проверяем другие данные
          CoreDataRequestPacket* requestPacket = (CoreDataRequestPacket*) packet.packetData;
          if(packet.header1 == CORE_HEADER1 && 
          packet.header2 == CORE_HEADER2 && 
          packet.header3 == CORE_HEADER3 && 
          packet.clusterID == Core.ClusterID && 
          packet.deviceID == currentClientNumber && 
          requestPacket->toDeviceID == Core.DeviceID)
          {
            #ifdef _CORE_DEBUG
              Serial.println(F("RS485: our packet, parse..."));
            #endif  

            // получаем кол-во показаний
            byte dataCount = requestPacket->dataCount;

            // теперь для каждого из показаний посылаем запрос на чтение этих показаний, чтобы добавить их в систему
            for(byte kk=0;kk < dataCount; kk++)
            {
              // формируем пакет запроса показаний, и отправляем его на шину
              packet.packetType = CoreSensorData; // запрашиваем показания с датчиков
              packet.packetData[0] =  currentClientNumber; //говорим, какое устройство опрашивается
              packet.packetData[1] = kk; // говорим, с какого датчика запрашиваем

              // считаем CRC
              packet.crc = Core.crc8((byte*)&packet,sizeof(CoreTransportPacket)-1);

              // отсылаем данные
              sendData((byte*)&packet,sizeof(CoreTransportPacket));
              startReadingTime = micros();
              bytesReaded = 0;
              writePtr = (byte*)&packet;

              // читаем из шины ответ
                while(1)
                {
                  if( micros() - startReadingTime > readTimeout)
                  {
                    /*
                    #ifdef _CORE_DEBUG
                      Serial.print(F("RS485: client #"));
                      Serial.print(currentClientNumber);
                      Serial.println(F(" not answering!"));
                    #endif
                    */
                    
                    break;
                  } // if
          
                  if(workStream->available())
                  {
                    startReadingTime = micros(); // сбрасываем таймаут
                    *writePtr++ = (byte) workStream->read();
                    bytesReaded++;
                  } // if available
          
                  if(bytesReaded == sizeof(CoreTransportPacket)) // прочитали весь пакет
                  {
                    #ifdef _CORE_DEBUG
                      Serial.println(F("RS485: Data packet received from client!"));
                    #endif

                    byte crc = Core.crc8((byte*)&packet,sizeof(CoreTransportPacket)-1);
          
                      // тут разбираем, что там пришло
                      if(packet.header1 == CORE_HEADER1 && 
                        packet.header2 == CORE_HEADER2 && 
                        packet.header3 == CORE_HEADER3 && 
                        packet.clusterID == Core.ClusterID && 
                        packet.deviceID == currentClientNumber &&
                        packet.crc == crc &&
                        packet.packetType == CoreSensorData)
                        {
                              // нам пришла информация о датчике, добавляем его в систему
                            DBGLN(F("RS485: Sensor data packet received, parse..."));

                            CoreSensorDataPacket* sensorData = (CoreSensorDataPacket*) packet.packetData;
                            bool hasSensorOnSlave = (sensorData->hasData != 0xFF);

                            if(hasSensorOnSlave)
                            {
                                  
                              String sensorName;
                              
                              char* namePtr = sensorData->sensorName;
                              while(*namePtr)
                              {
                                sensorName += *namePtr++;
                              }
      
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
                                if(sensorData->hasData == 1)
                                {
                                  // данные есть, можно читать
                                  newSensor->setData(sensorData->data,sensorData->dataLen);
                                }
                                else
                                  newSensor->setData(NULL,0); // сбрасываем данные
      
                                // назначаем тип данных
                                newSensor->setUserDataType((CoreDataType)sensorData->dataType);
                              
                                // также помещаем его показания в хранилище
                                Core.pushToStorage(newSensor);
                                
                                DBGLN(F("RS485: Userdata sensor added!"));                          
                                
                              } // if(newSensor)   

                            } // if(hasSensorOnSlave)
                            else
                            {
                              DBGLN(F("RS485: Sensor on slave not found!!!"));
                            }
                                              
                        } // if good packet

                    break;
                  }
              
               } // while              
              
            } // for
            
          }
          /*
          else
          {
            #ifdef _CORE_DEBUG
              Serial.println(F("RS485: bad packet!"));
            #endif                      
          }
          */
          
        } // crc
        /*
        else
        {
          #ifdef _CORE_DEBUG
            Serial.println(F("RS485: BAD CRC!"));
          #endif          
        }
        */
      
     } // if(received)
     
     else
     {
      /*
          #ifdef _CORE_DEBUG
          if(!bytesReaded)
          {
            Serial.print(F("RS485: NO DATA FROM CLIENT #"));
            Serial.println(currentClientNumber);
          }
          else
          {
            Serial.print(F("RS485: client UNCOMPLETED DATA - "));
              // выводим ответ клиента
              char* ch = (char*)&packet;
              for(byte zz=0;zz<bytesReaded;zz++)
                Serial.print((char)*ch++);

             Serial.println();
          }
          #endif
          */

          // ничего не получили с модуля, добавляем в список несуществующих на шине
          addToExcludedList(currentClientNumber);
                          
     } // else
    

     currentClientNumber++;

     if(currentClientNumber > CORE_RS485_MAX_ADDRESS) // начнём сначала
      currentClientNumber = 0;

     past = millis();
  } // if

}
//--------------------------------------------------------------------------------------------------------------------------------------
byte CoreRS485::getOfflineModulesCount()
{
  byte result = 0;

 for(size_t i=0;i<excludedList.size();i++)
  {
    if(excludedList[i].readingAttempts >= CORE_RS485_MAX_BAD_READINGS)
      result++;
  }  

  return result;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreRS485::inExcludedList(byte clientNumber)
{
  for(size_t i=0;i<excludedList.size();i++)
  {
    if(excludedList[i].clientNumber == clientNumber && excludedList[i].readingAttempts >= CORE_RS485_MAX_BAD_READINGS)
      return true;
  }

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::addToExcludedList(byte clientNumber)
{
  for(size_t i=0;i<excludedList.size();i++)
  {
    if(excludedList[i].clientNumber == clientNumber)
    {
      excludedList[i].readingAttempts++;
      if(excludedList[i].readingAttempts >= CORE_RS485_MAX_BAD_READINGS)
      {
        excludedList[i].readingAttempts = CORE_RS485_MAX_BAD_READINGS;

        #ifdef _CORE_DEBUG
          Serial.print(F("RS485: Client #"));
          Serial.print(clientNumber);
          Serial.println(F(" excluded from query!"));
        #endif
      }

        return;
    }
  }

  CoreRS485ExcludeRecord rec;
  rec.clientNumber = clientNumber;
  rec.readingAttempts = 1;

  excludedList.push_back(rec);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::update()
{
  if(/*!dataBuffer || */!workStream) // нет буфера для данных или неизвестный Serial
    return;


  if(RS485Settings.isMasterMode)
    updateMasterMode();
  else
    updateSlaveMode();
    

 
  
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::clear()
{
  /*
  // тут очищаем наши данные
  delete [] dataBuffer;
  dataBuffer = NULL;
  dataBufferLen = 0;
  writeIterator = 0;
  machineState = rs485WaitingHeader;

  for(size_t i=0;i<knownHeaders.size();i++)
  {
    delete [] knownHeaders[i].header;
  }

  while(knownHeaders.size())
    knownHeaders.pop();
    */
}
//--------------------------------------------------------------------------------------------------------------------------------------
/*
void CoreRS485::addKnownPacketHeader(byte* header, byte headerSize, byte packetDataLen, byte packetID)
{
  RS485IncomingHeader knownHeader;
  
  knownHeader.headerLen = headerSize;
  knownHeader.header = new byte[headerSize];
  memcpy(knownHeader.header,header,headerSize);
  knownHeader.packetDataLen = packetDataLen;
  knownHeader.packetID = packetID;

  knownHeaders.push_back(knownHeader);

  // смотрим - хватает ли нам места под самый большой из известных заголовков пакетов на шине
  if(dataBufferLen < (headerSize + packetDataLen))
  {
    delete [] dataBuffer;
    dataBufferLen = headerSize + packetDataLen;
    dataBuffer = new byte[dataBufferLen];
  }
  
}
*/
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::waitTransmitComplete()
{
  if(!workStream)
    return;
    
  #if defined(__AVR_ATmega2560__)

    if(workStream == &Serial)
      while(!(UCSR0A & _BV(TXC0) ));
    else
    if(workStream == &Serial1)
      while(!(UCSR1A & _BV(TXC1) ));
    else
    if(workStream == &Serial2)
      while(!(UCSR2A & _BV(TXC2) ));
    else
    if(workStream == &Serial3)
      while(!(UCSR3A & _BV(TXC3) ));

  #elif defined (__arm__) && defined (__SAM3X8E__) // Arduino Due compatible

    if(workStream == &Serial)
      while((USART0->US_CSR & UART_SR_TXRDY) != UART_SR_TXRDY);
    else
    if(workStream == &Serial1)
      while((USART1->US_CSR & UART_SR_TXRDY) != UART_SR_TXRDY);
    else
    if(workStream == &Serial2)
      while((USART2->US_CSR & UART_SR_TXRDY) != UART_SR_TXRDY);
    else
    if(workStream == &Serial3)
      while((USART3->US_CSR & UART_SR_TXRDY) != UART_SR_TXRDY);

  #elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)      
    while(!(UCSR0A & _BV(TXC0) ));
  #elif defined(ESP8266)
    #error "NOT IMPLEMENTED !!!"      
  #else
    #error "Unknown target board!"
  #endif  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::sendData(byte* data, byte dataSize)
{
  if(!workStream)
    return;

  switchToSend();
  workStream->write(data,dataSize);
  waitTransmitComplete();
  switchToReceive();
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_RS485_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_ESP_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreESPTransport ESP;
//--------------------------------------------------------------------------------------------------------------------------------------
CoreESPTransport::CoreESPTransport() : CoreTransport()
{
  for(int i=0;i<ESP_MAX_CLIENTS;i++)
    clients[i] = NULL;

    lastSerial = NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::update()
{
  //TODO: update ESP
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::begin()
{
  initClients();

  HardwareSerial* hs = NULL;

  #if defined(__AVR_ATmega2560__) || (defined (__arm__) && defined (__SAM3X8E__))
  
  switch(ESPTransportSettings.SerialNumber)
  {
    case 0:
      hs = &Serial;
    break;

    case 1:
      hs = &Serial1;
    break;

    case 2:
      hs = &Serial2;
    break;

    case 3:
      hs = &Serial3;
    break;

  } // switch
  #elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
    hs = &Serial;
  #elif defined(ESP8266)
    #error "NOT IMPLEMENTED!!!"
  #else
    #error "Unknown target board!"
  #endif    

  if(lastSerial) // надо закончить работу на старом порту
    lastSerial->end();

  lastSerial = hs;

  workStream = hs;
  unsigned long uspeed = ESPTransportSettings.UARTSpeed;
  uspeed *= 9600;

  hs->end();
  hs->begin(uspeed);

  while(writeOutQueue.size())
    writeOutQueue.pop();

  while(connectQueue.size())
    connectQueue.pop();

  while(disconnectQueue.size())
    disconnectQueue.pop();


  //TODO: тут переинициализация очередей и т.п. !!!
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::isInQueue(ESPClientsQueue& queue,CoreTransportClient* client)
{
  for(size_t i=0;i<queue.size();i++)
  {
    if(queue[i].client == client)
      return true;
  }

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::addToQueue(ESPClientsQueue& queue,CoreTransportClient* client, const char* ip, uint16_t port)
{
  if(isInQueue(queue,client))
    return;

    ESPClientQueueData dt;
    dt.client = client;
    dt.ip = NULL;
    if(ip)
    {
      dt.ip = new char[strlen(ip)+1];
      strcpy(dt.ip,ip);
    }
    dt.port = port;

    queue.push_back(dt);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::removeFromQueue(ESPClientsQueue& queue,CoreTransportClient* client)
{
  for(size_t i=0;i<queue.size();i++)
  {
    if(queue[i].client == client)
    {
      delete [] queue[i].ip;
        for(size_t j=i+1;j<queue.size();j++)
        {
          queue[i] = queue[j];
        }
        queue.pop();
        break;
    }
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::initClients()
{
  for(int i=0;i<ESP_MAX_CLIENTS;i++)
  {
    if(clients[i])
      clients[i]->Destroy();
      
    CoreTransportClient* client = CoreTransportClient::Create(this);
    setClientID(*client,i);
    clients[i] = client;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransportClient* CoreESPTransport::getFreeClient()
{
  //TODO: Тут проверяем - готовы ли мы к работе вообще (законнекчены к роутеру и т.п.)
  
  for(int i=0;i<ESP_MAX_CLIENTS;i++)
  {
    if(!clients[i]->connected() && !clients[i]->busy()) // возвращаем первого неподсоединённого и незанятого чем-либо клиента
      return clients[i];
  }

  return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::beginWrite(CoreTransportClient& client)
{
  #ifdef _CORE_DEBUG
    Serial.println(F("Write to ESP client..."));
  #endif

  if(!client.connected())
  {
    #ifdef _CORE_DEBUG
      Serial.println(F("ESP client not connected!"));
    #endif

    return;
  }

  //говорим, что клиент занят
  setClientBusy(client,true);
  
  // добавляем клиента в очередь на запись
  addToQueue(writeOutQueue,&client);

  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с записью в поток с этого клиента
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::beginConnect(CoreTransportClient& client, const char* ip, uint16_t port)
{
   #ifdef _CORE_DEBUG
    Serial.println(F("Connect ESP client to IP..."));
  #endif 

  if(client.connected())
  {
    #ifdef _CORE_DEBUG
      Serial.println(F("ESP client already connected!"));
    #endif

    return;
  }

  //говорим, что клиент занят
  setClientBusy(client,true);

  // добавляем клиента в очередь на соединение
  addToQueue(connectQueue,&client, ip, port);

  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с соединением клиента

  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::beginDisconnect(CoreTransportClient& client)
{
   #ifdef _CORE_DEBUG
    Serial.println(F("Disconnect ESP client..."));
  #endif 

  if(!client.connected())
  {
    #ifdef _CORE_DEBUG
      Serial.println(F("ESP client not connected!"));
    #endif

    return;
  }

  //говорим, что клиент занят
  setClientBusy(client,true);

  // добавляем клиента в очередь на соединение
  addToQueue(disconnectQueue,&client);

  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с отсоединением клиента
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_ESP_TRANSPORT_ENABLED


