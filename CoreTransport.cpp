#include "CoreTransport.h"
#include "Core.h"
//--------------------------------------------------------------------------------------------------------------------------------------
extern "C" {
static void __nolora(uint8_t* b, int dummy){}
static void __noincomingcall(const String& phoneNumber, bool isKnownNumber, bool& shouldHangUp) {}
static void __nosmsreceived(const String& phoneNumber, const String& message, bool isKnownNumber) {}
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_LORA_RECEIVE(uint8_t*, int) __attribute__ ((weak, alias("__nolora")));
void ON_INCOMING_CALL(const String& phoneNumber, bool isKnownNumber, bool& shouldHangUp) __attribute__ ((weak, alias("__noincomingcall")));
void ON_SMS_RECEIVED(const String& phoneNumber,const String& message, bool isKnownNumber) __attribute__ ((weak, alias("__nosmsreceived")));
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
// CoreTransportClient
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransportClient::CoreTransportClient()
{
  clientID = 0xFF;
  isConnected = false;
  isBusy = false;
  dataBuffer = NULL;
  dataBufferSize = 0;
  parent = NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransportClient::~CoreTransportClient()
{
  clearBuffer();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::setData(uint8_t* buff, size_t sz, bool copy)
{
  clearBuffer();
  
  dataBufferSize = sz;
  
  if(dataBufferSize)
  {
    if(copy) // попросили скопировать буфер
    {
      dataBuffer = new  uint8_t[dataBufferSize];
      memcpy(dataBuffer,buff,dataBufferSize);
    }
    else // нам просто передали буфер, который выделил сам транспорт
      dataBuffer = buff;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::setID(uint8_t id)
{
  clientID = id;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::setConnected(bool flag, int errorCode)
{
  // если была ошибка - по-любому постим событие с ошибкой
  if(errorCode != CT_ERROR_NONE)
  {
     isConnected = flag;
     raiseEvent(etConnect,errorCode);
     return;
  }

  // здесь нет ошибки, постим событие, только если изменился флаг
  if(isConnected != flag)
  {
    isConnected = flag;
    // постим подписчикам событие
    raiseEvent(etConnect,errorCode);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::setBusy(bool flag)
{
  isBusy = flag;
}
//--------------------------------------------------------------------------------------------------------------------------------------
size_t CoreTransportClient::getDataSize()
{
  return dataBufferSize;
}
//--------------------------------------------------------------------------------------------------------------------------------------
const uint8_t* CoreTransportClient::getData()
{
  return dataBuffer;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::clearBuffer()
{
    if(dataBuffer)
      delete [] dataBuffer; 

    dataBufferSize = 0;
    dataBuffer = NULL;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::disconnect()
{
    if(!connected())
      return;

    parent->beginDisconnect(*this);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreTransportClient::busy()
{
 return isBusy; 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::connect(const char* ip, uint16_t port)
{
    if(connected()) // уже присоединены, нельзя коннектится до отсоединения!!!
      return;
          
    parent->beginConnect(*this,ip,port);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreTransportClient::write(uint8_t* buff, size_t sz, bool takeBufferOwnership)
{
    if(!sz || !buff || !connected() || clientID == 0xFF)
    {
      DBGLN(F("Client - CAN'T WRITE!"));
      return false;
    }
/*
    DBG(F("Client #"));
    DBG(clientID);
    DBG(F(": write, data size="));
    DBGLN(sz);
*/
    setData(buff,sz,!takeBufferOwnership);
    parent->beginWrite(*this);

   return true;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransportClient* CoreTransportClient::Create(CoreTransport* transport)
{
    CoreTransportClient* instance = new CoreTransportClient();
    instance->parent = transport;
    return instance;    
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransport* CoreTransportClient::getTransport()
{
   return parent;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::Destroy()
{
  delete this;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreTransportClient::connected() 
{
  return isConnected;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreTransportClient::getID()
{
  return clientID;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::notifyDataWritten(int errorCode)
{
  // говорим, что данные записаны в поток
  raiseEvent(etDataWritten, errorCode);

  // и очищаем внутренний буфер с данными
 // clearBuffer(); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
int CoreTransportClient::getSubscriberIndex(IClientEventsSubscriber* subscriber)
{
  for(size_t i=0;i<subscribers.size();i++)
  {
    if(subscribers[i] == subscriber)
      return i;
  }

  return -1;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::subscribe(IClientEventsSubscriber* subscriber)
{
  if(!subscriber)
    return;
    
  int subIdx = getSubscriberIndex(subscriber);
  if(subIdx != -1) // этот подписчик уже подписан
    return;

   subscribers.push_back(subscriber);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::unsubscribe(IClientEventsSubscriber* subscriber)
{
  int subIdx = getSubscriberIndex(subscriber);
  
  if(subIdx == -1) // подписчик не найден
    return;

  for(size_t i=subIdx+1; i<subscribers.size();i++)
  {
    subscribers[i-1] = subscribers[i];
  }

  subscribers.pop();
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::raiseEvent(ClientEventType et, int errorCode)
{
  for(size_t i=0;i<subscribers.size();i++)
  {
    IClientEventsSubscriber* sub = subscribers[i];

    switch(et)
    {
      case etConnect:
        sub->OnClientConnect(*this,this->isConnected,errorCode);
      break;

      case etDataWritten:
        sub->OnClientDataWritten(*this, errorCode);
      break;

      case etDataAvailable:
        sub->OnClientDataAvailable(*this, errorCode);
      break;
      
    } // switch
  } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::notifyDataAvailable(bool isDone)
{
  // сообщаем подписчикам, что у нас есть данные
  raiseEvent(etDataAvailable,isDone);

}
//--------------------------------------------------------------------------------------------------------------------------------------
// CoreTransport
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransport::CoreTransport()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransport::~CoreTransport()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::subscribeClient(CoreTransportClient& client, IClientEventsSubscriber* subscriber)
{
  client.subscribe(subscriber);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::unsubscribeClient(CoreTransportClient& client, IClientEventsSubscriber* subscriber)
{
  client.unsubscribe(subscriber);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::notifyClientDataWritten(CoreTransportClient& client,int errorCode)
{
  client.notifyDataWritten(errorCode);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::notifyClientDataAvailable(CoreTransportClient& client, bool isDone)
{
  client.notifyDataAvailable(isDone);
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
void CoreTransport::setClientConnected(CoreTransportClient& client, bool isConnected, int errorCode)
{
  client.setConnected(isConnected, errorCode);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::setClientData(CoreTransportClient& client,uint8_t* buff, size_t sz)
{
  return client.setData(buff,sz,false);
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_RS485_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreRS485::CoreRS485()
{
  memset(&RS485Settings,0,sizeof(RS485Settings));
  RS485Settings.isMasterMode = true; // по умолчанию - в режиме мастера
  workStream = NULL;

#ifndef CORE_RS485_DISABLE_CORE_LOGIC

  rsPacketPtr = (uint8_t*)&rs485Packet;
  rs485WritePtr = 0;
  
#endif // CORE_RS485_DISABLE_CORE_LOGIC
  
/*  
  dataBuffer = NULL;
  dataBufferLen = 0;
  writeIterator = 0;
  
  machineState = rs485WaitingHeader;
*/  
}
//--------------------------------------------------------------------------------------------------------------------------------------
HardwareSerial* CoreRS485::getMyStream(uint8_t SerialNumber)
{
#if (TARGET_BOARD == MEGA_BOARD) || (TARGET_BOARD == DUE_BOARD)
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
#elif TARGET_BOARD == ESP_BOARD
  #error "NOT IMPLEMENTED!!!"
#else
  #error "Unknown target board!"
#endif        
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::begin()
{  

  workStream = NULL;

  if(!RS485Settings.enabled)
    return;
  
  if(RS485Settings.SerialNumber == 0 || RS485Settings.UARTSpeed == 0) // не можем работать через Serial или с нулевой скоростью!
    return;


  workStream = getMyStream(RS485Settings.SerialNumber);
    
    unsigned long uspeed = RS485Settings.UARTSpeed;
    uspeed *= 9600;
    workStream->begin(uspeed);

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
#ifndef CORE_RS485_DISABLE_CORE_LOGIC
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
     uint8_t readPtr = 0;
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
     uint8_t writePtr = 0;
     uint8_t bytesWritten = 0;
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
         uint8_t crc = Core.crc8((uint8_t*)&rs485Packet,sizeof(CoreTransportPacket)- 1);
      
         if(crc != rs485Packet.crc)
         {
          String str = F("BAD CRC!");
          sendData((uint8_t*)str.c_str(),str.length());
          rs485WritePtr = 0;
          return;
         }
      
        if(!(rs485Packet.header1 == CORE_HEADER1 && rs485Packet.header2 == CORE_HEADER2 && rs485Packet.header3 == CORE_HEADER3))
        {
          /*
          String str = F("BAD HEADERS!");
          sendData((uint8_t*)str.c_str(),str.length());
          */
          rs485WritePtr = 0;
          return;
        }
      
        if(rs485Packet.clusterID != Core.ClusterID)
        {
          /*
          String str = F("BAD CLUSTER!");
          sendData((uint8_t*)str.c_str(),str.length());
          */
          rs485WritePtr = 0;
          return;
        }
      
      
        if(!(rs485Packet.packetType == CoreDataRequest || rs485Packet.packetType == CoreSensorData))
        {
          /*
          String str = F("BAD PACKET TYPE!");
          sendData((uint8_t*)str.c_str(),str.length());
          */
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
              /*
              String str = F("NOT FOT MY DEVICE!");
              sendData((uint8_t*)str.c_str(),str.length());
              */
              rs485WritePtr = 0;
              return;
            }
      
            // говорим, что у нас N датчиков
            drp->dataCount = CoreDataStore.size();

            drp->toDeviceID = rs485Packet.deviceID;
            rs485Packet.deviceID = Core.DeviceID;
      
            // пересчитываем CRC
            rs485Packet.crc = Core.crc8((uint8_t*)&rs485Packet,sizeof(CoreTransportPacket)- 1);
      
            // пишем в поток
            sendData((uint8_t*)&rs485Packet,sizeof(CoreTransportPacket));
            
          }
          break; // CoreDataRequest
      
          case CoreSensorData:
          {
             // попросили отдать данные с датчика, в первом байте данных пакета - лежит номер датчика, в нулевом байте - номер устройства на шине
             uint8_t requestID = rs485Packet.packetData[0];
             uint8_t sensorNumber = rs485Packet.packetData[1];
      
             if(requestID != Core.DeviceID) // пакет не нам
             {
              /*
              String str = F("NOT FOT MY DEVICE!");
              sendData((uint8_t*)str.c_str(),str.length());
              */
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
              } // if(storedData.sensor)
              else
              {
                sdp->hasData = 0xFF; // датчик не найден
              }
      
              // отсылаем данные взад
              rs485Packet.crc = Core.crc8((uint8_t*)&rs485Packet,sizeof(CoreTransportPacket)- 1);
              sendData((uint8_t*)&rs485Packet,sizeof(CoreTransportPacket));
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
    rsPacketPtr[rs485WritePtr++] = (uint8_t) workStream->read();
   
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
   uint8_t offlineClients = getOfflineModulesCount();
   uint8_t aliveClients = CORE_RS485_MAX_ADDRESS - offlineClients + 1;

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
     packet.crc = Core.crc8((uint8_t*) &packet, sizeof(CoreTransportPacket)-1);


     // теперь пишем в шину запрос
     sendData((uint8_t*)&packet,sizeof(CoreTransportPacket));


     
/*
     #ifdef _CORE_DEBUG
      Serial.print(F("RS485: Data for client #"));
      Serial.print(currentClientNumber);
      Serial.println(F(" was sent, waiting answer..."));
     #endif
*/
     // обнуляем пакет
     memset(&packet,0,sizeof(CoreTransportPacket));
     uint8_t bytesReaded = 0;
     bool received = false;
     uint8_t* writePtr = (uint8_t*)&packet;
     
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
          uint8_t bIncoming = (uint8_t) workStream->read();
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
        uint8_t crc = Core.crc8((uint8_t*)&packet,sizeof(CoreTransportPacket)-1);

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
            uint8_t dataCount = requestPacket->dataCount;

            // теперь для каждого из показаний посылаем запрос на чтение этих показаний, чтобы добавить их в систему
            for(uint8_t kk=0;kk < dataCount; kk++)
            {
              // формируем пакет запроса показаний, и отправляем его на шину
              packet.packetType = CoreSensorData; // запрашиваем показания с датчиков
              packet.packetData[0] =  currentClientNumber; //говорим, какое устройство опрашивается
              packet.packetData[1] = kk; // говорим, с какого датчика запрашиваем

              // считаем CRC
              packet.crc = Core.crc8((uint8_t*)&packet,sizeof(CoreTransportPacket)-1);

              // отсылаем данные
              sendData((uint8_t*)&packet,sizeof(CoreTransportPacket));
              startReadingTime = micros();
              bytesReaded = 0;
              writePtr = (uint8_t*)&packet;

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
                    *writePtr++ = (uint8_t) workStream->read();
                    bytesReaded++;
                  } // if available
          
                  if(bytesReaded == sizeof(CoreTransportPacket)) // прочитали весь пакет
                  {
                    #ifdef _CORE_DEBUG
                      Serial.println(F("RS485: Data packet received from client!"));
                    #endif

                    uint8_t crc = Core.crc8((uint8_t*)&packet,sizeof(CoreTransportPacket)-1);
          
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

                              #endif // CORE_USERDATA_SENSOR_ENABLED

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
              for(uint8_t zz=0;zz<bytesReaded;zz++)
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
uint8_t CoreRS485::getOfflineModulesCount()
{
  uint8_t result = 0;

 for(size_t i=0;i<excludedList.size();i++)
  {
    if(excludedList[i].readingAttempts >= CORE_RS485_MAX_BAD_READINGS)
      result++;
  }  

  return result;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreRS485::inExcludedList(uint8_t clientNumber)
{
  for(size_t i=0;i<excludedList.size();i++)
  {
    if(excludedList[i].clientNumber == clientNumber && excludedList[i].readingAttempts >= CORE_RS485_MAX_BAD_READINGS)
      return true;
  }

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::addToExcludedList(uint8_t clientNumber)
{
  for(size_t i=0;i<excludedList.size();i++)
  {
    if(excludedList[i].clientNumber == clientNumber)
    {
      excludedList[i].readingAttempts++;
      if(excludedList[i].readingAttempts >= CORE_RS485_MAX_BAD_READINGS)
      {
        excludedList[i].readingAttempts = CORE_RS485_MAX_BAD_READINGS;
/*
        #ifdef _CORE_DEBUG
          Serial.print(F("RS485: Client #"));
          Serial.print(clientNumber);
          Serial.println(F(" excluded from query!"));
        #endif
*/
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
#endif // CORE_RS485_DISABLE_CORE_LOGIC
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::update()
{
  if(!RS485Settings.enabled)
    return;  
  
  if(!workStream) // нет буфера для данных или неизвестный Serial
    return;

  #ifndef CORE_RS485_DISABLE_CORE_LOGIC

  if(RS485Settings.isMasterMode)
    updateMasterMode();
  else
    updateSlaveMode();

    
  #endif // CORE_RS485_DISABLE_CORE_LOGIC
 
  
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::reset()
{
  if(!RS485Settings.enabled)
    return;
  
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

  #ifndef CORE_RS485_DISABLE_CORE_LOGIC
  
    rsPacketPtr = (uint8_t*)&rs485Packet;
    rs485WritePtr = 0;

    excludedList.empty();
    
  #endif // CORE_RS485_DISABLE_CORE_LOGIC
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
/*
void CoreRS485::addKnownPacketHeader(uint8_t* header, uint8_t headerSize, uint8_t packetDataLen, uint8_t packetID)
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
    dataBuffer = new uint8_t[dataBufferLen];
  }
  
}
*/
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::waitTransmitComplete()
{
  if(!workStream)
    return;
    
  #if TARGET_BOARD == MEGA_BOARD

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

  #elif TARGET_BOARD == DUE_BOARD

    if(workStream == &Serial)
      while((UART->UART_SR & UART_SR_TXRDY) != UART_SR_TXRDY);
    else
    if(workStream == &Serial1)
      while((USART0->US_CSR & US_CSR_TXEMPTY) == 0);
    else
    if(workStream == &Serial2)
      while((USART1->US_CSR & US_CSR_TXEMPTY)  == 0);      
    else
    if(workStream == &Serial3)
      while((USART3->US_CSR & US_CSR_TXEMPTY)  == 0);

  #elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)      
    while(!(UCSR0A & _BV(TXC0) ));
  #elif TARGET_BOARD == ESP_BOARD
    #error "NOT IMPLEMENTED !!!"      
  #else
    #error "Unknown target board!"
  #endif  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::sendData(uint8_t* data, uint8_t dataSize)
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

  wiFiReceiveBuff = new String();

}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::sendCommand(const String& command, bool addNewLine)
{
  DBG(F("ESP: ==>> "));
  DBGLN(command);
  
  workStream->write(command.c_str(),command.length());
  
  if(addNewLine)
  {
    workStream->println();
  }  

  machineState = espWaitAnswer; // говорим, что надо ждать ответа от ESP
  // запоминаем время отсылки последней команды
  timer = millis();
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::pingGoogle(bool& result)
{
    if(machineState != espIdle || !workStream || !ready() || initCommandsQueue.size()) // чего-то делаем, не могём
    {
      //DBGLN(F("ESP: BUSY!!!"));
      return false;
    }

        ESPKnownAnswer ka;
        workStream->println(F("AT+PING=\"google.com\""));
        // поскольку у нас serialEvent не основан на прерываниях, на самом-то деле (!),
        // то мы должны получить ответ вот прямо вот здесь, и разобрать его.

        String line; // тут принимаем данные до конца строки
        bool  pingDone = false;
        
        char ch;
        while(1)
        { 
          if(pingDone) // получили ответ на PING
            break;
            
          while(workStream->available())
          {
            ch = workStream->read();
        
            if(ch == '\r')
              continue;
            
            if(ch == '\n')
            {
              // получили строку, разбираем её
                 if(isKnownAnswer(line, ka))
                 {
                    result = (ka == kaOK);
                    pingDone = true;
                 }
             line = "";
            } // ch == '\n'
            else
              line += ch;
        
          if(pingDone) // получили ответ на PING
              break;
 
          } // while
          
        } // while(1)    


  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::getMAC(String& staMAC, String& apMAC)
{
    if(machineState != espIdle || !workStream || !ready() || initCommandsQueue.size()) // чего-то делаем, не могём
    {
      //DBGLN(F("ESP: BUSY!!!"));
      return false;
    }

        ESPKnownAnswer ka;
        workStream->println(F("AT+CIPSTAMAC?"));

        String line; // тут принимаем данные до конца строки
        staMAC = "-";
        apMAC = "-";
        
        bool  apMACDone = false, staMACDone=false;
        char ch;
        while(1)
        { 
          if(staMACDone) // получили MAC-адрес станции
            break;
            
          while(workStream->available())
          {
            ch = workStream->read();
        
            if(ch == '\r')
              continue;
            
            if(ch == '\n')
            {
              // получили строку, разбираем её
                 if(line.startsWith(F("+CIPSTAMAC:"))) // MAC станции
                 {
                    DBGLN(F("Station MAC found, parse..."));
            
                   staMAC = line.substring(11);                      
                  
                 } // if(line.startsWith
                 else
                 if(isKnownAnswer(line, ka))
                 {
                    staMACDone = true;
                 }
             line = "";
            } // ch == '\n'
            else
              line += ch;
        
          if(staMACDone) // получили MAC станции
              break;
 
          } // while
          
        } // while(1)

        // теперь получаем MAC точки доступа
        workStream->println(F("AT+CIPAPMAC?"));
        
        while(1)
        { 
          if(apMACDone) // получили MAC-адрес точки доступа
            break;
            
          while(workStream->available())
          {
            ch = workStream->read();
        
            if(ch == '\r')
              continue;
            
            if(ch == '\n')
            {
              // получили строку, разбираем её
                 if(line.startsWith(F("+CIPAPMAC:"))) // MAC нашей точки доступа
                 {
                   DBGLN(F("softAP MAC found, parse..."));
            
                   apMAC = line.substring(10);                      
                  
                 } // if(line.startsWith
                 else
                 if(isKnownAnswer(line,ka))
                 {
                    apMACDone = true;
                 }
             line = "";
            } // ch == '\n'
            else
              line += ch;
        
          if(apMACDone) // получили MAC точки доступа
              break;
 
          } // while
          
        } // while(1)

  return true;              
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::getIP(String& stationCurrentIP, String& apCurrentIP)
{
    if(machineState != espIdle || !workStream || !ready() || initCommandsQueue.size()) // чего-то делаем, не могём
    {
      //DBGLN(F("ESP: BUSY!!!"));
      return false;
    }

    workStream->println(F("AT+CIFSR"));  
    
    String line; // тут принимаем данные до конца строки
    bool knownAnswerFound = false;
    ESPKnownAnswer ka;  

    char ch;
    while(1)
    { 
      if(knownAnswerFound) // получили оба IP
        break;
        
      while(workStream->available())
      {
        ch = (char) workStream->read();
    
        if(ch == '\r')
          continue;
        
        if(ch == '\n')
        {
          // получили строку, разбираем её
            if(line.startsWith(F("+CIFSR:APIP"))) // IP нашей точки доступа
             {
               DBGLN(F("AP IP found, parse..."));
        
               int idx = line.indexOf("\"");
               if(idx != -1)
               {
                  apCurrentIP = line.substring(idx+1);
                  idx = apCurrentIP.indexOf("\"");
                  if(idx != -1)
                    apCurrentIP = apCurrentIP.substring(0,idx);
                  
               }
               else
                apCurrentIP = F("0.0.0.0");

             } // if(line.startsWith(F("+CIFSR:APIP")))
             else
              if(line.startsWith(F("+CIFSR:STAIP"))) // IP нашей точки доступа, назначенный роутером
             {
                  DBGLN(F("STA IP found, parse..."));
        
               int idx = line.indexOf("\"");
               if(idx != -1)
               {
                  stationCurrentIP = line.substring(idx+1);
                  idx = stationCurrentIP.indexOf("\"");
                  if(idx != -1)
                    stationCurrentIP = stationCurrentIP.substring(0,idx);
                  
               }
               else
                stationCurrentIP = F("0.0.0.0");

             } // if(line.startsWith(F("+CIFSR:STAIP")))

           if(isKnownAnswer(line,ka))
            knownAnswerFound = true;
         
          line = F("");
        } // ch == '\n'
        else
        {
              line += ch;
        }
    
     if(knownAnswerFound) // получили оба IP
        break;

      } // while
      
    } // while(1)


    return true;
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::sendCommand(ESPCommands command)
{
  currentCommand = command;
  
  // тут посылаем команду в ESP
  switch(command)
  {
    case cmdNone:
    case cmdCIPCLOSE: // ничего тут не надо, эти команды формируем не здесь
    case cmdCIPSTART:
    case cmdCIPSEND:
    case cmdWaitSendDone:
    break;

    case cmdWantReady:
    {
      DBGLN(F("ESP: reset..."));
      sendCommand(F("AT+RST"));
    }
    break;

    case cmdEchoOff:
    {
      DBGLN(F("ESP: echo OFF..."));
      sendCommand(F("ATE0"));
    }
    break;

    case cmdCWMODE:
    {
      DBGLN(F("ESP: softAP mode..."));
      sendCommand(F("AT+CWMODE_DEF=3"));
    }
    break;

    case cmdCWSAP:
    {
        DBGLN(F("ESP: Creating the access point..."));
      
        String com = F("AT+CWSAP_DEF=\"");
        com += ESPTransportSettings.APName;
        com += F("\",\"");
        com += ESPTransportSettings.APPassword;
        com += F("\",8,4");
        
        sendCommand(com);      
    }
    break;

    case cmdCWJAP:
    {
      DBGLN(F("ESP: Connecting to the router..."));
              
        String com = F("AT+CWJAP_DEF=\"");
        com += ESPTransportSettings.RouterID;
        com += F("\",\"");
        com += ESPTransportSettings.RouterPassword;
        com += F("\"");
        sendCommand(com);      
    }
    break;

    case cmdCWQAP:
    {
      DBGLN(F("ESP: Disconnect from router..."));
      sendCommand(F("AT+CWQAP"));
    }
    break;

    case cmdCIPMODE:
    {
      DBGLN(F("ESP: Set the TCP server mode to 0..."));
      sendCommand(F("AT+CIPMODE=0"));
    }
    break;

    case cmdCIPMUX:
    {
      DBGLN(F("ESP: Allow multiple connections..."));
      sendCommand(F("AT+CIPMUX=1"));
    }
    break;

    case cmdCIPSERVER:
    {
      DBGLN(F("ESP: Starting TCP-server..."));
      sendCommand(F("AT+CIPSERVER=1,80"));
    }
    break;

    case cmdCheckModemHang:
    {
      DBGLN(F("ESP: check for ESP available..."));
      
      flags.wantReconnect = false;
      sendCommand(F("AT+CWJAP?")); // проверяем, подконнекчены ли к роутеру
    }
    break;
    
  } // switch

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::isESPBootFound(const String& line)
{
  return (line == F("ready")) || line.startsWith(F("Ai-Thinker Technology"));
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::isKnownAnswer(const String& line, ESPKnownAnswer& result)
{
  result = kaNone;
  
  if(line == F("OK"))
  {
    result = kaOK;
    return true;
  }
  if(line == F("ERROR"))
  {
    result = kaError;
    return true;
  }
  if(line == F("FAIL"))
  {
    result = kaFail;
    return true;
  }
  if(line.endsWith(F("SEND OK")))
  {
    result = kaSendOk;
    return true;
  }
  if(line.endsWith(F("SEND FAIL")))
  {
    result = kaSendFail;
    return true;
  }
  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::processIPD()
{
  DBG(F("ESP: start parse +IPD, received="));
  DBGLN(*wiFiReceiveBuff);

  // здесь в wiFiReceiveBuff лежит только команда вида +IPD,<id>,<len>:
  // все данные надо вычитывать из потока
        
    int idx = wiFiReceiveBuff->indexOf(F(",")); // ищем первую запятую после +IPD
    const char* ptr = wiFiReceiveBuff->c_str();
    ptr += idx+1;
    // перешли за запятую, парсим ID клиента
    String connectedClientID = F("");
    while(*ptr != ',')
    {
      connectedClientID += (char) *ptr;
      ptr++;
    }
    ptr++; // за запятую
    String dataLen;
    while(*ptr != ':')
    {
      dataLen += (char) *ptr;
      ptr++; // перешли на начало данных
    }

    // получили ID клиента и длину его данных, которые - пока в потоке, и надо их быстро попакетно вычитать
    int clientID = connectedClientID.toInt();
    size_t lengthOfData = dataLen.toInt();
    
    if(clientID >=0 && clientID < ESP_MAX_CLIENTS)
    {

      /*
      DBG(F("ESP: data for client  #"));
      DBG(clientID);
      DBG(F("; len="));
      DBGLN(lengthOfData);
      */

       CoreTransportClient* client = clients[clientID];

       // у нас есть lengthOfData с данными для клиента, нам надо побить это на пакеты длиной N байт,
       // и последовательно вызывать событие прихода данных. Это нужно для того, чтобы не переполнить оперативку,
       // поскольку у нас её - не вагон.

      // пусть у нас будет максимум 512 байт на пакет
      const int MAX_PACKET_SIZE = 512;
      
      // если длина всех данных меньше 512 - просто тупо все сразу вычитаем
       uint16_t packetSize = min(MAX_PACKET_SIZE,lengthOfData);

        // теперь выделяем буфер под данные
        uint8_t* buff = new uint8_t[packetSize];

        // у нас есть буфер, в него надо скопировать данные из потока
        uint8_t* writePtr = buff;
        
        size_t packetWritten = 0; // записано в пакет
        size_t totalWritten = 0; // всего записано

            while(totalWritten < lengthOfData) // пока не запишем все данные с клиента
            {
                if(workStream->available())
                {                  
                  *writePtr++ = (uint8_t) workStream->read();
                  packetWritten++;
                  totalWritten++;
                }

                if(packetWritten >= packetSize)
                {
                  // скопировали один пакет
                  // передаём клиенту буфер, он сам его освободит, когда надо
                  setClientData(*client,buff,packetWritten);
    
                  // сообщаем подписчикам, что данные для клиента получены
                  notifyClientDataAvailable(*client, totalWritten >= lengthOfData);
    
                   
                  // пересчитываем длину пакета, вдруг там мало осталось, и незачем выделять под несколько байт огромный буфер
                  packetSize =  min(MAX_PACKET_SIZE, lengthOfData - totalWritten);
                  buff = new uint8_t[packetSize];
                  writePtr = buff; // на начало буфера
                  packetWritten = 0;
                }
              
            } // while

            // проверяем - есть ли остаток?
            if(packetWritten > 0)
            {
              // после прохода цикла есть остаток данных, уведомляем клиента
             // передаём клиенту буфер, он сам его освободит, когда надо
              setClientData(*client,buff,packetWritten);

              // сообщаем подписчикам, что данные для клиента получены
              notifyClientDataAvailable(*client, totalWritten >= lengthOfData);

            }
            else
            {
                // нет остатка, чистим буфер
                delete [] buff;
            }
                  
       
    } // if(clientID >=0 && clientID < ESP_MAX_CLIENTS)
    

 // DBGLN(F("ESP: +IPD parsed."));  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::update()
{
  if(!ESPTransportSettings.enabled)
    return;
 
  if(!workStream)
    return;

  if(flags.onIdleTimer) // попросили подождать определённое время, в течение которого просто ничего не надо делать
  {
      if(millis() - timer > idleTime)
      {
        //DBGLN(F("ESP: idle done!"));
        flags.onIdleTimer = false;
      }
  } 

  // Здесь проблема - мы не можем игнорировать входящие команды, мы не должны только обновлять очередь!
  //if(flags.onIdleTimer) // мы в режиме простоя
//    return;

  // флаг, что есть ответ от ESP, выставляется по признаку наличия хоть чего-то в буфере приёма
  bool hasAnswer = workStream->available() > 0;

  // выставляем флаг, что мы хотя бы раз получили хоть чего-то от ESP
  flags.isAnyAnswerReceived = flags.isAnyAnswerReceived || hasAnswer;

  bool hasAnswerLine = false; // флаг, что мы получили строку ответа от модема

  char ch;
  while(workStream->available())
  {
    // здесь мы должны детектировать - пришло IPD или нет.
    // если пришли данные - мы должны вычитать их длину, и уже после этого - отправить данные клиенту,
    // напрямую читая из потока. Это нужно, потому что данные могут быть бинарными, и мы никогда в них не дождёмся
    // перевода строки.

     if(wiFiReceiveBuff->startsWith(F("+IPD")) && wiFiReceiveBuff->indexOf(":") != -1)
     {
       // DBGLN(F("ESP: +IPD detected, parse!"));
       
        processIPD();
                
        hasAnswerLine = false; // говорим, что нет у нас строки с ответом, ибо нечего проверять ответ на +IPD - мы разобрали его сами
        delete wiFiReceiveBuff;
        wiFiReceiveBuff = new String();
        timer = millis();        
        return; // надо вывалится из цикла, поскольку мы уже всё отослали клиенту, для которого пришли данные
     }
    
    ch = workStream->read();

    if(ch == '\r') // ненужный нам символ
      continue;
    else
    if(ch == '\n')
    {   
        hasAnswerLine = true; // выставляем флаг, что у нас есть строка ответа от ESP  
        break; // выходим из цикла, остальное дочитаем позже, если это потребуется кому-либо
    }
    else
    {     
        if(flags.waitForDataWelcome && ch == '>') // ждём команду >  (на ввод данных)
        {
          flags.waitForDataWelcome = false;
          *wiFiReceiveBuff = F(">");
          hasAnswerLine = true;
          break; // выходим из цикла, получили приглашение на ввод данных
        }
        else
        {
          *wiFiReceiveBuff += ch;
                         
          if(wiFiReceiveBuff->length() > 2500) // буфер слишком длинный
          {
            DBGLN(F("ESP: incoming data too long, skip it!"));
            delete wiFiReceiveBuff;
            wiFiReceiveBuff = new String();
          }
        }
    } // any char except '\r' and '\n' 
    
  } // while(workStream->available())

  if(hasAnswer)
  {
     timer = millis(); // не забываем обновлять таймер ответа - поскольку у нас что-то пришло - значит, модем отвечает
  }
/*
  #ifdef _CORE_DEBUG

      if(hasAnswerLine && wiFiReceiveBuff->length())
      {
        
        // выводим то, что получено, для теста
        DBG(F("ESP: <<==(c:"));
        DBG(currentCommand);
        DBG(F(", m:"));        
        DBG(machineState);
        DBG(F(", l:"));
        DBG(wiFiReceiveBuff->length());
        DBG(F("): "));
        DBGLN(*wiFiReceiveBuff);
        
      }
  
  #endif // _CORE_DEBUG
*/
    if(hasAnswerLine && !wiFiReceiveBuff->length()) // пустая строка, не надо обрабатывать
      hasAnswerLine = false;

    // при разборе ответа тут будет лежать тип ответа, чтобы часто не сравнивать со строкой
    ESPKnownAnswer knownAnswer = kaNone;

  if(!flags.onIdleTimer) // только если мы не в режиме простоя
  {
    // анализируем состояние конечного автомата, чтобы понять, что делать
    switch(machineState)
    {
        case espIdle: // ничего не делаем, можем работать с очередью команд и клиентами
        {            
            // смотрим - если есть хоть одна команда в очереди инициализации - значит, мы в процессе инициализации, иначе - можно работать с очередью клиентов
            if(initCommandsQueue.size())
            {
                DBGLN(F("ESP: process next init command..."));
                currentCommand = initCommandsQueue[initCommandsQueue.size()-1];
                initCommandsQueue.pop();
                sendCommand(currentCommand);
            } // if
            else
            {
              // в очереди команд инициализации ничего нет, значит, можем выставить флаг, что мы готовы к работе с клиентами
              flags.ready = true;
              
              if(clientsQueue.size())
              {
                  // получаем первого клиента в очереди
                  ESPClientQueueData dt = clientsQueue[0];
                  int clientID = dt.client->getID();
                  
                  // смотрим, чего он хочет от нас
                  switch(dt.action)
                  {
                    case actionDisconnect:
                    {
                      // хочет отсоединиться
                      /*
                      DBG(F("ESP: client #"));
                      DBG(clientID);
                      DBGLN(F(" wants to disconnect..."));
                      */

                      currentCommand = cmdCIPCLOSE;
                      String cmd = F("AT+CIPCLOSE=");
                      cmd += clientID;
                      sendCommand(cmd);
                      
                    }
                    break; // actionDisconnect

                    case actionConnect:
                    {
                      // хочет подсоединиться
                      /*
                      DBG(F("ESP: client #"));
                      DBG(clientID);
                      DBGLN(F(" want to connect..."));
                      */

                      currentCommand = cmdCIPSTART;
                      String comm = F("AT+CIPSTART=");
                      comm += clientID;
                      comm += F(",\"TCP\",\"");
                      comm += dt.ip;
                      comm += F("\",");
                      comm += dt.port;
              
                      // и отсылаем её
                      sendCommand(comm);
                    }
                    break; // actionConnect

                    case actionWrite:
                    {
                      // хочет отослать данные
                      /*
                      DBG(F("ESP: client #"));
                      DBG(clientID);
                      DBG(F(" has data="));
                      DBG(dt.client->getDataSize());
                      DBGLN(F(" and wants to send it..."));
                      */

                      currentCommand = cmdCIPSEND;

                      String command = F("AT+CIPSENDBUF=");
                      command += clientID;
                      command += F(",");
                      command += dt.client->getDataSize();
                      flags.waitForDataWelcome = true; // выставляем флаг, что мы ждём >
                      
                      sendCommand(command);
                      
                    }
                    break; // actionWrite
                  } // switch
              }
              else
              {
                timer = millis(); // обновляем таймер в режиме ожидания, поскольку мы не ждём ответа на команды

                // у нас прошла инициализация, нет клиентов в очереди на обработку, следовательно - мы можем проверять модем на зависание
                // тут смотрим - не пора ли послать команду для проверки на зависание. Слишком часто её звать нельзя, что очевидно,
                // поэтому мы будем звать её минимум раз в N секунд. При этом следует учитывать, что мы всё равно должны звать эту команду
                // вне зависимости от того, откликается ли ESP или нет, т.к. в этой команде мы проверяем - есть ли соединение с роутером.
                // эту проверку надо делать периодически, чтобы форсировать переподсоединение, если роутер отвалился.
                static unsigned long hangTimer = 0;
                if(millis() - hangTimer > 30000)
                {
                 // DBGLN(F("ESP: want to check modem availability..."));
                  hangTimer = millis();
                  sendCommand(cmdCheckModemHang);
                  
                } // if
                
              } // else
            } // else inited
        }
        break; // espIdle

        case espWaitAnswer: // ждём ответа от модема на посланную ранее команду (функция sendCommand переводит конечный автомат в эту ветку)
        {
          // команда, которую послали - лежит в currentCommand, время, когда её послали - лежит в timer.
              if(hasAnswerLine)
              {                
                // есть строка ответа от модема, можем её анализировать, в зависимости от посланной команды (лежит в currentCommand)
                switch(currentCommand)
                {
                  case cmdNone:
                  {
                    // ничего не делаем
                    DBGLN(F("ESP: NO COMMAND!!!!"));
                  }
                  break; // cmdNone

                  case cmdCIPCLOSE:
                  {
                    // отсоединялись
                    if(isKnownAnswer(*wiFiReceiveBuff,knownAnswer))
                    {
                      if(clientsQueue.size())
                      {
                       // DBGLN(F("ESP: Client disconnected."));
                        // клиент отсоединён, ставим ему соответствующий флаг, освобождаем его и удаляем из очереди
                        ESPClientQueueData dt = clientsQueue[0];

                        CoreTransportClient* thisClient = dt.client;
                        removeClientFromQueue(thisClient);

                        setClientBusy(*thisClient,false);
                        setClientConnected(*thisClient,false,CT_ERROR_NONE);
                        
                      } // if(clientsQueue.size()) 
                      
                        machineState = espIdle; // переходим к следующей команде
                    }
                  }
                  break; // cmdCIPCLOSE

                  case cmdCIPSTART:
                  {
                    // соединялись
                        if(isKnownAnswer(*wiFiReceiveBuff,knownAnswer))
                        {
                          if(knownAnswer == kaOK)
                          {
                            // законнектились удачно
                            if(clientsQueue.size())
                            {
                             //  DBGLN(F("ESP: Client connected."));
                               ESPClientQueueData dt = clientsQueue[0];
                               
                               CoreTransportClient* thisClient = dt.client;
                               removeClientFromQueue(dt.client);
                               
                               setClientBusy(*thisClient,false);
                            }
                          }
                          else
                          {
                            // ошибка соединения
                            if(clientsQueue.size())
                            {
                               DBG(F("ESP: Client connect ERROR, received: "));
                               DBGLN(*wiFiReceiveBuff);
                               
                               ESPClientQueueData dt = clientsQueue[0];

                               CoreTransportClient* thisClient = dt.client;
                               removeClientFromQueue(thisClient);
                               
                               setClientBusy(*thisClient,false);
                               setClientConnected(*thisClient,false,CT_ERROR_CANT_CONNECT);
                            }
                          }
                          machineState = espIdle; // переходим к следующей команде
                        }                    
                    
                  }
                  break; // cmdCIPSTART


                  case cmdWaitSendDone:
                  {
                    // дожидаемся результата отсыла данных
                      
                      if(isKnownAnswer(*wiFiReceiveBuff,knownAnswer))
                      {
                        if(knownAnswer == kaSendOk)
                        {
                          // send ok
                          if(clientsQueue.size())
                          {
                             //DBGLN(F("ESP: data was sent."));
                             ESPClientQueueData dt = clientsQueue[0];
                             
                             CoreTransportClient* thisClient = dt.client;
                             removeClientFromQueue(thisClient);

                             setClientBusy(*thisClient,false);

                             // очищаем данные у клиента
                             setClientData(*thisClient,NULL,0);

                             notifyClientDataWritten(*thisClient,CT_ERROR_NONE);
                          }                     
                        } // send ok
                        else
                        {
                          // send fail
                          if(clientsQueue.size())
                          {
                             DBGLN(F("ESP: send data fail!"));
                             ESPClientQueueData dt = clientsQueue[0];

                             CoreTransportClient* thisClient = dt.client;
                             removeClientFromQueue(thisClient);
                             
                             setClientBusy(*thisClient,false);
                             
                             // очищаем данные у клиента
                             setClientData(*thisClient,NULL,0);
                             
                             notifyClientDataWritten(*thisClient,CT_ERROR_CANT_WRITE);
                          }                     
                        } // else send fail
  
                        machineState = espIdle; // переходим к следующей команде
                        
                      } // if(isKnownAnswer(*wiFiReceiveBuff,knownAnswer))
                       

                  }
                  break; // cmdWaitSendDone

                  case cmdCIPSEND:
                  {
                    // тут отсылали запрос на запись данных с клиента
                    if(*wiFiReceiveBuff == F(">"))
                    {
                       // дождались приглашения, можем писать в ESP
                       // тут пишем напрямую
                       if(clientsQueue.size())
                       {
                          // говорим, что ждём окончания отсыла данных
                          currentCommand = cmdWaitSendDone;                          
                          ESPClientQueueData dt = clientsQueue[0];

                          DBGLN(F("ESP: > received, start write from client to ESP..."));
                          
                          workStream->write(dt.client->getData(),dt.client->getDataSize());

                          // очищаем данные у клиента сразу после отсыла
                          setClientData(*(dt.client),NULL,0);
                       }
                    } // if
                    else
                    if(wiFiReceiveBuff->indexOf(F("FAIL")) != -1 || wiFiReceiveBuff->indexOf(F("ERROR")) != -1)
                    {
                       // всё плохо, не получилось ничего записать
                      if(clientsQueue.size())
                      {
                         DBGLN(F("ESP: Client write ERROR!"));
                         ESPClientQueueData dt = clientsQueue[0];

                         CoreTransportClient* thisClient = dt.client;
                         removeClientFromQueue(thisClient);

                         setClientBusy(*thisClient,false);
                         
                         // очищаем данные у клиента
                         setClientData(*thisClient,NULL,0);

                         notifyClientDataWritten(*thisClient,CT_ERROR_CANT_WRITE);
                        
                      }                     
                      
                      machineState = espIdle; // переходим к следующей команде
              
                    } // else can't write
                    
                  }
                  break; // cmdCIPSEND
                  
                  case cmdWantReady: // ждём загрузки модема в ответ на команду AT+RST
                  {
                    if(isESPBootFound(*wiFiReceiveBuff))
                    {
                      DBGLN(F("ESP: BOOT FOUND!!!"));
                      machineState = espIdle; // переходим к следующей команде
                    }
                  }
                  break; // cmdWantReady

                  case cmdEchoOff:
                  {
                    if(isKnownAnswer(*wiFiReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("ESP: Echo OFF command processed."));
                      machineState = espIdle; // переходим к следующей команде
                    }
                  }
                  break; // cmdEchoOff

                  case cmdCWMODE:
                  {
                    if(isKnownAnswer(*wiFiReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("ESP: CWMODE command processed."));
                      machineState = espIdle; // переходим к следующей команде
                    }
                  }
                  break; // cmdCWMODE

                  case cmdCWSAP:
                  {
                    if(isKnownAnswer(*wiFiReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("ESP: CWSAP command processed."));
                      machineState = espIdle; // переходим к следующей команде
                    }  
                  }
                  break; // cmdCWSAP

                  case cmdCWJAP:
                  {                    
                    if(isKnownAnswer(*wiFiReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("ESP: CWJAP command processed."));
                      machineState = espIdle; // переходим к следующей команде
                    }  
                  }
                  break; // cmdCWJAP

                  case cmdCWQAP:
                  {                    
                    if(isKnownAnswer(*wiFiReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("ESP: CWQAP command processed."));
                      machineState = espIdle; // переходим к следующей команде
                    }  
                  }
                  break; // cmdCWQAP

                  case cmdCIPMODE:
                  {                    
                    if(isKnownAnswer(*wiFiReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("ESP: CIPMODE command processed."));
                      machineState = espIdle; // переходим к следующей команде
                    }  
                  }
                  break; // cmdCIPMODE

                  case cmdCIPMUX:
                  {                    
                    if(isKnownAnswer(*wiFiReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("ESP: CIPMUX command processed."));
                      machineState = espIdle; // переходим к следующей команде
                    }  
                  }
                  break; // cmdCIPMUX
                  
                  case cmdCIPSERVER:
                  {                    
                    if(isKnownAnswer(*wiFiReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("ESP: CIPSERVER command processed."));
                      machineState = espIdle; // переходим к следующей команде
                    }  
                  }
                  break; // cmdCIPSERVER

                  case cmdCheckModemHang:
                  {                    
                    if(isKnownAnswer(*wiFiReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("ESP: ESP answered and available."));
                      machineState = espIdle; // переходим к следующей команде

                       if(flags.wantReconnect)
                       {
                          // требуется переподсоединение к роутеру. Проще всего это сделать вызовом restart - тогда весь цикл пойдёт с начала
                          restart();

                          // чтобы часто не дёргать реконнект - мы говорим, что после рестарта надо подождать 5 секунд перед тем, как обрабатывать следующую команду
                          DBGLN(F("ESP: Wait 5 seconds before reconnect..."));
                          flags.onIdleTimer = true;
                          timer = millis();
                          idleTime = 5000;
                          
                       } // if(flags.wantReconnect)
                      
                    } // if(isKnownAnswer

                     if(*wiFiReceiveBuff == F("No AP"))
                     {
                        if(ESPTransportSettings.Flags.ConnectToRouter)
                        {
                          DBGLN(F("ESP: No connect to router, want to reconnect..."));
                          // нет соединения с роутером, надо переподсоединиться, как только это будет возможно.
                          flags.wantReconnect = true;
                        }
                      
                     } // if
                  }
                  break; // cmdCheckModemHang
                                    
                } // switch

                
              } // if(hasAnswerLine)
              
         
        }
        break; // espWaitAnswer

        case espReboot:
        {
          // ждём перезагрузки модема
          unsigned long powerOffTime = ESPTransportSettings.HangPowerOffTime;
          powerOffTime *= 1000;
          if(millis() - timer > powerOffTime)
          {
            DBGLN(F("ESP: turn power ON!"));
            if(ESPTransportSettings.Flags.UseRebootPin)
            {
              pinMode(ESPTransportSettings.RebootPin,OUTPUT);
              digitalWrite(ESPTransportSettings.RebootPin,ESPTransportSettings.PowerOnLevel);
            }

            machineState = espWaitInit;
            timer = millis();
            
          } // if
        }
        break; // espReboot

        case espWaitInit:
        {
          unsigned long waitTime = ESPTransportSettings.WaitInitTIme;
          waitTime *= 1000;
          if(millis() - timer > waitTime)
          {            
            restart();
            DBGLN(F("ESP: inited after reboot!"));
          } // 
        }
        break;
      
    } // switch

  } // if(!flags.onIdleTimer)


    // тут просто анализируем ответ от ESP, если он есть, на предмет того - соединён ли клиент, отсоединён ли клиент и т.п.
    if(hasAnswerLine)
    {
      
       // смотрим, подсоединился ли клиент?
       int idx = wiFiReceiveBuff->indexOf(F(",CONNECT"));
       if(idx != -1)
       {
          // клиент подсоединился
          String s = wiFiReceiveBuff->substring(0,idx);
          int clientID = s.toInt();
          if(clientID >=0 && clientID < ESP_MAX_CLIENTS)
          {
            DBG(F("ESP: client connected - #"));
            DBGLN(clientID);

            // выставляем клиенту флаг, что он подсоединён
            CoreTransportClient* client = clients[clientID];
            
            setClientBusy(*client,false);
            setClientConnected(*client,true,CT_ERROR_NONE);
          }
       } // if

       idx = wiFiReceiveBuff->indexOf(F(",CLOSED"));
       if(idx != -1)
       {
        // клиент отсоединился
          String s = wiFiReceiveBuff->substring(0,idx);
          int clientID = s.toInt();
          if(clientID >=0 && clientID < ESP_MAX_CLIENTS)
          {
            DBG(F("ESP: client disconnected - #"));
            DBGLN(clientID);

            // выставляем клиенту флаг, что он подсоединён
            CoreTransportClient* client = clients[clientID];
            
            setClientBusy(*client,false);
            setClientConnected(*client,false,CT_ERROR_NONE);
          }        
        
       } // if(idx != -1)

       if(*wiFiReceiveBuff == F("WIFI CONNECTED"))
       {
          flags.connectedToRouter = true;
          DBGLN(F("ESP: connected to router!"));
       }
       else
       if(*wiFiReceiveBuff == F("WIFI DISCONNECT"))
       {
          flags.connectedToRouter = false;
          DBGLN(F("ESP: disconnected from router!"));
       }


      // не забываем чистить за собой
        delete wiFiReceiveBuff;
        wiFiReceiveBuff = new String();   
           
    } // if(hasAnswerLine)

    if(!hasAnswer) // проверяем на зависание
    {

      // нет ответа от ESP, проверяем, зависла ли она?
      unsigned long hangTime = ESPTransportSettings.HangTimeout;
      hangTime *= 1000;

      if(millis() - timer > hangTime)
      {
        DBGLN(F("ESP: modem not answering, reboot!"));

        if(ESPTransportSettings.Flags.UseRebootPin)
        {
          // есть пин, который надо использовать при зависании
          pinMode(ESPTransportSettings.RebootPin,OUTPUT);
          digitalWrite(ESPTransportSettings.RebootPin,!ESPTransportSettings.PowerOnLevel);
        }

        machineState = espReboot;
        timer = millis();
        
      } // if   
         
    }
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::begin()
{
  workStream = NULL;

  if(!ESPTransportSettings.enabled)
    return;
  
  if(ESPTransportSettings.SerialNumber == 0 || ESPTransportSettings.UARTSpeed == 0) // не можем работать через Serial или с нулевой скоростью!
    return;

//  DBG(F("ESP: begin, serial is: "));
//  DBGLN(ESPTransportSettings.SerialNumber);
  
  initClients();

  #ifdef CORE_ESP_WEB_SERVER
    subscribe(&ESPWebServer);
  #endif  

  HardwareSerial* hs = NULL;

  #if (TARGET_BOARD == MEGA_BOARD) || (TARGET_BOARD == DUE_BOARD)
  
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
  #elif TARGET_BOARD == ESP_BOARD
    #error "NOT IMPLEMENTED!!!"
  #else
    #error "Unknown target board!"
  #endif    

  workStream = hs;
  unsigned long uspeed = ESPTransportSettings.UARTSpeed;
  uspeed *= 9600;

  hs->begin(uspeed);

  restart();

  if(ESPTransportSettings.Flags.UseRebootPin)
  {
    // есть пин, который надо использовать при зависании
    pinMode(ESPTransportSettings.RebootPin,OUTPUT);
    digitalWrite(ESPTransportSettings.RebootPin,!ESPTransportSettings.PowerOnLevel);
    machineState = espReboot;
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::restart()
{
  delete wiFiReceiveBuff;
  wiFiReceiveBuff = new String();

  // очищаем очередь клиентов, заодно им рассылаем события
  clearClientsQueue(true);  

  // т.к. мы ничего не инициализировали - говорим, что мы не готовы предоставлять клиентов
  flags.ready = false;
  flags.isAnyAnswerReceived = false;
  flags.waitForDataWelcome = false;
  flags.connectedToRouter = false;
  flags.wantReconnect = false;
  flags.onIdleTimer = false;
  
  timer = millis();

  currentCommand = cmdNone;
  machineState = espIdle;

  // инициализируем очередь командами по умолчанию
 createInitCommands(true);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::createInitCommands(bool addResetCommand)
{  
  // очищаем очередь команд
  clearInitCommands();

//  DBGLN(F("ESP: Create init queue..."));
  
  if(ESPTransportSettings.Flags.ConnectToRouter) // коннектимся к роутеру
    initCommandsQueue.push_back(cmdCWJAP); // коннектимся к роутеру совсем в конце
  else  
    initCommandsQueue.push_back(cmdCWQAP); // отсоединяемся от роутера
    
  initCommandsQueue.push_back(cmdCIPSERVER); // сервер поднимаем в последнюю очередь
  initCommandsQueue.push_back(cmdCIPMUX); // разрешаем множественные подключения
  initCommandsQueue.push_back(cmdCIPMODE); // устанавливаем режим работы
  initCommandsQueue.push_back(cmdCWSAP); // создаём точку доступа
  initCommandsQueue.push_back(cmdCWMODE); // // переводим в смешанный режим
  initCommandsQueue.push_back(cmdEchoOff); // выключаем эхо
  
  if(addResetCommand)
    initCommandsQueue.push_back(cmdWantReady); // надо получить ready от модуля путём его перезагрузки      
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::clearInitCommands()
{
 // DBGLN(F("ESP: Clear init queue..."));  

  initCommandsQueue.empty();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::clearClientsQueue(bool raiseEvents)
{
//  DBG(F("ESP: Clear clients queue: "));
//  DBGLN(clientsQueue.size());
  
  // тут попросили освободить очередь клиентов.
  // для этого нам надо выставить каждому клиенту флаг того, что он свободен,
  // плюс - сообщить, что текущее действие над ним не удалось.  

    for(size_t i=0;i<clientsQueue.size();i++)
    {
        ESPClientQueueData dt = clientsQueue[i];
        if(raiseEvents)
        {
          switch(dt.action)
          {
            case actionDisconnect:
                // при дисконнекте всегда считаем, что ошибок нет
                setClientConnected(*(dt.client),false,CT_ERROR_NONE);
            break;
  
            case actionConnect:
                // если было запрошено соединение клиента с адресом - говорим, что соединиться не можем
                setClientConnected(*(dt.client),false,CT_ERROR_CANT_CONNECT);
            break;
  
            case actionWrite:
              // если попросили записать данные - надо сообщить подписчикам, что не можем записать данные
              notifyClientDataWritten(*(dt.client),CT_ERROR_CANT_WRITE);
            break;
          } // switch
          
        } // if(raiseEvents)

        // освобождаем клиента
        setClientBusy(*(dt.client),false);
    } // for

  clientsQueue.empty();

  // помимо этого - надо принудительно всем клиентам выставить статус "Отсоединён"
  for(int i=0;i<ESP_MAX_CLIENTS;i++)
  {
    setClientBusy(*(clients[i]),false);
    setClientConnected(*(clients[i]),false,CT_ERROR_NONE); 
  }  

//  DBGLN(F("ESP: clients queue cleared."));


  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::isClientInQueue(CoreTransportClient* client, ESPClientAction action)
{
  for(size_t i=0;i<clientsQueue.size();i++)
  {
    if(clientsQueue[i].client == client && clientsQueue[i].action == action)
      return true;
  }

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::addClientToQueue(CoreTransportClient* client, ESPClientAction action, const char* ip, uint16_t port)
{
  if(isClientInQueue(client, action))
  {
    DBGLN(F("ESP: Client already in queue!"));
    return;
  }

    ESPClientQueueData dt;
    dt.client = client;
    dt.action = action;
    
    dt.ip = NULL;
    if(ip)
    {
      dt.ip = new char[strlen(ip)+1];
      strcpy(dt.ip,ip);
    }
    dt.port = port;

    clientsQueue.push_back(dt);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::removeClientFromQueue(CoreTransportClient* client)
{
  for(size_t i=0;i<clientsQueue.size();i++)
  {
    if(clientsQueue[i].client == client)
    {
      delete [] clientsQueue[i].ip;
      
        for(size_t j=i+1;j<clientsQueue.size();j++)
        {
          clientsQueue[j-1] = clientsQueue[j];
        }
        
        clientsQueue.pop();
        break;
    } // if
    
  } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::initClients()
{
  for(int i=0;i<ESP_MAX_CLIENTS;i++)
  {
    if(clients[i])
      continue;
      
    CoreTransportClient* client = CoreTransportClient::Create(this);
    setClientID(*client,i);
    clients[i] = client;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::subscribe(IClientEventsSubscriber* subscriber)
{
  initClients();
  
  for(int i=0;i<ESP_MAX_CLIENTS;i++)
  {
    subscribeClient(*(clients[i]),subscriber);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::unsubscribe(IClientEventsSubscriber* subscriber)
{
  initClients();
  
  for(int i=0;i<ESP_MAX_CLIENTS;i++)
  {
    unsubscribeClient(*(clients[i]),subscriber);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransportClient* CoreESPTransport::getFreeClient()
{
  if(!ready()) // если ещё не готовы к работе - ничего не возвращаем
    return NULL;
  
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
//  DBGLN(F("ESP: write from client..."));

  if(!client.connected())
  {
    DBGLN(F("ESP: client not connected!"));
    return;
  }

  //говорим, что клиент занят
  setClientBusy(client,true);
  
  // добавляем клиента в очередь на запись
  addClientToQueue(&client, actionWrite);

/*
  DBG(F("ESP: Client #"));
  DBG(client.getID());
  DBG(F(" has data size: "));
  DBGLN(client.getDataSize());
*/
  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с записью в поток с этого клиента
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::beginConnect(CoreTransportClient& client, const char* ip, uint16_t port)
{
//   DBGLN(F("ESP: connect client to IP..."));

  if(client.connected())
  {
    DBGLN(F("ESP: client already connected!"));
    return;
  }

  //говорим, что клиент занят
  setClientBusy(client,true);

  // добавляем клиента в очередь на соединение
  addClientToQueue(&client, actionConnect, ip, port);

  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с соединением клиента

  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::beginDisconnect(CoreTransportClient& client)
{
 // DBGLN(F("ESP: Disconnect client..."));

  if(!client.connected())
  {
//    DBGLN(F("ESP: client not connected!"));
    return;
  }

  //говорим, что клиент занят
  setClientBusy(client,true);

  // добавляем клиента в очередь на соединение
  addClientToQueue(&client, actionDisconnect);

  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с отсоединением клиента
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::ready()
{
  return flags.ready && flags.isAnyAnswerReceived; // если мы полностью инициализировали ESP - значит, можем работать
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_ESP_WEB_SERVER
//--------------------------------------------------------------------------------------------------------------------------------------
CoreESPWebServerClass ESPWebServer;
//--------------------------------------------------------------------------------------------------------------------------------------
CoreESPWebServerClass::CoreESPWebServerClass()
{
  internalBuffer = new String();
  dynamicHandlerClient = NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPWebServerClass::OnClientConnect(CoreTransportClient& client, bool connected, int errorCode)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPWebServerClass::OnClientDataWritten(CoreTransportClient& client, int errorCode)
{
//  DBG(F("WEB: Client DATA WRITTEN, errorCode = "));
//  DBGLN(errorCode);
  
    CoreWebServerQuery* pending = getPendingQuery(&client);
    if(pending)
    {
      removePendingQuery(pending);
  //    DBGLN(F("WEB: disconnect client 1!"));    
      client.disconnect();
    }
    else
    {
      // тут проверяем, нет ли у нас ещё данных для клиента?
      CoreWebServerPendingFileData* pfd = getPendingFileData(&client);
      
      if(errorCode != CT_ERROR_NONE)
      {
        if(pfd)
          removePendingFileData(&client);
        
//        DBGLN(F("WEB: disconnect client 2!"));     
        client.disconnect();
        return;
      }

      // всё норм, проверяем, есть ли для этого клиента данные?
      if(!pfd)
      {
        // нет у нас клиента такого
        return;
      }

      if(pfd->pendingBytes < 1)
      {
        // данные закончились
        pfd->client->disconnect();
        removePendingFileData(pfd->client);    
      }
      else
      {
        // данные ещё есть, отсылаем
        sendNextFileData(pfd);
      }
      
    } // else

}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPWebServerClass::sendNextFileData(CoreWebServerPendingFileData* pfd)
{
  if(!pfd)
    return;

  // тут читаем в буфер, и отсылаем
  const int BUFFER_SIZE = CORE_ESP_WEB_SERVER_CLIENT_BUFFER; // будем читать по N байт

  unsigned long toSend = min(BUFFER_SIZE,pfd->pendingBytes);

 // DBG(F("WEB: sendNextFileData, dataLen="));
 // DBGLN(toSend);

  uint8_t* buff = new uint8_t[toSend];
  
  //тут чтение из файла
  for(unsigned long i=0;i<toSend;i++)
  {
     int iCh = pfd->file.read();
      if(iCh == -1)
      {
        // ОШИБКА ЧТЕНИЯ С ФАЙЛА!!!
        break;
      }
       buff[i] = (byte) iCh;
  }

  pfd->pendingBytes -= toSend;

  // посылаем новую порцию данных
  if(!pfd->client->write(buff,toSend,true))
  {
    delete [] buff;
    pfd->client->disconnect();
    removePendingFileData(pfd->client);
  }
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreWebServerQuery* CoreESPWebServerClass::getPendingQuery(CoreTransportClient* client)
{
   for(size_t i=0;i<pendingQueries.size();i++)
  {
    if(pendingQueries[i].client == client)
      return &(pendingQueries[i]);
  } 

  return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPWebServerClass::isOurClient(CoreTransportClient* client)
{
  for(size_t i=0;i<pendingQueries.size();i++)
  {
    if(pendingQueries[i].client == client)
      return true;
  }

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
String UrlDecode(const String& uri) // декодирует URI-строку
{
  String result;
  int len = uri.length();
  result.reserve(len);
  int s = 0;

 while (s < len) 
 {
    char c = uri[s++];

    if (c == '%' && s + 2 < len) 
    {
        char c2 = uri[s++];
        char c3 = uri[s++];
        if (isxdigit(c2) && isxdigit(c3)) 
        {
            c2 = tolower(c2);
            c3 = tolower(c3);

            if (c2 <= '9')
                c2 = c2 - '0';
            else
                c2 = c2 - 'a' + 10;

            if (c3 <= '9')
                c3 = c3 - '0';
            else
                c3 = c3 - 'a' + 10;

            result  += char(16 * c2 + c3);

        } 
        else 
        { 
            result += c;
            result += c2;
            result += c3;
        }
    } 
    else 
    if (c == '+') 
        result += ' ';
    else 
        result += c;
 } // while 

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPWebServerClass::processQuery(CoreTransportClient* client, char* query)
{
    if(!query)
      return;

   // надо перейти за слеш, поскольку нам нужен только адрес страницы


    // теперь нам надо выцепить URI, который мы будем анализировать
    char* space = strstr(query, " ");
    String method, uri;

    while(query != space)
    {
      method += (char) *query++;
    }

    query++;
    
    if(*query == '/') 
      query++;
          
    while(*query && *query != ' ')
    {
      uri += (char) *query++;
    }

    uri = UrlDecode(uri);
    
/*
    DBGLN(F("///////////////////////////////////////////////////////////////////"));
    DBGLN(uri);
    DBGLN(F("///////////////////////////////////////////////////////////////////"));
*/
    // теперь посмотрим, не команда ли это к ядру?
    if(uri.startsWith(CORE_COMMAND_SET) || uri.startsWith(CORE_COMMAND_GET))
    {
      // меняем слеши на разделители команд, т.е. мы можем обрабатывать два типа команд:
      // GET=ESP|IP, где | кодируется как последовательность %7С, и
      // GET=ESP/IP
      
      uri.replace("/",String(CORE_COMMAND_PARAM_DELIMITER));
      
      // это команда к ядру, выполняем её
      Core.processCommand(uri,this);

      String data = WEB_HEADER_BEGIN;
      data += F("200 OK");
      data += WEB_HEADER_LINE;

      data += WEB_HEADER_CONNECTION;
      data += WEB_HEADER_LINE;

      data += WEB_HEADER_CONTENT_TYPE;
      data += F("text/plain");
      data += WEB_HEADER_LINE;

      data += WEB_HEADER_CONTENT_LENGTH;
      data += internalBuffer->length();
      data += WEB_HEADER_LINE;
      data += WEB_HEADER_LINE;
      
      data += *internalBuffer;
      
      delete internalBuffer;
      internalBuffer = new String();

      client->write((uint8_t*)data.c_str(),data.length());
      
    }
    else
    {
      // другие запросы, пытаемся разобрать
      processURI(client,uri);

    }

  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPWebServerClass::send404(CoreTransportClient* client)
{
    String headers = WEB_HEADER_BEGIN;
    headers += F("404 Not Found");
    headers += WEB_HEADER_LINE;

    headers += WEB_HEADER_CONNECTION;
    headers += WEB_HEADER_LINE;

    headers += WEB_HEADER_CONTENT_TYPE;
    headers += F("text/plain");
    headers += WEB_HEADER_LINE;

    headers += WEB_HEADER_CONTENT_LENGTH;
    headers += "0";
    headers += WEB_HEADER_LINE;
    headers += WEB_HEADER_LINE;

    client->write((uint8_t*)headers.c_str(),headers.length());
}
//--------------------------------------------------------------------------------------------------------------------------------------
String CoreESPWebServerClass::getContentType(const String& fileName)
{
  String s = fileName;
  s.toLowerCase();

  if(s.endsWith(F(".txt")))
    return F("text/plain");
  else
  if(s.endsWith(F(".html")) || s.endsWith(F(".htm")))
    return F("text/html");
  else
  if(s.endsWith(F(".css")))
    return F("text/css");
  else
  if(s.endsWith(F(".bin")))
    return F("application/octet-stream");
  else
  if(s.endsWith(F(".csv")))
    return F("text/csv");
  else
  if(s.endsWith(F(".doc")) || s.endsWith(F(".docx")))
    return F("application/msword");
  else
  if(s.endsWith(F(".gif")))
    return F("image/gif");
  else
  if(s.endsWith(F(".ico")))
    return F("image/x-icon");
  else
  if(s.endsWith(F(".jpeg")) || s.endsWith(F(".jpg")))
    return F("image/jpeg");
  else
  if(s.endsWith(F(".js")))
    return F("application/javascript");
  else
  if(s.endsWith(F(".json")))
    return F("application/json");
  else
  if(s.endsWith(F(".png")))
    return F("image/png");
  else
  if(s.endsWith(F(".pdf")))
    return F("application/pdf");
  else
  if(s.endsWith(F(".rar")))
    return F("application/x-rar-compressed");
  else
  if(s.endsWith(F(".rtf")))
    return F("application/rtf");
  else
  if(s.endsWith(F(".wav")))
    return F("audio/x-wav");
  else
  if(s.endsWith(F(".xml")))
    return F("application/xml");
  else
  if(s.endsWith(F(".zip")))
    return F("application/zip");

  return F("application/octet-stream");
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPWebServerClass::send(int statusCode, const char* contentType, const char* data)
{
  if(!dynamicHandlerClient)
    return;

  String headers = WEB_HEADER_BEGIN;
  headers += statusCode;
  headers += F(" Answer");
  headers += WEB_HEADER_LINE;

  headers += WEB_HEADER_CONNECTION;
  headers += WEB_HEADER_LINE;

  headers += WEB_HEADER_CONTENT_TYPE;
  headers += contentType;
  headers += WEB_HEADER_LINE;

  headers += WEB_HEADER_CONTENT_LENGTH;
  headers += strlen(data);
  headers += WEB_HEADER_LINE;
  headers += WEB_HEADER_LINE;

  headers += data;

  dynamicHandlerClient->write((uint8_t*) headers.c_str(),headers.length());
  dynamicHandlerClient = NULL;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPWebServerClass::on(const char* uri, WebServerRequestHandler handler)
{
 for(size_t i=0;i<dynamicHandlers.size();i++)
  {
    if(!strcmp(dynamicHandlers[i].uri,uri))
      return;
  } 

   WebServerRequestHandlerData dt;
   dt.uri = uri;
   dt.handler = handler;

   dynamicHandlers.push_back(dt);
}
//--------------------------------------------------------------------------------------------------------------------------------------
WebServerRequestHandler CoreESPWebServerClass::getDynamicHandler(const char* uri)
{
  for(size_t i=0;i<dynamicHandlers.size();i++)
  {
    if(!strcmp(dynamicHandlers[i].uri,uri))
      return dynamicHandlers[i].handler;
  }

  return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPWebServerClass::processURI(CoreTransportClient* client, String& uri)
{
    const char* filename = uri.c_str();
    const char* paramPtr = strstr(filename,"?");
    if(paramPtr)
    {
      uri[paramPtr - filename] = '\0';
      paramPtr++;
    }

    DBG(F("URI: "));
    DBGLN(filename);

    if(paramPtr)
    {
      DBG(F("PARAMS: "));
      DBGLN(paramPtr);      
    }

    // тут смотрим - если есть обработчик, прявязанный к имени файла - вызываем его, иначе - пытаемся читать с SD
      WebServerRequestHandler handler = getDynamicHandler(strlen(filename) ? filename : "/");
      if(handler)
      {
        dynamicHandlerClient = client;
        handler(filename,paramPtr);
        dynamicHandlerClient = NULL;
      } // handler exists
      else
      {
          // обработчика не назначено, действуем по умолчанию
          
          #ifdef CORE_SD_SUPPORT_ENABLED
          
            // поддержка SD присутствует, читаем файл
      
            #ifdef CORE_SD_USE_SDFAT
              SdFile f;
              f.open(filename,O_READ);
              if(!f.isOpen())
              {
                send404(client);
                return;
              }
            #else
              File f = SD.open(filename,FILE_READ);
              if(!f)
              {
                send404(client);
                return;          
              }
            #endif
           
                // файл открыт, получаем его длину, формируем заголовки и отсылаем его клиенту
                unsigned long contentLength = 
            
                #ifdef CORE_SD_USE_SDFAT
                  f.fileSize();
                #else
                  f.size();
                #endif
            
                  String headers = WEB_HEADER_BEGIN;
                  headers += F("200 OK");
                  headers += WEB_HEADER_LINE;
            
                  headers += WEB_HEADER_CONNECTION;
                  headers += WEB_HEADER_LINE;
            
                  headers += WEB_HEADER_CONTENT_TYPE;
                  headers += getContentType(filename);
                  headers += WEB_HEADER_LINE;
            
                  headers += WEB_HEADER_CONTENT_LENGTH;
                  headers += contentLength;
                  headers += WEB_HEADER_LINE;
                  headers += WEB_HEADER_LINE;
            
            
                // запоминаем, сколько надо отослать данных и в какого клиента
                CoreWebServerPendingFileData pfd;
                pfd.pendingBytes = contentLength;
                pfd.client = client;
                pfd.file = f;
                pendingFiles.push_back(pfd);
            
                // отсылаем заголовки
                client->write((uint8_t*)headers.c_str(),headers.length());    
      
          #else
            // не включена поддержка SD, ничего не выдаём
            send404(client);
          #endif
          
    } // else no linked handler
    
     
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPWebServerClass::removePendingFileData(CoreTransportClient* client)
{
  for(size_t i=0;i<pendingFiles.size();)
  {
     if(pendingFiles[i].client == client)
     {
        //Тут закрытие файла
        if(pendingFiles[i].file
         #ifdef CORE_SD_USE_SDFAT
         .isOpen()
         #endif
         )
         {
          pendingFiles[i].file.close();
         }
         
        for(size_t k=i+1;k<pendingFiles.size();k++)
        {
          pendingFiles[k-1] = pendingFiles[k];
        }
        pendingFiles.pop();
        break;
     }
     else
     {
      i++;
     }
  } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreWebServerPendingFileData* CoreESPWebServerClass::getPendingFileData(CoreTransportClient* client)
{
  for(size_t i=0;i<pendingFiles.size();i++)
  {
     if(pendingFiles[i].client == client)
     {
        return &(pendingFiles[i]);
     }
  }

  return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPWebServerClass::OnClientDataAvailable(CoreTransportClient& client, bool isDone)
{
    // данные для клиента пришли
    const char* data = (const char*) client.getData();
    size_t dataSize = client.getDataSize();
      
    if(!isOurClient(&client))
    {
      // нет в списке клиентов, проверяем, возможно, запрос к нам

      if(dataSize > 4)
      {
          bool httpQueryFound = Core.memFind(data,dataSize,"GET ",4) == data && Core.memFind(data,dataSize,"HTTP/",5) != NULL;
          
          if(httpQueryFound)
          {
            bool hasCompletedQuery  = Core.memFind(data,dataSize,"\r\n\r\n",4) != NULL;
            if(hasCompletedQuery)
            {
              /*
              DBGLN(F("WEB: Completed query found!"));
              DBGLN(F("================================================"));

              #ifdef _CORE_DEBUG
                Serial.write(data,dataSize);
              #endif

              DBGLN(F("================================================"));
              */
              
              // уже есть готовый запрос, выщемляем первую строку - и вперёд

              const char* rn = (const char*) Core.memFind(data,dataSize,"\r\n",2);
                            
              char* query = new char[rn-data+1];              
              memcpy(query,data,rn-data);
              query[rn-data] = 0;
              
              processQuery(&client, query);
              
              delete [] query;
            }
            else
            {
              DBGLN(F("WEB: Uncompleted query, save first line"));
                // нет полного запроса, выщемляем первую строку - и сохраняем
              const char* rn = (const char*) Core.memFind((const uint8_t*)data,dataSize,(const uint8_t*)"\r\n",2);
              if(rn)
              {
                char* query = new char[rn-data+1];
                memcpy(query,data,rn-data);
                query[rn-data] = 0;

                CoreWebServerQuery pending;
                pending.client = &client;
                pending.query = query;
                pendingQueries.push_back(pending);
              }
              
              
            } // else
          } // httpQueryFound
          
      } // if(dataSize > 4)
      
    } // not our client
    else
    {
      // наш клиент, уже есть в списке клиентов, надо проверить - пришёл ли весь запрос
      bool hasCompletedQuery = isDone;
      if(hasCompletedQuery)
      {
        DBGLN(F("WEB: client data done, process query..."));
        CoreWebServerQuery* pending = getPendingQuery(&client);
        if(pending)
        {
          processQuery(&client, pending->query);
        }
        
      }
    } // else our client
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPWebServerClass::removePendingQuery(CoreWebServerQuery* query)
{
  for(size_t i=0;i<pendingQueries.size();)
  {
      if(&(pendingQueries[i]) == query)
      {
        delete[] query->query;
        for(size_t k=i+1;k<pendingQueries.size();k++)
        {
          pendingQueries[k-1] = pendingQueries[k];
        }
        pendingQueries.pop();
        break;
      }
      else
        i++;
  } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_ESP_WEB_SERVER
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_ESP_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_MQTT_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreMQTTSettings MQTTSettings;
//--------------------------------------------------------------------------------------------------------------------------------------
CoreMQTT MQTT;
//--------------------------------------------------------------------------------------------------------------------------------------
CoreMQTT::CoreMQTT()
{
  currentClient = NULL;
  timer = 0;
  machineState = mqttWaitClient;
  currentTransport = NULL;
  mqttMessageId = 0;
  streamBuffer = new String();
  currentStoreNumber = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::reset()
{
  // тут сброс - вызывается, когда конфиг перезагружается  
  // освобождаем клиента
  currentClient = NULL;
  timer = 0;
  machineState = mqttWaitClient;
  currentTransport = NULL;
  mqttMessageId = 0;
  currentStoreNumber = 0;
  clearReportsQueue();
  clearPublishQueue();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::clearPublishQueue()
{
  for(size_t i=0;i<publishList.size();i++)
  {
    delete [] publishList[i].payload;
    delete [] publishList[i].topic;
  }

  publishList.empty();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::processIncomingPacket(CoreTransportClient* client)
{
  const uint8_t* packet = client->getData();
  size_t dataLen = client->getDataSize();

  if(!dataLen)
    return;


  if(dataLen > 0)
  {

    uint8_t bCommand = packet[0];
    if((bCommand & MQTT_PUBLISH_COMMAND) == MQTT_PUBLISH_COMMAND)
    {
      // это к нам опубликовали топик
        DBGLN(F("MQTT: PUBLISH topic found!!!"));

      bool isQoS1 = (bCommand & 6) == MQTT_QOS1;

      // декодируем длину сообщения
      
        unsigned long multiplier = 1;
        int remainingLength = 0;
        unsigned int curReadPos = 1;
        uint8_t encodedByte;
        
        do
        {
          encodedByte =  packet[curReadPos];
          curReadPos++;
          
          remainingLength += (encodedByte & 127) * multiplier;
          multiplier *= 128;
          
        if (multiplier > 0x200000)
          break; // malformed
          
        } while ((encodedByte & 128) != 0);


        DBG(F("MQTT: Remaining length: "));
        DBGLN(remainingLength);

      if(curReadPos >= dataLen) // malformed
      {
          DBGLN(F("MQTT: MALFORMED 1"));
        return;
      }

      // теперь получаем имя топика
      uint8_t topicLengthMSB = packet[curReadPos];    
      curReadPos++;

      if(curReadPos >= dataLen) // malformed
      {
          DBGLN(F("MQTT: MALFORMED 2"));
        return;
      }
            
      uint8_t topicLengthLSB = packet[curReadPos];
      curReadPos++;

      uint16_t topicLength = (topicLengthMSB<<8)+topicLengthLSB;
      
      DBG(F("MQTT: Topic length: "));
      DBGLN(topicLength);


      // теперь собираем топик
      String topic;
      for(uint16_t j=0;j<topicLength;j++)
      {
        if(curReadPos >= dataLen) // malformed
        {
            DBGLN(F("MQTT: MALFORMED 3"));
          return;
        }        
        topic += (char) packet[curReadPos];
        curReadPos++;
      }

      // тут работаем с payload, склеивая его с топиком
      if(isQoS1)
      {
       // игнорируем ID сообщения
       curReadPos += 2; // два байта на ID сообщения
      }


      String* payload = new String();

      for(size_t p=curReadPos;p<dataLen;p++)
      {
        (*payload) += (char) packet[p];
      }

      if(payload->length())
      {
            DBG(F("MQTT: Payload are: "));
            DBGLN(*payload);

          // теперь склеиваем payload с топиком
          if(topic.length() && topic[topic.length()-1] != '/')
          {
            if((*payload)[0] != '/')
              topic += '/';
          }

          topic += *payload;
      }
      
      delete payload;
      
      if(topic.length())
      {
            DBG(F("MQTT: Topic are: "));
            DBGLN(topic);

          const char* setCommandPtr = strstr_P(topic.c_str(),(const char*) F("SET/") );
          const char* getCommandPtr = strstr_P(topic.c_str(),(const char*) F("GET/") );
          bool isSetCommand = setCommandPtr != NULL;
          bool isGetCommand = getCommandPtr != NULL;

          if(isSetCommand || isGetCommand)
          {
            const char* normalizedTopic = isSetCommand ? setCommandPtr : getCommandPtr;

            // нашли команду SET или GET, перемещаемся за неё
            //normalizedTopic += 4;

            // удаляем ненужные префиксы
            topic.remove(0,normalizedTopic - topic.c_str() );
            bool bFirst = true;
            for(unsigned int k=0;k<topic.length();k++)
            {
              if(topic[k] == '/')
              {
                if(bFirst)
                {
                  bFirst = false;
                  topic[k] = '='; 
                }
                else
                  topic[k] = CORE_COMMAND_PARAM_DELIMITER;             
              }
            } // for

              DBG(F("Normalized topic are: "));
              DBGLN(topic);

              delete streamBuffer;
              streamBuffer = new String();
              Core.processCommand(topic,this);

              // тут получили ответ, и надо опубликовать его в брокер
              pushToReportQueue(streamBuffer);
            
          } // if(isSetCommand || isGetCommand)
          else // unsupported topic
          {
              DBG(F("Unsupported topic: "));
              DBGLN(topic);
          } // else
          
      } // if(topic.length())
      else
      {
        DBGLN(F("Malformed topic name!!!"));
      }

    } // if((bCommand & MQTT_PUBLISH_COMMAND) == MQTT_PUBLISH_COMMAND)
    
  } // if(dataLen > 0)    
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::pushToReportQueue(String* toReport)
{
  
  String* newReport = new String();
  *newReport = *toReport;

  DBG(F("MQTT: Want to report - "));
  DBGLN(*newReport);

  reportQueue.push_back(newReport);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::OnClientDataAvailable(CoreTransportClient& client, bool isDone)
{
  if(!currentClient || &client != currentClient) // не наш клиент
    return;

  timer = millis();

//  DBGLN(F("MQTT: DATA FROM CLIENT !!!"));

  if(machineState == mqttWaitSendConnectPacketDone)
  {
//    DBGLN(F("MQTT: CONNECT packet was sent!"));
    machineState = mqttSendSubscribePacket;
  }
  else
  if(machineState == mqttWaitSendSubscribePacketDone)
  {
//    DBGLN(F("MQTT: SUBSCRIBE packet was sent!"));
    machineState = mqttSendPublishPacket;
  }
  else
  if(machineState == mqttWaitSendPublishPacketDone)
  {
//    DBGLN(F("MQTT: PUBLISH packet was sent!"));
    // отсылали пакет публикации, тут к нам пришла обратка,
    // поскольку мы подписались на все топики для нашего клиента, на будущее
     machineState = mqttSendPublishPacket;
  }
  else
  {
//      DBG(F("MQTT: OnClientDataAvailable - UNHANDLED MACHINE STATE: "));
//      DBGLN(machineState);

      // тут разбираем, что пришло от брокера. Если мы здесь, значит данные от брокера
      // пришли в необрабатываемую ветку, т.е. это публикация прямо с брокера.
      processIncomingPacket(currentClient);
  }
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::OnClientDataWritten(CoreTransportClient& client, int errorCode)
{
  
  if(!currentClient || &client != currentClient) // не наш клиент
    return;
  
  timer = millis();
   
  if(errorCode != CT_ERROR_NONE)
  {
    DBGLN(F("MQTT: Can't write to client!"));
    clearReportsQueue();
    clearPublishQueue();
    currentClient = NULL;
    machineState = mqttWaitReconnect;

    return;
  }
  
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::OnClientConnect(CoreTransportClient& client, bool connected, int errorCode)
{
  if(!currentClient || &client != currentClient) // не наш клиент
    return;

//  DBG(F("MQTT: OnClientConnect, connected = "));
//  DBGLN(connected);

  if(!connected)
  {
    // клиент не подсоединился, сбрасываем текущего клиента и вываливаемся в ожидание переподсоединения.
    DBGLN(F("MQTT: Can't connect to broker, try to reconnect..."));
    clearReportsQueue();
    clearPublishQueue();
    currentClient = NULL;
    machineState = mqttWaitReconnect;
    timer = millis();    
  }
  else
  {
    // клиент подсоединён, переходим на отсыл пакета с авторизацией
    machineState = mqttSendConnectPacket;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::convertAnswerToJSON(const String& answer, String* resultBuffer)
{
  // тут мы должны сформировать объект JSON из ответа, для этого надо разбить ответ по разделителям, и для каждого параметра создать именованное поле
  // в анонимном JSON-объекте
  // прикинем, сколько нам памяти надо резервировать, чтобы вместиться
  int neededJsonLen = 3; // {} - под скобки и завершающий ноль
  // считаем кол-во параметров ответа
  int jsonParamsCount=1; // всегда есть один ответ
  int answerLen = answer.length();
  
  for(int j=0;j<answerLen;j++)
  {
    if(answer[j] == CORE_COMMAND_PARAM_DELIMITER) // разделитель
      jsonParamsCount++;
  }
  // у нас есть количество параметров, под каждый параметр нужно минимум 6 символов ("p":""), плюс длина числа, которое будет как имя
  // параметра, плюс длина самого параметра, плюс запятые между параметрами
  int paramNameCharsCount = jsonParamsCount > 9 ? 2 : 1;

   neededJsonLen += (6 + paramNameCharsCount)*jsonParamsCount + (jsonParamsCount-1) + answer.length();

   // теперь можем резервировать память
   resultBuffer->reserve(neededJsonLen);

   // теперь формируем наш JSON-объект
   *resultBuffer = '{'; // начали объект

    if(answerLen > 0)
    {
       int currentParamNumber = 1;

       *resultBuffer += F("\"p");
       *resultBuffer += currentParamNumber;
       *resultBuffer += F("\":\"");
       
       for(int j=0;j<answerLen;j++)
       {
         if(answer[j] == CORE_COMMAND_PARAM_DELIMITER)
         {
           // достигли нового параметра, закрываем предыдущий и формируем новый
           currentParamNumber++;
           *resultBuffer += F("\",\"p");
           *resultBuffer += currentParamNumber;
           *resultBuffer += F("\":\"");
         }
         else
         {
            char ch = answer[j];
            
            if(ch == '"' || ch == '\\')
              *resultBuffer += '\\'; // экранируем двойные кавычки и обратный слеш
              
            *resultBuffer += ch;
         }
       } // for

       // закрываем последний параметр
       *resultBuffer += '"';
    } // answerLen > 0

   *resultBuffer += '}'; // закончили объект

}
//--------------------------------------------------------------------------------------------------------------------------------
bool CoreMQTT::publish(const char* topicName, const char* payload)
{
  
  if(!MQTTSettings.enabled || MQTTSettings.workMode == workModeDisabled || !currentTransport || !currentClient || !topicName) // выключены
    return false; 
    
  MQTTPublishQueue pq;
  int tnLen = strlen(topicName);
  pq.topic = new char[tnLen+1];
  memset(pq.topic,0,tnLen+1);
  strcpy(pq.topic,topicName);

  pq.payload = NULL;
  if(payload)
  {
    int pllen = strlen(payload);
    pq.payload = new char[pllen+1];
    memset(pq.payload,0,pllen+1);
    strcpy(pq.payload,payload);    
  }

  publishList.push_back(pq);

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::update()
{
  if(!MQTTSettings.enabled || MQTTSettings.workMode == workModeDisabled || !currentTransport) // выключены
    return; 
  
  switch(machineState)
  {
    
      case mqttWaitClient:
      {
        if(currentTransport->ready())
        {
            currentClient = currentTransport->getFreeClient();
            if(currentClient)
            {
              DBG(F("MQTT: Catch free client #"));
              DBGLN(currentClient->getID());
              DBG(F("MQTT: connect to "));
              DBG(MQTTSettings.serverAddress);
              DBG(":");
              DBGLN(MQTTSettings.serverPort);

              currentClient->connect(MQTTSettings.serverAddress.c_str(), MQTTSettings.serverPort);
              machineState = mqttWaitConnection; 
              timer = millis();
            }
        } // if(currentTransport->ready())
      }
      break; // mqttWaitClient

      case mqttWaitConnection:
      {
        unsigned long toWait = 20000;
        if(MQTTSettings.workMode == workModeThroughSIM800)
          toWait = 80000; // максимальное время ответа для SIM800 - 75 секунд
        
        if(millis() - timer > toWait)
        {
          DBG(F("MQTT: unable to connect within "));
          DBG(toWait/1000);
          DBGLN(F(" seconds, try to reconnect..."));
          
          // долго ждали, переподсоединяемся
          clearReportsQueue();
          clearPublishQueue();
          currentClient = NULL;
          machineState = mqttWaitReconnect;
          timer = millis();
        }
      }
      break; // mqttWaitConnection

      case mqttWaitReconnect:
      {
        if(millis() - timer > 10000)
        {
          DBGLN(F("MQTT: start reconnect!"));
          clearReportsQueue();
          clearPublishQueue();
          currentClient = NULL;
          machineState = mqttWaitClient;
        }
      }
      break; // mqttWaitReconnect

      case mqttSendConnectPacket:
      {
        if(currentClient)
        {
          DBGLN(F("MQTT: start send connect packet!"));
  
          String mqttBuffer;
          int mqttBufferLength;
          
          constructConnectPacket(mqttBuffer,mqttBufferLength,
            MQTTSettings.clientID.c_str() // client id
          , MQTTSettings.userName.length() ? MQTTSettings.userName.c_str() : NULL // user
          , MQTTSettings.password.length() ? MQTTSettings.password.c_str() : NULL // pass
          , NULL // will topic
          , 0 // willQoS
          , 0 // willRetain
          , NULL // will message
          );

          // переключаемся на ожидание результата отсылки пакета
          machineState = mqttWaitSendConnectPacketDone;
          
          // сформировали пакет CONNECT, теперь отсылаем его брокеру
          currentClient->write((uint8_t*) mqttBuffer.c_str(),mqttBufferLength);

        //   DBGLN(F("MQTT: CONNECT packet written!"));
          
          timer = millis();
        }  // if(currentClient)
        else
        {
          DBGLN(F("MQTT: No client in construct CONNECT packet mode!"));
          machineState = mqttWaitReconnect;
          timer = millis();          
        } // no client
        
      }
      break; // mqttSendConnectPacket

      case mqttSendSubscribePacket:
      {
        DBGLN(F("MQTT: Subscribe to topics!"));

        if(currentClient)
        {
          String mqttBuffer;
          int mqttBufferLength;
            
          // конструируем пакет подписки
          String topic = MQTTSettings.clientID;
          topic +=  F("/#");
          constructSubscribePacket(mqttBuffer,mqttBufferLength, topic.c_str());
  
          // переключаемся на ожидание результата отсылки пакета
          machineState = mqttWaitSendSubscribePacketDone;
          
          // сформировали пакет SUBSCRIBE, теперь отсылаем его брокеру
          currentClient->write((uint8_t*) mqttBuffer.c_str(),mqttBufferLength);
          timer = millis();
        }
        else
        {
          DBGLN(F("MQTT: No client in construct SUBSCRIBE packet mode!"));
          machineState = mqttWaitReconnect;
          timer = millis();          
        } // no client
      
      }
      break; // mqttSendSubscribePacket

      case mqttSendPublishPacket:
      {
        // тут мы находимся в процессе публикации, поэтому можем проверять - есть ли топики для репорта
        bool hasReportTopics = reportQueue.size() > 0;
        bool hasPublishTopics = publishList.size() > 0;
        
        unsigned long interval = MQTTSettings.intervalBetweenTopics;
        if(hasReportTopics || hasPublishTopics || millis() - timer > interval)
        {
          DBGLN(F("MQTT: SEND NEXT TOPIC!"));

          if(currentClient)
          {
            String mqttBuffer;
            int mqttBufferLength;
  
            // пока просто потестируем
            String topicName, data;

            if(hasReportTopics)
            {
              // у нас есть топик для репорта
              topicName =  MQTTSettings.clientID + F("/REPORT/");

              // удаляем перевод строки
              reportQueue[0]->trim();

              // тут в имя топика надо добавить запрошенную команду, чтобы в клиенте можно было ориентироваться
              // на конкретные топики отчёта
              int idx = reportQueue[0]->indexOf("=");
              String commandStatus = reportQueue[0]->substring(0,idx);
              reportQueue[0]->remove(0,idx+1);

              // теперь в reportQueue[0] у нас лежит ответ после OK= или ER=
              String delim = String(CORE_COMMAND_PARAM_DELIMITER);
              idx = reportQueue[0]->indexOf(delim);
              if(idx != -1)
              {
                // есть ответ с параметрами, выцепляем первый - это и будет дополнением к имени топика
                topicName += reportQueue[0]->substring(0,idx);
                reportQueue[0]->remove(0,idx);
                *reportQueue[0] = commandStatus + *reportQueue[0];
              }
              else
              {
                // только один ответ - имя команды, без возвращённых параметров
                topicName += *reportQueue[0];
                *reportQueue[0] = commandStatus;
              }
              

              #ifdef MQTT_REPORT_AS_JSON
                // сперва убираем =, и заменяем его на |
                //reportQueue[0]->replace("=", String(CORE_COMMAND_PARAM_DELIMITER));
                // и конвертируем в JSON
                convertAnswerToJSON(*(reportQueue[0]),&data);
              #else
                data = *(reportQueue[0]);            
              #endif

              // тут удаляем из очереди первое вхождение отчёта
              if(reportQueue.size() < 2)
                clearReportsQueue();
              else
              {
                  delete reportQueue[0];
                  for(size_t k=1;k<reportQueue.size();k++)
                  {
                    reportQueue[k-1] = reportQueue[k];
                  }
                  reportQueue.pop();
              }
            } // hasReportTopics
            else
            if(hasPublishTopics)
            {
              // есть пакеты для публикации
              MQTTPublishQueue pq = publishList[0];

              // тут публикуем из пакета для публикации
              topicName =  MQTTSettings.clientID + "/";
              topicName += pq.topic;

              if(pq.payload)
                data = pq.payload;

              // чистим память
              delete [] pq.topic;
              delete [] pq.payload;
              
              // и удаляем из списка
              if(publishList.size() < 2)
                publishList.empty();
              else
              {
                for(size_t kk=1;kk<publishList.size();kk++)
                {
                  publishList[kk-1] = publishList[kk];  
                }
                publishList.pop();
              }
            } // hasPublishTopics
            else
            {
              // обычный режим работы, отсылаем показания с хранилища
                if(CoreDataStore.size())
                {
                  // есть данные в хранилище, получаем их
                  CoreStoredData dataStored = CoreDataStore.get(currentStoreNumber);

                  CoreTextFormatProvider textFormatter;
                  data = textFormatter.format(dataStored,0,false);
                  topicName = MQTTSettings.clientID + "/" + dataStored.sensor->getName();

                  currentStoreNumber++;
                  if(currentStoreNumber >= CoreDataStore.size())
                    currentStoreNumber = 0;
                } // if
            }

              if(data.length() && topicName.length())
              {
                 // конструируем пакет публикации
                 constructPublishPacket(mqttBuffer,mqttBufferLength,topicName.c_str(), data.c_str()); 
      
                // переключаемся на ожидание результата отсылки пакета
                machineState = mqttWaitSendPublishPacketDone;
              
                // сформировали пакет PUBLISH, теперь отсылаем его брокеру
                currentClient->write((uint8_t*) mqttBuffer.c_str(),mqttBufferLength);
                timer = millis();
              }
          }
          else
          {
            DBGLN(F("MQTT: No client in construct PUBLISH packet mode!"));
            machineState = mqttWaitReconnect;
            timer = millis();          
          } // no client          
           
        }
      }
      break; // mqttSendPublishPacket

      case mqttWaitSendConnectPacketDone:
      case mqttWaitSendSubscribePacketDone:
      case mqttWaitSendPublishPacketDone:
      {
        if(millis() - timer > 20000)
        {
          DBGLN(F("MQTT: wait for send results timeout, reconnect!"));
          // долго ждали результата записи в клиента, переподсоединяемся
          clearReportsQueue();
          clearPublishQueue();
          currentClient = NULL;
          machineState = mqttWaitReconnect;
          timer = millis();
        }        
      }
      break;
      
    
  } // switch

  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::clearReportsQueue()
{
  for(size_t i=0;i<reportQueue.size();i++)
  {
    delete reportQueue[i];
  }

  reportQueue.empty();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::constructPublishPacket(String& mqttBuffer,int& mqttBufferLength, const char* topic, const char* payload)
{
  MQTTBuffer byteBuffer; // наш буфер из байт, в котором будет содержаться пакет

  // тут формируем пакет

  // кодируем топик
  encode(byteBuffer,topic);

  // теперь пишем данные топика
  int sz = strlen(payload);
  const char* readPtr = payload;
  for(int i=0;i<sz;i++)
  {
    byteBuffer.push_back(*readPtr++);
  }   

  size_t payloadSize = byteBuffer.size();

  MQTTBuffer fixedHeader;
  
  constructFixedHeader(MQTT_PUBLISH_COMMAND,fixedHeader,payloadSize);

  writePacket(fixedHeader,byteBuffer,mqttBuffer,mqttBufferLength);
  
}
//--------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::constructSubscribePacket(String& mqttBuffer,int& mqttBufferLength, const char* topic)
{
  MQTTBuffer byteBuffer; // наш буфер из байт, в котором будет содержаться пакет

  // тут формируем пакет подписки

  // сначала записываем ID сообщения
  mqttMessageId++;
  
  if(!mqttMessageId)
    mqttMessageId = 1;
    
  byteBuffer.push_back((mqttMessageId >> 8));
  byteBuffer.push_back((mqttMessageId & 0xFF));

  // кодируем топик, на который подписываемся
  encode(byteBuffer,topic);

  // теперь пишем байт QoS
  byteBuffer.push_back(1);

  size_t payloadSize = byteBuffer.size();

  MQTTBuffer fixedHeader;
  
  constructFixedHeader(MQTT_SUBSCRIBE_COMMAND | MQTT_QOS1, fixedHeader, payloadSize);

  writePacket(fixedHeader,byteBuffer,mqttBuffer,mqttBufferLength);  
}
//--------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::constructConnectPacket(String& mqttBuffer,int& mqttBufferLength,const char* id, const char* user, const char* pass
,const char* willTopic,uint8_t willQoS, uint8_t willRetain, const char* willMessage)
{
  mqttBuffer = "";

  MQTTBuffer byteBuffer; // наш буфер из байт, в котором будет содержаться пакет

  // теперь формируем переменный заголовок

  // переменный заголовок, для команды CONNECT
  byteBuffer.push_back(0);
  byteBuffer.push_back(6); // длина версии протокола MQTT
  byteBuffer.push_back('M');
  byteBuffer.push_back('Q');
  byteBuffer.push_back('I');
  byteBuffer.push_back('s');
  byteBuffer.push_back('d');
  byteBuffer.push_back('p');

  byteBuffer.push_back(3); // версия протокола - 3

  // теперь рассчитываем флаги
  byte flags = 0;

  if(willTopic)
    flags = 0x06 | (willQoS << 3) | (willRetain << 5);
  else
    flags = 0x02;

  if(user) // есть имя пользователя
    flags |= (1 << 7);

  if(pass) // есть пароль
    flags |= (1 << 6);
  
   byteBuffer.push_back(flags);

   // теперь смотрим настройки keep-alive
   int keepAlive = 60; // 60 секунд
   byteBuffer.push_back((keepAlive >> 8));
   byteBuffer.push_back((keepAlive & 0xFF));

   // теперь записываем payload, для этого каждую строку надо закодировать
   encode(byteBuffer,id);
   encode(byteBuffer,willTopic);
   encode(byteBuffer,willMessage);
   encode(byteBuffer,user);
   encode(byteBuffer,pass);

   // теперь мы имеем буфер переменной длины, нам надо подсчитать его длину, сворфировать фиксированный заголовок,
   // и сохранить всё в буфере
   size_t payloadSize = byteBuffer.size();
   MQTTBuffer fixedHeader;
   constructFixedHeader(MQTT_CONNECT_COMMAND,fixedHeader,payloadSize);

   writePacket(fixedHeader,byteBuffer,mqttBuffer,mqttBufferLength);


   // всё, пакет сформирован
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::writePacket(MQTTBuffer& fixedHeader, MQTTBuffer& payload, String& mqttBuffer,int& mqttBufferLength)
{
  mqttBuffer = "";
  
// запомнили, сколько байт надо послать в ESP
   mqttBufferLength = fixedHeader.size() + payload.size();

   // теперь записываем это в строку, перед этим зарезервировав память, и заполнив строку пробелами
   mqttBuffer.reserve(mqttBufferLength);
   for(int i=0;i<mqttBufferLength;i++)
    mqttBuffer += ' ';

  // теперь можем копировать данные в строку побайтово
  int writePos = 0;

  // пишем фиксированный заголовок
  for(size_t i=0;i<fixedHeader.size();i++)
  {
    mqttBuffer[writePos++] = fixedHeader[i];
  }
  
  // и переменный
  for(size_t i=0;i<payload.size();i++)
  {
    mqttBuffer[writePos++] = payload[i];
  }  
}
//--------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::constructFixedHeader(byte command, MQTTBuffer& fixedHeader, size_t payloadSize)
{
    fixedHeader.push_back(command); // пишем тип команды
  
    uint8_t remainingLength[4];
    uint8_t digit;
    uint8_t written = 0;
    uint16_t len = payloadSize;
    
    do 
    {
        digit = len % 128;
        len = len / 128;
        if (len > 0) 
        {
            digit |= 0x80;
        }
        
        remainingLength[written++] = digit;
        
    } while(len > 0);

    // мы записали written символов, как длину переменного заголовка - теперь пишем эти байты в фиксированный
    
    for(uint8_t i=0;i<written;i++)
    {
      fixedHeader.push_back(remainingLength[i]);
    }

}
//--------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::encode(MQTTBuffer& buff,const char* str)
{
  if(!str)
    return;

    size_t sz = buff.size(); // запоминаем текущий размер

    // записываем нули, как длину строки, потом мы это поправим
    buff.push_back(0);
    buff.push_back(0);

    const char* ptr = str;
    int strLen = 0;
    while(*ptr)
    {
      buff.push_back(*ptr++);
      strLen++;
    }

    // теперь записываем актуальную длину
    buff[sz] = (strLen >> 8);
    buff[sz+1] = (strLen & 0xFF);
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::begin()
{
  // попросили начать работу
  // для начала - освободим клиента
  currentClient = NULL;
  machineState = mqttWaitClient;
  currentTransport = NULL;
  mqttMessageId = 0;
  currentStoreNumber = 0;

    switch(MQTTSettings.workMode)
    {
      case workModeDisabled:
      break;

      case workModeThroughESP:
        #ifdef CORE_ESP_TRANSPORT_ENABLED
          currentTransport = &ESP;
        #endif
      break;

      case workModeThroughSIM800:
      #ifdef CORE_SIM800_TRANSPORT_ENABLED
          currentTransport = &SIM800;
      #endif
      break;
    }

  // подписываемся на события клиентов
  if(currentTransport)
  {
    currentTransport->subscribe(this);  
  }
    
  // ну и запомним, когда вызвали начало работы
  timer = millis();
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_MQTT_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_SIM800_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#include "CorePDU.h"
//--------------------------------------------------------------------------------------------------------------------------------------
SIM800TransportSettingsClass SIM800TransportSettings;
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSIM800Transport SIM800;
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSIM800Transport::CoreSIM800Transport() : CoreTransport()
{
  for(int i=0;i<SIM800_MAX_CLIENTS;i++)
    clients[i] = NULL;

  sim800ReceiveBuff = new String();
  smsToSend = new String();

}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::sendCommand(const String& command, bool addNewLine)
{
  DBG(F("SIM800: ==>> "));
  DBGLN(command);
  
  workStream->write(command.c_str(),command.length());
  
  if(addNewLine)
  {
    workStream->println();
  }  

  machineState = sim800WaitAnswer; // говорим, что надо ждать ответа от SIM800
  // запоминаем время отсылки последней команды
  timer = millis();
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::sendCommand(SIM800Commands command)
{
  currentCommand = command;
  
  // тут посылаем команду в ESP
  switch(command)
  {
    case smaNone:
    case smaCIPSTART:
    case smaCIPSEND:
    case smaWaitSendDone:
    case smaCIPCLOSE:
    case smaCMGS:
    case smaWaitForSMSClearance:
    case smaWaitSMSSendDone:
    break;

    case smaCheckReady:
    {
      DBGLN(F("SIM800: Check for modem READY..."));
      sendCommand(F("AT+CPAS"));
    }
    break;

    case smaCIPHEAD:
    {
      DBGLN(F("SIM800: Set IPD setting..."));
      sendCommand(F("AT+CIPHEAD=1"));      
    }
    break;

    case smaCIICR:
    {
      DBGLN(F("SIM800: Activate GPRS connection..."));
      sendCommand(F("AT+CIICR"));            
    }
    break;

    case smaCIFSR:
    {
      sendCommand(F("AT+CIFSR"));      
    }
    break;

    case smaCSTT:
    {
      DBGLN(F("SIM800: Setup GPRS connection..."));
      
      String comm = F("AT+CSTT=\"");
      comm += SIM800TransportSettings.APN;
      comm += F("\",\"");
      comm += SIM800TransportSettings.APNUser;
      comm += F("\",\"");
      comm += SIM800TransportSettings.APNPassword;
      comm += F("\"");

      sendCommand(comm);
    }
    break;

    case smaCIPMODE:
    {
      DBGLN(F("SIM800: Set CIPMODE..."));
      sendCommand(F("AT+CIPMODE=0"));            
    }
    break;

    case smaCIPMUX:
    {
      DBGLN(F("SIM800: Set CIPMUX..."));
      sendCommand(F("AT+CIPMUX=1"));            
    }
    break;

    case smaEchoOff:
    {
      DBGLN(F("SIM800: echo OFF..."));
      sendCommand(F("ATE0"));
    }
    break;

    case smaDisableCellBroadcastMessages:
    {
      DBGLN(F("SIM800: disable cell broadcast messagess..."));
      sendCommand(F("AT+CSCB=1"));
    }
    break;

    case smaAON:
    {
        DBGLN(F("SIM800: Turn AON ON..."));      
        sendCommand(F("AT+CLIP=1"));  
    }
    break;

    case smaPDUEncoding:
    {
      DBGLN(F("SIM800: Set PDU format..."));
      sendCommand(F("AT+CMGF=0"));
    }
    break;

    case smaUCS2Encoding:
    {
      DBGLN(F("SIM800: Set UCS2 format..."));
      sendCommand(F("AT+CSCS=\"UCS2\""));
    }
    break;

    case smaSMSSettings:
    {
      DBGLN(F("SIM800: Set SMS output mode..."));
      sendCommand(F("AT+CNMI=2,2"));
    }
    break;

    case smaWaitReg:
    {
      DBGLN(F("SIM800: Check registration status..."));
      sendCommand(F("AT+CREG?"));
    }
    break;

    case smaCheckModemHang:
    {
      DBGLN(F("SIM800: Check if modem available..."));      
      sendCommand(F("AT"));
    }
    break;

    case smaHangUp:
    {
      DBGLN(F("SIM800: Hang up..."));      
      sendCommand(F("ATH"));      
    }
    break;
    
  } // switch

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSIM800Transport::isKnownAnswer(const String& line, SIM800KnownAnswer& result)
{
  result = gsmNone;
  
  if(line == F("OK"))
  {
    result = gsmOK;
    return true;
  }
  if(line == F("ERROR"))
  {
    result = gsmError;
    return true;
  }
  if(line == F("FAIL"))
  {
    result = gsmFail;
    return true;
  }
  if(line.endsWith(F("SEND OK")))
  {
    result = gsmSendOk;
    return true;
  }
  if(line.endsWith(F("SEND FAIL")))
  {
    result = gsmSendFail;
    return true;
  }
  if(line.endsWith(F("CONNECT OK")))
  {
    result = gsmConnectOk;
    return true;
  }
  if(line.endsWith(F("CONNECT FAIL")))
  {
    result = gsmConnectFail;
    return true;
  }
  if(line.endsWith(F("ALREADY CONNECT")))
  {
    result = gsmAlreadyConnect;
    return true;
  }
  if(line.endsWith(F("CLOSE OK")))
  {
    result = gsmCloseOk;
    return true;
  }
  

  
  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::processIPD()
{
  DBG(F("SIM800: start parse +RECEIVE, received="));
  DBGLN(*sim800ReceiveBuff);

  // здесь в sim800ReceiveBuff лежит только команда вида +RECEIVE,<id>,<len>:
  // все данные надо вычитывать из потока
        
    int idx = sim800ReceiveBuff->indexOf(F(",")); // ищем первую запятую после +IPD
    const char* ptr = sim800ReceiveBuff->c_str();
    ptr += idx+1;
    // перешли за запятую, парсим ID клиента
    String connectedClientID = F("");
    while(*ptr != ',')
    {
      connectedClientID += (char) *ptr;
      ptr++;
    }
    ptr++; // за запятую
    String dataLen;
    while(*ptr != ':')
    {
      dataLen += (char) *ptr;
      ptr++; // перешли на начало данных
    }

    // получили ID клиента и длину его данных, которые - пока в потоке, и надо их быстро попакетно вычитать
    int clientID = connectedClientID.toInt();
    size_t lengthOfData = dataLen.toInt();
    
    if(clientID >=0 && clientID < SIM800_MAX_CLIENTS)
    {

     /* 
      DBG(F("SIM800: data for client  #"));
      DBG(clientID);
      DBG(F("; len="));
      DBGLN(lengthOfData);
      */
       CoreTransportClient* client = clients[clientID];

       // у нас есть lengthOfData с данными для клиента, нам надо побить это на пакеты длиной N байт,
       // и последовательно вызывать событие прихода данных. Это нужно для того, чтобы не переполнить оперативку,
       // поскольку у нас её - не вагон.

      // пусть у нас будет максимум 512 байт на пакет
      const int MAX_PACKET_SIZE = 512;
      
      // если длина всех данных меньше 512 - просто тупо все сразу вычитаем
       uint16_t packetSize = min(MAX_PACKET_SIZE,lengthOfData);

        // теперь выделяем буфер под данные
        uint8_t* buff = new uint8_t[packetSize];

        // у нас есть буфер, в него надо скопировать данные из потока
        uint8_t* writePtr = buff;
        
        size_t packetWritten = 0; // записано в пакет
        size_t totalWritten = 0; // всего записано

            while(totalWritten < lengthOfData) // пока не запишем все данные с клиента
            {
                if(workStream->available())
                {   
                  *writePtr++ = (uint8_t) workStream->read();
                  packetWritten++;
                  totalWritten++;
                }

                if(packetWritten >= packetSize)
                {
                  // скопировали один пакет
                  // передаём клиенту буфер, он сам его освободит, когда надо
                  setClientData(*client,buff,packetWritten);
    
                  // сообщаем подписчикам, что данные для клиента получены
                  notifyClientDataAvailable(*client, totalWritten >= lengthOfData);
    
                   
                  // пересчитываем длину пакета, вдруг там мало осталось, и незачем выделять под несколько байт огромный буфер
                  packetSize =  min(MAX_PACKET_SIZE, lengthOfData - totalWritten);
                  buff = new uint8_t[packetSize];
                  writePtr = buff; // на начало буфера
                  packetWritten = 0;
                }
              
            } // while

            // проверяем - есть ли остаток?
            if(packetWritten > 0)
            {
              // после прохода цикла есть остаток данных, уведомляем клиента
             // передаём клиенту буфер, он сам его освободит, когда надо
              setClientData(*client,buff,packetWritten);

              // сообщаем подписчикам, что данные для клиента получены
              notifyClientDataAvailable(*client, totalWritten >= lengthOfData);

            }
            else
            {
                // нет остатка, чистим буфер
                delete [] buff;
            }
                  
       
    } // if(clientID >=0 && clientID < ESP_MAX_CLIENTS)
    

  DBGLN(F("SIM800: +RECEIVE parsed."));  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::processIncomingCall(const String& line)
{
 // приходит строка вида
  // +CLIP: "79182900063",145,,,"",0
  
   // входящий звонок, проверяем, приняли ли мы конец строки?
    String ring = line.substring(8); // пропускаем команду +CLIP:, пробел и открывающую кавычку "

    int idx = ring.indexOf("\"");
    if(idx != -1)
      ring = ring.substring(0,idx);

    if(ring.length() && ring[0] != '+')
      ring = String(F("+")) + ring;

  // ищем - есть ли у нас этот номер среди известных
  bool isKnownNumber = false;

  for(size_t i=0;i<SIM800TransportSettings.KnownNumbers.size();i++)
  {
    if(SIM800TransportSettings.KnownNumbers[i]->startsWith(ring))
    {
      isKnownNumber = true;
      break;
    }
  }

  bool shouldHangUp = false;
  
  // вызываем событие
  ON_INCOMING_CALL(ring,isKnownNumber,shouldHangUp);
  
 // добавляем команду "положить трубку" - она выполнится первой, поскольку очередь инициализации у нас имеет приоритет
  if(shouldHangUp)
    initCommandsQueue.push_back(smaHangUp);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::processCMT(const String& pdu)
{
  if(pdu.length())
  {
    
    PDUIncomingMessage sms = PDU.Decode(pdu);

    if(sms.IsDecodingSucceed)
    {
      // СМС пришло, вызываем событие
      bool anyKnownNumbersFound = false;
        for(size_t i=0;i<SIM800TransportSettings.KnownNumbers.size();i++)
        {
          if(SIM800TransportSettings.KnownNumbers[i]->startsWith(sms.SenderNumber))
          {
            anyKnownNumbersFound = true;
            break;
          }
        }
      ON_SMS_RECEIVED(sms.SenderNumber,sms.Message, anyKnownNumbersFound);
    }

    
  } // if(pdu.length())
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::sendQueuedSMS()
{
  if(!outgoingSMSList.size())
    return;

  delete smsToSend;
  smsToSend = new String();
  
  int messageLength = 0;

  SIM800OutgoingSMS* sms = &(outgoingSMSList[0]);  
 
  PDUOutgoingMessage encodedMessage = PDU.Encode(*(sms->phone),*(sms->message),sms->isFlash, smsToSend);
  messageLength = encodedMessage.MessageLength;
  
/*
  ///////////////////////////////////////////////////
  *smsToSend = "0001000B919781920060F30008080074006500730074";
  messageLength = 21;
  ///////////////////////////////////////////////////
*/
    
  delete sms->phone;
  delete sms->message;

  if(outgoingSMSList.size() < 2)
  {
    outgoingSMSList.empty();
  }
  else
  {
    for(size_t i=1;i<outgoingSMSList.size();i++)
    {
      outgoingSMSList[i-1] = outgoingSMSList[i];
    }
    outgoingSMSList.pop();
  }
    
  // тут отсылаем СМС
  String command = F("AT+CMGS=");
  command += String(messageLength);

  flags.waitForDataWelcome = true;
  sendCommand(command);
  currentCommand = smaCMGS;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::update()
{
  if(!SIM800TransportSettings.enabled || !workStream)
    return;
 
  if(flags.onIdleTimer) // попросили подождать определённое время, в течение которого просто ничего не надо делать
  {
      if(millis() - timer > idleTime)
      {
        DBGLN(F("SIM800: idle done!"));
        flags.onIdleTimer = false;
      }
  } 


  // флаг, что есть ответ от ESP, выставляется по признаку наличия хоть чего-то в буфере приёма
  bool hasAnswer = workStream->available() > 0;

  // выставляем флаг, что мы хотя бы раз получили хоть чего-то от ESP
  flags.isAnyAnswerReceived = flags.isAnyAnswerReceived || hasAnswer;

  bool hasAnswerLine = false; // флаг, что мы получили строку ответа от модема

  char ch;
  while(workStream->available())
  {
    // здесь мы должны детектировать - пришло IPD или нет.
    // если пришли данные - мы должны вычитать их длину, и уже после этого - отправить данные клиенту,
    // напрямую читая из потока. Это нужно, потому что данные могут быть бинарными, и мы никогда в них не дождёмся
    // перевода строки.

     if(sim800ReceiveBuff->startsWith(F("+RECEIVE")) && sim800ReceiveBuff->indexOf(":") != -1)
     {
        DBGLN(F("SIM800: +RECEIVE detected, parse!"));
       
        processIPD();
                
        delete sim800ReceiveBuff;
        sim800ReceiveBuff = new String();
        timer = millis();        
        
        return;
     }

    ch = workStream->read();
    
    if(ch == '\r') // ненужный нам символ
      continue;
    else
    if(ch == '\n')
    {   
        hasAnswerLine = sim800ReceiveBuff->length(); // выставляем флаг, что у нас есть строка ответа от ESP  
        break; // выходим из цикла, остальное дочитаем позже, если это потребуется кому-либо
    }
    else
    {     
        if(flags.waitForDataWelcome && ch == '>') // ждём команду >  (на ввод данных)
        {
          flags.waitForDataWelcome = false;
          *sim800ReceiveBuff = F(">");
          hasAnswerLine = true;
          break; // выходим из цикла, получили приглашение на ввод данных
        }
        else
        {
          *sim800ReceiveBuff += ch;
                         
          if(sim800ReceiveBuff->length() > 2500) // буфер слишком длинный
          {
            DBGLN(F("SIM800: incoming data too long, skip it!"));
            delete sim800ReceiveBuff;
            sim800ReceiveBuff = new String();
          }
        }
    } // any char except '\r' and '\n' 
    
  } // while(workStream->available())


  if(hasAnswer)
  {
     timer = millis(); // не забываем обновлять таймер ответа - поскольку у нас что-то пришло - значит, модем отвечает
  }

      if(hasAnswerLine && sim800ReceiveBuff->length())
      {

        if(flags.pduInNextLine) // в прошлой строке пришло +CMT, поэтому в текущей - содержится PDU
        {
            flags.pduInNextLine = false;

            // разбираем, чего там пришло
            processCMT(*sim800ReceiveBuff);
          
            delete sim800ReceiveBuff;
            sim800ReceiveBuff = new String();
            timer = millis();
            return;
          
        }
        
        /*
        // выводим то, что получено, для теста
        DBG(F("SIM800: <<==(c:"));
        DBG(currentCommand);
        DBG(F(", m:"));        
        DBG(machineState);
        DBG(F(", l:"));
        DBG(sim800ReceiveBuff->length());
        DBG(F("): "));
        DBGLN(*sim800ReceiveBuff);
        */

        DBG(F("<<== "));
        DBGLN(*sim800ReceiveBuff);

        if(sim800ReceiveBuff->startsWith(F("+CLIP:")))
        {
          DBGLN(F("SIM800: +CLIP detected, parse!"));
         
          processIncomingCall(*sim800ReceiveBuff);
                            
          delete sim800ReceiveBuff;
          sim800ReceiveBuff = new String();
          
          timer = millis();        
          return; // поскольку мы сами отработали входящий звонок - выходим
        }
        else
        if(sim800ReceiveBuff->startsWith(F("+CMT:")))
          {
            flags.pduInNextLine = true;
            hasAnswerLine = false;
            delete sim800ReceiveBuff;
            sim800ReceiveBuff = new String();            
          }
        
      }

   if(hasAnswerLine && sim800ReceiveBuff->startsWith(F("AT+")))
   {
    // это эхо, игнорируем
      *sim800ReceiveBuff = "";
      hasAnswerLine = false;
   }

    // при разборе ответа тут будет лежать тип ответа, чтобы часто не сравнивать со строкой
    SIM800KnownAnswer knownAnswer = gsmNone;

    if(!flags.onIdleTimer)
    {
    // анализируем состояние конечного автомата, чтобы понять, что делать
    switch(machineState)
    {
        case sim800Idle: // ничего не делаем, можем работать с очередью команд и клиентами
        {            
            // смотрим - если есть хоть одна команда в очереди инициализации - значит, мы в процессе инициализации, иначе - можно работать с очередью клиентов
            if(initCommandsQueue.size())
            {
                DBGLN(F("SIM800: process next init command..."));
                currentCommand = initCommandsQueue[initCommandsQueue.size()-1];
                initCommandsQueue.pop();
                sendCommand(currentCommand);
            } // if
            else
            {
              // в очереди команд инициализации ничего нет, значит, можем выставить флаг, что мы готовы к работе с клиентами
              flags.ready = true;

              if(outgoingSMSList.size())
              {
                sendQueuedSMS();
              }
              else
              if(clientsQueue.size())
              {
                  // получаем первого клиента в очереди
                  SIM800ClientQueueData dt = clientsQueue[0];
                  int clientID = dt.client->getID();
                  
                  // смотрим, чего он хочет от нас
                  switch(dt.action)
                  {
                    case sim800DisconnectAction:
                    {
                      // хочет отсоединиться
                      /*
                      DBG(F("SIM800: client #"));
                      DBG(clientID);
                      DBGLN(F(" wants to disconnect..."));
                      */
                      currentCommand = smaCIPCLOSE;
                      String cmd = F("AT+CIPCLOSE=");
                      cmd += clientID;
                      sendCommand(cmd);                      
                      
                    }
                    break; // sim800DisconnectAction

                    case sim800ConnectAction:
                    {
                      // хочет подсоединиться
                      /*
                      DBG(F("SIM800: client #"));
                      DBG(clientID);
                      DBGLN(F(" want to connect..."));
                      */
                      currentCommand = smaCIPSTART;
                      String comm = F("AT+CIPSTART=");
                      comm += clientID;
                      comm += F(",\"TCP\",\"");
                      comm += dt.ip;
                      comm += F("\",");
                      comm += dt.port;
              
                      // и отсылаем её
                      sendCommand(comm);
                      
                    }
                    break; // sim800ConnectAction

                    case sim800WriteAction:
                    {
                      // хочет отослать данные
                      /*
                      DBG(F("SIM800: client #"));
                      DBG(clientID);
                      DBG(F(" has data="));
                      DBG(dt.client->getDataSize());
                      DBGLN(F(" and wants to send it..."));
                      */
                      currentCommand = smaCIPSEND;

                      String command = F("AT+CIPSEND=");
                      command += clientID;
                      command += F(",");
                      command += dt.client->getDataSize();
                      flags.waitForDataWelcome = true; // выставляем флаг, что мы ждём >
                      
                      sendCommand(command);                      
                    }
                    break; // sim800WriteAction
                  } // switch
              }
              else
              {
                timer = millis(); // обновляем таймер в режиме ожидания, поскольку мы не ждём ответа на команды

                static unsigned long hangTimer = 0;
                if(millis() - hangTimer > 30000)
                {
                  DBGLN(F("SIM800: want to check modem availability..."));
                  hangTimer = millis();
                  sendCommand(smaCheckModemHang);
                  
                } // if
                
              } // else
            } // else inited
        }
        break; // sim800Idle

        case sim800WaitAnswer: // ждём ответа от модема на посланную ранее команду (функция sendCommand переводит конечный автомат в эту ветку)
        {
          // команда, которую послали - лежит в currentCommand, время, когда её послали - лежит в timer.
              if(hasAnswerLine)
              {                
                // есть строка ответа от модема, можем её анализировать, в зависимости от посланной команды (лежит в currentCommand)
                switch(currentCommand)
                {
                  case smaNone:
                  {
                    // ничего не делаем
                    DBGLN(F("SIM800: NO COMMAND!!!!"));
                  }
                  break; // cmdNone

                  case smaWaitSMSSendDone:
                  {
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("SIM800: SMS was sent."));
                      sendCommand(F("AT+CMGD=1,4"));
                      currentCommand = smaWaitForSMSClearance;
                    }                       
                  }
                  break; // smaWaitSMSSendDone

                  case smaWaitForSMSClearance:
                  {
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("SIM800: SMS cleared."));
                      machineState = sim800Idle;
                    }                       
                    
                  }
                  break; // smaWaitForSMSClearance

                  case smaCMGS:
                  {
                    // отсылаем SMS
                    
                          if(*sim800ReceiveBuff == F(">")) 
                          {
                            
                            // дождались приглашения, можно посылать    
                            DBGLN(F("SIM800: Welcome received, continue sending..."));

                            sendCommand(*smsToSend,false);
                            workStream->write(0x1A); // посылаем символ окончания посыла

                            delete smsToSend;
                            smsToSend = new String();

                            currentCommand = smaWaitSMSSendDone;
                          } 
                          else 
                          {
                  
                            DBG(F("SIM800: BAD ANWER TO SMS COMMAND: WANT '>', RECEIVED: "));
                            DBGLN(*sim800ReceiveBuff);

                            delete smsToSend;
                            smsToSend = new String();
                            
                            // пришло не то, что ждали - просто игнорируем отсыл СМС
                             machineState = sim800Idle;
                              
                          }
                  }
                  break; // smaCMGS

                  case smaCIPCLOSE:
                  {
                    // отсоединялись
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      int clientID = sim800ReceiveBuff->toInt();
                      
                      if(clientsQueue.size())
                      {
                        DBGLN(F("SIM800: Client disconnected."));
                        // клиент отсоединён, ставим ему соответствующий флаг, освобождаем его и удаляем из очереди
                       CoreTransportClient* thisClient = getClientFromQueue(clientID,sim800DisconnectAction);
                       if(!thisClient)
                        thisClient = clients[clientID];
                       
                       removeClientFromQueue(thisClient, sim800DisconnectAction);

                        setClientBusy(*thisClient,false);
                        setClientConnected(*thisClient,false,CT_ERROR_NONE);
                        
                      } // if(clientsQueue.size()) 
                      
                        machineState = sim800Idle; // переходим к следующей команде
                    }
                  }
                  break; // smaCIPCLOSE

                  case smaCIPSTART:
                  {
                    // соединялись
                    
                        if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                        {
                          int clientID = sim800ReceiveBuff->toInt();
                                                                                                        
                          if(knownAnswer == gsmConnectOk)
                          {
                            // законнектились удачно
                            if(clientsQueue.size())
                            {
                               DBGLN(F("SIM800: Client connected."));
                               
                               CoreTransportClient* thisClient = getClientFromQueue(clientID,sim800ConnectAction);
                               if(!thisClient)
                                thisClient = clients[clientID];
                               
                               removeClientFromQueue(thisClient, sim800ConnectAction);
                               
                               setClientBusy(*thisClient,false);
                               setClientConnected(*thisClient,true,CT_ERROR_NONE);
                            }
                            machineState = sim800Idle; // переходим к следующей команде
                          } // gsmConnectOk
                          else if(knownAnswer == gsmConnectFail)
                          {
                            // ошибка соединения
                            if(clientsQueue.size())
                            {
                               DBGLN(F("SIM800: Client connect ERROR!"));
                               
                               CoreTransportClient* thisClient = getClientFromQueue(clientID,sim800ConnectAction);
                               if(!thisClient)
                                thisClient = clients[clientID];
                               
                               removeClientFromQueue(thisClient, sim800ConnectAction);
                               
                               setClientBusy(*thisClient,false);
                               setClientConnected(*thisClient,false,CT_ERROR_CANT_CONNECT);
                            }
                            machineState = sim800Idle; // переходим к следующей команде
                          } // gsmConnectFail
                        } // isKnownAnswer                   
                    
                  }
                  break; // cmdCIPSTART


                  case smaWaitSendDone:
                  {
                    // дожидаемся конца отсыла данных от клиента в SIM800
                      
                      if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                      {
                        int clientID = sim800ReceiveBuff->toInt();
                                                
                        if(knownAnswer == gsmSendOk)
                        {
                          // send ok
                          if(clientsQueue.size())
                          {
                             DBGLN(F("SIM800: data was sent."));
                             CoreTransportClient* thisClient = getClientFromQueue(clientID,sim800WriteAction);
                             if(!thisClient)
                              thisClient = clients[clientID];
                             
                             removeClientFromQueue(thisClient, sim800WriteAction);

                             setClientBusy(*thisClient,false);

                             // очищаем данные у клиента
                             setClientData(*thisClient,NULL,0);

                             notifyClientDataWritten(*thisClient,CT_ERROR_NONE);
                          }                     
                        } // send ok
                        else
                        {
                          // send fail
                          if(clientsQueue.size())
                          {
                             DBGLN(F("SIM800: send data fail!"));
                             
                             CoreTransportClient* thisClient = getClientFromQueue(clientID,sim800WriteAction);
                             if(!thisClient)
                              thisClient = clients[clientID];
                             
                             removeClientFromQueue(thisClient, sim800WriteAction);
                             
                             setClientBusy(*thisClient,false);
                             
                             // очищаем данные у клиента
                             setClientData(*thisClient,NULL,0);
                             
                             notifyClientDataWritten(*thisClient,CT_ERROR_CANT_WRITE);
                          }                     
                        } // else send fail
  
                        machineState = sim800Idle; // переходим к следующей команде
                        
                      } // if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                       

                  }
                  break; // smaWaitSendDone

                  case smaCIPSEND:
                  {
                    // тут отсылали запрос на запись данных с клиента
                    if(*sim800ReceiveBuff == F(">"))
                    {
                       // дождались приглашения, можем писать в ESP
                       // тут пишем напрямую
                       if(clientsQueue.size())
                       {
                          // говорим, что ждём окончания отсыла данных
                          currentCommand = smaWaitSendDone;                          
                          SIM800ClientQueueData dt = clientsQueue[0];

                          DBGLN(F("SIM800: > received, start write from client to SIM800..."));
                          
                          workStream->write(dt.client->getData(),dt.client->getDataSize());

                          // очищаем данные у клиента сразу после отсыла
                          setClientData(*(dt.client),NULL,0);
                       }
                    } // if
                    else
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      if(knownAnswer == gsmError || knownAnswer == gsmSendFail)
                      {
                           // всё плохо, не получилось ничего записать
                          if(clientsQueue.size())
                          {
                             DBGLN(F("SIM800: Client write ERROR!"));
                             SIM800ClientQueueData dt = clientsQueue[0];
    
                             CoreTransportClient* thisClient = dt.client;
                             removeClientFromQueue(thisClient,sim800WriteAction);
    
                             setClientBusy(*thisClient,false);
                             
                             // очищаем данные у клиента
                             setClientData(*thisClient,NULL,0);
    
                             notifyClientDataWritten(*thisClient,CT_ERROR_CANT_WRITE);
                            
                          }
                          
                        machineState = sim800Idle; // переходим к следующей команде
                      }                                           
              
                    } // else can't write
                    
                  }
                  break; // smaCIPSEND
                 
                  case smaCheckReady: // ждём готовности модема, ответ на команду AT+CPAS?
                  {
                      if( sim800ReceiveBuff->startsWith( F("+CPAS:") ) ) 
                      {
                          // это ответ на команду AT+CPAS, можем его разбирать
                          if(*sim800ReceiveBuff == F("+CPAS: 0")) 
                          {
                            // модем готов, можем убирать команду из очереди и переходить к следующей
                            DBGLN(F("SIM800: Modem ready."));
                            machineState = sim800Idle; // и переходим на следующую
                        }
                        else 
                        {
                           DBGLN(F("SIM800: Modem NOT ready, try again later..."));
                           idleTime = 2000; // повторим через 2 секунды
                           flags.onIdleTimer = true;
                           // добавляем ещё раз эту команду
                           initCommandsQueue.push_back(smaCheckReady);
                           machineState = sim800Idle; // и пошлём ещё раз команду проверки готовности           
                        }
                      }                    
                  }
                  break; // cmdWantReady


                  case smaHangUp:
                  {
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("SIM800: Call dropped."));
                      machineState = sim800Idle; // переходим к следующей команде
                    }                    
                  }
                  break; // smaHangUp

                  case smaCIPHEAD:
                  {
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("SIM800: CIPHEAD command processed."));
                      machineState = sim800Idle; // переходим к следующей команде
                    }
                  }
                  break; // smaCIPHEAD

                  case smaCIICR:
                  {
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("SIM800: CIICR command processed."));
                      machineState = sim800Idle; // переходим к следующей команде

                      if(knownAnswer == gsmOK)
                      {
                        // тут можем добавлять новые команды для GPRS
                        idleTime = 1000; // обработаем ответ через 1 секунд
                        flags.onIdleTimer = true;                           
                        DBGLN(F("SIM800: start checking GPRS connection..."));
                        initCommandsQueue.push_back(smaCIFSR);
                        gprsCheckingAttempts = 0;
                      }
                    }
                  }
                  break; // smaCIICR

                  case smaCIFSR:
                  {
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      // если мы здесь - мы не получили IP-адреса, т.к. ответ - один из известных
                      
                      if(knownAnswer == gsmOK)
                      {

                        // Тут пробуем чуть позже ещё раз эту команду
                        if(++gprsCheckingAttempts <=5)
                        {
                          DBGLN(F("SIM800: try to get GPRS IP address a little bit later..."));
                          
                          idleTime = 5000; // обработаем ответ через 5 секунд
                          flags.onIdleTimer = true;
                          initCommandsQueue.push_back(smaCIFSR);
                          machineState = sim800Idle; // переходим к следующей команде  
                        }
                        else
                        {
                          DBGLN(F("SIM800: Unable to get GPRS IP address!"));
                          // всё, исчерпали лимит на попытки получить IP-адрес
                          machineState = sim800Idle; // переходим к следующей команде
                          flags.gprsAvailable = false;
                        }
                      }
                      else
                      {
                        DBGLN(F("SIM800: GPRS connection fail!"));
                        flags.gprsAvailable = false;
                        machineState = sim800Idle; // переходим к следующей команде
                      }
                    } // isKnownAnswer
                    else
                    if(sim800ReceiveBuff->length() && sim800ReceiveBuff->indexOf(".") != -1)
                    {
                      DBG(F("SIM800: GPRS IP address found - "));
                      DBGLN(*sim800ReceiveBuff);
                      flags.gprsAvailable = true;
                      machineState = sim800Idle; // переходим к следующей команде          
                    }
                  }
                  break; // smaCIFSR

                  case smaCSTT:
                  {
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("SIM800: CSTT command processed."));
                      machineState = sim800Idle; // переходим к следующей команде

                      if(knownAnswer == gsmOK)
                      {
                        // тут можем добавлять новые команды для GPRS
                        DBGLN(F("SIM800: start GPRS connection..."));
                        initCommandsQueue.push_back(smaCIICR);
                      }
                      else
                      {
                        DBGLN(F("SIM800: Can't start GPRS connection!"));
                      }
                    }                    
                  }
                  break; // smaCSTT

                  case smaCIPMODE:
                  {
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("SIM800: CIPMODE command processed."));
                      machineState = sim800Idle; // переходим к следующей команде
                    }
                  }
                  break; // smaCIPMODE
                  
                  case smaCIPMUX:
                  {
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("SIM800: CIPMUX command processed."));
                      machineState = sim800Idle; // переходим к следующей команде
                    }
                  }
                  break; // smaCIPMUX


                  case smaEchoOff:
                  {
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      if(gsmOK == knownAnswer)
                      {
                        DBGLN(F("SIM800: Echo OFF command processed."));
                      }
                      else
                      {
                        DBGLN(F("SIM800: Echo OFF command FAIL!"));
                      }
                      machineState = sim800Idle; // переходим к следующей команде
                    }
                  }
                  break; // smaEchoOff

                  case smaDisableCellBroadcastMessages:
                  {                    
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      if(gsmOK == knownAnswer)
                      {
                        DBGLN(F("SIM800: Broadcast SMS disabled."));
                      }
                      else
                      {
                        DBGLN(F("SIM800: Broadcast SMS command FAIL!"));
                      }
                      machineState = sim800Idle; // переходим к следующей команде
                    }
                  }
                  break; // smaDisableCellBroadcastMessages

                  case smaAON:
                  {                    
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      if(gsmOK == knownAnswer)
                      {
                        DBGLN(F("SIM800: AON is ON."));
                      }
                      else
                      {
                        DBGLN(F("SIM800: AON command FAIL!"));
                      }
                      machineState = sim800Idle; // переходим к следующей команде
                    }
                  }
                  break; // smaAON

                  case smaPDUEncoding:
                  {
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      if(gsmOK == knownAnswer)
                      {
                        DBGLN(F("SIM800: PDU format is set."));
                      }
                      else
                      {
                        DBGLN(F("SIM800: PDU format command FAIL!"));
                      }
                      machineState = sim800Idle; // переходим к следующей команде
                    }                    
                  }
                  break; // smaPDUEncoding

                  case smaUCS2Encoding:
                  {
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      if(gsmOK == knownAnswer)
                      {
                        DBGLN(F("SIM800: UCS2 encoding is set."));
                      }
                      else
                      {
                        DBGLN(F("SIM800: UCS2 encoding command FAIL!"));
                      }
                      machineState = sim800Idle; // переходим к следующей команде
                    }                                        
                  }
                  break; // smaUCS2Encoding

                  case smaSMSSettings:
                  {
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      if(gsmOK == knownAnswer)
                      {
                        DBGLN(F("SIM800: SMS settings is set."));
                      }
                      else
                      {
                        DBGLN(F("SIM800: SMS settings command FAIL!"));
                      }
                      machineState = sim800Idle; // переходим к следующей команде
                    }                                        
                  }
                  break; // smaSMSSettings

                  case smaWaitReg:
                  {
                     if(sim800ReceiveBuff->indexOf(F("+CREG: 0,1")) != -1)
                        {
                          // зарегистрированы в GSM-сети
                             flags.isModuleRegistered = true;
                             DBGLN(F("SIM800: Modem registered in GSM!"));
                             machineState = sim800Idle;
                        } // if
                        else
                        {
                          // ещё не зарегистрированы
                            flags.isModuleRegistered = false;
                            idleTime = 5000; // повторим через 5 секунд
                            flags.onIdleTimer = true;
                            // добавляем ещё раз эту команду
                            initCommandsQueue.push_back(smaWaitReg);
                            machineState = sim800Idle;
                        } // else                    
                  }
                  break; // smaWaitReg
                  case smaCheckModemHang:
                  {                    
                    if(isKnownAnswer(*sim800ReceiveBuff,knownAnswer))
                    {
                      DBGLN(F("SIM800: modem answered and available."));
                      machineState = sim800Idle; // переходим к следующей команде
                      
                    } // if(isKnownAnswer

                  }
                  break; // smaCheckModemHang
                                    
                } // switch

                
              } // if(hasAnswerLine)
              
         
        }
        break; // sim800WaitAnswer

        case sim800Reboot:
        {
          // ждём перезагрузки модема
          unsigned long powerOffTime = SIM800TransportSettings.HangPowerOffTime;
          powerOffTime *= 1000;

          if(!SIM800TransportSettings.UseRebootPin)
            powerOffTime = 0;
          
          if(millis() - timer > powerOffTime)
          {
            if(SIM800TransportSettings.UseRebootPin)
            {
              DBGLN(F("SIM800: turn power ON!"));
              digitalWrite(SIM800TransportSettings.RebootPin,SIM800TransportSettings.PowerOnLevel);
            }

            if(SIM800TransportSettings.UsePowerKey)
            {
                digitalWrite(SIM800TransportSettings.PowerKeyPin,!SIM800TransportSettings.PowerKeyOnLevel);
            }

            machineState = sim800WaitInit;
            timer = millis();
            
          } // if
        }
        break; // smaReboot

        case sim800WaitInit:
        {
          unsigned long waitTime = SIM800TransportSettings.PowerKeyInitTime;

          if(!SIM800TransportSettings.UsePowerKey)
            waitTime = 0;
            
          if(millis() - timer > waitTime)
          { 
            DBGLN(F("SIM800: Power ON completed!"));
            
              if(SIM800TransportSettings.UsePowerKey)
              {
                DBGLN(F("SIM800: use POWERKEY!"));
                                
                digitalWrite(SIM800TransportSettings.PowerKeyPin,SIM800TransportSettings.PowerKeyOnLevel);
                delay(SIM800TransportSettings.PowerKeyPulseDuration);        
                digitalWrite(SIM800TransportSettings.PowerKeyPin,!SIM800TransportSettings.PowerKeyOnLevel);
            
                idleTime = SIM800TransportSettings.HangPowerOffTime; // подождём чуть-чуть...
                idleTime *= 1000;
                flags.onIdleTimer = true;
                
              }              

            // теперь ждём загрузки модема
           
            machineState = sim800WaitBootBegin;
            timer = millis();
                
            DBGLN(F("SIM800: inited after reboot!"));
          } // 
        }
        break; // sim800WaitInit

        case sim800WaitBootBegin:
        {
          sendCommand("AT");
          machineState = sim800WaitBoot;          
        }
        break; // sim800WaitBootBegin

        case sim800WaitBoot:
        {
          if(hasAnswerLine)
          {
            if(*sim800ReceiveBuff == F("Call Ready") || *sim800ReceiveBuff == F("SMS Ready"))
            {
              DBGLN(F("SIM800: BOOT FOUND, INIT!"));
              restart();
            }
          }
             
        }
        break; // sim800WaitBoot
      
      } // switch

    } // if(!flags.onIdleTimer)


    // тут просто анализируем ответ от SIM800, если он есть, на предмет того - соединён ли клиент, отсоединён ли клиент и т.п.
    if(hasAnswerLine)
    {
      
       // смотрим, подсоединился ли клиент?
       int idx = sim800ReceiveBuff->indexOf(F(",CONNECT OK"));
       if(idx != -1)
       {
          // клиент подсоединился
          String s = sim800ReceiveBuff->substring(0,idx);
          int clientID = s.toInt();
          if(clientID >=0 && clientID < SIM800_MAX_CLIENTS)
          {
            DBG(F("SIM800: client connected - #"));
            DBGLN(clientID);

            // выставляем клиенту флаг, что он подсоединён
            CoreTransportClient* client = clients[clientID];
            
            setClientBusy(*client,false);
            setClientConnected(*client,true,CT_ERROR_NONE);
          }
       } // if

       idx = sim800ReceiveBuff->indexOf(F(",CLOSED"));
       bool isConnectFail = sim800ReceiveBuff->endsWith(F("CONNECT FAIL"));
       if(idx != -1 || isConnectFail)
       {
        // клиент отсоединился
          String s = sim800ReceiveBuff->substring(0,idx);
          int clientID = s.toInt();
          if(clientID >=0 && clientID < SIM800_MAX_CLIENTS)
          {
            DBG(F("SIM800: client disconnected - #"));
            DBGLN(clientID);

            // выставляем клиенту флаг, что он отсоединён
            CoreTransportClient* client = clients[clientID];
            
            setClientBusy(*client,false);
            setClientConnected(*client,false,CT_ERROR_NONE);
          }        
        
       } // if(idx != -1)
       

      // не забываем чистить за собой
        delete sim800ReceiveBuff;
        sim800ReceiveBuff = new String();   
           
    } // if(hasAnswerLine)

    if(!hasAnswer) // проверяем на зависание
    {

      // нет ответа от ESP, проверяем, зависла ли она?
      unsigned long hangTime = SIM800TransportSettings.HangTimeout;
      hangTime *= 1000;

      if(millis() - timer > hangTime)
      {
        DBGLN(F("SIM800: modem not answering, reboot!"));

        if(SIM800TransportSettings.UseRebootPin)
        {
          // есть пин, который надо использовать при зависании
          digitalWrite(SIM800TransportSettings.RebootPin,!SIM800TransportSettings.PowerOnLevel);
        }

        machineState = sim800Reboot;
        timer = millis();
        
      } // if   
         
    } // if(!hasAnswer) 
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::begin()
{
  workStream = NULL;

  if(!SIM800TransportSettings.enabled)
    return;
  
  if(SIM800TransportSettings.SerialNumber == 0 || SIM800TransportSettings.UARTSpeed == 0) // не можем работать через Serial или с нулевой скоростью!
    return;
  
  initClients();

  HardwareSerial* hs = NULL;

  #if (TARGET_BOARD == MEGA_BOARD) || (TARGET_BOARD == DUE_BOARD)
  
  switch(SIM800TransportSettings.SerialNumber)
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
  #elif TARGET_BOARD == ESP_BOARD
    #error "NOT IMPLEMENTED!!!"
  #else
    #error "Unknown target board!"
  #endif    


  workStream = hs;
  unsigned long uspeed = SIM800TransportSettings.UARTSpeed;
  uspeed *= 9600;
  
  hs->begin(uspeed);

  restart();

  if(SIM800TransportSettings.UseRebootPin)
  {
    DBGLN(F("SIM800: power OFF!"));
    // есть пин, который надо использовать при зависании
    pinMode(SIM800TransportSettings.RebootPin,OUTPUT);
    digitalWrite(SIM800TransportSettings.RebootPin,!SIM800TransportSettings.PowerOnLevel);
  }

  if(SIM800TransportSettings.UsePowerKey)
  {      
      pinMode(SIM800TransportSettings.PowerKeyPin,OUTPUT);
  }

  machineState = sim800Reboot;

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSIM800Transport::sendSMS(const String& phoneNumber, const String& message, bool isFlash)
{
  if(!ready())
    return false;
    
    SIM800OutgoingSMS queuedSMS;
    
    queuedSMS.isFlash = isFlash;   
    queuedSMS.phone = new String(phoneNumber.c_str());    
    queuedSMS.message = new String(message.c_str());    

    outgoingSMSList.push_back(queuedSMS);

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::restart()
{
  delete sim800ReceiveBuff;
  sim800ReceiveBuff = new String();

  delete smsToSend;
  smsToSend = new String();

  // очищаем очередь клиентов, заодно им рассылаем события
  clearClientsQueue(true);  

  // т.к. мы ничего не инициализировали - говорим, что мы не готовы предоставлять клиентов
  flags.ready = false;
  flags.isAnyAnswerReceived = false;
  flags.waitForDataWelcome = false;
  flags.onIdleTimer = false;
  flags.isModuleRegistered = false;
  flags.gprsAvailable = false;
  
  timer = millis();

  currentCommand = smaNone;
  machineState = sim800Idle;

  // инициализируем очередь командами по умолчанию
 createInitCommands(true);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::createInitCommands(bool addResetCommand)
{  
  // очищаем очередь команд
  clearInitCommands();

  // если указаны параметры APN - при старте поднимаем GPRS
  if(SIM800TransportSettings.APN.length())
  {
    initCommandsQueue.push_back(smaCSTT);
  }
  
  initCommandsQueue.push_back(smaCIPMUX);
  initCommandsQueue.push_back(smaCIPMODE);
  initCommandsQueue.push_back(smaWaitReg); // ждём регистрации
  
  initCommandsQueue.push_back(smaCIPHEAD);
  initCommandsQueue.push_back(smaSMSSettings); // настройки вывода SMS
  initCommandsQueue.push_back(smaUCS2Encoding); // кодировка сообщений
  initCommandsQueue.push_back(smaPDUEncoding); // формат сообщений
  initCommandsQueue.push_back(smaAON); // включение АОН
  initCommandsQueue.push_back(smaDisableCellBroadcastMessages); // выключение броадкастовых SMS
  initCommandsQueue.push_back(smaCheckReady); // проверка готовности    
  initCommandsQueue.push_back(smaEchoOff); // выключение эха
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::clearInitCommands()
{
  initCommandsQueue.empty();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::clearClientsQueue(bool raiseEvents)
{
  
  // тут попросили освободить очередь клиентов.
  // для этого нам надо выставить каждому клиенту флаг того, что он свободен,
  // плюс - сообщить, что текущее действие над ним не удалось.  

    for(size_t i=0;i<clientsQueue.size();i++)
    {
        SIM800ClientQueueData dt = clientsQueue[i];
        if(raiseEvents)
        {
          switch(dt.action)
          {

            case sim800DisconnectAction:
                // при дисконнекте всегда считаем, что ошибок нет
                setClientConnected(*(dt.client),false,CT_ERROR_NONE); 
            break;
  
            case sim800ConnectAction:
                // если было запрошено соединение клиента с адресом - говорим, что соединиться не можем
                setClientConnected(*(dt.client),false,CT_ERROR_CANT_CONNECT);
            break;
  
            case sim800WriteAction:
              // если попросили записать данные - надо сообщить подписчикам, что не можем записать данные
              notifyClientDataWritten(*(dt.client),CT_ERROR_CANT_WRITE);
            break;
          } // switch
          
        } // if(raiseEvents)

        // освобождаем клиента
        setClientBusy(*(dt.client),false);
    } // for

  clientsQueue.empty();

  // помимо этого - надо принудительно всем клиентам выставить статус "Отсоединён"
   for(int i=0;i<SIM800_MAX_CLIENTS;i++)
  {
    setClientBusy(*(clients[i]),false);
    setClientConnected(*(clients[i]),false,CT_ERROR_NONE); 
  }  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransportClient* CoreSIM800Transport::getClientFromQueue(int clientID, SIM800ClientAction action)
{
   for(size_t i=0;i<clientsQueue.size();i++)
  {
    if(clientsQueue[i].client->getID() == clientID && clientsQueue[i].action == action)
      return clientsQueue[i].client;
  } 

  return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSIM800Transport::isClientInQueue(CoreTransportClient* client, SIM800ClientAction action)
{
  for(size_t i=0;i<clientsQueue.size();i++)
  {
    if(clientsQueue[i].client == client && clientsQueue[i].action == action)
      return true;
  }

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::addClientToQueue(CoreTransportClient* client, SIM800ClientAction action, const char* ip, uint16_t port)
{
  if(isClientInQueue(client, action))
  {
    DBGLN(F("SIM800: Client already in queue!"));
    return;
  }

    SIM800ClientQueueData dt;
    dt.client = client;
    dt.action = action;
    
    dt.ip = NULL;
    if(ip)
    {
      dt.ip = new char[strlen(ip)+1];
      strcpy(dt.ip,ip);
    }
    dt.port = port;

    clientsQueue.push_back(dt);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::removeClientFromQueue(CoreTransportClient* client, SIM800ClientAction action)
{
  for(size_t i=0;i<clientsQueue.size();i++)
  {
    if(clientsQueue[i].client == client && clientsQueue[i].action == action)
    {
      delete [] clientsQueue[i].ip;
      
        for(size_t j=i+1;j<clientsQueue.size();j++)
        {
          clientsQueue[j-1] = clientsQueue[j];
        }
        
        clientsQueue.pop();
        break;
    } // if
    
  } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::initClients()
{
  for(int i=0;i<SIM800_MAX_CLIENTS;i++)
  {
    if(clients[i])
      continue;
      
    CoreTransportClient* client = CoreTransportClient::Create(this);
    setClientID(*client,i);
    clients[i] = client;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::subscribe(IClientEventsSubscriber* subscriber)
{
  initClients();
  for(int i=0;i<SIM800_MAX_CLIENTS;i++)
  {
    subscribeClient(*(clients[i]),subscriber);
  }  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::unsubscribe(IClientEventsSubscriber* subscriber)
{
  initClients();
  for(int i=0;i<SIM800_MAX_CLIENTS;i++)
  {
    unsubscribeClient(*(clients[i]),subscriber);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransportClient* CoreSIM800Transport::getFreeClient()
{
  if(!ready() || !flags.gprsAvailable) // если ещё не готовы к работе - ничего не возвращаем
    return NULL;
  
  for(int i=0;i<SIM800_MAX_CLIENTS;i++)
  {
    if(!clients[i]->connected() && !clients[i]->busy()) // возвращаем первого неподсоединённого и незанятого чем-либо клиента
      return clients[i];
  }

  return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::beginWrite(CoreTransportClient& client)
{
  if(!client.connected())
  {
    DBGLN(F("SIM800: client not connected!"));
    return;
  }

  //говорим, что клиент занят
  setClientBusy(client,true);
  
  // добавляем клиента в очередь на запись
  addClientToQueue(&client, sim800WriteAction);


  DBG(F("SIM800: Client #"));
  DBG(client.getID());
  DBG(F(" has data size: "));
  DBGLN(client.getDataSize());

  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с записью в поток с этого клиента
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::beginConnect(CoreTransportClient& client, const char* ip, uint16_t port)
{
  if(client.connected())
  {
    DBGLN(F("SIM800: client already connected!"));
    return;
  }

  //говорим, что клиент занят
  setClientBusy(client,true);

  // добавляем клиента в очередь на соединение
  addClientToQueue(&client, sim800ConnectAction, ip, port);

  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с соединением клиента

  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::beginDisconnect(CoreTransportClient& client)
{
  if(!client.connected())
  {
//    DBGLN(F("SIM800: client not connected!"));
    return;
  }

  //говорим, что клиент занят
  setClientBusy(client,true);

  // добавляем клиента в очередь на соединение
  addClientToQueue(&client, sim800DisconnectAction);

  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с отсоединением клиента
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSIM800Transport::ready()
{
  return flags.ready && flags.isAnyAnswerReceived && flags.isModuleRegistered; // если мы полностью инициализировали SIM800 - значит, можем работать
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_SIM800_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_THINGSPEAK_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreThingSpeakSettings ThingSpeakSettings;
//--------------------------------------------------------------------------------------------------------------------------------------
CoreThingSpeak ThingSpeak;
//--------------------------------------------------------------------------------------------------------------------------------------
CoreThingSpeak::CoreThingSpeak()
{
  currentClient = NULL;
  currentTransport = NULL;
  timer = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreThingSpeak::begin()
{
  DBGLN(F("TS: begin."));
  
  // попросили начать работу
  currentClient = NULL;
  currentTransport = NULL;
  initSubstitutions();

    switch(ThingSpeakSettings.workMode)
    {
      case workModeDisabled:
      break;

      case workModeThroughESP:
        #ifdef CORE_ESP_TRANSPORT_ENABLED
          currentTransport = &ESP;
        #endif
      break;

      case workModeThroughSIM800:
      #ifdef CORE_SIM800_TRANSPORT_ENABLED
          currentTransport = &SIM800;
      #endif
      break;
    }

  // подписываемся на события клиентов
  if(currentTransport)
  {
    currentTransport->subscribe(this);  
  }
    
  // ну и запомним, когда вызвали начало работы
  timer = millis();
  machineState = tsWaitingInterval;
  onIdleTimer = false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreThingSpeak::reset()
{
  DBGLN(F("TS: reset."));
  
  currentClient = NULL;
  currentTransport = NULL;

  initSubstitutions();

  machineState = tsWaitingInterval;
  onIdleTimer = false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreThingSpeak::update()
{
  if(ThingSpeakSettings.workMode == workModeDisabled || !currentTransport || !ThingSpeakSettings.sensors.size() || !ThingSpeakSettings.apiKey.length())
    return;

  if(onIdleTimer)
  {
    if(millis() - timer > idleInterval)
    {
      onIdleTimer = false;
    }
  }

  if(onIdleTimer)
    return;

    switch(machineState)
    {
      case tsWaitingInterval:
      {
        // ждём момента начала отсыла данных
          unsigned long now = millis();

          // интервал обновления у нас - в секундах
          unsigned long needed = ThingSpeakSettings.updateInterval;
          needed *= 1000;
        
          if(now - timer > needed)
          {
              DBGLN(F("TS: WANT TO SEND DATA TO ThingSpeak!"));

              if(!currentTransport->ready())
              {
                // транспорт не готов работать, надо подождать какое-то время, прежде чем попытаться опять постучаться в транспорт
                DBGLN(F("TS: Transport not ready, try again later..."));
                
                onIdleTimer = true;
                idleInterval = 5000;
                timer = millis();

                machineState = tsWaitingTransport;
                
              } // if
              else
              {
                // транспорт готов к работе, можем начинать
                machineState = tsCatchClient;
              } // else
                    
          } // if        
      }
      break; // tsWaitingInterval

      case tsWaitingTransport:
      {
        // тут ещё раз проверяем - готов ли транспорт?
        if(currentTransport->ready())
        {
          DBGLN(F("TS: Transport ready, start catching client..."));
          machineState = tsCatchClient;
        }
        else
        {
          // не готов ещё транспорт
          onIdleTimer = true;
          idleInterval = 5000;
          timer = millis();
        }
      }
      break; // tsWaitingTransport

      case tsCatchClient:
      {
        // пытаемся подловить свободного клиента у транспорта
        currentClient = currentTransport->getFreeClient();
        if(currentClient)
        {
          DBGLN(F("TS: Free client acquired, connecting to ThingSpeak..."));
          machineState = tsStartConnect;
        }
        else
        {
          DBGLN(F("TS: No free client, try again later..."));
          onIdleTimer = true;
          idleInterval = 5000;
          timer = millis();
          
        }
      }
      break; // tsCatchClient

      case tsStartConnect:
      {
        // начинаем коннектиться к ThingSpeak
        currentClient->connect("api.thingspeak.com",80);
        machineState = tsConnectMode;
        timer = millis();
        
      }
      break; // tsStartConnect

      
      case tsDisconnectMode:
      case tsConnectMode:
      case tsWriteMode:
      case tsReadMode:
      {
        // в этих режимах проверяем - возможно, мы ооочень долго пишем/читаем/коннектимся/дисконнектимся
        unsigned long needed = 60000;
        if(ThingSpeakSettings.workMode == workModeThroughSIM800)
          needed = 80000; // максимальное время ответа от SIM800 - 75 секунд

        if(millis() - timer > needed)
        {
          // Тут ждали больше, чем надо!!!
            DBGLN(F("TS: TOO LONG WAITING DURING READ/WRITE/CONNECT/DISCONNECT OPERATIONS!"));
          
            timer = millis();
            
            if(currentClient && currentClient->connected())
            {
              machineState = tsDisconnectMode;
              currentClient->disconnect();
            }
            else
            {
              machineState = tsWaitingInterval;
            }
        }
      }
      break;
      
    } // switch


}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreThingSpeak::OnClientConnect(CoreTransportClient& client, bool connected, int errorCode)
{
  if(!currentClient || currentClient != &client) // не наш клиент
    return;

  if(errorCode != CT_ERROR_NONE)
  {
      // ошибка соединения - по-любому переходим в режим ожидания интервала
      DBGLN(F("TS: OnClientConnect - ERROR detected!"));
      currentClient = NULL;
      machineState = tsWaitingInterval;
      timer = millis();

      return;
  }

  if(machineState == tsDisconnectMode)
  {
    // отсоединялись
    if(!connected)
    {
      // отсоединились
      DBGLN(F("TS: work cicle done, switch to wait..."));
      currentClient = NULL;
      machineState = tsWaitingInterval;
      timer = millis();
    }
  }
  else if(machineState == tsConnectMode)
  {
    if(!connected)
    {
      // error!
      DBGLN(F("TS: unable to connect to ThingSpeak!"));
      currentClient = NULL;
      machineState = tsWaitingInterval;
      timer = millis();
    }
    else
    {
      // connected
      DBGLN(F("TS: connected to ThingSpeak, start sending data..."));
      sendData();
    }
  }
  else if(machineState == tsReadMode || machineState == tsWriteMode)
  {
    if(!connected)
    {
      // error!
      DBGLN(F("TS: disconnected during read/write mode!"));
      currentClient = NULL;
      machineState = tsWaitingInterval;
      timer = millis();
    }    
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreThingSpeak::publish(int fieldNumber,const String& data)
{
  if(fieldNumber < 1)
    fieldNumber = 1;

  if(fieldNumber > 8)
    fieldNumber = 8;

   substitutions[fieldNumber-1].active = true;
   substitutions[fieldNumber-1].data = data;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreThingSpeak::sendData()
{
  DBGLN(F("TS: collect data for ThingSpeak..."));
  // переключаемся в ветку отсыла данных
  machineState = tsWriteMode;

   // собираем данные с датчиков
   int currentFieldNumber = 1;

   CoreTextFormatProvider textFormatter;
   String httpQuery = F("GET /update?headers=false&api_key=");
   httpQuery += ThingSpeakSettings.apiKey;

   for(size_t i=0;i<ThingSpeakSettings.sensors.size();i++)
   {
      String* sensorName = ThingSpeakSettings.sensors[i];
      CoreStoredData dataStored = CoreDataStore.get(*sensorName);

      if(dataStored.hasData())
      {
            // есть данные с датчика, можно их отформатировать в строку
            // используем композитный форматтер, чтобы отделить температуру от влажности в сборных датчиках
            Vector<String*> formattedData = textFormatter.formatComposite(dataStored,0,false);
            
            for(size_t k=0;k<formattedData.size();k++,currentFieldNumber++)
            {
                httpQuery += F("&field");
                httpQuery += currentFieldNumber;
                httpQuery += '=';
    
                String compositeData = *(formattedData[k]);
    
                if(substitutions[currentFieldNumber-1].active)
                {
                  compositeData = substitutions[currentFieldNumber-1].data;
                  substitutions[currentFieldNumber-1].active = false;
                }
        
                // ThingSpeak просит float с точкой, поэтому заменяем запятую на точку
                compositeData.replace(',','.');
                httpQuery += encodeURI(compositeData);
    
                if(currentFieldNumber > 8)
                  break;
            } // for

            // не забываем чистить память
            for(size_t k=0;k<formattedData.size();k++)
            {
              delete formattedData[k];
            }
        
      } // if(dataStored.hasData())
      else
        currentFieldNumber++;

        if(currentFieldNumber > 8)
          break;
    
   } // for

    if(currentFieldNumber <=8)
    {
      // ещё остались свободные слоты
      for(int i=currentFieldNumber;i<=8;i++)
      {
        if(substitutions[i-1].active)
        {
          substitutions[i-1].active = false;
          String compositeData = substitutions[i-1].data;

          httpQuery += F("&field");
          httpQuery += i;
          httpQuery += '=';
          
          compositeData.replace(',','.');
          httpQuery += encodeURI(compositeData);
        }
      }
    }

   // тут собрали все данные для ThingSpeak, теперь можно формировать запрос
   httpQuery += F(" HTTP/1.1\r\nHost: api.thingspeak.com\r\n\r\n");

   // запрос сформирован, отсылаем его в клиент
   currentClient->write((uint8_t*)httpQuery.c_str(),httpQuery.length());
   timer = millis();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreThingSpeak::initSubstitutions()
{
  for(int i=0;i<8;i++)
  {
    substitutions[i].active = false;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreThingSpeak::OnClientDataWritten(CoreTransportClient& client, int errorCode)
{
  if(!currentClient || currentClient != &client)
    return;

  // писали данные в клиента - проверяем
  if(errorCode != CT_ERROR_NONE)
  {
      // ошибка записи в клиент
      DBGLN(F("TS: CLIENT WRITE ERROR, disconnecting..."));
      machineState = tsDisconnectMode;
      timer = millis();
      currentClient->disconnect();
      return;
  }
  DBGLN(F("TS: Client data written, wait for results..."));
  machineState = tsReadMode;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreThingSpeak::OnClientDataAvailable(CoreTransportClient& client, bool isDone)
{
  if(!currentClient || currentClient != &client)
    return;

  if(isDone)
  {
    DBGLN(F("TS: all data received, disconnect client..."));
    machineState = tsDisconnectMode;
    timer = millis();
    currentClient->disconnect();
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
String CoreThingSpeak::encodeURI(const String& uri)
{
  String result;
  
  typedef struct
  {
    char fromChar;
    uint8_t toCode;
    
  } URIReplace;

  static URIReplace replacements[] = 
  {
    {'$',0x24},
    {'&',0x26},
    {'+',0x2B},
    {',',0x2C},
    {'/',0x2F},
    {':',0x3A},
    {';',0x3B},
    {'=',0x3D},
    {'?',0x3F},
    {'@',0x40},
    {' ',0x20},
    {'"',0x22},
    {'<',0x3C},
    {'>',0x3E},
    {'#',0x23},
    {'%',0x25},
    {'{',0x7B},
    {'}',0x7D},
    {'|',0x7C},
    {'\\',0x5C},
    {'^',0x5E},
    {'~',0x7E},
    {'[',0x5B},
    {']',0x5D},
    {'`',0x60},
    {0,0}
  };

    for(size_t i=0;i<uri.length();i++)
    {
      char ch = uri[i];
      bool found = false;
      int counter = 0;
      while(replacements[counter].fromChar != 0)
      {
        if(ch == replacements[counter].fromChar)
        {
          result += '%';
          result += String(replacements[counter].toCode,16);
          
          found = true;
          break;
        }
        counter++;
      } // while
      
      if(!found)
        result += ch;
      
    } // for
  

//  DBG(F("TS: encoded data="));
//  DBGLN(result);

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_THINGSPEAK_TRANSPORT_ENABLED
