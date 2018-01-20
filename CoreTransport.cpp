#include "CoreTransport.h"
#include "Core.h"
//--------------------------------------------------------------------------------------------------------------------------------------
extern "C" {
static void __nors485(byte packetID, byte dataLen, byte* data){}
static void __nolora(byte* b, int dummy){}
static void __noclientconnect(CoreTransportClient& client) {}
static void __noclientdatareceived(CoreTransportClient& client) {}
static void __noclientwritedone(CoreTransportClient& client, bool isWriteSucceeded) {}
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_LORA_RECEIVE(byte*, int) __attribute__ ((weak, alias("__nolora")));
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_RS485_RECEIVE(byte packetID, byte dataLen, byte* data) __attribute__ ((weak, alias("__nors485")));
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
  workStream = NULL;
  
  dataBuffer = NULL;
  dataBufferLen = 0;
  writeIterator = 0;
  
  machineState = rs485WaitingHeader;
}
//--------------------------------------------------------------------------------------------------------------------------------------
HardwareSerial* CoreRS485::getMyStream(byte SerialNumber)
{
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
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::begin()
{
  workStream = getMyStream(RS485Settings.SerialNumber);
  
  if(RS485Settings.UARTSpeed > 0)
    workStream->begin(RS485Settings.UARTSpeed*9600);

  if(RS485Settings.DEPin > 0)
    pinMode(RS485Settings.DEPin,OUTPUT);

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
void CoreRS485::update()
{
  if(!dataBuffer || !workStream) // нет буфера для данных или неизвестный Serial
    return;

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
  
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreRS485::clear()
{
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
}
//--------------------------------------------------------------------------------------------------------------------------------------
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

  workStream = hs;
  hs->begin(9600*ESPTransportSettings.UARTSpeed);

  //TODO: Start job!!!
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


