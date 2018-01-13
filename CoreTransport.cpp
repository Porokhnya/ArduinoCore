#include "CoreTransport.h"
#include "Core.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_RS485_TRANSPORT_ENABLED
CoreRS485Settings RS485Settings;
CoreRS485 RS485;
//--------------------------------------------------------------------------------------------------------------------------------------
extern "C" {
static void __nors485(byte packetID, byte dataLen, byte* data){}
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_RS485_RECEIVE(byte packetID, byte dataLen, byte* data) __attribute__ ((weak, alias("__nors485")));
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_RS485_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_ESP_TRANSPORT_ENABLED
ESPTransportSettingsClass ESPTransportSettings;
#endif // CORE_ESP_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransport::CoreTransport(Stream* stream) : pStream(stream)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransport::~CoreTransport()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::init(CoreTransportEvents ev)
{
  events = ev;
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


