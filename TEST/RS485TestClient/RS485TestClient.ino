#include "Config.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#define MY_DEVICE_ID  3 // номер нашего устройства
#define CLUSTER_ID 2 // номер кластера
#define RS485_DE_PIN 5 // пин направления приёма-передачи RS485
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransportPacket rs485Packet;
byte* rsPacketPtr = (byte*)&rs485Packet;
byte rs485WritePtr = 0;
//--------------------------------------------------------------------------------------------------------------------------------------
void processRS485Packet()
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
         byte crc = crc8((byte*)&rs485Packet,sizeof(CoreTransportPacket)- 1);
      
         if(crc != rs485Packet.crc)
         {
          String str = F("BAD CRC!");
          SendDataToRS485((byte*)str.c_str(),str.length());
          rs485WritePtr = 0;
          return;
         }
      
        if(!(rs485Packet.header1 == CORE_HEADER1 && rs485Packet.header2 == CORE_HEADER2 && rs485Packet.header3 == CORE_HEADER3))
        {
          String str = F("BAD HEADERS!");
          SendDataToRS485((byte*)str.c_str(),str.length());
          rs485WritePtr = 0;
          return;
        }
      
        if(rs485Packet.clusterID != CLUSTER_ID)
        {
          String str = F("BAD CLUSTER!");
          SendDataToRS485((byte*)str.c_str(),str.length());
          rs485WritePtr = 0;
          return;
        }
      
      
        if(!(rs485Packet.packetType == CoreDataRequest || rs485Packet.packetType == CoreSensorData))
        {
          String str = F("BAD PACKET TYPE!");
          SendDataToRS485((byte*)str.c_str(),str.length());
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
            
            if(drp->toDeviceID != MY_DEVICE_ID) // пакет не нам
            {
              String str = F("NOT FOT MY DEVICE!");
              SendDataToRS485((byte*)str.c_str(),str.length());
              rs485WritePtr = 0;
              return;
            }
      
            // говорим, что у нас два датчика
            drp->dataCount = 2;
      
            rs485Packet.deviceID = MY_DEVICE_ID;
      
            // пересчитываем CRC
            rs485Packet.crc = crc8((byte*)&rs485Packet,sizeof(CoreTransportPacket)- 1);
      
            // пишем в поток
            SendDataToRS485((byte*)&rs485Packet,sizeof(CoreTransportPacket));
            
          }
          break; // CoreDataRequest
      
          case CoreSensorData:
          {
             // попросили отдать данные с датчика, в первом байте данных пакета - лежит номер датчика, в нулевом байте - номер устройства на шине
             byte requestID = rs485Packet.packetData[0];
             byte sensorNumber = rs485Packet.packetData[1];
      
             if(requestID != MY_DEVICE_ID) // пакет не нам
             {
              String str = F("NOT FOT MY DEVICE!");
              SendDataToRS485((byte*)str.c_str(),str.length());
              rs485WritePtr = 0;
              return;
             }
      
              rs485Packet.deviceID = MY_DEVICE_ID;
      
              CoreSensorDataPacket* sdp = (CoreSensorDataPacket*) rs485Packet.packetData;
              memset(sdp->sensorName,0,sizeof(sdp->sensorName));
      
              switch(sensorNumber)
              {
                case 0:
                {
                  strcpy(sdp->sensorName, "Remote1");
                  sdp->dataType = Temperature;
                  sdp->dataLen = 2;
                  sdp->data[0] = 12;
                  sdp->data[1] = 25;
                }
                break;
      
                case 1:
                {
                  strcpy(sdp->sensorName, "Remote2");
                  sdp->dataType = Luminosity;
                  sdp->dataLen = 2;
      
                  uint16_t lum = 1234;
                  byte* b = (byte*) &lum;
                  sdp->data[0] = *b++;
                  sdp->data[1] = *b;
                }
                break;
                
              } // switch(sensorNumber)
      
              // отсылаем данные взад
              rs485Packet.crc = crc8((byte*)&rs485Packet,sizeof(CoreTransportPacket)- 1);
              SendDataToRS485((byte*)&rs485Packet,sizeof(CoreTransportPacket));
          }
          break; // CoreSensorData
          
        } // switch

    rs485WritePtr = 0; // обнуляем указатель записи, т.к. всё уже приняли и обработали

  } // правильный заголовок
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool GotRS485Packet()
{
  // проверяем, есть ли у нас валидный RS-485 пакет
  return rs485WritePtr > ( sizeof(CoreTransportPacket)-1 );
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ProcessIncomingRS485Packets() // обрабатываем входящие пакеты по RS-485
{
  while(Serial.available())
  {
    rsPacketPtr[rs485WritePtr++] = (byte) Serial.read();
   
    if(GotRS485Packet())
    {
      processRS485Packet();
    }
  } // while
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SendDataToRS485(byte* data, byte dataSize)
{
  RS485Send();

  for(byte i=0;i<dataSize;i++)
    Serial.write(data[i]);

  RS485waitTransmitComplete();

  RS485Receive();
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RS485Receive()
{
  // переводим контроллер RS-485 на приём
    digitalWrite(RS485_DE_PIN,LOW);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RS485Send()
{
  // переводим контроллер RS-485 на передачу
    digitalWrite(RS485_DE_PIN,HIGH);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RS485waitTransmitComplete()
{
  // ждём завершения передачи по UART
  while(!(UCSR0A & _BV(TXC0) ));
}
//--------------------------------------------------------------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(57600);

  pinMode(RS485_DE_PIN,OUTPUT);
  RS485Receive();

}
//--------------------------------------------------------------------------------------------------------------------------------------
void loop() 
{
  
  ProcessIncomingRS485Packets();

}
//--------------------------------------------------------------------------------------------------------------------------------------

