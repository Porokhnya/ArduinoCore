#ifndef _CORE_TRANSPORT_H
#define _CORE_TRANSPORT_H
//--------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "CoreConfig.h"
#include "CoreArray.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreTransportClient;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef void (*OnConnectFunc)(CoreTransportClient& client); // обработчик обратного вызова результата соединения/отсоединения
typedef void (*OnClientDataReceivedFunc)(CoreTransportClient& client); // обработчик прихода данных для клиента
typedef void (*OnClientWriteDoneFunc)(CoreTransportClient& client, bool isWriteSucceeded); // обработчик завершения записи данных из клиента в транспорт
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  OnConnectFunc OnConnect; // событие "статус соединения клиента изменился"
  OnClientDataReceivedFunc OnClientDataReceived; // событие "получены данные для клиента"
  OnClientWriteDoneFunc OnClientWriteDone; // событие "данные клиента записаны в поток"
  
} CoreTransportEvents;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreTransportClient;
//--------------------------------------------------------------------------------------------------------------------------------------
// класс асинхронного транспорта, предназначен для предоставления интерфейса неблокирующей работы с AT-прошивками железок,
// типа SIM800 или ESP.
// производные классы могут держать свой пул клиентов, при этом должны заботиться о том,
// чтобы при запросе клиента на соединение этому клиенту назначался ID свободного слота в пуле
// клиентов. Класс транспорта по запросу открывает соединение, пишет туда данные асинхронно,
// также принимает данные от железки и рассовывает эти данные по клиентам.
// информирование внешнего мира осуществляется при помощи событий структуры CoreTransportEvents.
// запись в транспорт осуществляется через класс CoreTransportClient и его методы connect и write.
// методы connect и write класса CoreTransportClient являются неблокирующими, поэтому запись вызовом
// write следует осуществлять только, если клиент соединён, т.е. вызов connected() возвращает true.
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreTransport
{
  public:
  
    CoreTransport(Stream* stream);
    virtual ~CoreTransport();

    virtual void init(CoreTransportEvents events); // начинаем работу, запоминаем привязку к событиям
    virtual void update() = 0; // обновляем состояние транспорта

    virtual CoreTransportClient* getFreeClient() = 0; // возвращает свободного клиента (не законнекченного и не занятого делами)

protected:

  void setClientID(CoreTransportClient& client, uint8_t id);
  void setClientConnected(CoreTransportClient& client, bool connected);
  void setClientBuffer(CoreTransportClient& client,const uint8_t* buff, size_t sz);
  

  CoreTransportEvents events;
  Stream* pStream;    

  friend class CoreTransportClient;
  virtual void beginWrite(CoreTransportClient& client) = 0; // начинаем писать в транспорт с клиента
  virtual void beginConnect(CoreTransportClient& client, const char* ip, uint16_t port) = 0; // начинаем коннектиться к адресу
  virtual void beginDisconnect(CoreTransportClient& client) = 0; // начинаем отсоединение от адреса
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
// класс клиента транспорта. Держит буфер (для приёма или передачи), обеспечивает пересылку запроса на неблокирующую запись в транспорт,
// а также запроса на неблокирующее соединение с IP в транспорт.
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreTransportClient
{
  public:

  static CoreTransportClient* Create(CoreTransport* transport)
  {
    CoreTransportClient* instance = new CoreTransportClient();
    instance->parent = transport;
    return instance;    
  }

  void Destroy()
  {
    delete this;
  }
  
   bool connected() 
   {
    return isConnected;
   }
     
   uint8_t getID()
   {
    return clientID;
   }

  operator bool()
  {
    return  (clientID != 0xFF); 
  }


  void write(const uint8_t* buff, size_t sz)
  {
    if(!sz || !buff || !connected() || clientID == 0xFF)
      return;

    setBuffer(buff,sz);

    parent->beginWrite(*this);
      
  }

  void connect(const char* ip, uint16_t port)
  {
    if(connected()) // уже присоединены, нельзя коннектится до отсоединения!!!
      return;
          
    parent->beginConnect(*this,ip,port);
  }

  void disconnect()
  {
    if(!connected())
      return;

    parent->beginDisconnect(*this);
  }


  void clear()
  {
    if(buffer)
      delete [] buffer; 

    bufferSize = 0;
    buffer = NULL;
  }

  const uint8_t* getBuffer()
  {
    return buffer;
  }

  size_t getBufferSize()
  {
    return bufferSize;
  }

 protected:

    friend class CoreTransport;

    void setConnected(bool flag)
    {
     isConnected = flag; 
    }
    
    void setID(uint8_t id)
    {
      clientID = id;
    }
 
  void setBuffer(const uint8_t* buff, size_t sz)
  {
    clear();
    bufferSize = sz;
    buffer = new  uint8_t[bufferSize];
    memcpy(buffer,buff,bufferSize);
  }


 private:

    CoreTransportClient(const CoreTransportClient& rhs);
    CoreTransportClient operator=(const CoreTransportClient& rhs);
        
   ~CoreTransportClient() 
   {
     clear();
   }

    CoreTransportClient()
    {
      clientID = 0xFF;
      isConnected = false;
      buffer = NULL;
      bufferSize = 0;
      parent = NULL;
    }

   
     

  uint8_t clientID;
  bool isConnected;
  CoreTransport* parent;

  uint8_t* buffer;
  size_t bufferSize;
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
// тестовый транспорт для Serial
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef _CORE_DEBUG
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSerialTransport : public CoreTransport
{
  public:
    CoreSerialTransport() : CoreTransport(&Serial) {thisClient = NULL;}
    ~CoreSerialTransport() {
      thisClient->Destroy();
      }

    virtual void init(CoreTransportEvents events)
    {
      CoreTransport::init(events);

    }

    virtual CoreTransportClient* getFreeClient() // всегда возвращаем одного клиента
    {
        if(!thisClient)
          thisClient = CoreTransportClient::Create(this);

       return thisClient;
    }

    virtual void update() {}

  private:

       CoreTransportClient* thisClient;

  protected:
    
    virtual void beginConnect(CoreTransportClient& client, const char* ip, uint16_t port)
    {
      // тут коннектимся, когда надо - дёргаем функцию OnConnect и сообщаем, что клиент соединён
      setClientID(client,123);
      setClientConnected(client,true);
      
      if(events.OnConnect)
      {        
        events.OnConnect(client);
      }
    }

    virtual void beginDisconnect(CoreTransportClient& client)
    {
       // отсоединяем клиента асинхронно, если надо. По результату дёргаем событие OnConnect.
        setClientConnected(client,false);
        
        if(events.OnConnect)
        {
          events.OnConnect(client);
        }
      
    }
    
    virtual void beginWrite(CoreTransportClient& client)
    {
       // тут начинаем асинхронную запись. Когда надо - получаем с клиента буфер и пишем в поток
        pStream->write(client.getBuffer(),client.getBufferSize());

        if(events.OnClientWriteDone)
        {
          events.OnClientWriteDone(client, true); // говорим, что данные из клиента записаны
        }

        // очищаем буфер клиента
        client.clear();

        // тестово отсоединяем клиента
        setClientConnected(client,false);
                
        if(events.OnConnect)
        {
          events.OnConnect(client);
        }
   
    }
    
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // _CORE_DEBUG
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_ESP_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  bool CreateAP: 1; // поднимать ли точку доступа ?
  bool ConnectToRouter: 1; // коннектиться ли к роутеру ?
  bool UseRebootPin : 1; // использовать ли пин пересброса питания при зависании ESP ?
  
} ESPTransportSettingsFlags;
//--------------------------------------------------------------------------------------------------------------------------------------
struct ESPTransportSettingsClass
{
  ESPTransportSettingsFlags Flags; // флаги
  
  String APName; // имя точки доступа
  String APPassword; // пароль точки доступа
  
  String RouterID; // SSID роутера
  String RouterPassword; // пароль роутера
  
  byte UARTSpeed; // скорость работы с ESP (1 - 9600, 2 - 19200, 4 - 38400, 6 - 57600, 12 - 115200)
  byte SerialNumber; // номер Serial, который используется для работы с ESP (1 - Serial1, 2 - Serial2, 3 - Serial 3)
  
  byte RebootPin; // номер пина для пересброса питания ESP
  byte PowerOnLevel; // уровень для включения питания (1 - HIGH, 0 - LOW)
  
  byte HangTimeout; // кол-во секунд, по истечении которых модем считается зависшим (не пришёл ответ на команду)
  byte HangPowerOffTime; // сколько секунд держать питание выключенным при перезагрузке ESP, если он завис
  byte WaitInitTIme; // сколько секунд ждать загрузки модема при инициализации/переинициализации
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern ESPTransportSettingsClass ESPTransportSettings;
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_ESP_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_RS485_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
extern "C" {
  void ON_RS485_RECEIVE(byte packetID, byte dataLen, byte* data);
}
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  byte UARTSpeed; // скорость работы с RS-485 (1 - 9600, 2 - 19200, 4 - 38400, 6 - 57600, 12 - 115200)
  byte SerialNumber; // номер Serial, который используется для работы с RS-485 (1 - Serial1, 2 - Serial2, 3 - Serial 3)
  byte DEPin; // пин управления приёмом/передачей по RS-485
  
} CoreRS485Settings;
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreRS485Settings RS485Settings;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct // информация об известном входящем пакете
{
  byte headerLen; // длина известного заголовка
  byte* header; // байты известного заголовка
  byte packetDataLen; // длина данных пакета, который ждётся на шине
  byte packetID; // внутренний идентификатор пакета
  
} RS485IncomingHeader;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<RS485IncomingHeader> RS485KnownHeadersList;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  rs485WaitingHeader,
  rs485WaitingData
  
} RS485State;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreRS485
{
  public:
    CoreRS485();

    void begin();
    void update();
    void clear();

    void sendData(byte* data, byte dataSize); // отправляет данные в шину
    void addKnownPacketHeader(byte* header, byte headerSize, byte packetDataLen, byte packetID); // добавляем пакет в известные пакеты на шине


private:

  HardwareSerial* workStream;
  HardwareSerial* getMyStream(byte SerialNumber);

  byte* dataBuffer;
  byte dataBufferLen;
  byte writeIterator;
  RS485IncomingHeader* currentHeader;

  RS485State machineState;

  RS485KnownHeadersList knownHeaders;

  void switchToReceive();
  void switchToSend();
  void waitTransmitComplete();
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreRS485 RS485;
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_RS485_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------

#endif
