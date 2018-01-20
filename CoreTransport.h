#ifndef _CORE_TRANSPORT_H
#define _CORE_TRANSPORT_H
//--------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "CoreConfig.h"
#include "CoreArray.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreTransportClient;
//--------------------------------------------------------------------------------------------------------------------------------------
extern "C" {
  void ON_LORA_RECEIVE(byte* packet, int packetSize);
  void ON_RS485_RECEIVE(byte packetID, byte dataLen, byte* data);
  void ON_CLIENT_CONNECT(CoreTransportClient& client);
  void ON_CLIENT_DATA_RECEIVED(CoreTransportClient& client);
  void ON_CLIENT_WRITE_DONE(CoreTransportClient& client, bool isWriteSucceeded);
}
//--------------------------------------------------------------------------------------------------------------------------------------
// типы пакетов, гоняющихся по транспортам
//--------------------------------------------------------------------------------------------------------------------------------------
enum
{
  CoreSensorData = 1, // пакет с показаниями датчика от какого либо устройства в системе
};
//--------------------------------------------------------------------------------------------------------------------------------------
// общее описание пакета, гоняющегося по транспортам
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  // заголовок пакета
  byte header1;
  byte header2;
  byte header3;

  // идентификатор кластера
  byte clusterID;

  // идентификатор устройства
  byte deviceID;

  // тип пакета
  byte packetType;

  byte packetData[23]; // данные пакета, в зависимости от типа пакета
  

  // контрольная сумма
  byte crc; 
  
} CoreTransportPacket;
//--------------------------------------------------------------------------------------------------------------------------------------
// описание пакета данных с показаниями датчика, гоняющегося по транспортам (23 байта)
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  // имя датчика, максимум 10 символов, заканчивается '\0'
  char sensorName[11]; 

  // тип данных (температура, влажность, прочее)
  byte dataType;
    
  // длина данных
  byte dataLen;

  // данные, максимум 10 байт
  byte data[10];
  
} CoreSensorDataPacket;
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
  
    CoreTransport();
    virtual ~CoreTransport();

    virtual void update() = 0; // обновляем состояние транспорта
    virtual void begin() = 0; // начинаем работу

    virtual CoreTransportClient* getFreeClient() = 0; // возвращает свободного клиента (не законнекченного и не занятого делами)

protected:

  void setClientID(CoreTransportClient& client, uint8_t id);
  void setClientConnected(CoreTransportClient& client, bool connected);
  void setClientBuffer(CoreTransportClient& client,const uint8_t* buff, size_t sz);
  void setClientBusy(CoreTransportClient& client,bool busy);
  

  Stream* workStream; // поток, с которым мы работаем (читаем/пишем в/из него)

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

  CoreTransport* getTransport() // возвращает транспорт
  {
    return parent;
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

  bool busy() // проверяет, занят ли клиент чем-либо
  {
    return isBusy;
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

    void setBusy(bool flag)
    {
      isBusy = flag;
    }

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
      isBusy = false;
      buffer = NULL;
      bufferSize = 0;
      parent = NULL;
    }

   
     

  uint8_t clientID;
  bool isConnected;
  bool isBusy;
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
    CoreSerialTransport() : CoreTransport() {thisClient = NULL;}
    ~CoreSerialTransport() {
      thisClient->Destroy();
      }

    void begin()
    {
      workStream = &Serial;
    }

    virtual CoreTransportClient* getFreeClient() // возвращаем одного свободного, ничем не занятого клиента
    {
        if(!thisClient)
          thisClient = CoreTransportClient::Create(this);

       return thisClient;
    }

    void update() {}

  private:

       CoreTransportClient* thisClient;

  protected:
    
    virtual void beginConnect(CoreTransportClient& client, const char* ip, uint16_t port)
    {
      // тут коннектимся, когда надо - дёргаем функцию OnConnect и сообщаем, что клиент соединён
      setClientID(client,123);
      setClientConnected(client,true);

      ON_CLIENT_CONNECT(client);
      
    }

    virtual void beginDisconnect(CoreTransportClient& client)
    {
       // отсоединяем клиента асинхронно, если надо. По результату дёргаем событие OnConnect.
        setClientConnected(client,false);

        ON_CLIENT_CONNECT(client);
      
    }
    
    virtual void beginWrite(CoreTransportClient& client)
    {
       // тут начинаем асинхронную запись. Когда надо - получаем с клиента буфер и пишем в поток
        workStream->write(client.getBuffer(),client.getBufferSize());

        ON_CLIENT_WRITE_DONE(client,true); // говорим, что данные из клиента записаны


        // тестово будем писать в клиент из транспорта
        const char* pong = "pong";
        setClientBuffer(client,(const uint8_t*)pong, strlen(pong)+1);

        // говорим, что для клиента получены данные
        ON_CLIENT_DATA_RECEIVED(client);
        
        // очищаем буфер клиента
        client.clear();

        // тестово отсоединяем клиента
        setClientConnected(client,false);
        ON_CLIENT_CONNECT(client);
                   
    }
    
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // _CORE_DEBUG
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_ESP_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
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
// класс транспорта для ESP
//--------------------------------------------------------------------------------------------------------------------------------------
#define ESP_MAX_CLIENTS 4 // наш пул клиентов
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  CoreTransportClient* client; // ссылка на клиента
  char* ip; // IP для подсоединения
  uint16_t port; // порт для подсоединения
   
} ESPClientQueueData; // данные по клиенту в очереди
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<ESPClientQueueData> ESPClientsQueue; // очередь клиентов на совершение какой-либо исходящей операции (коннект, дисконнект, запись)
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreESPTransport : public CoreTransport
{
  public:
    CoreESPTransport();

    virtual void update(); // обновляем состояние транспорта
    virtual void begin(); // начинаем работу

    virtual CoreTransportClient* getFreeClient(); // возвращает свободного клиента (не законнекченного и не занятого делами)


  protected:

    virtual void beginWrite(CoreTransportClient& client); // начинаем писать в транспорт с клиента
    virtual void beginConnect(CoreTransportClient& client, const char* ip, uint16_t port); // начинаем коннектиться к адресу
    virtual void beginDisconnect(CoreTransportClient& client); // начинаем отсоединение от адреса


  private:

      CoreTransportClient* clients[ESP_MAX_CLIENTS];
      
      ESPClientsQueue writeOutQueue; // очередь на запись
      ESPClientsQueue connectQueue; // очередь на соединение
      ESPClientsQueue disconnectQueue; // очередь на отсоединение

      bool isInQueue(ESPClientsQueue& queue,CoreTransportClient* client); // тестирует - не в очереди ли уже клиент?
      void addToQueue(ESPClientsQueue& queue,CoreTransportClient* client, const char* ip=NULL, uint16_t port=0); // добавляет клиента в очередь
      void removeFromQueue(ESPClientsQueue& queue,CoreTransportClient* client); // удаляет клиента из очереди
      
      
      void initClients();
    
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreESPTransport ESP;
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_ESP_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_RS485_TRANSPORT_ENABLED
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
