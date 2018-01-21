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
  CoreDataRequest = 2, // пакет с запросом информации по данным к модулю в системе
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

  // идентификатор устройства, пославшего пакет
  byte deviceID;

  // тип пакета
  byte packetType;

  byte packetData[23]; // данные пакета, в зависимости от типа пакета
  

  // контрольная сумма
  byte crc; 
  
} CoreTransportPacket;
//--------------------------------------------------------------------------------------------------------------------------------------
// описание пакета данных запроса информации о кол-ве данных у модуля на шине (23 байта)
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  byte toDeviceID; // какому устройству предназначен запрос
  byte dataCount; // сколько у устройства записей с данными

  byte reserved[21]; // резерв
  
} CoreDataRequestPacket;
//--------------------------------------------------------------------------------------------------------------------------------------
// описание пакета данных с показаниями датчика, гоняющегося по транспортам (23 байта)
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  // имя датчика, максимум 9 символов, заканчивается '\0'
  char sensorName[10]; 

  // признак наличия данных
  byte hasData;

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

      HardwareSerial* lastSerial;
      
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
#define CORE_RS485_POLL_INTERVAL 100 // минимальный интервал перед опросом очередной адрес на шине (в режиме мастера), мс
#define CORE_RS485_ROUNDTRIP 5 // за сколько секунд мы хотим опросить всех клиентов (основываясь на этом значении - мы высчитываем интервал между опросами)
#define CORE_RS485_MAX_ADDRESS 49 // максимально 50 устройств на шине
#define CORE_RS485_MAX_BAD_READINGS 5 // максимальное кол-во неудачных попыток чтения с модуля, после которых он уже не опрашивается
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  byte UARTSpeed; // скорость работы с RS-485 (1 - 9600, 2 - 19200, 4 - 38400, 6 - 57600, 12 - 115200)
  byte SerialNumber; // номер Serial, который используется для работы с RS-485 (1 - Serial1, 2 - Serial2, 3 - Serial 3)
  byte DEPin; // пин управления приёмом/передачей по RS-485
  bool isMasterMode; // мы в режиме мастера или слейва?
  
} CoreRS485Settings;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  byte clientNumber;
  byte readingAttempts;
  
} CoreRS485ExcludeRecord;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CoreRS485ExcludeRecord> CoreRS485ExcludedList;
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreRS485Settings RS485Settings;
//--------------------------------------------------------------------------------------------------------------------------------------
/*
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
*/
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreRS485
{
  public:
    CoreRS485();

    void begin();
    void update();
    void clear();

    void sendData(byte* data, byte dataSize); // отправляет данные в шину
//    void addKnownPacketHeader(byte* header, byte headerSize, byte packetDataLen, byte packetID); // добавляем пакет в известные пакеты на шине


private:

  CoreRS485ExcludedList excludedList;
  bool inExcludedList(byte clientNumber);
  void addToExcludedList(byte clientNumber);
  byte getOfflineModulesCount();

  HardwareSerial* workStream;
  HardwareSerial* getMyStream(byte SerialNumber);

 // byte* dataBuffer;
//  byte dataBufferLen;
//  byte writeIterator;
//  RS485IncomingHeader* currentHeader;

//  RS485State machineState;

//  RS485KnownHeadersList knownHeaders;

  void switchToReceive();
  void switchToSend();
  void waitTransmitComplete();

  void updateSlaveMode();
  void updateMasterMode();

  CoreTransportPacket rs485Packet;
  byte* rsPacketPtr;
  byte rs485WritePtr;

  bool gotRS485Packet();
  void processIncomingRS485Packets();
  void processRS485Packet();
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreRS485 RS485;
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_RS485_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------

#endif
