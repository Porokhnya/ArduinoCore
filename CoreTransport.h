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
  void ON_LORA_RECEIVE(uint8_t* packet, int packetSize);
}
//--------------------------------------------------------------------------------------------------------------------------------------
// типы пакетов, гоняющихся по транспортам
//--------------------------------------------------------------------------------------------------------------------------------------
enum
{
  CoreSensorData = 1, // пакет с показаниями датчика от какого либо устройства в системе
  CoreDataRequest = 2, // пакет с запросом информации по данным к модулю в системе
  CoreDataReceipt = 3, // пакет с ответом на от мастера, что данные получены
};
//--------------------------------------------------------------------------------------------------------------------------------------
// общее описание пакета, гоняющегося по транспортам
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  // заголовок пакета
  uint8_t header1;
  uint8_t header2;
  uint8_t header3;

  // идентификатор кластера
  uint8_t clusterID;

  // идентификатор устройства, пославшего пакет
  uint8_t deviceID;

  // тип пакета
  uint8_t packetType;

  uint8_t packetData[23]; // данные пакета, в зависимости от типа пакета
  

  // контрольная сумма
  uint8_t crc; 
  
} CoreTransportPacket;
//--------------------------------------------------------------------------------------------------------------------------------------
// описание пакета данных запроса информации о кол-ве данных у модуля на шине (23 байта)
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  uint8_t toDeviceID; // какому устройству предназначен запрос
  uint8_t dataCount; // сколько у устройства записей с данными

  uint8_t reserved[21]; // резерв
  
} CoreDataRequestPacket;
//--------------------------------------------------------------------------------------------------------------------------------------
// описание пакета данных с показаниями датчика, гоняющегося по транспортам (23 байта)
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  // имя датчика, максимум 9 символов, заканчивается '\0'
  char sensorName[10]; 

  // признак наличия данных
  uint8_t hasData;

  // тип данных (температура, влажность, прочее)
  uint8_t dataType;
    
  // длина данных
  uint8_t dataLen;

  // данные, максимум 10 байт
  uint8_t data[10];
  
} CoreSensorDataPacket;
//--------------------------------------------------------------------------------------------------------------------------------------
// описание пакета-квитанции в получении данных
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  uint8_t toDeviceID; // какому устройству предназначен пакет
  char sensorName[10];  // имя датчика, данные с которого получены, максимум 9 символов, заканчивается '\0'
  
  uint8_t reserved[12]; // резерв
  
} CoreDataReceiptPacket;
//--------------------------------------------------------------------------------------------------------------------------------------
// Коды ошибок транспорта
//--------------------------------------------------------------------------------------------------------------------------------------
#define CT_ERROR_NONE             0 // нет ошибки
#define CT_ERROR_CANT_CONNECT     1 // не удалось установить соединение
#define CT_ERROR_CANT_WRITE       2 // ошибка записи данных из клиента в поток
//--------------------------------------------------------------------------------------------------------------------------------------
// типы событий
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  etConnected,      // соединено
  etDisconnected,   // отсоединено
  etDataWritten,    // данные записаны
  etDataAvailable   // доступны входящие данные
  
} ClientEventType;
//--------------------------------------------------------------------------------------------------------------------------------------
// класс подписчика на события клиента - каждый, использующий клиента, может подписаться и отписаться на его события.
//--------------------------------------------------------------------------------------------------------------------------------------
struct IClientEventsSubscriber
{
  virtual void OnClientConnect(CoreTransportClient& client, int errorCode) = 0; // событие "Клиент соединён"
  virtual void OnClientDisconnect(CoreTransportClient& client, int errorCode) = 0; // событие "Клиент отсоединён"
  virtual void OnClientDataWritten(CoreTransportClient& client, int errorCode) = 0; // событие "Данные из клиента записаны в поток"
  virtual void OnClientDataAvailable(CoreTransportClient& client) = 0; // событие "Для клиента поступили данные"
};
//--------------------------------------------------------------------------------------------------------------------------------------
// класс асинхронного транспорта, предназначен для предоставления интерфейса неблокирующей работы с AT-прошивками железок,
// типа SIM800 или ESP.
// производные классы могут держать свой пул клиентов, при этом должны заботиться о том,
// чтобы при запросе клиента на соединение этому клиенту назначался ID свободного слота в пуле
// клиентов. Класс транспорта по запросу открывает соединение, пишет туда данные асинхронно,
// также принимает данные от железки и рассовывает эти данные по клиентам.
// информирование внешнего мира осуществляется при помощи подписки на события клиента через интерфейс IClientEventsSubscriber.
// запись в транспорт осуществляется через класс CoreTransportClient и его методы connect и write.
// методы connect и write класса CoreTransportClient являются неблокирующими, поэтому запись вызовом
// write следует осуществлять только, если клиент соединён, т.е. вызов connected() возвращает true.
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreTransport
{
  public:
  
    CoreTransport();
    virtual ~CoreTransport();

    // обновляем состояние транспорта
    virtual void update() = 0; 

    // начинаем работу
    virtual void begin() = 0; 

    // проверяет, готов ли транспорт к работе (например, проведена ли первичная инициализация)
    virtual bool ready() = 0; 

    // возвращает свободного клиента (не законнекченного и не занятого делами)
    virtual CoreTransportClient* getFreeClient() = 0;

   // подписка на события клиентов
   virtual void subscribe(IClientEventsSubscriber* subscriber) = 0;
   
   // отписка от событий клиентов
   virtual void unsubscribe(IClientEventsSubscriber* subscriber) = 0;

protected:

  friend class CoreTransportClient;

  // подписка/отписка клиентов на события
  void subscribeClient(CoreTransportClient& client, IClientEventsSubscriber* subscriber);
  void unsubscribeClient(CoreTransportClient& client, IClientEventsSubscriber* subscriber);

  void setClientID(CoreTransportClient& client, uint8_t id);
  void setClientData(CoreTransportClient& client,const uint8_t* data, size_t sz);
  void setClientBusy(CoreTransportClient& client,bool busy);

  // функции, косвенно вызывающие события на клиенте
  void setClientConnected(CoreTransportClient& client, bool connected, int errorCode);  
  void notifyClientDataWritten(CoreTransportClient& client, int errorCode);
  void notifyClientDataAvailable(CoreTransportClient& client);
  
    
  virtual void beginWrite(CoreTransportClient& client) = 0; // начинаем писать в транспорт с клиента
  virtual void beginConnect(CoreTransportClient& client, const char* ip, uint16_t port) = 0; // начинаем коннектиться к адресу
  virtual void beginDisconnect(CoreTransportClient& client) = 0; // начинаем отсоединение от адреса
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<IClientEventsSubscriber*> ClientSubscribers;
//--------------------------------------------------------------------------------------------------------------------------------------
// класс клиента транспорта. Держит буфер (для приёма или передачи), обеспечивает пересылку запроса на неблокирующую запись в транспорт,
// а также запроса на неблокирующее соединение с IP в транспорт.
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreTransportClient
{
  public:

  static CoreTransportClient* Create(CoreTransport* transport);
  void Destroy();

  CoreTransport* getTransport(); // возвращает транспорт

  
   bool connected();     
   uint8_t getID();

  operator bool()
  {
    return  (clientID != 0xFF); 
  }

  bool operator==(const CoreTransportClient& rhs)
  {
    return (rhs.clientID == clientID);
  }

  bool operator!=(const CoreTransportClient& rhs)
  {
    return !(operator==(rhs));
  }

  void connect(const char* ip, uint16_t port);
  void disconnect();
  void write(const uint8_t* buff, size_t sz);

  bool busy(); // проверяет, занят ли клиент чем-либо  

  const uint8_t* getData();
  size_t getDataSize();


 protected:

    friend class CoreTransport;

    void subscribe(IClientEventsSubscriber* subscriber); // подписка на события клиента
    void unsubscribe(IClientEventsSubscriber* subscriber); // отписка от событий клиента

    // транспорт, когда надо - выставляет флаг занятоски клиента каким-то делом
    void setBusy(bool flag);

    // установка ID клиента транспортом
    void setID(uint8_t id);

    // транспорт может дёргать эту функцию, чтобы установить данные в клиент
    void setData(const uint8_t* buff, size_t sz);

    // транспорт дёргает эту функцию, чтобы клиент сообщил подписчикам статус соединения
    void setConnected(bool isConnected, int errorCode);

    // транспорт дёргает эту функцию, чтобы клиент сообщил подписчикам, что данные с него записаны в транспорт
    void notifyDataWritten(int errorCode);

    // транспорт дёргает эту функцию, чтобы клиент сообщил подписчикам, что в клиенте доступны данные
    void notifyDataAvailable();


 private:

    CoreTransportClient(const CoreTransportClient& rhs);
    CoreTransportClient operator=(const CoreTransportClient& rhs);
        
   ~CoreTransportClient();
    CoreTransportClient();


    ClientSubscribers subscribers;
    int getSubscriberIndex(IClientEventsSubscriber* subscriber);

    void raiseEvent(ClientEventType et, int errorCode);
 
    void clearBuffer();

    uint8_t clientID;
    bool isConnected;
    bool isBusy;
    CoreTransport* parent;
  
    uint8_t* dataBuffer;
    size_t dataBufferSize;
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_ESP_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  bool ConnectToRouter: 1; // коннектиться ли к роутеру ?
  bool UseRebootPin : 1; // использовать ли пин пересброса питания при зависании ESP ?
  
  uint8_t pad : 6;
  
} ESPTransportSettingsFlags;
//--------------------------------------------------------------------------------------------------------------------------------------
struct ESPTransportSettingsClass
{
  ESPTransportSettingsFlags Flags; // флаги
  
  String APName; // имя точки доступа
  String APPassword; // пароль точки доступа
  
  String RouterID; // SSID роутера
  String RouterPassword; // пароль роутера
  
  uint8_t UARTSpeed; // скорость работы с ESP (1 - 9600, 2 - 19200, 4 - 38400, 6 - 57600, 12 - 115200)
  uint8_t SerialNumber; // номер Serial, который используется для работы с ESP (1 - Serial1, 2 - Serial2, 3 - Serial 3)
  
  uint8_t RebootPin; // номер пина для пересброса питания ESP
  uint8_t PowerOnLevel; // уровень для включения питания (1 - HIGH, 0 - LOW)
  
  uint8_t HangTimeout; // кол-во секунд, по истечении которых модем считается зависшим (не пришёл ответ на команду)
  uint8_t HangPowerOffTime; // сколько секунд держать питание выключенным при перезагрузке ESP, если он завис
  uint8_t WaitInitTIme; // сколько секунд ждать загрузки модема при инициализации/переинициализации
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern ESPTransportSettingsClass ESPTransportSettings;
//--------------------------------------------------------------------------------------------------------------------------------------
// класс транспорта для ESP
//--------------------------------------------------------------------------------------------------------------------------------------
#define ESP_MAX_CLIENTS 4 // наш пул клиентов
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  
  actionNone,
  actionDisconnect, // запрошено отсоединение клиента
  actionConnect, // запрошено подсоединение клиента
  actionWrite, // запрошена запись из клиента в ESP
  
} ESPClientAction;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  ESPClientAction action; // действие, которое надо выполнить с клиентом
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

    virtual bool ready(); // проверяем на готовность к работе

   // подписка на события клиентов
   virtual void subscribe(IClientEventsSubscriber* subscriber);
   
   // отписка от событий клиентов
   virtual void unsubscribe(IClientEventsSubscriber* subscriber);
   
    virtual CoreTransportClient* getFreeClient(); // возвращает свободного клиента (не законнекченного и не занятого делами)

  protected:

    virtual void beginWrite(CoreTransportClient& client); // начинаем писать в транспорт с клиента
    virtual void beginConnect(CoreTransportClient& client, const char* ip, uint16_t port); // начинаем коннектиться к адресу
    virtual void beginDisconnect(CoreTransportClient& client); // начинаем отсоединение от адреса

  private:

      Stream* workStream; // поток, с которым мы работаем (читаем/пишем в/из него)

      // пул клиентов
      CoreTransportClient* clients[ESP_MAX_CLIENTS];

      ESPClientsQueue clientsQueue; // очередь действий с клиентами

      bool isInQueue(ESPClientsQueue& queue,CoreTransportClient* client); // тестирует - не в очереди ли уже клиент?
      void addToQueue(ESPClientsQueue& queue,CoreTransportClient* client, ESPClientAction action, const char* ip=NULL, uint16_t port=0); // добавляет клиента в очередь
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
  uint8_t UARTSpeed; // скорость работы с RS-485 (1 - 9600, 2 - 19200, 4 - 38400, 6 - 57600, 12 - 115200)
  uint8_t SerialNumber; // номер Serial, который используется для работы с RS-485 (1 - Serial1, 2 - Serial2, 3 - Serial 3)
  uint8_t DEPin; // пин управления приёмом/передачей по RS-485
  bool isMasterMode; // мы в режиме мастера или слейва?
  
} CoreRS485Settings;
//--------------------------------------------------------------------------------------------------------------------------------------
#ifndef CORE_RS485_DISABLE_CORE_LOGIC
typedef struct
{
  uint8_t clientNumber;
  uint8_t readingAttempts;
  
} CoreRS485ExcludeRecord;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CoreRS485ExcludeRecord> CoreRS485ExcludedList;
#endif // CORE_RS485_DISABLE_CORE_LOGIC
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreRS485Settings RS485Settings;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreRS485
{
  public:
    CoreRS485();

    void begin();
    void update();
    void reset();

    // возвращает UART, используемый для RS-485
    HardwareSerial* getSerial() {return workStream; }

    void sendData(uint8_t* data, uint8_t dataSize); // отправляет данные в шину: переключается на передачу, посылает данные, после отсыла - переключается на приём
    void switchToReceive(); // переключается на приём
    void switchToSend(); // переключается на передачу
    void waitTransmitComplete(); //ждёт окончания передачи
    
//    void addKnownPacketHeader(uint8_t* header, uint8_t headerSize, uint8_t packetDataLen, uint8_t packetID); // добавляем пакет в известные пакеты на шине


private:

#ifndef CORE_RS485_DISABLE_CORE_LOGIC

  CoreRS485ExcludedList excludedList;
  bool inExcludedList(uint8_t clientNumber);
  void addToExcludedList(uint8_t clientNumber);
  uint8_t getOfflineModulesCount();
  void updateSlaveMode();
  void updateMasterMode();

  CoreTransportPacket rs485Packet;
  uint8_t* rsPacketPtr;
  uint8_t rs485WritePtr;

  bool gotRS485Packet();
  void processIncomingRS485Packets();
  void processRS485Packet();
#endif // CORE_RS485_DISABLE_CORE_LOGIC

  HardwareSerial* workStream;
  HardwareSerial* getMyStream(uint8_t SerialNumber);

 // uint8_t* dataBuffer;
//  uint8_t dataBufferLen;
//  uint8_t writeIterator;
//  RS485IncomingHeader* currentHeader;

//  RS485State machineState;

//  RS485KnownHeadersList knownHeaders;


  
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreRS485 RS485;
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_RS485_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------

#endif
