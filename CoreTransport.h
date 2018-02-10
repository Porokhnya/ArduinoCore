#ifndef _CORE_TRANSPORT_H
#define _CORE_TRANSPORT_H
//--------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "CoreConfig.h"
#include "CoreArray.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_SD_USE_SDFAT
  #include <SdFat.h>
  extern SdFat SD;
#else
  #include <SD.h>
#endif
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
  etConnect,      // соединено
  etDataWritten,    // данные записаны
  etDataAvailable   // доступны входящие данные
  
} ClientEventType;
//--------------------------------------------------------------------------------------------------------------------------------------
// класс подписчика на события клиента - каждый, использующий клиента, может подписаться и отписаться на его события.
//--------------------------------------------------------------------------------------------------------------------------------------
struct IClientEventsSubscriber
{
  virtual void OnClientConnect(CoreTransportClient& client, bool connected, int errorCode) = 0; // событие "Статус соединения клиента"
  virtual void OnClientDataWritten(CoreTransportClient& client, int errorCode) = 0; // событие "Данные из клиента записаны в поток"
  virtual void OnClientDataAvailable(CoreTransportClient& client, bool isDone) = 0; // событие "Для клиента поступили данные", флаг - все ли данные приняты
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
  void setClientData(CoreTransportClient& client,uint8_t* data, size_t sz);
  void setClientBusy(CoreTransportClient& client,bool busy);

  // функции, косвенно вызывающие события на клиенте
  void setClientConnected(CoreTransportClient& client, bool connected, int errorCode);  
  void notifyClientDataWritten(CoreTransportClient& client, int errorCode);
  void notifyClientDataAvailable(CoreTransportClient& client, bool isDone);
  
    
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
  bool write(uint8_t* buff, size_t sz, bool takeBufferOwnership=false);

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
    void setData(uint8_t* buff, size_t sz, bool copy);

    // транспорт дёргает эту функцию, чтобы клиент сообщил подписчикам статус соединения
    void setConnected(bool isConnected, int errorCode);

    // транспорт дёргает эту функцию, чтобы клиент сообщил подписчикам, что данные с него записаны в транспорт
    void notifyDataWritten(int errorCode);

    // транспорт дёргает эту функцию, чтобы клиент сообщил подписчикам, что в клиенте доступны данные
    void notifyDataAvailable(bool isDone);


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
  bool enabled;
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
typedef struct
{
  bool ready                : 1; // флаг готовности
  bool connectedToRouter    : 1; // флаг того, что заокннекчены к роутеру
  bool isAnyAnswerReceived  : 1; // флаг, что мы получили хотя бы один ответ от модема
  bool waitForDataWelcome   : 1; // флаг, что ждём приглашения на отсыл данных
  bool wantReconnect        : 1; // флаг, что мы должны переподсоединиться к роутеру
  bool onIdleTimer          : 1; // флаг, что мы в режиме простоя
  
} CoreESPTransportFlags;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  cmdNone, // ничего не делаем
  cmdWantReady, // надо получить ready от модуля
  cmdEchoOff, // выключаем эхо
  cmdCWMODE, // переводим в смешанный режим
  cmdCWSAP, // создаём точку доступа
  cmdCWJAP, // коннектимся к роутеру
  cmdCWQAP, // отсоединяемся от роутера
  cmdCIPMODE, // устанавливаем режим работы
  cmdCIPMUX, // разрешаем множественные подключения
  cmdCIPSERVER, // запускаем сервер
  cmdCheckModemHang, // проверяем на зависание модема 
  cmdCIPCLOSE, // отсоединямся
  cmdCIPSTART, // соединяемся
  cmdCIPSEND, // начинаем слать данные
  cmdWaitSendDone, // ждём окончания отсылки данных
  
} ESPCommands;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<ESPCommands> ESPCommandsList;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  espIdle,        // состояние "ничего не делаем"
  espWaitAnswer,  // состояние "ждём ответа на команду, посланную ESP"
  espReboot,      // состояние "ESP в процессе перезагрузки"
  espWaitInit,    // ждём инициализации после подачи питания
  
} ESPMachineState;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  kaNone,
  kaOK,             // OK
  kaError,          // ERROR
  kaFail,           // FAIL
  kaSendOk,         // SEND OK
  kaSendFail,       // SEND FAIL
  
} ESPKnownAnswer;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreESPTransport : public CoreTransport
{
  public:
    CoreESPTransport();

    virtual void update(); // обновляем состояние транспорта
    virtual void begin(); // начинаем работу

    // возвращает IP клиента и станции
    bool getIP(String& staIP, String& apIP);
    
    // возвращает MAC клиента и станции
    bool getMAC(String& staMAC, String& apMAC);

    // пропинговать гугл
    bool pingGoogle(bool& result);

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


      void restart();
      void processIPD();

      bool isESPBootFound(const String& line);
      bool isKnownAnswer(const String& line, ESPKnownAnswer& result);

      CoreESPTransportFlags flags; // флаги состояния
      ESPMachineState machineState; // состояние конечного автомата

      ESPCommands currentCommand;
      ESPCommandsList initCommandsQueue; // очередь команд на инициализацию
      unsigned long timer; // общий таймер
      unsigned long idleTime; // время, которое нам надо подождать, ничего не делая
      
      void clearInitCommands();
      void createInitCommands(bool addResetCommand);
      
      void sendCommand(const String& command, bool addNewLine=true);
      void sendCommand(ESPCommands command);
      
      Stream* workStream; // поток, с которым мы работаем (читаем/пишем в/из него)

      String* wiFiReceiveBuff;

      // пул клиентов
      CoreTransportClient* clients[ESP_MAX_CLIENTS];


      void clearClientsQueue(bool raiseEvents);

      ESPClientsQueue clientsQueue; // очередь действий с клиентами

      bool isClientInQueue(CoreTransportClient* client, ESPClientAction action); // тестирует - не в очереди ли уже клиент?
      void addClientToQueue(CoreTransportClient* client, ESPClientAction action, const char* ip=NULL, uint16_t port=0); // добавляет клиента в очередь
      void removeClientFromQueue(CoreTransportClient* client); // удаляет клиента из очереди  
      
      void initClients();
    
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreESPTransport ESP;
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_ESP_WEB_SERVER
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  CoreTransportClient* client;
  char* query;
  
} CoreWebServerQuery;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CoreWebServerQuery> CoreWebServerQueries;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  unsigned long pendingBytes;
  CoreTransportClient* client;
#ifdef CORE_SD_USE_SDFAT
  SdFile file;
#else
  File file;
#endif  
} CoreWebServerPendingFileData;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CoreWebServerPendingFileData> CoreWebServerPendingFiles;
//--------------------------------------------------------------------------------------------------------------------------------------
#define WEB_HEADER_BEGIN F("HTTP/1.1 ")
#define WEB_HEADER_CONNECTION F("Connection: close")
#define WEB_HEADER_CONTENT_TYPE F("Content-Type: ")
#define WEB_HEADER_CONTENT_LENGTH F("Content-Length: ")
#define WEB_HEADER_LINE F("\r\n")
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreESPWebServerClass : public IClientEventsSubscriber, public Stream
{
  public:
    CoreESPWebServerClass();

  // IClientEventsSubscriber
  virtual void OnClientConnect(CoreTransportClient& client, bool connected, int errorCode); // событие "Статус соединения клиента"
  virtual void OnClientDataWritten(CoreTransportClient& client, int errorCode); // событие "Данные из клиента записаны в поток"
  virtual void OnClientDataAvailable(CoreTransportClient& client, bool isDone); // событие "Для клиента поступили данные", флаг - все ли данные приняты

  // Stream
  virtual void flush(){}
  virtual int peek() {return 0;}
  virtual int read() {return 0;}
  virtual int available() {return 0;}
  virtual size_t write(uint8_t ch) { *internalBuffer += (char) ch; return 1;}

  private:

    CoreWebServerQueries pendingQueries;
    bool isOurClient(CoreTransportClient* client);
    void removePendingQuery(CoreWebServerQuery* query);
    CoreWebServerQuery* getPendingQuery(CoreTransportClient* client);

    void processQuery(CoreTransportClient* client, char* query);
    void processURI(CoreTransportClient* client, String& uri);

    String* internalBuffer;
    
    CoreWebServerPendingFiles pendingFiles;
    CoreWebServerPendingFileData* getPendingFileData(CoreTransportClient* client);
    void removePendingFileData(CoreTransportClient* client);
    void sendNextFileData(CoreWebServerPendingFileData* pfd);

    void send404(CoreTransportClient* client);
    String getContentType(const String& fileName);
    
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreESPWebServerClass CoreESPWebServer;
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_ESP_WEB_SERVER
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
  bool enabled;
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
#ifdef CORE_MQTT_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  mqttDisabled, // выключено
  mqttThroughESP, // через ESP
  
} MQTTWorkMode;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  bool enabled;
  MQTTWorkMode workMode;
  String clientID;
  String serverAddress;
  int serverPort;
  String userName;
  String password;
  uint16_t intervalBetweenTopics;

  void reset()
  {
    enabled = false;
  }
  
  
} CoreMQTTSettings;
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreMQTTSettings MQTTSettings;
//--------------------------------------------------------------------------------------------------------------------------------------
#define MQTT_CONNECT_COMMAND (1 << 4)
#define MQTT_PUBLISH_COMMAND (3 << 4)
#define MQTT_SUBSCRIBE_COMMAND (8 << 4)
#define MQTT_QOS1 (1 << 1)
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<uint8_t> MQTTBuffer;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  mqttWaitClient, // ожидаем свободного клиента
  mqttWaitConnection, // ожидаем подсоединения к брокеру
  mqttWaitReconnect, // ожидаем переподсоединения к брокеру (в случае неуспешного соединения)
  mqttSendConnectPacket, // отсылаем пакет с информацией о подсоединении к брокеру
  mqttWaitSendConnectPacketDone,
  mqttSendSubscribePacket, // отсылаем пакет с информацией о подписке
  mqttWaitSendSubscribePacketDone,
  mqttSendPublishPacket, // отсылаем пакет публикации
  mqttWaitSendPublishPacketDone,
  
} MQTTState;
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreMQTT : public IClientEventsSubscriber, public Stream
{
  public:
    CoreMQTT();
    void reset();
    void update();
    void begin();

  virtual void OnClientConnect(CoreTransportClient& client, bool connected, int errorCode); // событие "Статус соединения клиента"
  virtual void OnClientDataWritten(CoreTransportClient& client, int errorCode); // событие "Данные из клиента записаны в поток"
  virtual void OnClientDataAvailable(CoreTransportClient& client, bool isDone); // событие "Для клиента поступили данные", флаг - все ли данные приняты

  // Stream
  virtual void flush(){}
  virtual int peek() {return 0;}
  virtual int read() {return 0;}
  virtual int available() {return 0;}
  virtual size_t write(uint8_t ch) { *streamBuffer += (char) ch; return 1;}


private:

  CoreTransportClient* currentClient;
  CoreTransport* currentTransport;
  unsigned long timer;

  MQTTState machineState;
  uint16_t mqttMessageId;
  String* streamBuffer;

  size_t currentStoreNumber;

  void pushToReportQueue(String* toReport);
  Vector<String*> reportQueue;
  void clearReportsQueue();

  void constructFixedHeader(byte command, MQTTBuffer& fixedHeader,size_t payloadSize);

  void constructConnectPacket(String& mqttBuffer,int& mqttBufferLength,const char* id, const char* user, const char* pass,const char* willTopic,uint8_t willQoS, uint8_t willRetain, const char* willMessage);
  void constructSubscribePacket(String& mqttBuffer,int& mqttBufferLength, const char* topic);
  void constructPublishPacket(String& mqttBuffer,int& mqttBufferLength, const char* topic, const char* payload);
  
  void encode(MQTTBuffer& buff,const char* str);

  void writePacket(MQTTBuffer& fixedHeader,MQTTBuffer& payload, String& mqttBuffer,int& mqttBufferLength);

  void processIncomingPacket(CoreTransportClient* client);

  void convertAnswerToJSON(const String& answer, String* resultBuffer);
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreMQTT MQTT;
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_MQTT_TRANSPORT_ENABLED

#endif
