//--------------------------------------------------------------------------------------------------------------------------------------
#include "Core.h"
#include "CoreConfig.h"
//--------------------------------------------------------------------------------------------------------------------------------------
// обработчик неизвестных команд, входящих по Serial
//--------------------------------------------------------------------------------------------------------------------------------------
void unhandledCommandHandler(const String& command, Stream* outStream)
{
  outStream->print(CORE_COMMAND_ANSWER_ERROR);
  outStream->println(F("UNKNOWN_COMMAND"));
}
//--------------------------------------------------------------------------------------------------------------------------------------
// Конфиг ко умолчанию
//--------------------------------------------------------------------------------------------------------------------------------------
const uint8_t defaultConfig[] PROGMEM = {

  // заголовок конфига
  CORE_HEADER1,
  CORE_HEADER2,
  CORE_HEADER3,
    //---------------------------------------
      DeviceIDRecord, // уникальный ID устройства в системе (0-255)
      1, // у нас устройство с адресом 1, адрес всегда доступен вызовом Core.DeviceID
    //---------------------------------------
      ClusterIDRecord, // уникальный ID кластера (0-255)
      1, // у нас кластер номер 1, номер кластера всегда доступен вызовом Core.ClusterID
    //---------------------------------------
      FractDelimiterRecord, // // разделитель целой и дробной частей
      ',', // у нас этим разделителем будет запятая
    //---------------------------------------
      TemperatureUnitRecord, // // вид измеряемой температуры (цельсии - UnitCelsius или фаренгейты - UnitFahrenheit)
      UnitCelsius, // у нас будут цельсии
    //---------------------------------------
      SensorsUpdateIntervalRecord, // интервал опроса датчиков, в секундах
      5, // у нас будет 5 секунд между опросами
    //---------------------------------------
  
  // окончание конфига
  CORE_HEADER1
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
// Событие "Поступил входящий звонок". В первом параметре - номер телефона, второй параметр - сообщает, известный ли это номер телефона, 
// третий - если установлен, то трубка будет положена сразу же.
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_INCOMING_CALL(const String& phoneNumber, bool isKnownNumber, bool& shouldHangUp)
{
  Serial.print(F("Incoming call from: "));
  Serial.print(phoneNumber);
  Serial.print(F("; known number? "));
  if(isKnownNumber)
    Serial.println(F("KNOWN"));
  else
    Serial.println(F("UNKNOWN"));
  
  // кладём трубку
  shouldHangUp = true;

  // отправляем СМС в ответ
  #ifdef CORE_SIM800_TRANSPORT_ENABLED
    SIM800.sendSMS(phoneNumber, F("Не звони мне больше, редиска :)"), true);
  #endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
// Событие "Пришло СМС". В первом параметре - номер телефона, с которого пришло СМС, во втором - декодированное сообщение,
// в третьем - флаг, что сообщение получено с известного номера
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_SMS_RECEIVED(const String& phoneNumber,const String& message, bool isKnownNumber)
{
    Serial.print(F("INCOMING SMS FROM: "));
    Serial.println(phoneNumber);
    
    Serial.print(F("INCOMING SMS TEXT: "));
    Serial.println(message);

    // отправляем СМС назад
    #ifdef CORE_SIM800_TRANSPORT_ENABLED
      static int smsCounter = 0;
      smsCounter++;
      String smsText = "Ответочка номер #";
      smsText += smsCounter;
      SIM800.sendSMS(phoneNumber, smsText, true); // последний параметр - flash или обычное смс
    #endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
// Событие ON_CORE_BEGIN вызывается, когда запрошен старт ядра в работу, например, при перезагрузке конфига из конфигуратора. 
// Здесь можно добавлять свои датчики в систему.
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_CORE_BEGIN()
{
  // пример добавления датчика в систему см. в файле "Пример добавления датчика.txt"
}
//--------------------------------------------------------------------------------------------------------------------------------------
// LoRa: обработчик входящих данных. Если логика ядра при работе с лова выключена (см. CORE_LORA_DISABLE_CORE_LOGIC) - то это событие
// вызывается каждый раз по приходу любых данных по радиоканалу. Если логика ядра включена - то это событие вызывается только тогда, когда
// ядро не смогло обработать входящий пакет данных (например, получен неизвестный пакет).
// в packet - данные пакета длиной packetSize.
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_LORA_RECEIVE(uint8_t* packet, int packetSize) 
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
/*
#define SCENE_PIN 8
CorePinScenario scene; // сценарий мигания светодиодом на пине 8
*/
//--------------------------------------------------------------------------------------------------------------------------------------
void setup() 
{ 
  Serial.begin(CORE_COMMUNICATION_SPEED);

  // все необработанные данные из Serial будут перенаправлены в функцию unhandledCommandHandler
  Core.setup(unhandledCommandHandler);
  
  // пробуем загрузить конфиг из EEPROM, если не получится - грузим из флеша конфиг по умолчанию
  if(!Core.loadConfig())
  {
     Core.saveConfig(defaultConfig,sizeof(defaultConfig),true); 
     Core.loadConfig();
  }

  // говорим ядру, чтобы начинало работу
  Core.begin();


  // мы можем назначить свои обработчики запросов к веб-серверу, если надо
  #ifdef CORE_ESP_WEB_SERVER
    // для примера - назначим обработчик обращения к файлу test.txt при помощи лямбда-функции
    ESPWebServer.on("test.txt", [](const char* uri, const char* params){

          ESPWebServer.send(
            200,  // код ответа
            "text/plain", // тип контента
            "This is dynamic handler!" // данные контентаы
            );
      
      });
  #endif


/*
  // моргаем светодиодом по сценарию, по кругу
  scene.add({SCENE_PIN,1,100}); // 100 мс - высокий уровень
  scene.add({SCENE_PIN,0,100}); // 100 мс - низкий уровень
  scene.add({SCENE_PIN,1,100}); // 100 мс - высокий уровень
  scene.add({SCENE_PIN,0,100}); // 100 мс - низкий уровень
  scene.add({SCENE_PIN,1,100}); // 100 мс - высокий уровень
  scene.add({SCENE_PIN,0,2000}); // 2 секунды - низкий уровень
*/
}
//--------------------------------------------------------------------------------------------------------------------------------------
void loop() 
{  
  // обновляем сцену
 // scene.update();
  
  // обновляем показания с датчиков
  Core.update();
  
  // обрабатываем входящие по Serial команды
  Core.handleCommands();

  // тут раз в 30 секунд будем публиковать свой топик в брокера MQTT
  #ifdef CORE_MQTT_TRANSPORT_ENABLED
    static unsigned long freeramMillis = 0;
    if(millis() - freeramMillis > 30000)
    {
      freeramMillis = millis();
      String topicName = F("FREERAM");
      String payload = String(Core.getFreeMemory());
      MQTT.publish(topicName.c_str(),payload.c_str());
    }
  #endif // CORE_MQTT_TRANSPORT_ENABLED


  // просим отправить на ThingSpeak наше значение в одно из полей канала
  #ifdef CORE_THINGSPEAK_TRANSPORT_ENABLED
    static unsigned long thingSpeakMillis = 0;
    if(millis() - thingSpeakMillis > 5000)
    {
      static int thingspeakData = 0;
      thingspeakData = random(50,500);

      // просим опубликовать наше рандомное число в поле номер 3. Допустимые диапазоны полей - 1-8.
      // при очередной публикации в ThingSpeak это рандомное число там появится.
      ThingSpeak.publish(3,String(thingspeakData));

      // ну и попросим опубликовать данные о свободной памяти контроллера в поле номер 4
      ThingSpeak.publish(4,String(Core.getFreeMemory()));
      
      thingSpeakMillis = millis();
    }
  #endif // CORE_THINGSPEAK_TRANSPORT_ENABLED

/*
  if(Signals[12])
  {
    Serial.println(F("!!!!!!!!!!!!!Signal #12 raised!!!!!!!!!!!!!!!!"));
    Signals[12] = 0; // сбрасываем сигнал до следующего его обновления
  }
*/
}
//--------------------------------------------------------------------------------------------------------------------------------------

