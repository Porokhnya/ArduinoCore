#include "CoreLOGIC.h"
//--------------------------------------------------------------------------------------------------------------------------------------
CoreLogic Logic;
//--------------------------------------------------------------------------------------------------------------------------------------
CoreLogic::CoreLogic()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
// LoRa: обработчик входящих данных. Если логика ядра при работе с лова выключена (см. CORE_LORA_DISABLE_CORE_LOGIC) - то это событие
// вызывается каждый раз по приходу любых данных по радиоканалу. Если логика ядра включена - то это событие вызывается только тогда, когда
// ядро не смогло обработать входящий пакет данных (например, получен неизвестный пакет).
// в packet - данные пакета длиной packetSize.
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreLogic::onIncomingLoRaData(uint8_t* packet, int16_t packetSize)
{
#ifdef CORE_LORA_TRANSPORT_ENABLED  

  // доступен глобальный объект LoRa (см. CoreLora.h)
  // поместите ваш код сюда.
  
#endif  
}
//--------------------------------------------------------------------------------------------------------------------------------------
// обработчик события прихода пользовательских данных по RS-485 от мастера к слейву. Мастер может в любой момент отослать любые пользовательские данные
// из логики следующим образом:
/*
    byte* data = ....;
    uint16_t dataLength = ...;
    Stream* writeStream = RS485.beginUserPacket(dataLength);
    if(writeStream)
    {
      writeStream->write(data,dataLength);
      RS485.endUserPacket();
    }
 */
 // т.е. мастер всегда инициирует передачу вызовом beginUserPacket и указанием, сколько байт будет послано в дальнейшем.
 // при получении пакета с пользовательскими данными на слейве вызовется событие, внутри которого можно отсылать любые данные назад мастеру,
 // как показано в примере ниже:
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreLogic::onIncomingRS485Data(Stream* stream, uint16_t dataToRead)
{
#ifdef CORE_RS485_TRANSPORT_ENABLED

  // доступен глобальный объект RS485, см. CoreTransport.h
  // поместите ваш код сюда.

 /*
  DBG(F("USER DATA ON RS-485, LENGTH="));
  DBGLN(dataToRead);

  // просто пишем эти данные в Serial
  uint16_t readed = 0;
  while(readed < dataToRead)
  {
    while(stream->available())
    {
      Serial.write(stream->read());
      readed++;
    }
  }

  // вычитали данные, отправляем назад любые данные к мастеру, какие хочется
  String pingBack = F("Ping back from RS-485 slave!");
  // отсылаем данные мастеру, пусть делает с ними что хочет. 
  // Тут мы ожидаем, что после окончания передачи мастер переключится на приём, и будет ждать от нас ответа.
  RS485.echo((uint8_t*) pingBack.c_str(),pingBack.length());
  */

#endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
// Событие "Поступил входящий звонок". В первом параметре - номер телефона, второй параметр - сообщает, известный ли это номер телефона, 
// третий - если установлен, то трубка будет положена сразу же.
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreLogic::onIncomingCall(const String& phoneNumber, bool isKnownNumber, bool& shouldHangUp)
{
#ifdef CORE_SIM800_TRANSPORT_ENABLED

  // доступен глобальный объект SIM800 (см. CoreTransport.h)
  // поместите ваш код сюда.

  
/*  
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
  SIM800.sendSMS(phoneNumber, F("Не звони мне больше, редиска :)"), true);  
*/  
#endif  
}
//--------------------------------------------------------------------------------------------------------------------------------------
// Событие "Пришло СМС". В первом параметре - номер телефона, с которого пришло СМС, во втором - декодированное сообщение,
// в третьем - флаг, что сообщение получено с известного номера
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreLogic::onIncomingSMS(const String& phoneNumber,const String& message, bool isKnownNumber)
{
#ifdef CORE_SIM800_TRANSPORT_ENABLED

  // доступен глобальный объект SIM800 (см. CoreTransport.h)
  // поместите ваш код сюда.

  
/*
    Serial.print(F("INCOMING SMS FROM: "));
    Serial.println(phoneNumber);
    
    Serial.print(F("INCOMING SMS TEXT: "));
    Serial.println(message);

    // отправляем СМС назад
      static int smsCounter = 0;
      smsCounter++;
      String smsText = F("Ответочка номер #");
      smsText += smsCounter;
      SIM800.sendSMS(phoneNumber, smsText, true); // последний параметр - flash или обычное смс
*/
#endif  
}
//--------------------------------------------------------------------------------------------------------------------------------------
// Событие "ядро стартовало". В этом событии можно добавлять любые датчики в систему, например.
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreLogic::onCoreStarted()
{
  // поместите ваш код сюда.
}
//--------------------------------------------------------------------------------------------------------------------------------------
// событие "Поступила неизвестная команда по Serial". Если отработали - возвращаем true, иначе - false
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreLogic::onUnknownCommand(const String& command, Stream* outStream)
{
  // поместите ваш код сюда.
  
  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
// функция вызывается при первом старте ядра в работу
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreLogic::doBegin()
{
  // поместите ваш код сюда.

  
/*  
  // например, мы можем назначить свои обработчики запросов к веб-серверу, если надо
  #ifdef CORE_ESP_WEB_SERVER
    // для примера - назначим обработчик обращения к файлу test.txt при помощи лямбда-функции
    ESPWebServer.on("test.txt", [](const char* uri, const char* params){

          ESPWebServer.send(
            200,  // код ответа
            "text/plain", // тип контента
            "This is dynamic handler!" // данные контента
            );
      
      });
  #endif  
*/  
}
//--------------------------------------------------------------------------------------------------------------------------------------
// функция вызывается при обновлении ядра, здесь можно делать работу для логики конкретного проекта
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreLogic::doUpdate()
{
  // поместите ваш код сюда.

  
  
/*
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
*/  
}
//--------------------------------------------------------------------------------------------------------------------------------------
// ДАЛЕЕ ИДЁТ СЛУЖЕБНАЯ ОБВЯЗКА, ПРЕДОСТАВЛЯЮЩАЯ ДОСТУП К ТОМУ ИЛИ ИНОМУ СОБЫТИЮ и т.п.
//--------------------------------------------------------------------------------------------------------------------------------------
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// \/
//--------------------------------------------------------------------------------------------------------------------------------------
// обработчик неизвестных команд, входящих по Serial
//--------------------------------------------------------------------------------------------------------------------------------------
void unhandledCommandHandler(const String& command, Stream* outStream)
{
  if(!Logic.onUnknownCommand(command,outStream))
  {
    outStream->print(CORE_COMMAND_ANSWER_ERROR);
    outStream->println(F("UNKNOWN_COMMAND"));
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreLogic::begin(const uint8_t* defaultConfig, size_t configSize)
{
  Serial.begin(CORE_COMMUNICATION_SPEED);

  // все необработанные данные из Serial будут перенаправлены в функцию unhandledCommandHandler
  Core.setup(unhandledCommandHandler);
  
  // пробуем загрузить конфиг из EEPROM, если не получится - грузим из флеша конфиг по умолчанию
  if(!Core.loadConfig())
  {
     Core.saveConfig(defaultConfig,configSize,true); 
     Core.loadConfig();
  }

  // говорим ядру, чтобы начинало работу
  Core.begin();

  doBegin();


}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreLogic::update()
{
  // обновляем показания с датчиков
  Core.update();
  
  // обрабатываем входящие по Serial команды
  Core.handleCommands();

  doUpdate();

}
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_CORE_BEGIN()
{
  Logic.onCoreStarted();
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_SIM800_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_INCOMING_CALL(const String& phoneNumber, bool isKnownNumber, bool& shouldHangUp)
{
  Logic.onIncomingCall(phoneNumber, isKnownNumber, shouldHangUp); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_SMS_RECEIVED(const String& phoneNumber,const String& message, bool isKnownNumber)
{
  Logic.onIncomingSMS(phoneNumber,message,isKnownNumber);
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_SIM800_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef  CORE_RS485_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_RS485_DATA_RECEIVED(Stream* stream, uint16_t dataToRead)
{
  Logic.onIncomingRS485Data(stream,dataToRead);
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_RS485_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_LORA_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_LORA_RECEIVE(uint8_t* packet, int16_t packetSize)
{
  Logic.onIncomingLoRaData(packet,packetSize);
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_LORA_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------

