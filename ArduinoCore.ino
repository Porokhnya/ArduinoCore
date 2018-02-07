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

/*
  if(Signals[12])
  {
    Serial.println(F("!!!!!!!!!!!!!Signal #12 raised!!!!!!!!!!!!!!!!"));
    Signals[12] = 0; // сбрасываем сигнал до следующего его обновления
  }
*/  

}
//--------------------------------------------------------------------------------------------------------------------------------------

