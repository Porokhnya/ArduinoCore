#ifndef _CORE_CONFIG_H
#define _CORE_CONFIG_H
//--------------------------------------------------------------------------------------------------------------------------------------
// Конфигурация ядра
//--------------------------------------------------------------------------------------------------------------------------------------
// закомментировать для выключения отладочной информации
#define _CORE_DEBUG
//--------------------------------------------------------------------------------------------------------------------------------------
/* 
 *  какую память используем:
 *  1 - встроенную EEPROM
 *  2 - AT24C32
 *  3 - AT24C64
 *  4 - AT24C128
 *  5 - AT24C256
 *  6 - AT24C512
 */
#define MEMORY_USED 1
//--------------------------------------------------------------------------------------------------------------------------------------
// если забыли сменить используемую память при компиляции под DUE - сделаем это автоматически на память самой большой ёмкости
//--------------------------------------------------------------------------------------------------------------------------------------
#if defined (__arm__) && defined (__SAM3X8E__)
  #if MEMORY_USED == 1
    #pragma message "CHANGED MEMORY_USED AUTOMATICALLY!!!"
    #undef MEMORY_USED
    #define MEMORY_USED 2
  #endif
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
// начальный адрес хранения привязок адресов DS18B20 к их индексам в памяти (три байта - заголовок, одна запись - 8 байт, настройки по умолчанию
// позволяют сохранить до начала данных конфига - 20 датчиков)
#define CORE_DS18B20_BINDING_ADDRESS 0
//--------------------------------------------------------------------------------------------------------------------------------------
// начальный адрес хранения конфига по датчикам в памяти
#define CORE_STORE_ADDRESS 200
//--------------------------------------------------------------------------------------------------------------------------------------
// настройки коммуникации с программой-конфигуратором (по Serial)
#define CORE_COMMUNICATION_SPEED 57600 // скорость коммуникации
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_FRACT_DELIMITER ',' // разделитель по умолчанию целой и дробной частей для строкового представления показаний
#define CORE_HUMIDITY_DELIMITER '/' // разделитель между температурой и влажностью (для датчиков влажности) для строкового представления показаний
//--------------------------------------------------------------------------------------------------------------------------------------
// интервал обновления показаний с датчиков по умолчанию, миллисекунд
#define CORE_SENSORS_UPDATE_INTERVAL 2000
//--------------------------------------------------------------------------------------------------------------------------------------
// определяем поддерживаемые системой датчики, ненужные - закомментировать
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_DS18B20_ENABLED // поддержка датчика температуры DS18B20
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_BH1750_ENABLED // поддержка датчика освещённости BH1750 (шина I2C)
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_SI7021_ENABLED // поддержка датчика влажности Si7021 (шина I2C)
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_DHT_ENABLED // поддержка датчиков влажности семейства DHT
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_DS3231_ENABLED // поддержка часов реального времени DS3231
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_DIGITALPORT_ENABLED // поддержка датчиков состояния цифрового порта
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_ANALOGPORT_ENABLED // поддержка чтения данных с аналоговых портов
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_USERDATA_SENSOR_ENABLED // поддержка пользовательских данных в виде виртуального датчика с любым типом показаний
//--------------------------------------------------------------------------------------------------------------------------------------
// настройки транспортов, ненужные - закомментировать
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_ESP_TRANSPORT_ENABLED // поддержка транспорта ESP
//--------------------------------------------------------------------------------------------------------------------------------------
// настройки общения ядра с внешним миром
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_COMMAND_GET F("GET=") // префикс для команды получения данных из ядра
#define CORE_COMMAND_SET F("SET=") // префикс для команды сохранения данных в ядро
#define CORE_COMMAND_ANSWER_OK F("OK=") // какой префикс будет посылаться в ответ на команду получения данных и её успешной отработке
#define CORE_COMMAND_ANSWER_ERROR F("ER=") // какой префикс будет посылаться в ответ на команду получения данных и её неуспешной отработке
#define CORE_COMMAND_PARAM_DELIMITER "|" // разделитель параметров
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
