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
// начальный адрес хранения конфига в памяти
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
// включить/выключить поддержку сигналов (настраиваемые через конфигуратор флаги, выставляемые в зависимости от внешних условий -
// показаний датчиков и т.п.)
#define CORE_SIGNALS_ENABLED
#define CORE_SIGNALS_UPDATE_INTERVAL 2000 // через сколько миллисекунд обновлять состояние сигналов
//--------------------------------------------------------------------------------------------------------------------------------------
// настройки транспортов, ненужные - закомментировать
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_ESP_TRANSPORT_ENABLED // поддержка транспорта ESP
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_RS485_TRANSPORT_ENABLED // поддержка работы с шиной RS-485
// раскомментировать, если не надо использовать логику ядра при работе через RS-485 (при этом работа с RS-485 может производиться в логике вручную) - 
// отсыл данных, приём пакетов любого формата и т.п. При раскомментированной настройке ядро предоставляет только основные функции обвязки под
// RS-485 - переключение приёма/передачи и т.п.
//#define CORE_RS485_DISABLE_CORE_LOGIC 
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_LORA_TRANSPORT_ENABLED // поддержка работы с LoRa
#define CORE_LORA_TIME_SHIFT 2 // на сколько миллисекунд сдвигать попытку отправки пакета данных, если слейвом не получена квитанция от мастера
#define CORE_LORA_SEND_DURATION 30 // частота отсылки данных с датчиков, секунд (цикл будет повторяться через это значение)
#define CORE_LORA_RETRANSMIT_COUNT 5 // сколько попыток перепосыла пакета с данными предпринимать слейву
#define CORE_LORA_RECEIPT_TIMEOUT 100 // таймаут на получение квитанции от мастера, миллисекунд
// раскомментировать, если не надо использовать логику ядра при работе через LoRa (при этом работа с LoRa может производиться в логике вручную) - 
// отсыл данных, приём пакетов любого формата и т.п. При раскомментированной настройке ядро предоставляет глобальный объект LoRa,
// с помощью которого можно вручную работать по радиоканалу.
//#define CORE_LORA_DISABLE_CORE_LOGIC 
//--------------------------------------------------------------------------------------------------------------------------------------
// настройки SD
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_SD_SUPPORT_ENABLED // закомментировать, если не нужна поддержка SD
#define CORE_SD_USE_SDFAT // закомментировать, если не надо использьвать SdFat (вместо SdFat будет использоваться штатная библиотека SD)
#define CORE_SD_SDFAT_SPEED SPI_HALF_SPEED // скорость работы SD для библиотеки SdFat
#define CORE_SD_CS_PIN 4 // номер пина CS для SD-карты
//--------------------------------------------------------------------------------------------------------------------------------------
// настройки общения ядра с внешним миром
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_COMMAND_GET F("GET=") // префикс для команды получения данных из ядра
#define CORE_COMMAND_SET F("SET=") // префикс для команды сохранения данных в ядро
#define CORE_COMMAND_ANSWER_OK F("OK=") // какой префикс будет посылаться в ответ на команду получения данных и её успешной отработке
#define CORE_COMMAND_ANSWER_ERROR F("ER=") // какой префикс будет посылаться в ответ на команду получения данных и её неуспешной отработке
#define CORE_COMMAND_PARAM_DELIMITER "|" // разделитель параметров
#define CORE_END_OF_DATA F("[END]")
//--------------------------------------------------------------------------------------------------------------------------------------
#define DUE_BOARD 1
#define MEGA_BOARD 2
#define ESP_BOARD 3
//--------------------------------------------------------------------------------------------------------------------------------------
#if defined (__arm__) && defined (__SAM3X8E__)
  #define TARGET_BOARD DUE_BOARD
#elif  defined(__AVR_ATmega2560__) 
  #define TARGET_BOARD MEGA_BOARD
#elif defined(ESP8266)  
  #define TARGET_BOARD ESP_BOARD
#else
  #error "Unknown target board!"
#endif  
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
