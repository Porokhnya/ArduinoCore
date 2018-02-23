#include "CoreConfig.h"
#include "Core.h"
#include <avr/pgmspace.h>
//--------------------------------------------------------------------------------------------------------------------------------------
#if MEMORY_USED == 1
  #include <EEPROM.h>
#else
  #include "AT24CX.h"
  AT24CX* memory;  
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
extern "C" {
static void __nocorebegin(){}
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_CORE_BEGIN() __attribute__ ((weak, alias("__nocorebegin")));
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_SD_SUPPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
  #ifdef CORE_SD_USE_SDFAT
    SdFat SD;
  #endif
//--------------------------------------------------------------------------------------------------------------------------------------
SDSettingsStruct SDSettings;
//--------------------------------------------------------------------------------------------------------------------------------------
// FileUtils
//--------------------------------------------------------------------------------------------------------------------------------------
void FileUtils::printFile(const String& fileName, Stream* outStream)
{
  #ifdef CORE_SD_USE_SDFAT

    SdFile f;
    if(!f.open(fileName.c_str(),O_READ))
    return;

    if(f.isDir())
    {
      f.close();
      return;
    }
    
  #else
  
    File f = SD.open(fileName.c_str(),FILE_READ);
    if(!f)
    return;

    if(f.isDirectory())
    {
      f.close();
      return;
    }
  
  #endif

    while(1)
    {
      int iCh = f.read();
      if(iCh == -1)
        break;

        outStream->write((byte) iCh);
    }

    f.close();
  

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool FileUtils::readLine(
  #ifdef CORE_SD_USE_SDFAT
  SdFile
  #else
  File
  #endif
&f, String& result)
{
  result = "";

  #ifdef CORE_SD_USE_SDFAT
  if(!f.isOpen())
  #else
    if(!f)
  #endif
    return false;
    
    while(1)
    {
      int iCh = f.read();
      
      if(iCh == -1)
        return false;

      char ch = (char) iCh;

      if(ch == '\r')
        continue;

      if(ch == '\n')
        return true;

      result += ch;
    }
    return false;  
}
//--------------------------------------------------------------------------------------------------------------------------------
String FileUtils::getFileName(
  #ifdef CORE_SD_USE_SDFAT
  SdFile
  #else
  File
  #endif
  &f)
{
  #ifdef CORE_SD_USE_SDFAT
      char nameBuff[50] = {0};
      f.getName(nameBuff,50);
      return nameBuff;
  #else
    return f.name();
  #endif      
}
//--------------------------------------------------------------------------------------------------------------------------------------
void FileUtils::printFilesNames(const String& dirName, bool recursive, Stream* outStream)
{
  #ifdef CORE_SD_USE_SDFAT
  
  const char* dirP = dirName.c_str();
  

  SdFile root;
  if(!root.open(dirP,O_READ))
    return;

  root.rewind();

  SdFile entry;
  while(entry.openNext(&root,O_READ))
  {
    if(entry.isDir())
    {
      String currentDirName =  FileUtils::getFileName(entry);
      outStream->print(currentDirName);
      outStream->println(F("\t<DIR>"));
      
      if(recursive)
      {
        String subPath = dirName + "/";
        subPath += currentDirName;
        FileUtils::printFilesNames(subPath,recursive, outStream);      
      }
    }
    else
    {      
      outStream->println(FileUtils::getFileName(entry));
    }
    entry.close();
  } // while


  root.close();
  #else

    const char* dirP = dirName.c_str();

    File root = SD.open(dirP);
    if(!root)
      return;

    root.rewindDirectory();
    while(true)
    {
      File entry = root.openNextFile();
      if(!entry)
        break;

        if(entry.isDirectory())
        {
          String currentDirName =  FileUtils::getFileName(entry);
          outStream->print(currentDirName);
          outStream->println(F("\t<DIR>"));
          
          if(recursive)
          {
            String subPath = dirName + "/";
            subPath += currentDirName;
            FileUtils::printFilesNames(subPath,recursive, outStream);      
          }
        }
        else
        {
          outStream->println(FileUtils::getFileName(entry));
        }

        entry.close();
    }
    root.close();

  #endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t FileUtils::countFiles(const String& dirName, bool recursive)
{
#ifdef CORE_SD_USE_SDFAT  
  int16_t result = 0;
  const char* dirP = dirName.c_str();
  
  SdFile root;
  if(!root.open(dirP,O_READ))
    return result;

  root.rewind();

  SdFile entry;
  while(entry.openNext(&root,O_READ))
  {
    if(entry.isDir())
    {
      if(recursive)
      {
        String subPath = dirName + "/";
        subPath += FileUtils::getFileName(entry);
        result += FileUtils::countFiles(subPath,recursive);      
      }
    }
    else
    {      
      result++;
    }
    entry.close();
  } // while


  root.close();
  return result;

  #else

    int16_t result = 0;
    const char* dirP = dirName.c_str();

      File root = SD.open(dirP);
      if(!root)
        return result;

    root.rewindDirectory();
    while(true)
    {
      File entry = root.openNextFile();
      if(!entry)
        break;

        if(entry.isDirectory())
        {
          if(recursive)
          {
            String subPath = dirName + "/";
            subPath += FileUtils::getFileName(entry);
            result += FileUtils::countFiles(subPath,recursive);          
          }
        }
        else
        {
          result++;
        }

        entry.close();
    }
    root.close();
    return result;
        

  #endif

}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_SD_SUPPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
// CoreConfigIterator
//--------------------------------------------------------------------------------------------------------------------------------------
CoreConfigIterator::CoreConfigIterator()
{
  dataSize = 0;
  address = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreConfigIterator::writeOut(uint8_t b)
{
  if(outStream)
  {
    if(asHexString)
    {
      outStream->print(Core.byteToHexString(b));
    }
    else
    {
      outStream->write(b);
    }
    return true;
  }  
  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreConfigIterator::first(uint16_t addr, uint16_t sz, Stream* out, bool asHex)
{
  outStream = out; // запоминаем поток, в который нас попросили просто прочитать данные
  asHexString = asHex; // запоминаем формат, в котором нас попросили прочитать данные

#ifdef _CORE_DEBUG
  if(!outStream)
  {
   // DBGLN(F("Start iterate config..."));
  }
#endif  

  if(sz < 4) // у нас минимум 4 байта на голову и хвост
    return false;

  readed = 0;
  address = addr;
  dataSize = sz;

  // проверяем заголовок
  if(read() != CORE_HEADER1)
  {
    #ifdef _CORE_DEBUG
    if(!outStream)
    {
      DBGLN(F("!CORE_HEADER1"));
    }
    #endif
    return false;
  }

  if(read() != CORE_HEADER2)
  {
    #ifdef _CORE_DEBUG
    if(!outStream)
    {
      DBGLN(F("!CORE_HEADER2"));
    }
    #endif
    
    return false;
  }

  if(read() != CORE_HEADER3)
  {
    #ifdef _CORE_DEBUG
    if(!outStream)
    {
      DBGLN(F("!CORE_HEADER3"));
    }
    #endif
    return false;
  }
  // встали за заголовок, на начало данных
  #ifdef _CORE_DEBUG
  if(!outStream)
  {
   // DBGLN(F("Header OK."));
  }
  #endif

  // смотрим - надо ли вывести конфиг в поток?
  writeOut(CORE_HEADER1);
  writeOut(CORE_HEADER2);
  writeOut(CORE_HEADER3);

  return readRecord(); // если есть хоть одна запись - она прочитается
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreConfigIterator::next()
{
  return readRecord();
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreConfigIterator::read()
{
   if(readed < dataSize)
   {
     uint8_t b = doRead(address, readed);
     readed++; // запоминаем, сколько байт прочитали
     return b;
   }

   // достигнут конец буфера данных
   return 0xFF;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreConfigIterator::readString(String& result)
{
    while(1)
    {
      char ch = read();
      writeOut(ch);
      
      if(ch == '\0')
        break;
        
      if(!outStream)
        result += ch;            
    }
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreConfigIterator::skipString()
{
  char ch;
  
  while(1)
  {
    ch = read();
    writeOut(ch);

    if(ch == '\0')
      break;
  }  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreConfigIterator::readRecord()
{
  uint8_t recordType = read();

  if(!(recordType > DummyFirstRecord && recordType < DummyLastRecord))
  {
    // неизвестный тип записи, выходим
    #ifdef _CORE_DEBUG
    if(!outStream)
    {
     // DBGLN(F("End of config."));
    }
    #endif

    writeOut(CORE_HEADER1);
    
    return false;
  }

  writeOut(recordType);

  switch(recordType)
  {
    case SensorRecord: // данные по датчику
    {
      // читаем имя датчика
        String sensorName;
        readString(sensorName);
      
        // тут читаем тип датчика
        uint8_t b = read();
        writeOut(b);
        
        CoreSensorType type = (CoreSensorType) b;
      
        // читаем длину данных, сохранённых датчиком
        uint8_t dataLen = read();
        writeOut(dataLen);

        // есть информация по датчику, читаем её
        uint8_t* record = new uint8_t[dataLen];
        for(uint8_t i=0;i<dataLen;i++)
        {
          record[i] = read();
          writeOut(record[i]);
          
        }
      
        if(!outStream)
          applySensorRecord(sensorName,type,record);
          
        delete [] record;
            
    }
    return true; // SensorRecord

    case FractDelimiterRecord: // разделитель целой и дробной частей
    {
      uint8_t b = read();
      if(!writeOut(b))
      {
        Core.FractDelimiter = b;
      }
    }
    return true; // FractDelimiterRecord

    case DeviceIDRecord:
    {
      uint8_t b = read();
      if(!writeOut(b))
      {
        Core.DeviceID = b;
      }
      
    }
    return true; // DeviceIDRecord

    case ClusterIDRecord:
    {
      uint8_t b = read();
      if(!writeOut(b))
      {
        Core.ClusterID = b;
      }
      
    }
    return true; // ClusterIDRecord

    case WatchdogRecord: // запись настроек ватчдога
    {
             
       uint8_t b = read();
       if(!writeOut(b))
        CoreWatchdog.WatchdogEnabled = b;

        b = read();
       if(!writeOut(b))
        CoreWatchdog.WatchdogPin = b;

        uint16_t interval = 0;
        byte* writePtr = (byte*)&interval;
        bool shouldSave = false;
        
        for(uint8_t i=0;i<2;i++)
        {
          b = read();
          if(!writeOut(b))
          {
              shouldSave = true;
             *writePtr++ = b;
          }
        } // for

        if(shouldSave)
        {
          CoreWatchdog.WatchdogInterval = interval;
        }

        writePtr = (byte*)&interval;
        for(uint8_t i=0;i<2;i++)
        {
          b = read();
          if(!writeOut(b))
          {
              shouldSave = true;
             *writePtr++ = b;
          }
        } // for
        
        if(shouldSave)
        {
          CoreWatchdog.WatchdogPulseDuration = interval;
        }
      
    }
    return true;

    case SignalRecord:
    {
      uint8_t recordLength = read();
      
      if(writeOut(recordLength))
      {
        // нас попросили просто вывести всё в поток
        for(uint16_t skipBytes=0;skipBytes<recordLength;skipBytes++)
          writeOut(read());
      }
      else
      {
        
        #ifdef CORE_SIGNALS_ENABLED
          // надо сохранить в список сигналов - поэтому мы пропускаем всю запись, но перед этим запоминаем текущий адрес памяти.
          uint16_t recordStartAddress = address + readed;
          
          // скармливаем этот адрес менеджеру сигналов, и он будет знать, что по этому адресу лежит запись настроек одного сигнала
          Signals.addRecord(recordStartAddress);
        #endif

        // а теперь просто пропускаем N байт, увеличивая кол-во "прочитанных" (т.к. читать тут - не надо, всё сделает менеджер сигналов)
        readed += recordLength;
      }
      
    }
    return true; // SignalRecord

    case TemperatureUnitRecord: // вид измеряемой температуры
    {
      uint8_t b = read();
      if(!writeOut(b))
      {
        Core.TemperatureUnit = b;
      }
    }
    return true; // TemperatureUnitRecord

    case SensorsUpdateIntervalRecord: // интервал опроса датчиков
    {
      uint8_t b = read();
      if(!writeOut(b))
      {
        Core.SensorsUpdateInterval = b;
        Core.SensorsUpdateInterval *= 1000; // переводим в миллисекунды
      }
    }
    return true; // SensorsUpdateIntervalRecord

    case ThingSpeakSettingsRecord: // данные о настройках ThingSpeak
    {
      #ifdef CORE_THINGSPEAK_TRANSPORT_ENABLED
      
        // читаем запись
        ThingSpeakSettings.enabled = true;
          
        uint8_t b = read();
        bool inSaveMode = !writeOut(b);
        
        if(inSaveMode)
          ThingSpeakSettings.workMode = (TransportClientWorkMode) b;

         if(inSaveMode)
            ThingSpeakSettings.apiKey = "";

          readString(ThingSpeakSettings.apiKey);

          if(inSaveMode)
          {
            uint8_t* writePtr = (uint8_t*)&(ThingSpeakSettings.updateInterval);
            *writePtr++ = read();
            *writePtr = read();
          }
          else
          {
            writeOut(read());
            writeOut(read());
          }          

          b = read();
          writeOut(b);
          
          if(inSaveMode)
          {
            ThingSpeakSettings.clearSensors();
            
             for(uint8_t i=0;i<b;i++)
             {
               String s;
               readString(s);
               ThingSpeakSettings.addSensor(s.c_str());
             }
          }
          else
          {
            // тут просто попросили отдать данные в поток
            for(uint8_t i=0;i<b;i++)
            {
              skipString();
            }
          }
          
          
      #else
          // пропускаем запись

        // workMode
        writeOut(read());

        // apiKey
        skipString();

        // updateInterval
        writeOut(read());
        writeOut(read());

        // sensors
        uint8_t cnt = read();
        writeOut(cnt);

        for(uint8_t i=0;i<cnt;i++)
        {
          skipString();
        }
      #endif 
    }
    return true; // ThingSpeakSettingsRecord

    case SIM800SettingsRecord: // данные о настройках SIM800
    {
      #ifdef CORE_SIM800_TRANSPORT_ENABLED
        // читаем запись
        SIM800TransportSettings.enabled = true;

        // скорость UART
        uint8_t b = read();
        bool inSaveMode = !writeOut(b);
        if(inSaveMode)
          SIM800TransportSettings.UARTSpeed = b;

        // номер UART
        b = read();
        if(inSaveMode)
          SIM800TransportSettings.SerialNumber = b;
        else
          writeOut(b);

        // UseRebootPin
        b = read();
        if(inSaveMode)
          SIM800TransportSettings.UseRebootPin = b;
        else
          writeOut(b);

        // RebootPin
        b = read();
        if(inSaveMode)
          SIM800TransportSettings.RebootPin = b;
        else
          writeOut(b);

        // PowerOnLevel
        b = read();
        if(inSaveMode)
          SIM800TransportSettings.PowerOnLevel = b;
        else
          writeOut(b);

        // HangTimeout
        b = read();
        if(inSaveMode)
          SIM800TransportSettings.HangTimeout = b;
        else
          writeOut(b);

        // HangPowerOffTime
        b = read();
        if(inSaveMode)
          SIM800TransportSettings.HangPowerOffTime = b;
        else
          writeOut(b);

        // UsePowerKey
        b = read();
        if(inSaveMode)
          SIM800TransportSettings.UsePowerKey = b;
        else
          writeOut(b);

        // PowerKeyPin
        b = read();
        if(inSaveMode)
          SIM800TransportSettings.PowerKeyPin = b;
        else
          writeOut(b);

          // PowerKeyPulseDuration
          if(inSaveMode)
          {
            uint8_t* writePtr = (uint8_t*)&(SIM800TransportSettings.PowerKeyPulseDuration);
            *writePtr++ = read();
            *writePtr = read();
          }
          else
          {
            writeOut(read());
            writeOut(read());
          }          

        // PowerKeyOnLevel
        b = read();
        if(inSaveMode)
          SIM800TransportSettings.PowerKeyOnLevel = b;
        else
          writeOut(b);

          // PowerKeyInitTime
          if(inSaveMode)
          {
            uint8_t* writePtr = (uint8_t*)&(SIM800TransportSettings.PowerKeyInitTime);
            *writePtr++ = read();
            *writePtr = read();
          }
          else
          {
            writeOut(read());
            writeOut(read());
          }          

          // APN
          if(inSaveMode)
            SIM800TransportSettings.APN = "";

          readString(SIM800TransportSettings.APN);

          // APNUser
          if(inSaveMode)
            SIM800TransportSettings.APNUser = "";
            
          readString(SIM800TransportSettings.APNUser);

          // APNPassword
          if(inSaveMode)
            SIM800TransportSettings.APNPassword = "";

          readString(SIM800TransportSettings.APNPassword);
      
          //тут чтение известных номеров
          // читаем кол-во номеров
          b = read();
          writeOut(b);
          
          if(inSaveMode)
          {
            SIM800TransportSettings.clearKnownNumbers();
             for(uint8_t i=0;i<b;i++)
             {
               String s;
               readString(s);
               SIM800TransportSettings.addKnownNumber(s.c_str());
             }
          }
          else
          {
            // тут просто попросили отдать данные в поток
            for(uint8_t i=0;i<b;i++)
            {
              skipString();
            }
          }
        
      #else
      
        // пропускаем всю запись
        //UARTSpeed
        writeOut(read());

        // SerialNumber
        writeOut(read());
        
        // UseRebootPin        
        writeOut(read());

        // RebootPin
        writeOut(read());

        // PowerOnLevel
        writeOut(read());

        // HangTimeout
        writeOut(read());

        // HangPowerOffTime
        writeOut(read());

        // UsePowerKey
        writeOut(read());

        // PowerKeyPin
        writeOut(read());

        // PowerKeyPulseDuration
        writeOut(read());
        writeOut(read());

        // PowerKeyOnLevel
        writeOut(read());

        // PowerKeyInitTime
        writeOut(read());
        writeOut(read());

        // APN
        skipString();

        // APNUser
        skipString();

        // APNPassword
        skipString();

        // known numbers
        uint8_t cnt = read();
        writeOut(cnt);

        for(uint8_t i=0;i<cnt;i++)
        {
          skipString();
        }
        
      #endif // CORE_SIM800_TRANSPORT_ENABLED
    }
    return true; // SIM800SettingsRecord


    case MQTTSettingsRecord: // данные о настройках MQTT
    {
      #ifdef CORE_MQTT_TRANSPORT_ENABLED
      
          // тут читаем все настройки MQTT
          MQTTSettings.enabled = true;
          
          // читаем режим работы
          uint8_t b = read();
          bool inSaveMode = !writeOut(b);
          
          if(inSaveMode)
            MQTTSettings.workMode = (TransportClientWorkMode) b;

          // читаем ID клиента
          if(inSaveMode)
            MQTTSettings.clientID = "";

          readString(MQTTSettings.clientID);      

          // читаем адрес сервера
          if(inSaveMode)
            MQTTSettings.serverAddress = "";

          readString(MQTTSettings.serverAddress);        

          // читаем порт сервера
          if(inSaveMode)
          {
            uint8_t* writePtr = (uint8_t*)&(MQTTSettings.serverPort);
            *writePtr++ = read();
            *writePtr = read();
          }
          else
          {
            writeOut(read());
            writeOut(read());
          }

          // читаем имя пользователя
          if(inSaveMode)
            MQTTSettings.userName = "";

          readString(MQTTSettings.userName);

          // читаем пароль
          if(inSaveMode)
            MQTTSettings.password = "";

          readString(MQTTSettings.password);

          // читаем интервал обновления топиков
          if(inSaveMode)
          {
            uint8_t* writePtr = (uint8_t*)&(MQTTSettings.intervalBetweenTopics);
            *writePtr++ = read();
            *writePtr = read();            
          }
          else
          {
            writeOut(read());
            writeOut(read());
          }
      
      #else

          // тут пропускаем все настройки MQTT
          writeOut(read()); // пропускаем режим работы
          
          // пропускаем ID клиента
          skipString();

          // пропускаем адрес сервера
          skipString();

          // пропускаем порт
          writeOut(read());
          writeOut(read());

          // пропускаем имя пользователя
          skipString();

          // пропускаем пароль
          skipString();

          // пропускаем интервал обновлений топиков
          writeOut(read());
          writeOut(read());
         
      
      #endif // CORE_MQTT_TRANSPORT_ENABLED
    }
    return true;

    case ESPSettingsRecord: // данные о настройках ESP
    {
      
      // имя точки доступа (набор байт, заканчивающийся нулевым байтом)
      #ifdef CORE_ESP_TRANSPORT_ENABLED
      
      ESPTransportSettings.enabled = true;
      
        // сохраняем
        if(!outStream)
          ESPTransportSettings.APName = "";

        readString(ESPTransportSettings.APName);
                  
      #else
        // пропускаем
        skipString();
        
      #endif // CORE_ESP_TRANSPORT_ENABLED

      // пароль точки доступа (набор байт, заканчивающийся нулевым байтом)
      #ifdef CORE_ESP_TRANSPORT_ENABLED
        // сохраняем
        if(!outStream)
          ESPTransportSettings.APPassword = "";

        readString(ESPTransportSettings.APPassword);
        
      #else
        // пропускаем
       skipString();
        
      #endif // CORE_ESP_TRANSPORT_ENABLED
      
      // флаг - коннектиться ли к роутеру
      #ifdef CORE_ESP_TRANSPORT_ENABLED
        uint8_t b = read();
        if(!writeOut(b))
          ESPTransportSettings.Flags.ConnectToRouter = b;
      #else
        writeOut(read()); // пропускаем
      #endif // CORE_ESP_TRANSPORT_ENABLED


      // SSID роутера (набор байт, заканчивающийся нулевым байтом)
      #ifdef CORE_ESP_TRANSPORT_ENABLED
        // сохраняем
        if(!outStream)
          ESPTransportSettings.RouterID = "";

        readString(ESPTransportSettings.RouterID);

      #else
        // пропускаем
       skipString();
        
      #endif // CORE_ESP_TRANSPORT_ENABLED            

      // пароль роутера (набор байт, заканчивающийся нулевым байтом)
      #ifdef CORE_ESP_TRANSPORT_ENABLED
        // сохраняем
        if(!outStream)
          ESPTransportSettings.RouterPassword = "";

        readString(ESPTransportSettings.RouterPassword);
                
      #else
        // пропускаем
        skipString();
        
      #endif // CORE_ESP_TRANSPORT_ENABLED          


      // скорость работы с ESP (1 - 9600, 2 - 19200, 4 - 38400, 6 - 57600, 12 - 115200)
      #ifdef CORE_ESP_TRANSPORT_ENABLED
        // сохраняем
        b = read();
        if(!writeOut(b))
          ESPTransportSettings.UARTSpeed = b;
      #else
        // пропускаем
        writeOut(read());
      #endif // CORE_ESP_TRANSPORT_ENABLED

      // номер Serial, который используется для работы с ESP (1 - Serial1, 2 - Serial2, 3 - Serial 3)
      #ifdef CORE_ESP_TRANSPORT_ENABLED
        // сохраняем
        b = read();
        if(!writeOut(b))
          ESPTransportSettings.SerialNumber = b;
      #else
        // пропускаем
        writeOut(read());
      #endif // CORE_ESP_TRANSPORT_ENABLED
      
      // использовать ли пин пересброса питания при зависании ESP (0 - не использовать, 1 - использовать)
      #ifdef CORE_ESP_TRANSPORT_ENABLED
        // сохраняем
        b = read();
        if(!writeOut(b))
          ESPTransportSettings.Flags.UseRebootPin = b;
      #else
        // пропускаем
        writeOut(read());
      #endif // CORE_ESP_TRANSPORT_ENABLED
      
      // номер пина для пересброса питания ESP
      #ifdef CORE_ESP_TRANSPORT_ENABLED
        // сохраняем
        b = read();
        if(!writeOut(b))
          ESPTransportSettings.RebootPin = b;
      #else
        // пропускаем
        writeOut(read());
      #endif // CORE_ESP_TRANSPORT_ENABLED

      // кол-во секунд, по истечении которых модем считается зависшим (не пришёл ответ на команду)
      #ifdef CORE_ESP_TRANSPORT_ENABLED
        // сохраняем
        b = read();
        if(!writeOut(b))
          ESPTransportSettings.HangTimeout = b;
      #else
        // пропускаем
        writeOut(read());
      #endif // CORE_ESP_TRANSPORT_ENABLED

      // сколько секунд держать питание выключенным при перезагрузке ESP, если он завис
      #ifdef CORE_ESP_TRANSPORT_ENABLED
        // сохраняем
        b = read();
        if(!writeOut(b))
          ESPTransportSettings.HangPowerOffTime = b;
      #else
        // пропускаем
        writeOut(read());
      #endif // CORE_ESP_TRANSPORT_ENABLED
      
      // сколько секунд ждать загрузки модема при инициализации/переинициализации
      #ifdef CORE_ESP_TRANSPORT_ENABLED
        // сохраняем
        b = read();
        if(!writeOut(b))
          ESPTransportSettings.WaitInitTIme = b;
      #else
        // пропускаем
        writeOut(read());
      #endif // CORE_ESP_TRANSPORT_ENABLED

      // уровень для включения питания (1 - HIGH, 0 - LOW)
      #ifdef CORE_ESP_TRANSPORT_ENABLED
        // сохраняем
        b = read();
        if(!writeOut(b))
          ESPTransportSettings.PowerOnLevel = b;
      #else
        // пропускаем
        writeOut(read());
      #endif // CORE_ESP_TRANSPORT_ENABLED
      
      
    }
    return true; // ESPSettingsRecord


    case RS485SettingsRecord: // данные о настройках RS485
    {
    
      #ifdef CORE_RS485_TRANSPORT_ENABLED

        RS485Settings.enabled = true;
        
        uint8_t b = read();
        if(!writeOut(b))
          RS485Settings.UARTSpeed = b;

        b = read();
        if(!writeOut(b))
          RS485Settings.SerialNumber = b;

        b = read();
        if(!writeOut(b))          
          RS485Settings.DEPin = b;

        b = read();
        if(!writeOut(b))          
          RS485Settings.isMasterMode = (b == 1);
        
      #else
        // пропускаем 4 байта
        writeOut(read());
        writeOut(read());
        writeOut(read());
        writeOut(read());
         
      #endif
    }
    return true;

    case LoRaSettingsRecord:
    {
            
      #ifdef CORE_LORA_TRANSPORT_ENABLED
        LoRaSettings.enabled = true;
        
        uint8_t loraB = read();
        if(!writeOut(loraB))
          LoRaSettings.frequency = loraB;

        loraB = read();
        if(!writeOut(loraB))
          LoRaSettings.ss = loraB;

        loraB = read();
        if(!writeOut(loraB))
          LoRaSettings.reset = loraB;

        loraB = read();
        if(!writeOut(loraB))
          LoRaSettings.dio = loraB;

        loraB = read();
        if(!writeOut(loraB))
          LoRaSettings.txPower = loraB;

        loraB = read();
        if(!writeOut(loraB))
          LoRaSettings.bandwidth = loraB;

        loraB = read();
        if(!writeOut(loraB))
          LoRaSettings.useCrc = loraB;

        loraB = read();
        if(!writeOut(loraB))
          LoRaSettings.isMasterMode = loraB;

        loraB = read();
        if(!writeOut(loraB))
          LoRaSettings.retransmitCount = loraB;

        loraB = read();
        if(!writeOut(loraB))
        {
          LoRaSettings.sendDuration = loraB;
          LoRaSettings.sendDuration *= 1000;
        }
          
      #else
        for(uint8_t j=0;j<10;j++) // пропускаем 10 байт (настройки LoRa)
          writeOut(read());
      #endif
    }
    return true;
    
    
  } // switch(b)

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreConfigIterator::applySensorRecord(const String& sensorName, CoreSensorType type,uint8_t* record)
{
  // тут применяем настройки из записи
  CoreSensor* s = CoreSensorsFactory::createSensor(type);
  if(s)
  {
    s->setName(sensorName);
    s->begin(record); // просим датчик прочитать свои настройки
    // добавляем в список
    Core.Sensors()->add(s);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
// CoreEEPROMConfigIterator
//--------------------------------------------------------------------------------------------------------------------------------------
CoreEEPROMConfigIterator::CoreEEPROMConfigIterator() : CoreConfigIterator()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreEEPROMConfigIterator::doRead(uint16_t startAddress, uint16_t addressOffset)
{
  uint16_t addr = startAddress;
  addr += addressOffset;
  return Core.memRead(addr);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSensors::clear()
{
   for(size_t i=0;i<list.size();i++)
   {
    CoreSensor* s = list[i];
    delete s;
   }

   list.empty();
   //while(list.size())
   // list.pop();
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreClass Core;
//--------------------------------------------------------------------------------------------------------------------------------------
CoreClass::CoreClass()
{
	pUnhandled = NULL;
  FractDelimiter = CORE_FRACT_DELIMITER;
  TemperatureUnit = UnitCelsius;
  SensorsUpdateInterval = CORE_SENSORS_UPDATE_INTERVAL;
  DeviceID = 0;
  ClusterID = 0;
  wantRestart = false;
  configSaveAddress = CORE_STORE_ADDRESS;
  serialOwnedFlag = true;
  lastSerialCommandTimer = 0;
  anyCommandFromSerialReceived = false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreClass::crc8(const uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;
  while (len--) 
    {
    uint8_t inbyte = *addr++;
    for (uint8_t i = 8; i; i--)
      {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) 
        crc ^= 0x8C;
      inbyte >>= 1;
      }  // end of for
    }  // end of while
  return crc;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::saveConfig(const uint8_t* address, uint16_t sz, bool isInFlashSource)
{
   uint16_t writeAddress = CORE_STORE_ADDRESS;
   for(uint16_t i=0;i<sz;i++)
   {
      uint8_t b = isInFlashSource ?  pgm_read_byte_near((address + i)) : *(address + i);
      memWrite(writeAddress, b);
      writeAddress++;
   }
  
 
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::loadConfig()
{
  configLoaded = false;
  reset();
  
  CoreEEPROMConfigIterator iter;
  uint16_t sz = 0;

  #if MEMORY_USED == 1
    sz = E2END;
  #elif MEMORY_USED == 2
     sz = 4096-1;
  #elif MEMORY_USED == 3
    sz = 4096*2-1;
  #elif MEMORY_USED == 4
   sz = 4096*4-1;
  #elif MEMORY_USED == 5
   sz = 4096*8-1;
  #elif MEMORY_USED == 6
    sz = 4096*16-1;
  #endif   
  
  if(iter.first(CORE_STORE_ADDRESS,sz))
  {
    while(iter.next());
  }
  else
    return false;

   initSensors();


  lastMillis = SensorsUpdateInterval/2;
  configLoaded = true;

   return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void* CoreClass::memFind(const void *haystack, size_t n, const void *needle, size_t m)
{
        const unsigned char *y = (const unsigned char *)haystack;
        const unsigned char *x = (const unsigned char *)needle;

        size_t j, k, l;

        if (m > n || !m || !n)
                return NULL;

        if (1 != m) {
                if (x[0] == x[1]) {
                        k = 2;
                        l = 1;
                } else {
                        k = 1;
                        l = 2;
                }

                j = 0;
                while (j <= n - m) {
                        if (x[1] != y[j + 1]) {
                                j += k;
                        } else {
                                if (!memcmp(x + 2, y + j + 2, m - 2)
                                    && x[0] == y[j])
                                        return (void *)&y[j];
                                j += l;
                        }
                }
        } else
                do {
                        if (*y == *x)
                                return (void *)y;
                        y++;
                } while (--n);

        return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::reset()
{

  #ifdef CORE_DS18B20_ENABLED // надо попросить диспетчера DS18B20 очистить всех менеджеров линий
    CoreDS18B20Dispatcher.clear();
  #endif  

  #ifdef CORE_RS485_TRANSPORT_ENABLED
    RS485.reset();
  #endif

  #ifdef CORE_LORA_TRANSPORT_ENABLED
    LoraDispatcher.reset();    
  #endif // CORE_LORA_TRANSPORT_ENABLED

  #ifdef CORE_SIGNALS_ENABLED
    Signals.reset();
  #endif

  // сбрасываем настройки ватчдога по умолчанию
  CoreWatchdog.reset();

  #ifdef CORE_MQTT_TRANSPORT_ENABLED
    MQTTSettings.reset();
    MQTT.reset();
  #endif

  #ifdef CORE_THINGSPEAK_TRANSPORT_ENABLED
    ThingSpeakSettings.reset();
    ThingSpeak.reset();
  #endif

  #ifdef CORE_SIM800_TRANSPORT_ENABLED
    SIM800TransportSettings.reset();
  #endif
  
  // очищаем сигналы
  sensorTimers.empty();

  // очищаем список датчиков
  list.clear();


  // очищаем сохранённые данные
  CoreDataStore.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::initSensors()
{
    for(size_t i=0;i<list.size();i++)
    {
      // говорим, что пока нет данных с датчика
      CoreSensor* sensor = list.get(i);
      CoreDataStore.save(sensor,NULL,0);  
    }
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::setCONFIGSTART()
{
  configSaveAddress = CORE_STORE_ADDRESS;

  // поскольку у нас сигналы читаются напрямую из EEPROM - мы должны на время обновления конфига запретить им перечитывать данные
  #ifdef CORE_SIGNALS_ENABLED
    Signals.pause();
  #endif  

  return true;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::setCONFIGPART(const char* param)
{
  
  while(*param)
  {
    uint8_t b = hexStringToByte(param);
    param += 2;

    memWrite(configSaveAddress, b);
    configSaveAddress++;
  }

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreClass::hexStringToByte(const char* buff)
{
  uint8_t tens = makeNum(*buff++);
  uint8_t ones = makeNum(*buff);

   if(ones == 'X') 
    return  0;
    
  return  (tens * 16) + ones;   
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreClass::makeNum(char ch) 
{
  if((ch >= '0') && (ch <= '9'))
    return ((uint8_t) ch) - '0';
  
  switch(ch) 
  {
    case 'A':
    case 'a': return 10;
    
    case 'B':
    case 'b': return 11;
    
    case 'C':
    case 'c': return 12;
    
    case 'D':
    case 'd': return 13;

    case 'E':
    case 'e': return 14;
    
    case 'F':
    case 'f': return 15;
    
    default: return 16;
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::setDATETIME(const char* param)
{
  #ifdef CORE_DS3231_ENABLED

    // разбираем параметр на составные части
    int8_t day = 0;
    int8_t month = 0;
    int16_t year = 0;
    int8_t hour = 0;
    int8_t minute = 0;
    int8_t second = 0;

    // буфер под промежуточные данные
    char workBuff[5] = {0};
    char* writePtr = workBuff;

    // извлекаем день
    const char* delim = strchr(param,'.');
    if(!delim || (delim - param > 4))
      return false;

    while(param < delim)
      *writePtr++ = *param++;
    *writePtr = 0;
    writePtr = workBuff;

    day = atoi(workBuff);

    param = delim+1; // перемещаемся на следующий компонент - месяц
    delim = strchr(param,'.');
    if(!delim || (delim - param > 4))
      return false;

    while(param < delim)
      *writePtr++ = *param++;
    *writePtr = 0;
    writePtr = workBuff;

    month = atoi(workBuff);

    param = delim+1; // перемещаемся на следующий компонент - год
    delim = strchr(param,' ');
    if(!delim || (delim - param > 4))
      return false;

    while(param < delim)
      *writePtr++ = *param++;
    *writePtr = 0;
    writePtr = workBuff;

    year = atoi(workBuff);

    param = delim+1; // перемещаемся на следующий компонент - час
    delim = strchr(param,':');
    if(!delim || (delim - param > 4))
      return false;

    while(param < delim)
      *writePtr++ = *param++;
    *writePtr = 0;
    writePtr = workBuff;

    hour = atoi(workBuff);

    param = delim+1; // перемещаемся на следующий компонент - минута
    delim = strchr(param,':');
    if(!delim || (delim - param > 4))
      return false;

    while(param < delim)
      *writePtr++ = *param++;
    *writePtr = 0;
    writePtr = workBuff;

    minute = atoi(workBuff);

    param = delim+1; // перемещаемся на следующий компонент - секунда

    while(*param && writePtr < &(workBuff[4]))
      *writePtr++ = *param++;
    *writePtr = 0;

    second = atoi(workBuff);

    setCurrentDateTime(day, month, year,hour,minute,second);
    
  return true;
  #else  
    return false;
  #endif // CORE_DS3231_ENABLED
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_DS3231_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::setCurrentDateTime(uint8_t day, uint8_t month, uint16_t year, uint8_t hour, uint8_t minute, uint8_t second)
{
   // вычисляем день недели
    int16_t dow;
    uint8_t mArr[12] = {6,2,2,5,0,3,5,1,4,6,2,4};
    dow = (year % 100);
    dow = dow*1.25;
    dow += day;
    dow += mArr[month-1];
    
    if (((year % 4)==0) && (month<3))
     dow -= 1;
     
    while (dow>7)
     dow -= 7;  


    // получили список DS3231 в системе
    CoreSensorsList rtcSensors = list.getByType(DS3231);
    for(size_t i=0;i<rtcSensors.size();i++)
    {
      // теперь каждому девайсу устанавливаем время
      CoreSensorDS3231* sensor = (CoreSensorDS3231*) rtcSensors[i];
      sensor->setTime(second, minute, hour, dow, day, month, year);
    }
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_DS3231_ENABLED     
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::getFEATURES(const char* commandPassed, Stream* pStream)
{
  if(commandPassed)
  {
      pStream->print(CORE_COMMAND_ANSWER_OK);
      pStream->print(commandPassed);
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);    
  }

  int16_t written = 0;
  #ifdef CORE_ESP_TRANSPORT_ENABLED
   pStream->print(F("ESP")); 
   written++;
  #endif

  #ifdef CORE_RS485_TRANSPORT_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("RS485")); 
  #endif

  #ifdef CORE_LORA_TRANSPORT_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("LORA")); 
  #endif

  #ifdef CORE_SD_SUPPORT_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("SD")); 
  #endif
  
  #ifdef CORE_SIGNALS_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("SIG")); 
  #endif


  #ifdef CORE_MQTT_TRANSPORT_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("MQTT")); 
  #endif

  #ifdef CORE_SIM800_TRANSPORT_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("SIM800")); 
  #endif
 
  #ifdef CORE_THINGSPEAK_TRANSPORT_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("TS")); 
  #endif

  pStream->println();

  return true;
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::getFREERAM(const char* commandPassed, Stream* pStream)
{
  if(commandPassed)
  {
      pStream->print(CORE_COMMAND_ANSWER_OK);
      pStream->print(commandPassed);
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);    
  }

  pStream->println(getFreeMemory());

  return true;
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::getSTORAGE(const char* commandPassed, Stream* pStream)
{
   if(commandPassed)
  {
      pStream->print(CORE_COMMAND_ANSWER_OK);
      pStream->print(commandPassed);
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);    
  } 

  // проходим по всем датчикам хранилища
  for(size_t i=0;i<CoreDataStore.size();i++)
  {
    CoreStoredData dt = CoreDataStore.get(i);
    
    // выводим имя датчика
    String sensorName = dt.sensor->getName();

    for(size_t j=0;j<sensorName.length();j++)
    {
      pStream->print(byteToHexString(sensorName[j]));
    } // for

    // выводим завершающий 0 после имени
      pStream->print(byteToHexString(0));

   // выводим тип железки датчика
   pStream->print(byteToHexString(dt.sensor->getType()));

   // выводим тип показаний датчика
  #ifdef CORE_USERDATA_SENSOR_ENABLED 
   if(dt.sensor->isUserDataSensor())
   {
      CoreUserDataSensor* uds = (CoreUserDataSensor*) dt.sensor;
      pStream->print(byteToHexString(uds->getUserDataType()));
   }
   else
  #endif 
   {
    pStream->print(byteToHexString(CoreSensor::getDataType(dt.sensor->getType())));
   }

   // выводим длину данных датчика
    if(dt.hasData())
    {
      pStream->print(byteToHexString(dt.dataSize));
      // выводим данные датчика
      for(uint8_t k=0;k<dt.dataSize;k++)
      {
        pStream->print(byteToHexString(dt.data[k]));
      }
    }
    else
    {
      // нет данных с датчика
      pStream->print(byteToHexString(0));
    }


      
    
  } // for


  pStream->println();
  
  return true;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::getSIGNALS(const char* commandPassed, Stream* pStream)
{
   if(commandPassed)
  {
      pStream->print(CORE_COMMAND_ANSWER_OK);
      pStream->print(commandPassed);
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);    
  }

   #ifdef CORE_SIGNALS_ENABLED
      for(uint8_t i=0;i<CORE_SIGNAL_BYTES;i++)
      {
        pStream->print(byteToHexString(SIGNALS[i]));
      }
   #endif

   pStream->println();

   return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::getCONFIG(const char* commandPassed, Stream* pStream)
{
  if(commandPassed)
  {
      pStream->print(CORE_COMMAND_ANSWER_OK);
      pStream->print(commandPassed);
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);    
  }

  CoreEEPROMConfigIterator iter;
  uint16_t sz = 0;

  #if MEMORY_USED == 1
    sz = E2END;
  #elif MEMORY_USED == 2
     sz = 4096-1;
  #elif MEMORY_USED == 3
    sz = 4096*2-1;
  #elif MEMORY_USED == 4
   sz = 4096*4-1;
  #elif MEMORY_USED == 5
   sz = 4096*8-1;
  #elif MEMORY_USED == 6
    sz = 4096*16-1;
  #endif   
  
  if(iter.first(CORE_STORE_ADDRESS,sz,pStream,true))
  {
    while(iter.next());
  }

  pStream->println();
  
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::getCPU(const char* commandPassed, Stream* pStream)
{
  if(commandPassed)
  {
      pStream->print(CORE_COMMAND_ANSWER_OK);
      pStream->print(commandPassed);
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);    
  }

  #if TARGET_BOARD == MEGA_BOARD
    pStream->println(F("MEGA"));    
  #elif TARGET_BOARD == DUE_BOARD
    pStream->println(F("DUE"));
  #elif TARGET_BOARD == ESP_BOARD
    pStream->println(F("ESP"));
  #elif TARGET_BOARD == ATMEGA328_BOARD
    pStream->println(F("328"));
  #else
    pStream->println(F("NOP"));
    #error "Unknown target board!"
  #endif


  return true;
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::getSENSORS(const char* commandPassed, Stream* pStream)
{

  if(commandPassed)
  {
      pStream->print(CORE_COMMAND_ANSWER_OK);
      pStream->print(commandPassed);
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);    
  }
  
  int16_t written = 0;
  #ifdef CORE_BH1750_ENABLED
   pStream->print(F("BH1750")); 
   written++;
  #endif

  #ifdef CORE_SI7021_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("SI7021")); 
  #endif

  #ifdef CORE_DS3231_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("DS3231")); 
  #endif

  #ifdef CORE_DHT_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("DHT")); 
  #endif

  #ifdef CORE_DS18B20_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("DS18B20")); 
  #endif  

  #ifdef CORE_DIGITALPORT_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("DPORT")); 
  #endif  

  #ifdef CORE_ANALOGPORT_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("APORT")); 
  #endif  

  #ifdef CORE_USERDATA_SENSOR_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("USENSOR")); 
  #endif  

  #ifdef CORE_MAX6675_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("MAX6675")); 
  #endif  
  
  #ifdef CORE_BMP180_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("BMP180")); 
  #endif  

  #ifdef CORE_MAX44009_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("MAX44009")); 
  #endif  

  #ifdef CORE_HCSR04_ENABLED
    if(written)
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      
    written++;
    pStream->print(F("HCSR04")); 
  #endif  

  pStream->println();

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::getDATETIME(const char* commandPassed, Stream* pStream)
{
  // запросили получение времени/даты
  bool commandHandled = false;
  
  CoreDataList allDateTime = CoreDataStore.getByType(DateTime);
  for(size_t k=0;k<allDateTime.size();k++)
  {
    CoreStoredData dataStored = allDateTime[k];
    if(dataStored.hasData())
    {
      commandHandled = true;
      CoreTextFormatProvider textFormatter;
      String data = textFormatter.format(dataStored,k,true);
      // получили данные в текстовом виде, выводим из в тот поток, который слушает класс команд
      pStream->print(CORE_COMMAND_ANSWER_OK);
      pStream->print(commandPassed);
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);
      pStream->println(data);
      break;
    }
  } // for

  if(!commandHandled)
  {
    // нет ни одного датчика
    pStream->print(CORE_COMMAND_ANSWER_ERROR);
    pStream->print(commandPassed);
    pStream->print(CORE_COMMAND_PARAM_DELIMITER);
    pStream->println(F("NO_DATA"));

    commandHandled = true;
  }  

  return commandHandled;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::printBackSETResult(bool isOK, const char* command, Stream* pStream)
{
  if(isOK)
    pStream->print(CORE_COMMAND_ANSWER_OK);
  else
    pStream->print(CORE_COMMAND_ANSWER_ERROR);

  pStream->print(command);
  pStream->print(CORE_COMMAND_PARAM_DELIMITER);

  if(isOK)
    pStream->println(F("OK"));
  else
    pStream->println(F("BAD_PARAMS"));

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
// Список поддерживаемых команд
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_DS3231_ENABLED
const char DATETIME_COMMAND[] PROGMEM = "DATETIME"; // получить/установить дату/время на контроллер
#endif
const char SENSORS_COMMAND[] PROGMEM = "SENSORS"; // получить информацию о поддерживаемых датчиках
const char FEATURES_COMMAND[] PROGMEM = "FEATURES"; // получить информацию о поддерживаемых возможностях
const char FREERAM_COMMAND[] PROGMEM = "FREERAM"; // получить информацию о свободной памяти
const char CPU_COMMAND[] PROGMEM = "CPU"; // получить информацию о МК
const char CONFIG_COMMAND[] PROGMEM = "CONFIG"; // получить конфиг
const char CONFIGSTART_COMMAND[] PROGMEM = "CONFIG_START"; // начать сохранение конфига
const char CONFIGPART_COMMAND[] PROGMEM = "CONFIG_PART"; // записать часть конфига
const char STORAGE_COMMAND[] PROGMEM = "STORAGE"; // получить показания со всех датчиков хранилища
const char RESTART_COMMAND[] PROGMEM = "RESTART"; // перезапустить ядро
const char PIN_COMMAND[] PROGMEM = "PIN"; // установить уровень на пине
#ifdef CORE_SD_SUPPORT_ENABLED
const char LS_COMMAND[] PROGMEM = "LS"; // отдать список файлов
const char FILE_COMMAND[] PROGMEM = "FILE"; // отдать содержимое файла
#endif
#ifdef CORE_SIGNALS_ENABLED
const char SIGNALS_COMMAND[] PROGMEM = "SIG"; // получить статус сигналов
#endif

#ifdef CORE_ESP_TRANSPORT_ENABLED
const char ESP_COMMAND[] PROGMEM = "ESP"; // запрос к модулю ESP
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::processCommand(const String& command,Stream* pStream)
{
    bool commandHandled = false;

    if(command.startsWith(CORE_COMMAND_SET))
    {
      // команда на установку свойств

      CommandParser cParser;
      if(cParser.parse(command,true))
      {
        const char* commandName = cParser.getArg(0);

        if(!strcmp_P(commandName, RESTART_COMMAND))
        {
            // запросили начать запись конфига
            commandHandled = printBackSETResult(setRESTART(),commandName,pStream);
        } // RESTART_COMMAND
        else
        if(!strcmp_P(commandName, PIN_COMMAND))
        {
            // запросили установить уровень на пине SET=PIN|13|ON, SET=PIN|13|1, SET=PIN|13|OFF, SET=PIN|13|0, SET=PIN|13|ON|2000 
            if(cParser.argsCount() > 2)
            {
              commandHandled = setPIN(cParser, pStream);
            }
            else
            {
              // недостаточно параметров
              commandHandled = printBackSETResult(false,commandName,pStream);
            }
        } // PIN_COMMAND        
        else
        if(!strcmp_P(commandName, CONFIGSTART_COMMAND))
        {
            // запросили начать запись конфига
              commandHandled = printBackSETResult(setCONFIGSTART(),commandName,pStream);
        } // CONFIGSTART_COMMAND
        else
        if(!strcmp_P(commandName, CONFIGPART_COMMAND))
        {
            // запросили записать часть конфига
          if(cParser.argsCount() > 1)
          {
            const char* paramPtr = cParser.getArg(1);
            commandHandled = printBackSETResult(setCONFIGPART(paramPtr),commandName,pStream);
          }
          else
          {
            // недостаточно параметров
            commandHandled = printBackSETResult(false,commandName,pStream);
          }
        } // CONFIGPART_COMMAND        
        #ifdef CORE_DS3231_ENABLED
        else
        if(!strcmp_P(commandName, DATETIME_COMMAND)) // DATETIME
        {
          if(cParser.argsCount() > 1)
          {
          // запросили установку даты/времени, приходит строка вида 25.12.2017 12:23:49
            const char* paramPtr = cParser.getArg(1);
            commandHandled = printBackSETResult(setDATETIME(paramPtr),commandName,pStream);
          }
          else
          {
            // недостаточно параметров
            commandHandled = printBackSETResult(false,commandName,pStream);
          }
                    
        } // DATETIME
        #endif // CORE_DS3231_ENABLED
      
      
      //TODO: тут разбор команды !!!
      
      } // if(cParser.parse(command,true))
      
    } // SET COMMAND
    else
    if(command.startsWith(CORE_COMMAND_GET))
    {
      // команда на получение свойств
      CommandParser cParser;
      
      if(cParser.parse(command,false))
      {
        const char* commandName = cParser.getArg(0);
        #ifdef CORE_DS3231_ENABLED
        if(!strcmp_P(commandName, DATETIME_COMMAND))
        {
          commandHandled = getDATETIME(commandName,pStream);
                    
        } // DATETIME_COMMAND
        else
        #endif // CORE_DS3231_ENABLED
        if(!strcmp_P(commandName, SENSORS_COMMAND))
        {
          commandHandled = getSENSORS(commandName,pStream);
                    
        } // SENSORS_COMMAND
         else
        if(!strcmp_P(commandName, PIN_COMMAND))
        {
            commandHandled = getPIN(commandName,cParser,pStream);                    
          
        } // PIN_COMMAND       
        else
        if(!strcmp_P(commandName, FEATURES_COMMAND))
        {
          commandHandled = getFEATURES(commandName,pStream);
        } // FEATURES_COMMAND
        else
        if(!strcmp_P(commandName, FREERAM_COMMAND))
        {
          commandHandled = getFREERAM(commandName,pStream);
        } // FREERAM_COMMAND
        else
        if(!strcmp_P(commandName, CPU_COMMAND))
        {
          commandHandled = getCPU(commandName,pStream);
        } // CPU_COMMAND
        else
        if(!strcmp_P(commandName, CONFIG_COMMAND))
        {
          commandHandled = getCONFIG(commandName,pStream);
        } // CONFIG_COMMAND
        else
        if(!strcmp_P(commandName, STORAGE_COMMAND))
        {
          commandHandled = getSTORAGE(commandName,pStream);
        } // STORAGE_COMMAND
        #ifdef CORE_SIGNALS_ENABLED
        else
        if(!strcmp_P(commandName, SIGNALS_COMMAND))
        {
          commandHandled = getSIGNALS(commandName,pStream);
        } // STORAGE_COMMAND
        #endif
        #ifdef CORE_SD_SUPPORT_ENABLED
        else
        if(!strcmp_P(commandName, LS_COMMAND)) // LS
        {
            // запросили получить список файлов в папке, GET=LS|FolderName
            commandHandled = getLS(commandName,cParser,pStream);                    
        } // LS        
        else
        if(!strcmp_P(commandName, FILE_COMMAND)) // FILE
        {
            // запросили получить список файлов в папке, GET=FILE|FilePath
            commandHandled = getFILE(commandName,cParser,pStream);                    
        } // LS        
        #endif // CORE_SD_SUPPORT_ENABLED 

        #ifdef CORE_ESP_TRANSPORT_ENABLED
        else
        if(!strcmp_P(commandName, ESP_COMMAND)) // ESP
        {
          commandHandled = getESP(commandName,cParser,pStream); 
        }
        #endif //CORE_ESP_TRANSPORT_ENABLED
        
        //TODO: тут разбор команды !!!
        
      } // if(cParser.parse(command,false))
      
    } // GET COMMAND
    
    if(!commandHandled && pUnhandled)
      pUnhandled(command, pStream);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_ESP_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::getESP(const char* commandPassed, const CommandParser& parser, Stream* pStream)
{
  if(parser.argsCount() < 2)
    return false;


   const char* whichCommand = parser.getArg(1);

   if(!strcasecmp_P(whichCommand,(const char*) F("IP")))
   {
      // получить информацию о IP
      String staIP, apIP;
      bool res = ESP.getIP(staIP, apIP);
      
      if(res)
        pStream->print(CORE_COMMAND_ANSWER_OK);
      else
        pStream->print(CORE_COMMAND_ANSWER_ERROR);
      
      pStream->print(commandPassed);
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);    
      pStream->print(whichCommand);
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);

      if(!res)
      {
        pStream->println(F("BUSY"));
      }
      else
      {
        pStream->print(staIP);
        pStream->print(CORE_COMMAND_PARAM_DELIMITER);
        pStream->println(apIP);       
      }

      return true;
      
   } // IP command
   else
   if(!strcasecmp_P(whichCommand,(const char*) F("MAC")))
   {
      // получить информацию о MAC-адресах
      String staMAC, apMAC;
      bool res = ESP.getMAC(staMAC, apMAC);
      
      if(res)
        pStream->print(CORE_COMMAND_ANSWER_OK);
      else
        pStream->print(CORE_COMMAND_ANSWER_ERROR);
      
      pStream->print(commandPassed);
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);    
      pStream->print(whichCommand);
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);

      if(!res)
      {
        pStream->println(F("BUSY"));
      }
      else
      {
        pStream->print(staMAC);
        pStream->print(CORE_COMMAND_PARAM_DELIMITER);
        pStream->println(apMAC);       
      }

      return true;    
   } // MAC command
   else
   if(!strcasecmp_P(whichCommand,(const char*) F("PING")))
   {
      // пропинговать google.com
      bool pingSucceeded = false;
      bool res = ESP.pingGoogle(pingSucceeded);
      
      if(res)
        pStream->print(CORE_COMMAND_ANSWER_OK);
      else
        pStream->print(CORE_COMMAND_ANSWER_ERROR);
      
      pStream->print(commandPassed);
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);    
      pStream->print(whichCommand);
      pStream->print(CORE_COMMAND_PARAM_DELIMITER);

      if(!res)
      {
        pStream->println(F("BUSY"));
      }
      else
      {
        if(pingSucceeded)
          pStream->println(F("PING_OK"));
       else
          pStream->println(F("NO_PING"));
      }

      return true;        
   } // PING command

   return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_ESP_TRANSPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_SD_SUPPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::getFILE(const char* commandPassed, const CommandParser& parser, Stream* pStream)
{
  String endOfFile = CORE_END_OF_DATA;
  if(parser.argsCount() > 1)
  {
    String fileName;

    for(size_t i=1;i<parser.argsCount();i++)
    {
      if(fileName.length())
        fileName += F("/");

      fileName += parser.getArg(i);
    }
    
    FileUtils::printFile(fileName,pStream);
    pStream->println(endOfFile);
  }
  else
  {
    pStream->println(endOfFile);
  }
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::getLS(const char* commandPassed, const CommandParser& parser, Stream* pStream)
{
  String folderName = F("/");
  
  if(parser.argsCount() > 1)
  {
    folderName = "";
    for(size_t i=1;i<parser.argsCount();i++)
    {
      if(folderName.length())
        folderName += F("/");

      folderName += parser.getArg(i);
    }    
  }

  FileUtils::printFilesNames(folderName,false,pStream);
  pStream->println(CORE_END_OF_DATA);
  
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_SD_SUPPORT_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::getPIN(const char* commandPassed, const CommandParser& parser, Stream* pStream)
{
  if(parser.argsCount() < 2)
    return false;  

   int16_t pinNumber = atoi(parser.getArg(1));   
   int16_t pinState = getPinState(pinNumber);

  pStream->print(CORE_COMMAND_ANSWER_OK);

  pStream->print(commandPassed);
  pStream->print(CORE_COMMAND_PARAM_DELIMITER);
  pStream->print(pinNumber);
  pStream->print(CORE_COMMAND_PARAM_DELIMITER);
  pStream->println(pinState ? F("ON") : F("OFF"));   

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::setPIN(CommandParser& parser, Stream* pStream)
{

  if(parser.argsCount() < 3)
    return false;
  
  int16_t pinNumber = atoi(parser.getArg(1));
  const char* level = parser.getArg(2);
  
  bool isHigh = !strcasecmp_P(level,(const char*) F("ON")) || *level == '1';

  pinMode(pinNumber,OUTPUT);
  digitalWrite(pinNumber,isHigh);

  pStream->print(CORE_COMMAND_ANSWER_OK);

  pStream->print(parser.getArg(0));
  pStream->print(CORE_COMMAND_PARAM_DELIMITER);
  pStream->print(pinNumber);
  pStream->print(CORE_COMMAND_PARAM_DELIMITER);
  pStream->println(level);

  // если есть ещё один параметр - значит, надо держать сигнал определённое время, например, SET=PIN|13|ON|2000
  if(parser.argsCount() > 3)
  {
    uint32_t raiseDelay = atol(parser.getArg(3));
    CoreDelayedEvent.raise(raiseDelay,CoreDelayedEventClass::CoreDelayedEventPinChange,(void*) new CoreDelayedEventPinChangeArg(pinNumber,!isHigh));
  }
  

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::setRESTART()
{
  wantRestart = true;
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::handleCommands()
{
  if(!anyCommandFromSerialReceived)
  {
    if(serialOwnedFlag && millis() - lastSerialCommandTimer > CORE_RELEASE_SERIAL_DELAY)
    {
      // нет команд в течение N секунд, отпускаем Serial
      serialOwnedFlag = false;
      DBGLN(F("No incoming commands within 5 seconds, release Serial!"));
      beginSerialRelatedTransports();
    }
  }

  if(!serialOwnedFlag) // мы больше не владеем Serial, поэтому не должны ничего из него читать
    return;
  
  if(Commands.hasCommand())
  {    

    String command = Commands.getCommand();

    if(command.startsWith(CORE_COMMAND_GET) || command.startsWith(CORE_COMMAND_SET))
    {
      anyCommandFromSerialReceived = true;    
      Stream* pStream = Commands.getStream();
      processCommand(command,pStream);
    }
    

    Commands.clearCommand(); // очищаем буфер команд
  
  } // if(Commands.hasCommand())
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
#if TARGET_BOARD == DUE_BOARD // Arduino Due compatible
    #include <malloc.h>
    #include <stdlib.h>
    #include <stdio.h>
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
int CoreClass::getFreeMemory()
{
  #if TARGET_BOARD == MEGA_BOARD || TARGET_BOARD == ATMEGA328_BOARD
  
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
    
  #elif TARGET_BOARD == DUE_BOARD

    struct mallinfo mi = mallinfo();
    char* heapend = _sbrk(0);
    register char* stack_ptr asm("sp");

    return (stack_ptr - heapend + mi.fordblks);

  #elif TARGET_BOARD == ESP_BOARD
    #error "NOT IMPLEMENTED!!!"    
  #else    
    #error "Unknown target board!"
  #endif   
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::printVersion(Stream& stream)
{
  stream << CORE_VERSION  << ENDL; 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::printSupportedSensors(Stream& s)
{
  getSENSORS(NULL,&s); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::setup(CoreUnhandledCommandsHandler func)
{
   pUnhandled = func;
	 memInit(); // инициализируем память
 
  lastMillis = millis();
 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::addTimer(uint16_t timerDelay,CoreSensor* sensor,uint16_t storeIndex)
{
  if(!timerDelay)
  {
    //DBGLN(F("No timer needed..."));
    readFromSensor(sensor,storeIndex);
    return;
  }

  // тут добавляем в список сигналов
  CoreSensorTimerStruct ss;
  ss.startTimer = millis();
  ss.timerDelay = timerDelay;
  ss.sensor = sensor;
  ss.storeIndex = storeIndex;
  
  sensorTimers.push_back(ss);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::pushToStorage(CoreSensor* s)
{
  for(size_t i=0;i<list.size();i++)
  {
    CoreSensor* sensor = list.get(i);
    if(sensor == s)
    {
      readFromSensor(sensor,i);    
    }
  } // for  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::readFromSensor(CoreSensor* sensor,uint16_t storeIndex)
{
  while(storeIndex >= CoreDataStore.list.size())
  {
    // добавляем в хранилище записи, если необходимо
    CoreDataStore.save(sensor,NULL,0);  
  }
  
  CoreStoredData* stored = &(CoreDataStore.list[storeIndex]);

  // говорим, что с датчика нет данных
  if(stored->data)
    delete [] stored->data;
    
  stored->data = NULL;

  uint8_t sz = sensor->getDataSize();

  if(sz)
  {
    uint8_t* dt = new uint8_t[sz];
    
    if(sensor->read(dt))
    {      
        // на датчике есть данные
        stored->data = dt;
        stored->dataSize = sz;
          
    }   // if
    else
      delete [] dt;
      
  } // if(sz)
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t CoreClass::getPinMode(uint8_t p)
{

  #if (TARGET_BOARD == MEGA_BOARD) || (TARGET_BOARD == DUE_BOARD)
     const uint8_t max_pin = 69;
  #elif TARGET_BOARD == ESP_BOARD
    #error "NOT IMPLEMENTED!!!"  
  #elif TARGET_BOARD == ATMEGA328_BOARD       
     const uint8_t max_pin = 19;
  #else
     #error "Unknown target board!"
  #endif

   if (p > max_pin) 
       return -1;

#if (TARGET_BOARD == MEGA_BOARD) || (TARGET_BOARD == ATMEGA328_BOARD)

  uint8_t port = digitalPinToPort(p);
  uint8_t portBit  = digitalPinToBitMask(p);
  volatile uint8_t* mode =    portModeRegister(port);
  volatile uint8_t* out = portOutputRegister(port);
     
  if ((*mode) & portBit)
  {
    return OUTPUT;
  }
  else
  {     
    if( (*out) & portBit )
    { 
      return INPUT;
    }
    else
    { 
      return INPUT;
    }
  }
#elif (TARGET_BOARD == DUE_BOARD)

  if((g_pinStatus[p] & 0xF) == PIN_STATUS_DIGITAL_INPUT)
  {
    return INPUT;
  }
  else
  if((g_pinStatus[p] & 0xF) == PIN_STATUS_DIGITAL_INPUT_PULLUP)
  {
    return INPUT;
  }
 else
 if((g_pinStatus[p] & 0xF) == PIN_STATUS_DIGITAL_OUTPUT)
 {
    return OUTPUT;
 }

  return INPUT;
    
#endif

return -1;
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t CoreClass::getPinState(uint8_t pin)
{
#if (TARGET_BOARD == MEGA_BOARD) || (TARGET_BOARD == ATMEGA328_BOARD)
  
  int16_t mode = getPinMode(pin);
  if(mode == -1)
    return mode;

  uint8_t port = digitalPinToPort(pin);
  uint8_t portBit = digitalPinToBitMask(pin);

  if(mode == INPUT)
  {
    volatile uint8_t* pir = portInputRegister(port);
    return (*pir & portBit) ? HIGH : LOW;    
  }
  else
  {
    volatile uint8_t* por = portOutputRegister(port);
    return (*por & portBit) ? HIGH : LOW;
  }

#elif (TARGET_BOARD == DUE_BOARD)

  return digitalRead(pin);

#endif

  return LOW;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreClass::isOnTimer(CoreSensor* sensor)
{
  for(size_t i = 0;i < sensorTimers.size(); i++)
  {
    if(sensorTimers[i].sensor == sensor)
    {
      DBGLN(F("IN MEASURE..."));
      return true;
    }
  }

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::updateTimers()
{
  uint32_t curMillis = millis();
  
  for(size_t i = 0;i < sensorTimers.size();)
  {
    if(curMillis - sensorTimers[i].startTimer > sensorTimers[i].timerDelay)
    {
      // можно читать с датчика
      //DBGLN(F("TIMER!"));
      readFromSensor(sensorTimers[i].sensor,sensorTimers[i].storeIndex);

      // теперь сдвигаем всё к голове
      for(size_t k=i+1;k<sensorTimers.size();k++)
      {
         sensorTimers[k-1] = sensorTimers[k];
      }

      // убираем одну позицию
      sensorTimers.pop();

    } // if
    else
    {
       
      // время конвертации не закончилось, надо обновить датчик, вдруг он внутри себя что-то захочет сделать, во время конвертации,
      // например, если датчику требуется 10 семплов за 10 секунд - он при вызове startMeasure может взводить таймер, а в вызовах
      // update проверять, и каждую секунду делать семпл
      sensorTimers[i].sensor->update();
      
      i++;

    }
  } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
const char HEX_CHARS[]  PROGMEM = {"0123456789ABCDEF"};
//--------------------------------------------------------------------------------------------------------------------------------------
const char* CoreClass::byteToHexString(byte i)
{
  static char HEX_HOLDER[3] = {0}; // холдер для шестнадцатеричного представления байта в строковом виде
  
  int16_t idx = i & 0xF;
  char char1 = (char) pgm_read_byte_near( HEX_CHARS + idx );
  i>>=4;
  idx = i & 0xF;
  char char2 = (char) pgm_read_byte_near( HEX_CHARS + idx );
  
  HEX_HOLDER[0] = char2;
  HEX_HOLDER[1] = char1;
  
  //return Out; 
  return HEX_HOLDER;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::beginSerialRelatedTransports()
{ 
    // тут мы должны стартовать транспорты, только если они используют Serial. Иначе они уже стартовали, и незачем их дёргать лишний раз
  
  #ifdef CORE_RS485_TRANSPORT_ENABLED
    if(RS485Settings.enabled && RS485Settings.SerialNumber == 0)
    {
//      DBGLN(F("RS485 linked to Serial, restart it..."));
      RS485.begin();
    }
  #endif

  // сначала выясняем, надо ли нам резетить MQTT и ThingSpeak. Это надо делать в том случае,
  // если хотя бы один транспорт (ESP или SIM800) - рестартует, т.к. привязан к Serial.

  bool wantResetTransportClients = false;
  
  #ifdef CORE_ESP_TRANSPORT_ENABLED
    if(ESPTransportSettings.enabled && ESPTransportSettings.SerialNumber == 0)
      wantResetTransportClients = true;
  #endif

  #ifdef CORE_SIM800_TRANSPORT_ENABLED
    if(SIM800TransportSettings.enabled && SIM800TransportSettings.SerialNumber == 0)
    {
        wantResetTransportClients = true;
    }
  #endif

  // теперь резетим клиенты транспортов, при этом они освободят ссылки на транспорт и его клиентов
  if(wantResetTransportClients)
  {
    #ifdef CORE_MQTT_TRANSPORT_ENABLED
      MQTT.reset();
    #endif
  
    #ifdef CORE_THINGSPEAK_TRANSPORT_ENABLED
      ThingSpeak.reset();
    #endif    
  }
  
  bool isAnyTransportRestarted = false;

  #ifdef CORE_ESP_TRANSPORT_ENABLED
    if(ESPTransportSettings.enabled && ESPTransportSettings.SerialNumber == 0)
    {
//      DBGLN(F("ESP linked to Serial, restart it..."));
      isAnyTransportRestarted = true;
      ESP.begin();
    }
  #endif

  #ifdef CORE_SIM800_TRANSPORT_ENABLED
    if(SIM800TransportSettings.enabled && SIM800TransportSettings.SerialNumber == 0)
    {
//      DBGLN(F("SIM800 linked to Serial, restart it..."));
      isAnyTransportRestarted = true;
      SIM800.begin();
    }
  #endif 

  // тут надо рестартовать клиентов транспортов, если хотя бы один транспорт рестартован.
  // Это связано с тем, что клиенты держат в себе класс клиента транспорта, и он будет невалидным
  // пре рестарте транспорта.
  if(isAnyTransportRestarted)
  {
    #ifdef CORE_MQTT_TRANSPORT_ENABLED
      MQTT.begin();
    #endif
  
    #ifdef CORE_THINGSPEAK_TRANSPORT_ENABLED
      ThingSpeak.begin();
    #endif    
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::begin()
{
  if(!configLoaded)
    return;

  lastSerialCommandTimer = millis();

  #ifdef CORE_SD_SUPPORT_ENABLED
  
    SDSettings.CSPin = CORE_SD_CS_PIN;

    
    pinMode(SDSettings.CSPin,OUTPUT);
    digitalWrite(SDSettings.CSPin,LOW);
    
    #ifdef CORE_SD_USE_SDFAT

      bool sdInited = false;
    
      DBGLN(F("Init SD using SdFat..."));
      
      delay(400);
      
      for(uint8_t i=0;i<5;i++)
      {
        if(SD.begin(SDSettings.CSPin, CORE_SD_SDFAT_SPEED))
        {
          sdInited = true;
          break;
        }
        else
          delay(200);
       
      } // for

        if(sdInited)
        {
          DBGLN(F("SD inited."));
        }
        else
        {
           DBGLN(F("SD init error!"));
        }      
    #else
    
      DBGLN(F("Init SD..."));
      
      if(SD.begin(SDSettings.CSPin))
      {
         DBGLN(F("SD inited."));
      }
      else
      {
         DBGLN(F("SD init error!"));
      }
    #endif
  #endif // CORE_SD_SUPPORT_ENABLED

  #ifdef CORE_RS485_TRANSPORT_ENABLED
      RS485.begin();
  #endif  

  #ifdef CORE_ESP_TRANSPORT_ENABLED
    ESP.begin();
  #endif

  #ifdef CORE_SIM800_TRANSPORT_ENABLED
    SIM800.begin();
  #endif 

  #ifdef CORE_LORA_TRANSPORT_ENABLED

  LoraDispatcher.begin();
    
  #endif // CORE_LORA_TRANSPORT_ENABLED

  #ifdef CORE_SIGNALS_ENABLED
    Signals.begin();
    Signals.resume();
  #endif

  #ifdef CORE_MQTT_TRANSPORT_ENABLED
    MQTT.begin();
  #endif

  #ifdef CORE_THINGSPEAK_TRANSPORT_ENABLED
    ThingSpeak.begin();
  #endif


  ON_CORE_BEGIN();

  printVersion(Serial);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::yieldCritical()
{
  CoreWatchdog.update();
  
  #ifdef CORE_ESP_TRANSPORT_ENABLED
    ESP.update();
  #endif

  #ifdef CORE_SIM800_TRANSPORT_ENABLED
    SIM800.update();
  #endif  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::update()
{
  if(wantRestart) // запросили перезапуск ядра
  {
    wantRestart = false;

    // если конфиг загружен - перезапускаемся
    if(loadConfig())
    {
      lastMillis = 0;
      begin();
    }
      
    return;
  }

  CoreWatchdog.update();
  CoreDelayedEvent.update();

  #ifdef CORE_RS485_TRANSPORT_ENABLED
    // обновляем транспорт RS-485
    RS485.update();
  #endif

  #ifdef CORE_ESP_TRANSPORT_ENABLED
    ESP.update();
  #endif

  #ifdef CORE_SIM800_TRANSPORT_ENABLED
    SIM800.update();
  #endif

  #ifdef CORE_LORA_TRANSPORT_ENABLED
    LoraDispatcher.update();    
  #endif // CORE_LORA_TRANSPORT_ENABLED  

  updateTimers(); // обновляем таймеры

  #ifdef CORE_SIGNALS_ENABLED
    Signals.update();
  #endif

  #ifdef CORE_MQTT_TRANSPORT_ENABLED
    MQTT.update();
  #endif

  #ifdef CORE_THINGSPEAK_TRANSPORT_ENABLED
    ThingSpeak.update();
  #endif

  
  uint32_t curMillis = millis();

  #ifdef CORE_DIGITALPORT_ENABLED
  // тут проходим по всем датчикам, и для датчиков слежения за цифровыми портами обновляем их состояние постоянно, вне зависимости от выставленного интервала
  for(size_t i=0;i<list.size();i++)
  {
    CoreSensor* sensor = list.get(i);
    if(sensor->getType() == DigitalPortState)
    {
      readFromSensor(sensor,i);    
    }
  } // for
  #endif // CORE_DIGITALPORT_ENABLED


  #ifdef CORE_DS3231_ENABLED
  // с датчиков DS3231 надо обновлять показания каждую секунду, вне зависимости от выставленного интервала
  static uint32_t ds3231UpdateMillis = 0;
  if(!ds3231UpdateMillis || curMillis - ds3231UpdateMillis > 1000)
  {
    ds3231UpdateMillis = curMillis;
    for(size_t i=0;i<list.size();i++)
    {
      CoreSensor* sensor = list.get(i);
      if(sensor->getType() == DS3231) // только с часов реального времени обновляем раз в секунду
      {
        readFromSensor(sensor,i);    
      }
    } // for    
  }
 #endif // CORE_DS3231_ENABLED
 
  if(!lastMillis || curMillis - lastMillis > SensorsUpdateInterval)
  {
    
    lastMillis = curMillis;
    // тут можно обновлять показания
    
    for(size_t i=0;i<list.size();i++)
    {
      // говорим, что пока нет данных с датчика
      CoreSensor* sensor = list.get(i);

      #ifdef CORE_DS3231_ENABLED
      if(sensor->getType() != DS3231)
      {
        // игнорируем часы реального времени - они обновляются отдельно
      #endif // CORE_DS3231_ENABLED        

        if(!isOnTimer(sensor)) // если на датчик не взведён таймер
        {
          // запускаем конвертацию на датчике
          uint16_t readingInterval = sensor->startMeasure();
    
          // теперь мы знаем, через какое время читать данные с датчика - можно взводить таймер
          addTimer(readingInterval,sensor,i);
        }

      #ifdef CORE_DS3231_ENABLED
      } // if(sensor->getType() != DS3231)
      #endif // CORE_DS3231_ENABLED

    } // for

  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::memInit()
{
#if MEMORY_USED == 1
  // не надо инициализировать дополнительно
#elif MEMORY_USED == 2
   memory = new AT24C32();
#elif MEMORY_USED == 3
  memory = new AT24C64();
#elif MEMORY_USED == 4
 memory = new AT24C128();
#elif MEMORY_USED == 5
 memory = new AT24C256();
#elif MEMORY_USED == 6
  memory = new AT24C512();
#endif    
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t CoreClass::memRead(uint16_t address)
{
  #if MEMORY_USED == 1
    return EEPROM.read(address);
  #else
    return memory->read(address);
  #endif      

}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreClass::memWrite(uint16_t address, uint8_t val)
{
  #if MEMORY_USED == 1
    if(memRead(address) != val)
      EEPROM.write(address, val);
  #else
    if(memRead(address) != val)
      memory->write(address,val);
  #endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
// CoreStoredData
//--------------------------------------------------------------------------------------------------------------------------------------
CoreStoredData::operator LuminosityData() const
{
  LuminosityData result;  
  if(!hasData())
    return result;
    
  if(dataSize < sizeof(LuminosityData))
    return result;

  memcpy(&result,data,sizeof(LuminosityData));
  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreStoredData::operator DistanceData() const
{
  DistanceData result;
    
  if(!hasData())
    return result;
    
  if(dataSize < sizeof(DistanceData))
    return result;

  memcpy(&result,data,sizeof(DistanceData));
  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreStoredData::operator DigitalPortData() const
{
  DigitalPortData result;
  
  if(!hasData())
    return result;
    
  if(dataSize < sizeof(DigitalPortData))
    return result;

  memcpy(&result,data,sizeof(DigitalPortData));

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreStoredData::operator AnalogPortData() const
{
  AnalogPortData result;
  
  if(!hasData())
    return result;
    
  if(dataSize < sizeof(AnalogPortData))
    return result;

  memcpy(&result,data,sizeof(AnalogPortData));
  
  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreStoredData::operator TemperatureData() const
{
  TemperatureData result;

  if(!hasData())
  {
    return result;
  }

  if(dataSize < sizeof(TemperatureData))
  {
    return result;
  }

  memcpy(&result,data,sizeof(TemperatureData));

   return result;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreStoredData::operator DateTimeData() const
{
  DateTimeData result;
  memset(&result,0,sizeof(result));

  if(!hasData())
    return result;

  if(dataSize < sizeof(DateTimeData))
    return result;

  memcpy(&result,data,sizeof(DateTimeData));

  return result;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreStoredData::operator HumidityData() const
{
  HumidityData result;

  if(!hasData())
    return result;

  if(dataSize < sizeof(HumidityData))
    return result;

  memcpy(&result,data,sizeof(HumidityData));

   return result;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreStoredData::operator BarometricData() const
{
  BarometricData result;
  
  if(!hasData())
    return result;

  if(dataSize < sizeof(BarometricData))
    return result;

   memcpy(&result,data,sizeof(BarometricData));

   return result;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreStoredData::operator PressureData() const
{
  PressureData result;
  
  if(!hasData())
    return result;

  if(dataSize < sizeof(PressureData))
    return result;

   memcpy(&result,data,sizeof(PressureData));
 
   return result;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
// CoreDataStoreClass
//--------------------------------------------------------------------------------------------------------------------------------------
CoreDataStoreClass::CoreDataStoreClass()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreDataStoreClass::clear()
{
  for(size_t i=0;i<list.size();i++)
  {
    delete [] list[i].data;
  }

  list.empty();
  //while(list.size())
  //  list.pop();
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreDataList CoreDataStoreClass::getBySensor(CoreSensorType type)
{
  CoreDataList result;

  for(size_t i=0;i<list.size();i++)
  {
    CoreStoredData dt = list[i];
    if(dt.sensor->getType() == type)
      result.push_back(dt);
  }

   return result;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreStoredData CoreDataStoreClass::get(const String& name)
{
  for(size_t i=0;i<list.size();i++)
  {
    CoreStoredData dt = list[i];
    if(dt.sensor->getName() == name)
      return dt;
  }

  CoreStoredData dummy;
  dummy.data = NULL;
  return dummy;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreStoredData CoreDataStoreClass::get(CoreSensorType type, uint8_t pin)
{
  CoreStoredData result;
  if(!(type == AnalogPortState || type == DigitalPortState))
    return result;

  CoreDataList lst = getBySensor(type);
  for(size_t i=0;i<lst.size();i++)
  {
    CoreStoredData dt = lst[i];
    if(type == AnalogPortState)
    {
      AnalogPortData apd = dt;
      if(apd.Pin == pin)
        return dt;
    }
    else
    {
      DigitalPortData dpd = dt;
      if(dpd.Pin == pin)
        return dt;      
    }
  }

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreDataList CoreDataStoreClass::getByType(CoreDataType type)
{
  CoreDataList result;

  for(size_t i=0;i<list.size();i++)
  {
    CoreStoredData dt = list[i];
    if(CoreSensor::getDataType(dt.sensor->getType()) == type)
      result.push_back(dt);
  }

   return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
size_t CoreDataStoreClass::save(CoreSensor* sensor, uint8_t* data, uint8_t dataSize)
{
  CoreStoredData dt;
  dt.sensor = sensor;
  dt.data = data;
  dt.dataSize = dataSize;

  list.push_back(dt);
  return list.size()-1;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreDataStoreClass CoreDataStore;
//--------------------------------------------------------------------------------------------------------------------------------------
// CoreTextFormatProvider
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTextFormatProvider::CoreTextFormatProvider() : CoreDataFormatProvider()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
Vector<String*> CoreTextFormatProvider::formatComposite(const CoreStoredData& dataStored, size_t sensorIndex, bool showUnits)
{
  Vector<String*> result;

  if(!dataStored.hasData())
    return result;

    // получаем тип данных, который хранит железка определённого вида
    CoreSensorType st = dataStored.sensor->getType();
    CoreDataType typeOfData = CoreSensor::getDataType(st);

    if(dataStored.sensor->isUserDataSensor())
    {
      #ifdef CORE_USERDATA_SENSOR_ENABLED
        CoreUserDataSensor* uds = (CoreUserDataSensor*) dataStored.sensor;
        typeOfData = uds->getUserDataType();
      #endif
    }

    switch(typeOfData)
    {
      case UnknownType:
      break;

      case Temperature: // это температура?
      case Altitude: // высота у нас тоже в структуре температуры
      {
         // приводим к температуре
        TemperatureData tdt = dataStored;
        String* t = new String();
        *t = tdt;
        
        if(showUnits)
          *t += CoreSensor::getUnit(typeOfData);

          result.push_back(t);
      }
      break;
      
      case Luminosity: // это освещённость?
      {
        // приводим к освещённости
        LuminosityData lum = dataStored;
        String* t = new String(lum.Value);
        if(showUnits)
          *t += CoreSensor::getUnit(typeOfData);

          result.push_back(t);
      }
      break;

      case Distance: // это расстояние?
      {
        // приводим к расстоянию
        DistanceData dis = dataStored;
        String* t = new String(dis);
        if(showUnits)
          *t += CoreSensor::getUnit(typeOfData);

          result.push_back(t);
      }
      break;

      case DigitalPort: // это состояние цифрового порта?
      {
        DigitalPortData dpd = dataStored;
        result.push_back(new String(dpd.Value == LOW ? F("OFF") : F("ON")));
      }
      break;

      case AnalogPort: // это состояние аналогового порта?
      {
        AnalogPortData apd = dataStored;
        result.push_back(new String(apd.Value));
      }
      break;      

      case Humidity: // это влажность (пара температура/влажность)
      {
        HumidityData hData = dataStored;
        
        String* t = new String(hData.Temperature);
        
        if(showUnits)
          *t += CoreSensor::getUnit(Temperature);

        result.push_back(t);
                
        t = new String(hData.Humidity);
        
        if(showUnits)
          *t += CoreSensor::getUnit(Humidity);

        result.push_back(t);
      }
      break;

      case DateTime: // это дата/время?
      {
        // приводим к дате/времени
       DateTimeData dtt = dataStored;
       String* t = new String(dtt);
        if(showUnits)
          *t += CoreSensor::getUnit(typeOfData);

        result.push_back(t);
      }
      break;

      case UserData: // пользовательские данные
      {          
          String* t = new String();
          for(uint8_t i=0;i<dataStored.dataSize;i++)
          {
            *t += Core.byteToHexString(dataStored.data[i]);
            *t += ' ';
          }

          result.push_back(t);      
      }
      break;

      case Barometric:
      {
        BarometricData hData = dataStored;
        
        String* t = new String(hData.Temperature);
        
        if(showUnits)
          *t += CoreSensor::getUnit(Temperature);

        result.push_back(t);
                
        t = new String(hData.Pressure);
        
        if(showUnits)
          *t += CoreSensor::getUnit(Pressure, hData.Pressure.isInPA);

        result.push_back(t);        

        t = new String(hData.Altitude);
        
        if(showUnits)
          *t += CoreSensor::getUnit(Altitude);
        
        result.push_back(t);        
      }
      break;

      case Pressure:
      {
         // приводим к давлению
        PressureData tdt = dataStored;
        String* t = new String();
        *t = tdt;
        
        if(showUnits)
          *t += CoreSensor::getUnit(typeOfData,tdt.isInPA);

          result.push_back(t);
        
      }
      break;

      //TODO: тут другие типы показаний!!!
    }        

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
String CoreTextFormatProvider::format(const CoreStoredData& dataStored, size_t sensorIndex, bool showUnits)
{
 String result = "-"; // нет данных с датчика

 if(dataStored.hasData())
  {
    // получаем тип данных, который хранит железка определённого вида
    CoreSensorType st = dataStored.sensor->getType();
    CoreDataType typeOfData = CoreSensor::getDataType(st);

    if(dataStored.sensor->isUserDataSensor())
    {
      #ifdef CORE_USERDATA_SENSOR_ENABLED
        CoreUserDataSensor* uds = (CoreUserDataSensor*) dataStored.sensor;
        typeOfData = uds->getUserDataType();
      #endif
    }

    switch(typeOfData)
    {
      case UnknownType:
      break;

      case Temperature: // это температура?
      case Altitude: // высота у нас тоже в структуре температуры
      {
         // приводим к температуре
        TemperatureData tdt = dataStored;
        result = tdt;
        if(showUnits)
          result += CoreSensor::getUnit(typeOfData);       
      }
      break;
      
      case Luminosity: // это освещённость?
      {
        // приводим к освещённости
        LuminosityData lum = dataStored;
        result = lum.Value;
        if(showUnits)
          result += CoreSensor::getUnit(typeOfData);
      }
      break;

      case Distance: // это расстояние?
      {
        // приводим к расстоянию
        DistanceData dis = dataStored;
        result = dis;
        if(showUnits)
          result += CoreSensor::getUnit(typeOfData);
      }
      break;

      case DigitalPort: // это состояние цифрового порта?
      {
        DigitalPortData dpd = dataStored;
        result = dpd.Value == LOW ? F("OFF") : F("ON");
      }
      break;

      case AnalogPort: // это состояние аналогового порта?
      {
        AnalogPortData apd = dataStored;
        result = apd.Value;
      }
      break;      

      case Humidity: // это влажность (пара температура/влажность)
      {
        HumidityData hData = dataStored;
        
        result = hData.Temperature;
        
        if(showUnits)
          result += CoreSensor::getUnit(Temperature);
        
        result += CORE_HUMIDITY_DELIMITER;
        
        result += hData.Humidity;
        
        if(showUnits)
          result += CoreSensor::getUnit(Humidity);
      }
      break;

      case DateTime: // это дата/время?
      {
        // приводим к дате/времени
       DateTimeData dtt = dataStored;
        result = dtt;
        if(showUnits)
          result += CoreSensor::getUnit(typeOfData);
      }
      break;

      case UserData: // пользовательские данные
      {          
          result = "";
          for(uint8_t i=0;i<dataStored.dataSize;i++)
          {
            result += Core.byteToHexString(dataStored.data[i]);
            result += ' ';
          }
          
      }
      break;

      case Barometric:
      {
        BarometricData hData = dataStored;
        
        result = hData.Temperature;
        
        if(showUnits)
          result += CoreSensor::getUnit(Temperature);
        
        result += CORE_HUMIDITY_DELIMITER;
        
        result += hData.Pressure;
        
        if(showUnits)
          result += CoreSensor::getUnit(Pressure, hData.Pressure.isInPA);

        result += CORE_HUMIDITY_DELIMITER;
        
        result += hData.Altitude;
        
        if(showUnits)
          result += CoreSensor::getUnit(Altitude);
        
      }
      break;

      case Pressure: // это давление?
      {
         // приводим к температуре
        PressureData tdt = dataStored;
        result = tdt;
        if(showUnits)
          result += CoreSensor::getUnit(typeOfData, tdt.isInPA);       
      }
      break;      

      //TODO: тут другие типы показаний!!!
    }
    
    
  }

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
// WatchdogSettingsClass
//--------------------------------------------------------------------------------------------------------------------------------------
WatchdogSettingsClass CoreWatchdog;
//--------------------------------------------------------------------------------------------------------------------------------------
void WatchdogSettingsClass::reset()
{
  WatchdogEnabled = false;
  WatchdogPin = 0;
  WatchdogInterval = 0;
  WatchdogPulseDuration = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WatchdogSettingsClass::update()
{
  if(!WatchdogEnabled)
    return;

    static bool waitForHigh = true;
    static uint32_t watchdogTimer = 0;

    uint32_t wantedInterval = waitForHigh ? WatchdogInterval : WatchdogPulseDuration;
    
    if(millis() - watchdogTimer > wantedInterval)
    {
      pinMode(WatchdogPin,OUTPUT);
      digitalWrite(WatchdogPin, waitForHigh ? HIGH : LOW); 
      waitForHigh = !waitForHigh;
      watchdogTimer = millis();      
    }

      
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreDelayedEventClass CoreDelayedEvent;
//--------------------------------------------------------------------------------------------------------------------------------------
CoreDelayedEventClass::CoreDelayedEventClass()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreDelayedEventClass::raise(uint32_t raiseDelay,CoreDelayedEventHandler handler, void* param)
{
  CoreDelayedEventData rec;
  rec.timer = millis();
  rec.duration = raiseDelay;
  rec.handler = handler;
  rec.param = param;
  
  signals.push_back(rec);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreDelayedEventClass::update()
{
    for(size_t i=0;i<signals.size();)
    {
        if(millis() - signals[i].timer > signals[i].duration)
        {
          // сигнал сработал
          if(signals[i].handler)
            signals[i].handler(signals[i].param);

          for(size_t j=i+1; j< signals.size(); j++)
          {
            signals[j-1] = signals[j];
          }

          signals.pop();
        } // if
        else
        {
          i++;
        }
    } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreDelayedEventClass::CoreDelayedEventPinChange(void* param)
{
  CoreDelayedEventPinChangeArg* arg = (CoreDelayedEventPinChangeArg*) param;
  pinMode(arg->pin,OUTPUT);
  digitalWrite(arg->pin,arg->level);

  delete arg;
}
//--------------------------------------------------------------------------------------------------------------------------------------

