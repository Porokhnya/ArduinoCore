//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "Signals.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_SIGNALS_ENABLED
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t signals[CORE_SIGNAL_BYTES] = {0};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// SignalOneAction
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
SignalOneAction::SignalOneAction()
{
  action = saNone;
  actionDataLength = 0;
  actionData = NULL;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
SignalOneAction::~SignalOneAction()
{
  delete [] actionData;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// SignalHandler
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
SignalHandler::SignalHandler(uint16_t memAddr)
{
  memoryAddress = memAddr;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SignalHandler::analyze()
{
  uint16_t addr = memoryAddress;
  //uint8_t recordLength = Core.memRead(addr); addr++;

/*
  uint8_t* record = new uint8_t[recordLength];

  // вычитываем запись
  for(uint8_t i=0;i<recordLength;i++)
  {
    record[i] =  Core.memRead(addr); addr++;
  } // for
*/
  // начинаем анализировать

//  uint8_t recordIterator = 0;
  bool hasTimeSettings = Core.memRead(addr) == 1; addr++; //record[recordIterator] != 0;
//  recordIterator++;

  DBG(F("SIG: hasTimeSettings=")); DBGLN(hasTimeSettings);

  if(hasTimeSettings)
  {
    DBGLN(F("SIG: Check datetime..."));
    
    DateTimeData timeFrom, timeTo;
    
    uint8_t daymask = Core.memRead(addr); addr++;
    timeFrom.hour = Core.memRead(addr); addr++;
    timeFrom.minute = Core.memRead(addr); addr++;
    timeTo.hour = Core.memRead(addr); addr++;
    timeTo.minute = Core.memRead(addr); addr++;

    // тут, не вычитывая всю запись - мы можем анализировать - можем ли мы работать с сигналом.
    // если есть часы реального времени - считаем, что с сигналом можно работать, если с часов есть показания
    // и мы попадаем в указанный интервал. Если же часов реального времени нет - то считаем, что в этом случае
    // с сигналом работать нельзя - и выходим.

    CoreDataList rtcDataList = CoreDataStore.getByType(DateTime);
    if(!rtcDataList.size())
    {
      DBGLN(F("SIG: No datetime!"));
      // нет показаний времени, не можем работать
    //  delete [] record;
      return;
    }

    CoreStoredData storedData = rtcDataList[0];
    if(!storedData.hasData())
    {
      DBGLN(F("SIG: No datetime data!"));
      // нет данных с часов - работать не можем
      //delete [] record;
      return;
    }

    // тут превращаем данные из хранилища в формат даты/времени
    DateTimeData currentTime = storedData;

    // всё, теперь можем проверять - попадаем ли мы в настройки, когда сигнал может проверяться
    // понедельник - у нас день недели номер 1
    if(!(daymask & (1 << currentTime.dayOfWeek)))
    {
      DBGLN(F("SIG: Can't work today!"));
      // не можем работать в этот день недели, выходим
     // delete [] record;
      return;
    }

    // проверяем - возможно, у нас указаны нули в диапазонах - тогда работаем всегда, и незачем проверять диапазон
    if(!(timeFrom.hour == 0 && timeFrom.minute == 0 && timeTo.hour == 0 && timeTo.minute == 0))
    {
        // теперь проверяем, попадаем ли мы в диапазон по времени
        if(!(currentTime >= timeFrom && currentTime <= timeTo))
        {
          DBGLN(F("SIG: Can't work at this time!"));
          // не можем работать в это время, выходим
        //  delete [] record;
          return;
        }
    }

    // если мы здесь - значит, по времени у нас всё ОК, и можем работать
    DBGLN(F("SIG: Can work, continue checking..."));
    
  } // hasTimeSettings

  // вычитываем оператор сравнения
  SignalOperands operand = (SignalOperands) Core.memRead(addr); addr++;

  // затем идёт длина данных, с которыми сравниваем
  uint8_t dataLength = Core.memRead(addr); addr++;
  uint8_t* data = NULL;

  if(dataLength)
  {
    data = new uint8_t[dataLength];

    for(uint8_t i=0;i<dataLength;i++)
    {
      data[i] = Core.memRead(addr); addr++;
    }
  } // if

  // потом идёт кол-во действий, которые надо совершить, если сигнал сработал
  uint8_t actionsCount = Core.memRead(addr); addr++;

  // вычитываем все действия
  SignalActionsList actions;

  for(uint8_t i=0;i<actionsCount;i++)
  {
    uint8_t actionType = Core.memRead(addr); addr++;
    uint8_t actionDataLength = Core.memRead(addr); addr++;
    uint8_t* actionData = new uint8_t[actionDataLength];

    for(uint8_t k=0;k<actionDataLength;k++)
    {
      actionData[k] = Core.memRead(addr); addr++;
    }

    SignalOneAction* action = new SignalOneAction();
    action->action = actionType;
    action->actionDataLength = actionDataLength;
    action->actionData = actionData;

    actions.push_back(action);
    
  } // for

  // далее идёт имя датчика, с которого получаем показания, заканчивается '\0'
  String sensorName;
  // резервируем сразу память, чтобы не дёргать переаллокацию
  sensorName.reserve(10);
  char ch;
  do
  {
    ch = (char) Core.memRead(addr); addr++;
    if(ch != '\0')
      sensorName += ch;
      
  } while(ch != '\0');


  // разобрали всю запись, можем её анализировать. У нас есть оператор сравнения - operand,
  // и имя датчика. Если имени датчика нет - считаем, что сигнал сработал, и мы можем выполнять действия
  if(!sensorName.length())
  {
    DBGLN(F("SIG: No sensor name, simply execute actions..."));
    executeActions(actions);
  }
  else
  {
    // есть имя датчика, анализируем
    if(compare(sensorName, operand, data, dataLength))
      executeActions(actions);
    
  } // else


  // в самом конце чистим за собой
  for(size_t i=0;i<actions.size();i++)
  {
    delete actions[i];
  }

  //delete [] record;
  

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SignalHandler::executeActions(SignalActionsList& actions)
{
  DBG(F("SIG: Actions to execute="));
  DBGLN(actions.size());


  //Тут выполнение действий, когда сигнал сработал

  for(size_t i=0;i<actions.size();i++)
  {
    SignalOneAction* oneAction = actions[i];

    switch(oneAction->action)
    {
      case saNone:
      break;

      case saRaiseSignal:
      {
        // взвод сигнала
        if(oneAction->actionDataLength != 1 || ! oneAction->actionData)
        {
          DBGLN(F("SIG: saRaiseSignal action = MALFORMED!"));
        }
        else
        {
          // можем взводить сигнал
          uint8_t signalNumber = oneAction->actionData[0];
          DBG(F("SIG: saRaiseSignal action, signal #"));
          DBGLN(signalNumber);
          Signals[signalNumber] = 1;
        }
      }
      break;

      case saResetSignal:
      {
        if(oneAction->actionDataLength != 1|| ! oneAction->actionData)
        {
          DBGLN(F("SIG: saResetSignal action = MALFORMED!"));
        }
        else
        {
          // можем снимать сигнал
          uint8_t signalNumber = oneAction->actionData[0];
          DBG(F("SIG: saResetSignal action, signal #"));
          DBGLN(signalNumber);
          Signals[signalNumber] = 0;          
        }
        
      }
      break;

      //TODO: Тут другие типы действий на сигнале!!!

      default:
      {
        DBGLN(F("SIG: Unknown action!"));
      }
      break;
      
    } // switch
    
  } // for

  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool SignalHandler::compareNumber(long num,SignalOperands operand,long from, long to)
{
 switch(operand)
  {
    case sopNone:
    case sopCompareNoData:
      return false;
      
    case sopInterval:
      return (num >= from && num <= to);

    case sopEqual:
      return (num == from);

    case sopLess:
      return (num < from);

    case sopLessOrEqual:
      return (num <= from);

    case sopGreater:
      return (num > from);

    case sopGreaterOrEqual:
      return (num >= from);

    case sopNotEqual:
      return (num != from);

  } // switch

  return false;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool SignalHandler::compareDigitalPort(DigitalPortData& dataStored, SignalOperands operand, uint8_t* data, uint8_t dataLength)
{
    if(dataLength != 1)
    {
      DBGLN(F("SIG: compareDigitalPort - MALFORMED DATA!"));
      return false;
    }

  // поскольку цифровой порт сравнивается только на 1 и 0 - можно применить оператор совпадения с указанным уровнем,
  // игнорируя все остальные. Это мы и делаем.
  return compareNumber(dataStored.Value,sopEqual,data[0],0);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool SignalHandler::compareAnalogPort(AnalogPortData& dataStored, SignalOperands operand, uint8_t* data, uint8_t dataLength)
{
    if(dataLength != 4)
    {
      DBGLN(F("SIG: compareAnalogPort - MALFORMED DATA!"));
      return false;
    }

   uint16_t valueFrom, valueTo;
   memcpy(&valueFrom,data,sizeof(uint16_t));
   data += sizeof(uint16_t);
   memcpy(&valueTo,data,sizeof(uint16_t));

   return compareNumber(dataStored.Value,operand,valueFrom,valueTo); 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool SignalHandler::compareUserData(CoreStoredData& dataStored, SignalOperands operand, uint8_t* data, uint8_t dataLength)
{
  // тут ситуация поинтересней - у нас пользовательские данные могут быть переменной длины, следовательно, и сравнивать с ними надо
  // с этим учётом. В этом случае dataLength должно быть кратно 2, т.к. должна быть пара с совпадающей длиной.
    if(dataLength % 2)
    {
      DBGLN(F("SIG: compareUserData - MALFORMED DATA!"));
      return false;
    }

    uint8_t compareLength = dataLength/2;
    uint8_t* dataTo = (data + compareLength);

    uint8_t blockLength = min(dataStored.dataSize,compareLength);

  switch(operand)
  {
    case sopNone:
    case sopCompareNoData:
      return false;
      
    case sopInterval:
      return (
        (memcmp(dataStored.data, data, blockLength) > 0) 
        && 
        (memcmp(dataStored.data, dataTo, blockLength) < 0)
      );

    case sopEqual:
      return !memcmp(dataStored.data, data, blockLength);

    case sopLess:
    case sopLessOrEqual:
      return memcmp(dataStored.data, data, blockLength) < 0;

    case sopGreater:
    case sopGreaterOrEqual:
      return memcmp(dataStored.data, data, blockLength) > 0;

    case sopNotEqual:
      return memcmp(dataStored.data, data, blockLength) != 0;

  } // switch

  return false;
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool SignalHandler::compareLuminosity(LuminosityData& dataStored, SignalOperands operand, uint8_t* data, uint8_t dataLength)
{
    if(dataLength != 8)
    {
      DBGLN(F("SIG: compareLuminosity - MALFORMED DATA!"));
      return false;
    }
  
  // dataLength у нас равно 8, и в зависимости от операнда - там либо одно значение, либо - попадание в диапазон
  long lumFrom, lumTo;
  memcpy(&lumFrom,data,sizeof(long));
  data += sizeof(long);
  memcpy(&lumTo,data,sizeof(long));


  return compareNumber(dataStored.Value,operand,lumFrom,lumTo);
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool SignalHandler::compareHumidity(HumidityData& dataStored, SignalOperands operand, uint8_t* data, uint8_t dataLength)
{
    if(dataLength != 5)
    {
      DBGLN(F("SIG: compareHumidity - MALFORMED DATA!"));
      return false;
    }
  
  // dataLength у нас равно 5, и в зависимости от операнда - там либо одно значение, либо - попадание в диапазон
  // в пятом байте - указание, чего сравниваем - влажность или температуру
  if(data[4] == 1)
  {
    // сравниваем влажность
    return compareTemperature(dataStored.Humidity, operand, data, dataLength-1);
  }
  else
  {
    // сравниваем температуру
    return compareTemperature(dataStored.Temperature, operand, data, dataLength-1);
  }
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool SignalHandler::compareTemperature(TemperatureData& dataStored, SignalOperands operand, uint8_t* data, uint8_t dataLength)
{
    if(dataLength != 4)
    {
      DBGLN(F("SIG: compareTemperature - MALFORMED DATA!"));
      return false;
    }
  
  // dataLength у нас равно 4, и в зависимости от операнда - там либо одно значение, либо - попадание в диапазон

  TemperatureData tempFrom, tempTo;
  tempFrom.Value = (int8_t) data[0]; // в первом байте - значение температуры в целых

  // во втором байте - значение температуры в сотых
  tempFrom.Fract = data[1];

  // в третьем байте - значение конечного диапазона температуры, в целых
  tempTo.Value = (int8_t) data[2];
  
  // в четвёртом байте - значение конечного диапазона температуры, в сотых
  tempTo.Fract = data[3];

  switch(operand)
  {
    case sopNone:
    case sopCompareNoData:
      return false;
      
    case sopInterval:
      return (dataStored >= tempFrom && dataStored <= tempTo);

    case sopEqual:
      return (dataStored == tempFrom);

    case sopLess:
      return (dataStored < tempFrom);

    case sopLessOrEqual:
      return (dataStored <= tempFrom);

    case sopGreater:
      return (dataStored > tempFrom);

    case sopGreaterOrEqual:
      return (dataStored >= tempFrom);

    case sopNotEqual:
      return (dataStored != tempFrom);

  } // switch

  return false;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool SignalHandler::compare(const String& sensorName, SignalOperands operand, uint8_t* data, uint8_t dataLength)
{

  if(operand == sopNone) // нет операнда
  {
    DBGLN(F("SIG: No compare operand!"));
    return false;
  }
  
  DBG(F("SIG: Compare, sensor="));
  DBGLN(sensorName);

  // получаем данные из хранилища
  CoreStoredData dataStored = CoreDataStore.get(sensorName);

  if(!dataStored.hasData() || !dataStored.sensor) // нет данных
  {
    if(operand == sopCompareNoData) // если попросили сравнить с условием "с датчика нет данных" - считаем, что сигнал сработал
      return true;
    else  
      return false;
  }

  // с датчика есть данные, можем их анализировать
// получаем тип данных, который хранит железка определённого вида
    CoreSensorType st = dataStored.sensor->getType();
    CoreDataType typeOfData = CoreSensor::getDataType(st);

    if(dataStored.sensor->isUserDataSensor())
    {
      CoreUserDataSensor* uds = (CoreUserDataSensor*) dataStored.sensor;
      typeOfData = uds->getUserDataType();
    }

    switch(typeOfData)
    {
      case UnknownType:
      break;

      case Temperature: // это температура?
      {
        TemperatureData dt = dataStored;
        return compareTemperature(dt, operand, data, dataLength);
      }
      
      case Luminosity: // это освещённость?
      {
        LuminosityData dt = dataStored;
        return compareLuminosity(dt, operand, data, dataLength);
      }

      case DigitalPort: // это состояние цифрового порта?
      {
        DigitalPortData dt = dataStored;
        return compareDigitalPort(dt, operand, data, dataLength);
      }

      case AnalogPort: // это состояние аналогового порта?
      {
        AnalogPortData dt = dataStored;
        return compareAnalogPort(dt, operand, data, dataLength);
      }

      case Humidity: // это влажность (пара температура/влажность)
      {
        HumidityData dt = dataStored;
        return compareHumidity(dt, operand, data, dataLength);
      }

      case DateTime: // это дата/время?
        return false;

      case UserData: // пользовательские данные
        return compareUserData(dataStored, operand, data, dataLength);
   

      //TODO: тут другие типы показаний!!!  
    } // switch 

  return false;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Signal
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Signal::Signal()
{
  signalNumber = 0xFF;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Signal& Signal::operator=(bool val)
{
  set(val ? 1 : 0);
  return *this;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Signal& Signal::operator=(uint8_t val)
{
  return operator=(val != 0);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Signal& Signal::operator=(uint16_t val)
{
  return operator=(val != 0);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Signal& Signal::operator=(uint32_t val)
{
  return operator=(val != 0);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Signal& Signal::operator=(int8_t val)
{
  return operator=(val != 0);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Signal& Signal::operator=(int16_t val)
{
  return operator=(val != 0);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Signal& Signal::operator=(int32_t val)
{
  return operator=(val != 0);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Signal::setNumber(uint8_t num)
{
   signalNumber = num;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Signal::reset()
{
  set(0);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Signal::raise()
{
  set(1);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Signal::operator bool()
{
  return get();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t Signal::get()
{
  uint8_t byteNum, bitNum;
  offset(byteNum, bitNum);

  if(byteNum >= CORE_SIGNAL_BYTES)
    return 0;

  bool b = (signals[byteNum] & (1 << bitNum)) != 0;
  return (b ? 1 : 0);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Signal::set(uint8_t val)
{
  uint8_t byteNum, bitNum;
  offset(byteNum, bitNum);

  if(byteNum >= CORE_SIGNAL_BYTES)
    return;

  byte b = signals[byteNum];
  
  b &= ~(1 << bitNum);
  b |= (val << bitNum);

  signals[byteNum] = b;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Signal::offset(uint8_t& byteNum, uint8_t& bitNum)
{
  byteNum = signalNumber/8;
  bitNum = signalNumber%8;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// SignalsManager
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
SignalsManager Signals;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
SignalsManager::SignalsManager()
{
  reset();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Signal& SignalsManager::operator[](uint8_t signalNumber)
{
  thisSignal.setNumber(signalNumber);
  return thisSignal;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SignalsManager::addRecord(uint16_t memoryAddress)
{
  addresses.push_back(memoryAddress);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SignalsManager::reset()
{
  memset(signals,0,sizeof(signals));

  // чистим список адресов
  while(addresses.size())
    addresses.pop();

  updateTimer = millis();    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SignalsManager::begin()
{
  //TODO: Тут настройка и пуск в работу!!!
  updateTimer = millis();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SignalsManager::update()
{
  // Тут обновление сигналов
  unsigned long now = millis();
  if(now - updateTimer > CORE_SIGNALS_UPDATE_INTERVAL)
  {
    // пора обновлять сигналы
    DBGLN(F("SIG: Update signals..."));

    for(size_t i=0;i<addresses.size();i++)
    {
      SignalHandler handler(addresses[i]);
      handler.analyze();
    }
    
    DBGLN(F("SIG: Signals updated."));

    updateTimer = millis();
  } // if
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_SIGNALS_ENABLED
