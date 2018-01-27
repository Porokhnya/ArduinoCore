//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "Signals.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef CORE_SIGNALS_ENABLED
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t signals[CORE_SIGNAL_BYTES] = {0};
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
void SignalsManager::reset()
{
  memset(signals,0,sizeof(signals));
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SignalsManager::begin()
{
  //TODO: Тут настройка и пуск в работу!!!
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SignalsManager::update()
{
  //TODO: Тут обновление сигналов!!!
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // CORE_SIGNALS_ENABLED
