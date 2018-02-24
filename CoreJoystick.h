#pragma once
//--------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
//--------------------------------------------------------------------------------------------------------------------------------------
class Joystick
{
  public:

    Joystick();

   void begin(uint8_t xPin, uint8_t yPin, int16_t midPointX, int16_t midPointY, uint8_t histeresis = 20);
   void update();

   // возвращает сырые значения с потенциометров
   int16_t getXRaw();
   int16_t getYRaw();

   // возвращает нормализованные значения с потенциометров, приведённые к диапазону [-1.0, 1.0]
   float getX();
   float getY();

   // возвращает угол по каждой из осей. degreesFreedom - степень свободы по оси, градусов
   float getXAngle(uint8_t degreesFreedom=180);
   float getYAngle(uint8_t degreesFreedom=180);

   // возвращает угол в полярных координатах
   float getPolarAngle();

 private:

  uint8_t pinX, pinY;
  int16_t curX, curY;

  int16_t midX, midY;

  uint8_t histeresis;

  float reflect(int16_t current, int16_t midpoint, uint8_t histeresis);
  float computeAngle(float val, uint8_t degreesFreedom);
  
};
//--------------------------------------------------------------------------------------------------------------------------------------

