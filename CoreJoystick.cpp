#include "CoreJoystick.h"
//--------------------------------------------------------------------------------------------------------------------------------------
Joystick::Joystick()
{
  pinX = pinY = 0;
  curX = curY = 0;
  midX = midY = 512;
  histeresis = 20;
}
//--------------------------------------------------------------------------------------------------------------------------------------
float Joystick::reflect(int16_t current, int16_t midpoint, uint8_t histeresis)
{
  if(current >= (midpoint - histeresis) && current <= (midpoint + histeresis))
    return 0.0;

 float reflectedPos = current - midpoint;

  return (reflectedPos/midpoint);
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Joystick::begin(uint8_t xPin, uint8_t yPin, int16_t midPointX, int16_t midPointY, uint8_t _histeresis)
{
  pinX = xPin;
  pinY = yPin;

  midX = midPointX;
  midY = midPointY;

  histeresis = _histeresis;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Joystick::update()
{
  curX = analogRead(pinX);
  curY = analogRead(pinY);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t Joystick::getXRaw()
{
  return curX;
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t Joystick::getYRaw()
{
  return curY;
}
//--------------------------------------------------------------------------------------------------------------------------------------
float Joystick::getX()
{
  return reflect(curX,midX,histeresis);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
float Joystick::getY()
{
  return reflect(curY,midY,histeresis);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
float Joystick::getPolarAngle()
{
  float x = getX();
  float y = getY();

  float atanResult = atan2(y, x);
  
  float ang = degrees(atanResult);
  if (ang < 0)  
    ang += 360;
  
  if(ang > 0)
    ang = 360 - ang;

  return ang;
}
//--------------------------------------------------------------------------------------------------------------------------------------
float Joystick::computeAngle(float val, uint8_t degreesFreedom)
{
  val += 1.0;
  float stepSize = float(degreesFreedom)/2.0;
  return val*stepSize;
}
//--------------------------------------------------------------------------------------------------------------------------------------
float Joystick::getXAngle(uint8_t degreesFreedom)
{
   return computeAngle(getX(),degreesFreedom);
}
//--------------------------------------------------------------------------------------------------------------------------------------
float Joystick::getYAngle(uint8_t degreesFreedom)
{
  return computeAngle(getY(),degreesFreedom);
}
//--------------------------------------------------------------------------------------------------------------------------------------

