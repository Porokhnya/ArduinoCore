#pragma once
//--------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
//--------------------------------------------------------------------------------------------------------------------------------------
class Encoder
{
  unsigned int state;
  byte pin0, pin1;
  byte ppc;
  int change;
  bool pullup;

  unsigned int readState();

public:
  Encoder(byte A, byte B, byte pulsesPerClick, bool pullup = true);

  void begin();
  void update();
  int getChange();
};
//--------------------------------------------------------------------------------------------------------------------------------------
