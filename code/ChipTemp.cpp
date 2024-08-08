#include "ChipTemp.h"
#include <Arduino.h>

// Taken (stolen :) )  from:
// https://forum.arduino.cc/t/lightweight-avr-temperature-sensor-interface/40346

ChipTemp::ChipTemp()
{ 
}

inline void ChipTemp::initialize() 
{ ADMUX = 0xC8; // select reference, select temp sensor
  delay(10); // wait for the analog reference to stabilize
  readAdc(); // discard first sample (never hurts to be safe)  
}

inline int ChipTemp::readAdc()
{ ADCSRA |= _BV(ADSC); // start the conversion  
  while (bit_is_set(ADCSRA, ADSC)); // ADSC is cleared when the conversion finishes    
  return (ADCL | (ADCH << 8)); // combine bytes 
}

int ChipTemp::deciCelsius() 
{ long averageTemp=0; 
  initialize(); // must be done everytime
  for (int i=0; i<samples; i++) averageTemp += readAdc();
  averageTemp -= offsetFactor;
  return averageTemp / divideFactor; // return deci degree Celsius
}

int ChipTemp::celsius() 
{ return deciCelsius()/10;
}

int ChipTemp::deciFahrenheit() 
{ return (9 * deciCelsius()+1600) / 5;
}

int ChipTemp::fahrenheit() 
{ return (9 * deciCelsius()+1600) / 50; // do not use deciFahrenheit()/10;
}