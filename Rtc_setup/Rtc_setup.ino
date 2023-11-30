#include <Wire.h>
#include <I2C_RTC.h>

#define PCF8563_ADDRESS 0x51

static PCF8563 RTC;

void setup()
{
  RTC.begin();
  
  RTC.setDate(30, 11, 23);     //SetDate(Day,Month,Year)
  RTC.setTime(20, 00, 00);     //SetTime(Hours,Minutes,Seconds)

  Wire.beginTransmission(PCF8563_ADDRESS);
  Wire.write(0x0E);        // Address of register 0Eh
  Wire.write(B10000011);   // for timer enable and a timer clock freq of 1/60 Hz
  Wire.endTransmission();
  
  Wire.beginTransmission(PCF8563_ADDRESS);
  Wire.write(0x0F); // Address of register 0Fh
  //Wire.write(B00001010); // Set Value for timer, depend on clock freq, 10 minutes
  Wire.write(B00000001); // set timer for 1 minute
  Wire.endTransmission();

  Wire.beginTransmission(PCF8563_ADDRESS);
  Wire.write(0x01); // Address of register 01h
  Wire.write(B00010001); // Control of Interrupt - 4 bit, 0 bit - timer interrupt is enable
  Wire.endTransmission();

}


void loop()
{
  
}
 
