#include <Wire.h>
#include <I2C_RTC.h>

#define PCF8563_ADDRESS 0x51

static PCF8563 RTC;
bool flag = true;
void setup()
{
  RTC.begin();
  
  RTC.setDate(30, 11, 23);     //SetDate(Day,Month,Year)
  RTC.setTime(20, 00, 00);     //SetTime(Hours,Minutes,Seconds)

  Wire.beginTransmission(PCF8563_ADDRESS);
  Wire.write(0x0E);        // Address of register 0Eh
  Wire.write(B10000010);   // for timer enable and a timer clock freq of 1/60 Hz
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
  pinMode (3, INPUT); // set pin to input
  digitalWrite (3, HIGH); // turn on pullup resistors
  Serial.begin(115200);
  
}


void loop()
{
  if(digitalRead(3) == LOW && flag){
    Serial.println("Otrzymalem przerwanie");
    //Serial.println(millis());
    flag = false;
  }
  if(digitalRead(3) == HIGH){
    flag = true; 
  }

  }
