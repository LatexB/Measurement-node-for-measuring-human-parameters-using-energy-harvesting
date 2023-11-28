/*
 * This example is putting the board into deep sleep with power consumption around 2uA
 */
#include <bluefruit.h>
#include <Adafruit_FlashTransport.h>


void setup()
{
  Adafruit_FlashTransport_QSPI flashTransport;
  Bluefruit.begin();
  flashTransport.begin();
  //Send command to turn off flash P25Q16H
  flashTransport.runCommand(0xB9);
  flashTransport.end();
  sd_power_system_off();
}

void loop()
{
}
