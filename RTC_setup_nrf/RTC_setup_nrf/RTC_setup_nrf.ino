/*
 * This example is putting the board into deep sleep with power consumption around 2uA
 */
#include <bluefruit.h>
#include <Adafruit_FlashTransport.h>

#include <Wire.h>
#define PCF8563_ADDRESS 0x51

uint8_t cnt = 0u;

void setup() {
  Wire.begin(); 
  Wire.beginTransmission (PCF8563_ADDRESS);
  Wire.endTransmission();
   
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


void loop() {
  // put your main code here, to run repeatedly:
   delay(4000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(4000);
   digitalWrite(LED_BUILTIN, HIGH);
   cnt = cnt + 1u;
   if(cnt == 5u)
   { 
    shutdown();
   }

}

void sleepFlash() {
    Adafruit_FlashTransport_QSPI flashTransport;
    Bluefruit.begin();
    flashTransport.begin();
    flashTransport.runCommand(0xB9);
    flashTransport.end();
}
void disconnectPin(uint32_t ulPin) {
      if (ulPin >= PINS_COUNT) {
        return;
    }

    ulPin = g_ADigitalPinMap[ulPin];

    NRF_GPIO_Type * port = nrf_gpio_pin_port_decode(&ulPin);

    port->PIN_CNF[ulPin] = 
          ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
        | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
        | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
        | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
        | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
}

void shutdown() {
    //sleep flash
    cnt = 0u;
    sleepFlash();
    //disconnect any pins used
    //disconnectPin(D1);
    //disconnectPin(D2);
    //setup pin for wakeup
    nrf_gpio_cfg_sense_input(g_ADigitalPinMap[D3], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    //power off
    sd_power_system_off();
}
