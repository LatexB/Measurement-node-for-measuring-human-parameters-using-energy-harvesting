/*
 * This example is putting the board into deep sleep with power consumption around 2uA
 */
#include <bluefruit.h>
#include <Adafruit_FlashTransport.h>

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
    sleepFlash();
    //disconnect any pins used
    //disconnectPin(D1);
    //disconnectPin(D2);
    //setup pin for wakeup
    //nrf_gpio_cfg_sense_input(g_ADigitalPinMap[D3], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    //power off
    sd_power_system_off();
}
void setup()
{
  shutdown();
}

void loop()
{
}
