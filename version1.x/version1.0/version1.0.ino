#define PCF8563_ADDRESS 0x51

#include <bluefruit.h>
#include <Adafruit_FlashTransport.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Wire.h>

MAX30105 Sensor;

uint8_t cnt = 0u;
bool started_meas = false;
long t_start;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
int tempCnt = 0;
float tempSum;

void configure_RTC();
void configure_sensor();
void sleep_flash();
void disconnect_pin(uint32_t ulPin);
void shutdown();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(D2, OUTPUT);
  digitalWrite(D2,HIGH); //Power for MAX sensor
  configure_RTC();
}

void loop() {
  if(!started_meas){
    Serial.begin(115200);
    configure_sensor();
    //t_start = millis(); //Debug
  }
  started_meas = true;
  digitalWrite(LED_BUILTIN, LOW);
  
  tempSum += Sensor.readTemperature();
  tempCnt++; //Number of temperature measurments
  long irValue = Sensor.getIR();
  delay(30);
  if (checkForBeat(irValue) == true)
  {
    //sensed a beat
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  Serial.print(" IR=");
  Serial.print(irValue);
  Serial.print(" BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(" Avg BPM=");
  Serial.print(beatAvg); 
  if (irValue < 50000)
    Serial.print(" No finger?");
  Serial.print('\n');

   digitalWrite(LED_BUILTIN, HIGH);
   cnt = cnt + 1u;

  //Check if 10secs elapsed
   if (millis()-t_start >= 10000)
   { 
    Serial.println('\n');
    Serial.print(" Avg BPM=");
    Serial.print(beatAvg);
    Serial.print(" Avg TEMP=");
    Serial.print(tempSum/tempCnt);
    
    /*DEBUG*/
    /*Serial.print(" Counter= ");
    Serial.print(cnt);
    long t_stop = millis();
    Serial.print(" czas= ");
    Serial.print(millis()-t_start);
    Serial.print("N of temp meas= ");
    Serial.print(tempCnt);
    Serial.print("Sum of temp=");
    Serial.print(tempSum);*/
    
    Sensor.shutDown();
    delay(100);
    digitalWrite(D2,LOW);
    started_meas = false;
    delay(1000);
    shutdown();
   }
}

void configure_RTC(){
  Wire.begin(); 
  delay(1000);
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


void configure_sensor(){
  if (Sensor.begin(Wire, I2C_SPEED_FAST) == false) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  //Sensor.setup(0); //Configure sensor. Turn off LEDs
  Sensor.setup(); //Configure sensor. Use 25mA for LED drive
  Sensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  Sensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  Sensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.
}


void sleep_flash() {
    Adafruit_FlashTransport_QSPI flashTransport;
    Bluefruit.begin();
    flashTransport.begin();
    flashTransport.runCommand(0xB9);
    flashTransport.end();
}


/* DOES NOT WORK (?) */
void disconnect_pin(uint32_t ulPin) {
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
    cnt = 0u;
    sleep_flash();
    //disconnect any pins used
    disconnect_pin(D2); //Control of transistor
    disconnect_pin(D4); //SDA
    disconnect_pin(D5); //SCL
    //setup pin for wakeup
    nrf_gpio_cfg_sense_input(g_ADigitalPinMap[D3], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    //power off
    sd_power_system_off();
}
