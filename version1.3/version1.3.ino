#define PCF8563_ADDRESS 0x51

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <Adafruit_FlashTransport.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Wire.h>

MAX30105 Sensor;
BLEService  es_svc = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);

// GATT Characteristic and Object Type - 0x2A6E - Temperature
BLECharacteristic tchar = BLECharacteristic(UUID16_CHR_TEMPERATURE);
BLECharacteristic bchar = BLECharacteristic(UUID16_CHR_HEART_RATE_MEASUREMENT);

BLEDis  bledis;  // Initialization of Device Information Service

// Control of deep sleep
bool started_meas = false;
long t_start;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

// Data of BPM
float beatsPerMinute;
int beatAvg;
int beatAbove;
int nBeatAbove;
uint8_t BPMBLEData[2];

//Data of temperature
int tempCnt = 0;
float tempSum;
float temp;
uint8_t tempBLEData[4];

void configure_RTC();
void configure_sensor();
void configure_ESS();
void configure_BLE();
void start_adv();
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void sleep_flash();
void disconnect_pin(uint32_t ulPin);
void shutdown();

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(D2, OUTPUT);
  digitalWrite(D2,HIGH); //Power for MAX sensor
  configure_RTC();
  configure_BLE();
}

void loop() {
  if(!started_meas){
    configure_sensor();
    // Reset to default values
    tempSum = 0;
    tempCnt = 0;
    beatAbove = 0;
    nBeatAbove = 0;
    t_start = millis(); 
  }
  digitalWrite(LED_BUILTIN, LOW);
  started_meas = true;
  
  tempSum += Sensor.readTemperature();
  tempCnt++; //Number of temperature measurments
  
  // Measure BPM
  long irValue = Sensor.getIR();
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
    //Filter useless readings
    if (beatAvg>55){
      beatAbove+=beatAvg;
      nBeatAbove++;
    }
  }
  digitalWrite(LED_BUILTIN, HIGH);
  //Check if 30secs elapsed
   if (millis()-t_start >= 30000)
   { 
    Serial.print(" Avg BPM=");
    Serial.print(beatAbove/nBeatAbove);
    Serial.print(" Avg TEMP=");
    Serial.println(tempSum/tempCnt);

    temp = tempSum/tempCnt;
    beatAvg = beatAbove/nBeatAbove;
    memcpy(tempBLEData, &temp, sizeof(temp));
    memcpy(BPMBLEData, &beatAvg, sizeof(beatAvg));

    // Send data BLE
    tchar.notify(tempBLEData, sizeof(tempBLEData));
    delay(200);
    bchar.notify(BPMBLEData, sizeof(BPMBLEData));

    // Go to deep sleep mode
    Sensor.shutDown(); 
    delay(100);
    digitalWrite(D2,LOW); 
    started_meas = false; 
    t_start = 0;
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
  Sensor.setup(); //Configure sensor. Use 25mA for LED drive
  Sensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  Sensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  Sensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.
}

// Set required properties to BLE characteristics
void configure_ESS() {
  es_svc.begin();
  // Temperature
  tchar.setProperties(CHR_PROPS_NOTIFY);
  tchar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  tchar.setFixedLen(4);
  tchar.begin();

  // BPM
  bchar.setProperties(CHR_PROPS_NOTIFY);
  bchar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  bchar.setFixedLen(2);
  bchar.begin();

}

void configure_BLE(){
  Bluefruit.begin();
  Bluefruit.setName("BLEnergy VitaTrack");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  bledis.setManufacturer("BMB ENGINEERING");
  bledis.setModel("NRF52840-MAX30102-PCF8563");
  bledis.begin();

  configure_ESS();
  start_adv();
  
}

void start_adv(){
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(es_svc);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 1600);   // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void connect_callback(uint16_t conn_handle){
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

// Turn off internal flash, to lower power consumption during sleep
void sleep_flash() {
    Adafruit_FlashTransport_QSPI flashTransport;
    flashTransport.begin();
    flashTransport.runCommand(0xB9);
    flashTransport.end();
}


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
    sleep_flash();
    //disconnect any pins used
    disconnect_pin(D2); //Control of GPIOVIN for MAX30102
    disconnect_pin(D4); //SDA
    disconnect_pin(D5); //SCL
    //setup pin for wakeup
    nrf_gpio_cfg_sense_input(g_ADigitalPinMap[D3], NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    //power off
    sd_power_system_off();
}
