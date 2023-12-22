// Library needed for GPS
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11

// Used for hardware & software SPI
#define LIS3DH_CS 10

unsigned long acc_timer = millis();

const int taskDelay = 100;

// TTN Configuration
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui(u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { /* see report */ };
void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { /* see report */};
void os_getDevKey(u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

// payload to send to TTN gateway
static uint8_t payload[16];                                                                      
static osjob_t sendjob;


uint16_t converted_temperatureC;                                                                        

uint16_t converted_hallSensorValue;

int8_t a_x;
int8_t a_y;
int8_t a_z;    

int8_t latitude;
int8_t latitudeDelta;
int8_t longitude;
int8_t longitudeDelta;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 7;

// Pin mapping for Adafruit Feather M0 LoRa
const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = { 3, 6, LMIC_UNUSED_PIN },
  .rxtx_rx_active = 0,
  .rssi_cal = 8,  // LBT cal for the Adafruit Feather M0 LoRa, in dB
  .spi_freq = 8000000,
};



// Pin Definitions
// Not working pins:A3 A4 A2 A1, 11 12 6 9
// Pin for Temperature and Hall Effect Sensor
#define LM35_TEMP_SENSOR_PIN A0
#define HALL_3144_SENSOR_PIN A5



// // Init for GPS
// Adafruit_GPS GPS(&Serial1);

// // Init for I2C
// Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// Pin for Testing
// #define TEST_PIN 5

void setup() {
  // Initialize Serial Monitor
  delay(1000);
  Serial.begin(115200);

  latitude = random(40, 61); 
  longitude = random(-10, 11); 
  // LMIC init.
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Disable link-check mode and ADR, because ADR tends to complicate testing.
  LMIC_setLinkCheckMode(0);
  // Set the data rate to Spreading Factor 7.  This is the fastest supported rate for 125 kHz channels, and it
  // minimizes air time and battery power. Set the transmission power to 14 dBi (25 mW).
  LMIC_setDrTxpow(DR_SF7, 14);
  // in the US, with TTN, it saves join time if we start on subband 1 (channels 8-15). This will
  // get overridden after the join by parameters from the network. If working with other
  // networks or in other regions, this will need to be changed.
  //LMIC_selectSubBand(1);

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  // we call the LMIC's runloop processor. This will cause things to happen based on events and time. One
  // of the things that will happen is callbacks for transmission complete or received messages. We also
  // use this loop to queue periodic data transmissions.  You can put other things here in the loop() routine,
  // but beware that LoRaWAN timing is pretty tight, so if you do more than a few milliseconds of work, you
  // will want to call os_runloop_once() every so often, to keep the radio running.
  os_runloop_once();

  if (millis() - acc_timer > taskDelay) {
    buildPayload();
  
  // Temperature Sensor Reading
  float temperatureC = getTemperatureC();
  float temperatureF = celsiusToFahrenheit(temperatureC);

  converted_temperatureC = (uint16_t) (temperatureC * 100);   
  
  // Hall Effect Sensor Reading
  int hallSensorValue = analogRead(HALL_3144_SENSOR_PIN);

  converted_hallSensorValue = (uint16_t)(hallSensorValue);

  // Generating random acceleration values
  a_x = random(-128,128);                                       
  a_y = random(-128,128);
  a_z = random(-128,128);

  // Update Position
  latitudeDelta = random(-2, 3);
  longitudeDelta = random(-2, 3);

  if ((latitude + latitudeDelta) > 60 || (latitude + latitudeDelta) < 40) {

  } 
  else {
    latitude += latitudeDelta;
  }

  if ((longitude + longitudeDelta) > 10 || (longitude + longitudeDelta) < -10) {

  } 
  else {
    longitude += longitudeDelta;
  }

  // Display Acceleration
  displayAccelerometer(a_x,a_y,a_z);
  // Display Position
  displayGPS(longitude,latitude);
  // Display Temperature
  displayTemperature(temperatureC, temperatureF);
  // Display Hall effect Sensor
  displayHallEffectSensor(hallSensorValue);
  }
 
}

// Function to read temperature from LM35 sensor
float getTemperatureC() {
  int sensorReading = analogRead(LM35_TEMP_SENSOR_PIN);
  float voltage = sensorReading * (3.3 / 1023.0);
  return voltage * 100;
}

// Function to convert Celsius to Fahrenheit
float celsiusToFahrenheit(float temperatureC) {
  return (temperatureC * 9.0 / 5.0) + 32.0;
}

// Function to display temperature
void displayTemperature(float temperatureC, float temperatureF) {
  Serial.print("Temperature Value: ");
  Serial.print(temperatureC);
  Serial.print("\xC2\xB0");  // affiche le symbole degré
  Serial.print("C  |  ");
  Serial.print(temperatureF);
  Serial.print("\xC2\xB0");  // affiche le symbole degré
  Serial.println("F");
}

// Function to print sensor values
void displayHallEffectSensor(int hallSensorValue) {
  Serial.print("Hall Sensor Value: ");
  Serial.println(hallSensorValue);
  // Serial.print("Test Value: ");
  // Serial.println(testValue);
}
// Function to print accelerometer values
void displayAccelerometer(int8_t x, int8_t y, int8_t z){
  Serial.print("Acceleration in X: ");
  Serial.println(x);
  Serial.print("Acceleration in Y: ");
  Serial.println(y);
    Serial.print("Acceleration in Z: ");
  Serial.println(z);
}
// Function to print GPS values
void displayGPS(int8_t longitude, int8_t latitude){
  Serial.print("Longitude: ");
  Serial.println(longitude);
  Serial.print("Latitude: ");
  Serial.println(latitude);
}

void onEvent(ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      LMIC_setLinkCheckMode(0);
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("Payload sent successfully"));
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      Serial.println(F("Starting new transmission"));
      break;
    default:
      Serial.print(F("ERROR: Unknown event "));
      Serial.println(ev);
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // prepare upstream data transmission at the next possible time.
    // transmit on port 1 (the first parameter); you can use any value from 1 to 223 (others are reserved).
    // don't request an ack (the last parameter, if not zero, requests an ack from the network).
    // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.
    LMIC_setTxData2(1, payload, sizeof(payload) - 1, 0);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}


void buildPayload() {
  payload[0] = converted_temperatureC >> 8;                                                 
  payload[1] = converted_temperatureC & 0xFF; 
  payload[2] = converted_hallSensorValue >> 8;                                           
  payload[3] = converted_hallSensorValue & 0xFF;
  payload[4] = a_x;
  payload[5] = a_y;
  payload[6] = a_z;
  payload[7] = longitude;
  payload[8] = latitude;
}

  