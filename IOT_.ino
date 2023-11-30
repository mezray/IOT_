// Library needed for GPS
#include <Wire.h>
#include <Adafruit_GPS.h>

// Pin Definitions
// Not working pins:A3 A4 A2 A1, 11 12 6 9 
// Pin for Temperature and Hall Effect Sensor
#define LM35_TEMP_SENSOR_PIN A5
#define HALL_3144_SENSOR_PIN A0

// "Pin" for GPS
Adafruit_GPS GPS(&Serial1);

// Pin for Testing
#define TEST_PIN 5

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize GPS
  while (!Serial) delay(10);
  Serial.println("Adafruit GPS Feather M0 Test");
  // GPS Setup
  GPS.begin(115200);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  
  
  // Set pin modes to test board
  pinMode(TEST_PIN, INPUT);
  
}

void loop() {
  // Temperature Sensor Reading
  float temperatureC = getTemperatureC();
  float temperatureF = celsiusToFahrenheit(temperatureC);
  // Hall Effect Sensor Reading 
  int hallSensorValue = analogRead(HALL_3144_SENSOR_PIN);
  // Value For Testing
  int testValue = digitalRead(TEST_PIN);
  
  // Display Temperature
  displayTemperature(temperatureC, temperatureF);
  // Display Hall effect Sensor
  displayHallEffectSensor( hallSensorValue, digitalRead(TEST_PIN));
  // Print GPS Values 
  // TODO : Go outside to make it work, may take time
  displayGPS();
  // TODO: remove this if not needed
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }
  
  // Change to get more/less frequent readings
  delay(500);
}

// Function to read temperature from LM35 sensor
float getTemperatureC() {
  int sensorReading = analogRead(LM35_TEMP_SENSOR_PIN);
  float voltage = sensorReading * (5.0 / 1023.0);
  return voltage * 100;
}

// Function to convert Celsius to Fahrenheit
float celsiusToFahrenheit(float temperatureC) {
  return (temperatureC * 9.0 / 5.0) + 32.0;
}

void displayGPS() {
  // Display GPS Data
  Serial.print("Location: ");
  Serial.print(GPS.latitudeDegrees, 4);
  Serial.print(", ");
  Serial.print(GPS.longitudeDegrees, 4);
  Serial.print("  Fix: ");
  Serial.print((int)GPS.fix);
  Serial.print("  Satellites: ");
  Serial.println((int)GPS.satellites);
  // Display Time Data
  Serial.print("  Time: ");
  Serial.print(GPS.hour, DEC);
  Serial.print(':');
  Serial.print(GPS.minute, DEC);
  Serial.print(':');
  Serial.println(GPS.seconds, DEC);
}

// Function to display temperature
void displayTemperature(float temperatureC, float temperatureF) {
  Serial.print("Temperature : ");
  Serial.print(temperatureC);
  Serial.print("\xC2\xB0"); // affiche le symbole degré
  Serial.print("C  |  ");
  Serial.print(temperatureF);
  Serial.print("\xC2\xB0"); // affiche le symbole degré
  Serial.println("F");
}

// Function to print sensor values
void displayHallEffectSensor(int hallSensorValue, int testValue) {
  Serial.print("Hall Effect Sensor Value: ");
  Serial.println(hallSensorValue);
  Serial.print("Test Value: ");
  Serial.println(testValue);
}