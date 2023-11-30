// Pin Definitions
// Not working pins:A3 A4 A2 A1, 11 12 6 9 
#define LM35_TEMP_SENSOR_PIN A5
#define HALL_3144_SENSOR_PIN A0
#define TEST_PIN 5

void setup() {
  // Start serial communication at 9600 baud
  Serial.begin(9600);
  
  // Set pin modes to test board
  pinMode(TEST_PIN, INPUT);
}

void loop() {
  // Temperature Measurement
  float temperatureC = getTemperatureC();
  float temperatureF = celsiusToFahrenheit(temperatureC);

  // Display Temperature
  displayTemperature(temperatureC, temperatureF);

  // Hall Effect Sensor Reading
  int hallSensorValue = analogRead(HALL_3144_SENSOR_PIN);

  // Value For Testing
  int testValue = digitalRead(TEST_PIN);

  // Print Sensor Values
  printSensorValues(hallSensorValue, testValue);

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
void printSensorValues(int hallSensorValue, int testValue) {
  Serial.print("Hall Effect Sensor Value: ");
  Serial.println(hallSensorValue);
  // Serial.print("Test Value: ");
  // Serial.println(testValue);
}
