#define TempPin A5

// A3 A4 A2 A1
#define hallPin A0
// 11 12 6 9
int testPin = 5;


void setup() {
  // Commencer la communication série à un débit de 9600 bauds
  Serial.begin(9600);
  pinMode(testPin, INPUT);
}

void loop() {
  // Obtenir la lecture de tension de l'LM35
  int lecture = analogRead(TempPin);

  // Convertir cette lecture en tension
  float tension = lecture * (5.0 / 1023.0);

  // Convertir la tension en température en Celsius
  float temperatureC = tension * 100;

  // Afficher la température en Celsius
  Serial.print("Temperature : ");
  Serial.print(temperatureC);
  Serial.print("\xC2\xB0"); // affiche le symbole degré
  Serial.print("C  |  ");

  // Afficher la température en Fahrenheit
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
  Serial.print(temperatureF);
  Serial.print("\xC2\xB0"); // affiche le symbole degré
  Serial.println("F");

  // Contrôler les LEDs en fonction de la température
  // controlerLEDs(temperatureC);

  int sensorValue = analogRead(hallPin);
  int testValue = digitalRead(testPin);

  // Print the sensor value to the Serial Monitor
  Serial.print("Hall Effect Sensor Value: ");
  Serial.println(sensorValue);
  Serial.print("Test Value: ");
  Serial.println(testValue);

  delay(500); // attendre une seconde entre les lectures
}