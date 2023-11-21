#include "SHT85.h"
#include <RTC.h>
#include <Wire.h>
#include <WiFiS3.h>
#include <SPI.h>
#include <WiFiUdp.h>
#include <Arduino.h> 
#include <ArduinoHttpClient.h>
#include "Arduino_LED_Matrix.h"
//#include "UVX.h"
// WiFi Configuration
int status = WL_IDLE_STATUS;
char ssid[] = "INFINITUMCF42"; 
char pass[] = "wP4AkrMmvL";
//char ssid[] = "PINEMEDIA-008275"; 
//char pass[] = "severo36";
byte mac[6];
unsigned int localPort = 2390;
IPAddress timeServer(45, 231, 168, 6);
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];
char server[] = "44.235.86.3";
int port = 5000;
WiFiUDP Udp;
ArduinoLEDMatrix matrix;
// Sensor Configuration
#define SHT85_ADDRESS 0x44
SHT85 sht;

// Time Constants
#define MIN10 600000
#define HOUR 3600000
#define MIN5 300000
#define S30 30000

//
bool isPumpAndValveActive = false;
unsigned long pumpAndValveStartTime = 0;
const unsigned long pumpAndValveDuration = 8000;

// Device Pin Assignments
const int PIN_HEATER = 10;
const int PIN_FAN = 5;
const int PIN_WATER_PUMP = 13;
const int PIN_CUSTOM_LED = 3;
const int PIN_VALVE = 9;

// Thresholds
const float INITIAL_DAY_TEMP_MAX = 26.0;
const float INITIAL_DAY_TEMP_MIN = 24.0;
const float FINAL_DAY_TEMP_MAX = 30.0;
const float FINAL_DAY_TEMP_MIN = 15.0;

const float INITIAL_NIGHT_TEMP_MAX = 25.0; // Assuming a value, adjust as needed
const float INITIAL_NIGHT_TEMP_MIN = 23.0; // Assuming a value, adjust as needed
const float FINAL_NIGHT_TEMP_MAX = 25.0;   // Assuming a value, adjust as needed
const float FINAL_NIGHT_TEMP_MIN = 10.0;   // Assuming a value, adjust as needed

const float INITIAL_HUMIDITY = 90.0;
const float FINAL_HUMIDITY = 70.0;

unsigned long lastWateringMorning = 0;
unsigned long lastWateringEvening = 0;
unsigned long lastGustTime = 0;
int gustCount = 0;

int temperature; 
float airHumidity;
const int timeZone = -6;
bool isDST = false;
// Flag to check if system is in constant mode
bool isConstantMode = false;
float userInitialDayTempMax = 25;
float userFinalDayTempMax = 30;
bool isConstantTempDayMode;

float userInitialDayTempMin;  // Initial minimum day temperature set by the user
float userFinalDayTempMin;    // Final minimum day temperature set by the user
float userInitialNightTempMin;  // Initial minimum night temperature set by the user
float userFinalNightTempMin;

float userInitialNightTempMax = 20;
float userFinalNightTempMax = 25;
bool isConstantTempNightMode;

float userInitialHumidity = 70;
float userFinalHumidity = 70;
bool isConstantHumidityMode;
float userTargetHumidity;

int standbyFanSpeed = 90; // Standby speed for the fan (PWM value between 0-255)
int maxFanSpeed = 255; // Maximum speed for the fan (PWM value between 0-255)
int maxLightLevel = 255; // Maximum light level (PWM value between 0-255)

int constantLightLevel = 255; // Constant light level if user chooses constant mode (PWM value between 0-255)
bool isConstantLightMode; // Flag to check if system is in constant light mode
//tiempo
unsigned long previousMillisHeater = 0;
const long intervalHeaterOn = 7000;
const long intervalHeaterOff = 14000;
bool heaterOn = false;

unsigned long previousMillisFan = 0;
const long intervalFanHigh = 2000;

unsigned long previousMillisValve = 0;
const long intervalValveOn = 100;
const long intervalValveOff = 100;
bool valveOn = false;

unsigned long previousMillisWaterPump = 0;
const long intervalWaterPumpOn = 6000;
bool waterPumpOn = false;

unsigned long previousMillisGust = 0;
const long intervalGust = 5000;

bool enableWatering = true;
bool enableGusts = true;
int wateringHourMorning = 9;  // Default 9 AM
int wateringHourEvening = 17;  // Default 5 PM

unsigned long previousMillisWateringMorning = 0;
unsigned long previousMillisWateringEvening = 0;
unsigned long lastHumidityReadMillis = 0;
const long readInterval = 100; // 100 ms
int readCount = 0;
unsigned long lastNTPUpdate = 0;  // Stores the last time the NTP server was queried
const unsigned long oneDayMillis = 86400000;  // One day in milliseconds

unsigned long lastSyncTime = 0; 
const long syncInterval = 8 * 60 * 60 * 1000;  // 8 hours in milliseconds

//variables de riego
enum WateringState {
  STATE_IDLE,
  STATE_OPEN_VALVE,
  STATE_PUMP_ON,
  STATE_PUMP_OFF,
  STATE_CLOSE_VALVE
};
WateringState currentWateringState = STATE_IDLE;
WateringState currentWateringHumidityState = STATE_IDLE;
void readTempAndHumidity() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastHumidityReadMillis >= readInterval) {
    lastHumidityReadMillis = currentMillis;
    sht.read();
    temperature = sht.getTemperature();
    airHumidity = sht.getHumidity();
    if (isnan(airHumidity)) {
      Serial.println("Error reading humidity");
    }
    readCount++;
    if (readCount >= 50) {
      readCount = 0;
      // Do something with the 50 readings, like averaging or other logic
    }
  }
}
bool isLeapYear(int year) {
  return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
}
void calculateDate(int& day, int& month, int& year, String& epochDay, String& epochMonth, unsigned long epoch) {
  const int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  const String daysOfWeek[] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};
  const String months[] = {"JANUARY", "FEBRUARY", "MARCH", "APRIL", "MAY", "JUNE", "JULY", "AUGUST", "SEPTEMBER", "OCTOBER", "NOVEMBER", "DECEMBER"};

  year = 1970;
  unsigned long days = epoch / 86400;
  int dayOfWeek = (4 + days) % 7;  // 1 Jan 1970 was a Thursday

  while (days >= (isLeapYear(year) ? 366 : 365)) {
    days -= (isLeapYear(year) ? 366 : 365);
    year++;
  }

  if (isLeapYear(year)) {
    days++;
  }

  month = 0;
  while (days >= daysInMonth[month]) {
    days -= daysInMonth[month];
    month++;
  }

  day = days + 1;
  month++;  // Convert to 1-indexed

  epochDay = daysOfWeek[dayOfWeek];
  epochMonth = months[month - 1];
}
float readUserFloatInput(unsigned long timeout = 10000) {
  unsigned long startTime = millis();
  String inputString = "";

  while (millis() - startTime < timeout) {
    if (Serial.available() > 0) {
      char inChar = Serial.read();
      if (inChar == '\n') {
        return inputString.toFloat();
      } else {
        inputString += inChar;
      }
    }
  }

  Serial.println("Timed out waiting for input!");
  return -1.0; // You can use a sentinel value to indicate that no input was received
}
float readAndValidateUserFloatInput(float minLimit, float maxLimit, unsigned long timeout = 10000) {
  float value = readUserFloatInput(timeout);
  
  if (value >= minLimit && value <= maxLimit) {
    return value;
  } else {
    Serial.println("Invalid value entered. Must be between " + String(minLimit) + " and " + String(maxLimit));
    return -1.0;
  }
}
int readAndValidateUserIntInput(int minValue, int maxValue) {
    while (Serial.available() == 0) {}
    String userInput = Serial.readStringUntil('\n');
    int value = userInput.toInt();
    if (value >= minValue && value <= maxValue) {
        return value;
    }
    return -1;  // Invalid input
}
void clearSerialBuffer() {
    while(Serial.available() > 0) {
        Serial.read();
    }
}
String readStringFromSerial() {
    while (Serial.available() == 0) {} // Wait for user input
    return Serial.readStringUntil('\n');
}
void setup() {
  RTC.begin();
  Serial.begin(9600);
  // Initialize sensor tempHum()
  Wire.begin();
  sht.begin(SHT85_ADDRESS);
  Wire.setClock(100000);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
    // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("¡Comunicación con el módulo WiFi fallida!");
    // don't continue
    while (true);
  }
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Por favor, actualice el firmware");
  }
  while (status != WL_CONNECTED) {
    Serial.print("Intentando conectarse a SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  Serial.println("Conectado a WiFi");
  printWifiStatus();

  synchronizeWithNTP();

  // Initialize triggers
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_WATER_PUMP, OUTPUT);
  pinMode(PIN_CUSTOM_LED, OUTPUT);
  pinMode(PIN_VALVE, OUTPUT);

  // Start with everything off except the fan
  digitalWrite(PIN_WATER_PUMP, LOW);
  digitalWrite(PIN_HEATER, LOW);
  digitalWrite(PIN_FAN, LOW);
  digitalWrite(PIN_CUSTOM_LED, LOW);
  // Ask user to set initial and final conditions for day temperature
Serial.println("Ingrese 'c' para modo de temperatura diurna constante o 'v' para modo variable:");
String modeDayTemp = readStringFromSerial();
//clearSerialBuffer(); // Limpiar buffer
delay(200);
if (modeDayTemp == "c") {
  isConstantTempDayMode = true;
  Serial.println("Ingrese el valor constante de temperatura diurna:");
  float constantDayTemp = readAndValidateUserFloatInput(10.0, 50.0);
  if (constantDayTemp != -1.0) {
    userInitialDayTempMax = constantDayTemp;
    Serial.println("Temperatura diurna establecida en valor constante: " + String(userInitialDayTempMax) + "°C");
  } else {
    Serial.println("No se recibió entrada para temperatura diurna constante. Usando valor predeterminado.");
  }
} else if (modeDayTemp == "v") {
  isConstantTempDayMode = false;
  Serial.println("Ingrese la temperatura mínima diurna inicial:");
  userInitialDayTempMin = readAndValidateUserFloatInput(10.0, 50.0);  // Ajustar límites según sea necesario
  Serial.println("Temperatura mínima diurna inicial establecida en: " + String(userInitialDayTempMin) + "°C");
  Serial.println("Ingrese la temperatura mínima diurna final:");
  userFinalDayTempMin = readAndValidateUserFloatInput(10.0, 50.0);  // Ajustar límites según sea necesario
  Serial.println("Temperatura mínima diurna final establecida en: " + String(userFinalDayTempMin) + "°C");
  Serial.println("Ingrese la temperatura máxima diurna inicial:");
  float initialDayTempMax = readAndValidateUserFloatInput(10.0, 50.0);
  if (initialDayTempMax != -1.0) {
    userInitialDayTempMax = initialDayTempMax;
    Serial.println("Temperatura máxima diurna inicial establecida en: " + String(userInitialDayTempMax) + "°C");
  } else {
    Serial.println("No se recibió entrada para la temperatura máxima diurna inicial. Usando valor predeterminado.");
  }
  
  Serial.println("Ingrese la temperatura máxima diurna final:");
  float finalDayTempMax = readAndValidateUserFloatInput(10.0, 50.0);
  if (finalDayTempMax != -1.0) {
    userFinalDayTempMax = finalDayTempMax;
    Serial.println("Temperatura máxima diurna final establecida en: " + String(userFinalDayTempMax) + "°C");
  } else {
    Serial.println("No se recibió entrada para la temperatura máxima diurna final. Usando valor predeterminado.");
  }
}

// Solicitar al usuario que establezca las condiciones iniciales y finales para la temperatura nocturna
Serial.println("Ingrese 'c' para modo de temperatura nocturna constante o 'v' para modo variable:");
while (Serial.available() == 0) {}
String modeNightTemp = readStringFromSerial();
clearSerialBuffer(); // Limpiar buffer
delay(200);

if (modeNightTemp == "c") {
  isConstantTempNightMode = true;
  Serial.println("Ingrese el valor constante de temperatura nocturna:");
  float constantNightTemp = readAndValidateUserFloatInput(10.0, 50.0);
  if (constantNightTemp != -1.0) {
    userInitialNightTempMax = constantNightTemp;
    Serial.println("Temperatura nocturna establecida en valor constante: " + String(constantNightTemp) + "°C");
  } else {
    Serial.println("No se recibió entrada para temperatura nocturna constante. Usando valor predeterminado.");
  }
} else if (modeNightTemp == "v") {
  isConstantTempNightMode = false;
  Serial.println("Ingrese la temperatura mínima nocturna inicial:");
  userInitialNightTempMin = readAndValidateUserFloatInput(10.0, 50.0);
  Serial.println("Temperatura mínima nocturna inicial establecida en: " + String(userInitialNightTempMin) + "°C");
  Serial.println("Ingrese la temperatura mínima nocturna final:");
  userFinalNightTempMin = readAndValidateUserFloatInput(10.0, 50.0);
  Serial.println("Temperatura mínima nocturna final establecida en: " + String(userFinalNightTempMin) + "°C");
  Serial.println("Ingrese la temperatura máxima nocturna inicial:");
  userInitialNightTempMax = readAndValidateUserFloatInput(10.0, 50.0);
  Serial.println("Temperatura máxima nocturna inicial establecida en: " + String(userInitialNightTempMax) + "°C");
  Serial.println("Ingrese la temperatura máxima nocturna final:");
  userFinalNightTempMax = readAndValidateUserFloatInput(10.0, 50.0);
  Serial.println("Temperatura máxima nocturna final establecida en: " + String(userFinalNightTempMax) + "°C");
}


// Para la humedad
Serial.println("Ingrese 'c' para modo de humedad constante o 'v' para modo variable:");
while (Serial.available() == 0) {}
String modeHumidity = readStringFromSerial();
clearSerialBuffer(); // Limpiar buffer
delay(200);
if (modeHumidity == "c") {
  isConstantHumidityMode = true;
    // Si el usuario elige el modo constante, solo necesitamos un valor
  Serial.println("Ingrese el valor constante de humedad:");
  userInitialHumidity = readAndValidateUserFloatInput(20.0, 95.0);  // Ajustar límites según sea necesario
  if (userInitialHumidity != -1.0) {
    Serial.println("Humedad objetivo establecida en: " + String(userInitialHumidity) + "%");
  } else {
    Serial.println("No se recibió una entrada válida para la humedad objetivo. Usando valor predeterminado.");
  }
} else if (modeHumidity == "v") {
  isConstantHumidityMode = false;
  // Para el modo variable, necesitamos tanto la humedad inicial como la final
  Serial.println("Ingrese la humedad inicial:");
  userInitialHumidity = readAndValidateUserFloatInput(20.0, 95.0);
  if (userInitialHumidity != -1.0) {
    Serial.println("Humedad inicial establecida en: " + String(userInitialHumidity) + "%");
  } else {
    Serial.println("No se recibió una entrada válida para la humedad inicial. Usando valor predeterminado.");
  }

  Serial.println("Ingrese la humedad final:");
  userFinalHumidity = readAndValidateUserFloatInput(20.0, 95.0);
  if (userFinalHumidity != -1.0) {
    Serial.println("Humedad final establecida en: " + String(userFinalHumidity) + "%");
  } else {
    Serial.println("No se recibió una entrada válida para la humedad final. Usando valor predeterminado.");
  }
}

// Para el nivel de luz
Serial.println("Ingrese 'c' para nivel de luz constante o 'v' para modo variable:");
while (Serial.available() == 0) {}
String modeLight = readStringFromSerial();
clearSerialBuffer(); // Limpiar buffer
delay(200);
if (modeLight == "c") {
  Serial.println("Ingrese el nivel de luz constante (0-255):");
  constantLightLevel = readAndValidateUserFloatInput(0, 255);
  Serial.println("Nivel de Luz constante establecido en: " + String(constantLightLevel));
} else if (modeLight == "v") {
  Serial.println("Ingrese el nivel máximo de luz (0-255):");
  maxLightLevel = readAndValidateUserFloatInput(0, 255);
  Serial.println("Nivel máximo de Luz establecido en: " + String(maxLightLevel));
}

// Preguntar al usuario sobre la configuración de riego
Serial.println("¿Desea habilitar el riego? Ingrese 'y' para Sí o 'n' para No:");
while (Serial.available() == 0) {}
String wateringChoice = readStringFromSerial();
enableWatering = (wateringChoice == "y");
clearSerialBuffer(); // Limpiar buffer
delay(200);
if (enableWatering) {
  Serial.println("Ingrese la hora de riego matutino (0-23):");
  wateringHourMorning = readAndValidateUserFloatInput(0, 23);
  Serial.println("Riego matutino programado para: " + String(wateringHourMorning));
  Serial.println("Ingrese la hora de riego vespertino (0-23):");
  wateringHourEvening = readAndValidateUserFloatInput(0, 23);
  Serial.println("Riego vespertino programado para: " + String(wateringHourEvening));
}

// Preguntar al usuario sobre la configuración de ráfagas de viento
Serial.println("¿Desea habilitar ráfagas de viento? Ingrese 'y' para Sí o 'n' para No:");
while (Serial.available() == 0) {}
String gustsChoice = readStringFromSerial();
enableGusts = (gustsChoice == "y");
clearSerialBuffer(); // Limpiar buffer
delay(200);
// Preguntar al usuario sobre la velocidad del ventilador en espera
Serial.println("Ingrese la velocidad del ventilador en espera (valor PWM entre 0-255):");
standbyFanSpeed = readAndValidateUserIntInput(0, 255);
if (standbyFanSpeed != -1) {
  Serial.println("Velocidad del ventilador en espera establecida en: " + String(standbyFanSpeed));
} else {
  Serial.println("Entrada inválida recibida para la velocidad del ventilador en espera. Usando valor predeterminado.");
}

// Preguntar al usuario sobre la velocidad máxima del ventilador
Serial.println("Ingrese la velocidad máxima del ventilador (valor PWM entre 0-255):");
maxFanSpeed = readAndValidateUserIntInput(0, 255);
if (maxFanSpeed != -1) {
  Serial.println("Velocidad máxima del ventilador establecida en: " + String(maxFanSpeed));
} else {
  Serial.println("Entrada inválida recibida para la velocidad máxima del ventilador");
  }
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

unsigned long sendNTPpacket(IPAddress& address) {
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0; // Stratum, or type of clock
  packetBuffer[2] = 6; // Polling Interval
  packetBuffer[3] = 0xEC; // Peer Clock Precision
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  Udp.beginPacket(address, 123);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.println("MACarrones");
  WiFi.macAddress(mac);

  // Print the MAC address
  for (byte i = 0; i < 6; i++) {
    Serial.print(mac[i], HEX);
    if (i < 5) {
      Serial.print(":");
    }
  }
  Serial.println();
}


void loop() {
  updatePumpAndValve();
  unsigned long currentMillis = millis();
  analogWrite(PIN_FAN, standbyFanSpeed);
  if (currentMillis - lastSyncTime >= syncInterval) {
    lastSyncTime = currentMillis;
    synchronizeWithNTP();
  }
  RTCTime currentTime;
  RTC.getTime(currentTime);
  Serial.print(currentTime.getDayOfMonth());
  Serial.print("/");
  Serial.print(Month2int(currentTime.getMonth()));
  Serial.print("/");
  Serial.print(currentTime.getYear());
  Serial.print(" - ");

  // Print time (HH/MM/SS)
  Serial.print(currentTime.getHour());
  Serial.print(":");
  Serial.print(currentTime.getMinutes());
  Serial.print(":");
  Serial.println(currentTime.getSeconds());

  int currentHour = currentTime.getHour();
  int currentMinute = currentTime.getMinutes();
  int currentDay = currentTime.getDayOfMonth();
  readTempAndHumidity();
  Serial.println();
  Serial.print("temperatura: ");
  Serial.println(temperature, 1);
  Serial.print("humedad: ");
  Serial.println(airHumidity, 1);
  float dayTempMax, dayTempMin, nightTempMax, nightTempMin, targetHumidity;
  //osciladores climaticos usando funcion senoideal
  int minutesSinceMidnight = currentHour * 60 + currentMinute;
  float oscillationFactor;
  if (currentHour >= 7 && currentHour <= 22) {  // Daytime
    // Peak at 1pm (13:00 or 780 minutes since midnight)
    oscillationFactor = sin(PI * (minutesSinceMidnight - 780) / (15.5 * 60));
  } else {  // Nighttime
    // Lowest at 3am (3:00 or 180 minutes since midnight)
    oscillationFactor = cos(PI * (minutesSinceMidnight - 180) / (15.5 * 60));
  }

  int ledIntensity = 0;
  float ledOscillationFactor;
  
  if (isConstantLightMode) {
    // Set LED intensity to constant value
    ledIntensity = constantLightLevel;
    ledOscillationFactor = 0;
  } else {
   // Peak intensity at noon (12:00 or 720 minutes since midnight)
  if (currentHour >= 7 && currentHour <= 21) {  // Daytime from 7 AM to 9 PM
    ledOscillationFactor = sin(PI * (minutesSinceMidnight - 720) / (14 * 60));
    // Map oscillation factor to PWM value (0 to maxLightLevel)
    ledIntensity = (ledOscillationFactor * (maxLightLevel));
  } else {  // Nighttime
    ledOscillationFactor = 0; // LED off
    ledIntensity = 0;
  }
  }
  // Write PWM value to LED
  analogWrite(PIN_CUSTOM_LED, ledIntensity);

  if (isConstantTempDayMode) {
    // Use the constant value provided by the user
    dayTempMax = userInitialDayTempMax; // or userFinalDayTempMax, depending on which one holds the constant value
    dayTempMin = userInitialDayTempMax; // Similarly, choose the appropriate variable
  } else {
    // Use the variable values, changing over time
    dayTempMax = map(currentDay, 0, 90, userInitialDayTempMax, userFinalDayTempMax);
    dayTempMin = map(currentDay, 0, 90, userInitialDayTempMin, userFinalDayTempMin); // Assuming you have userInitialDayTempMin
  }

  // Do the same for night temperature
  if (isConstantTempNightMode) {
    nightTempMax = userInitialNightTempMax; // or userFinalNightTempMax, depending on which one holds the constant value
    nightTempMin = userInitialNightTempMax; // Similarly, choose the appropriate variable
  } else {
    nightTempMax = map(currentDay, 0, 90, userInitialNightTempMax, userFinalNightTempMax);
    nightTempMin = map(currentDay, 0, 90, userInitialNightTempMin, userFinalNightTempMin); // Assuming you have userInitialNightTempMin
  }

  // And also for humidity
  if (isConstantHumidityMode) {
    targetHumidity = userInitialHumidity; // or userFinalHumidity, depending on which one holds the constant value
  } else {
    targetHumidity = map(currentDay, 0, 90, userInitialHumidity, userFinalHumidity);
  }

  float currentTempMax = (currentHour >= 7 && currentHour <= 22) ? dayTempMax : nightTempMax;
  float currentTempMin = (currentHour >= 7 and currentHour <= 22) ? dayTempMin : nightTempMin;
  float targetTemperature = currentTempMin + (currentTempMax - currentTempMin) * (oscillationFactor + 1) / 2;

  // Temperature control
  if (temperature < targetTemperature) {
    if (!heaterOn && currentMillis - previousMillisHeater >= intervalHeaterOff) {
      heaterOn = true;
      analogWrite(PIN_FAN, maxFanSpeed);
      previousMillisHeater = currentMillis;
      digitalWrite(PIN_HEATER, HIGH);
      Serial.print("calentando");
    } else if (heaterOn && currentMillis - previousMillisHeater >= intervalHeaterOn) {
      heaterOn = false;
      previousMillisHeater = currentMillis;
      digitalWrite(PIN_HEATER, LOW);
    }
  }
  readTempAndHumidity();  // Update sensor readings
  if (temperature > targetTemperature && currentMillis - previousMillisFan >= intervalFanHigh) {
    previousMillisFan = currentMillis;
    analogWrite(PIN_FAN, maxFanSpeed);
    analogWrite(PIN_FAN, standbyFanSpeed);
  }

  // Humidity control
  if (airHumidity < targetHumidity) {
    analogWrite(PIN_FAN, maxFanSpeed);
    startPumpAndValve();
    Serial.print("regadnoo");
    delay(1000);
  } else if (airHumidity > (targetHumidity+20)) {
    analogWrite(PIN_FAN, maxFanSpeed);  // Max speed
    delay(30000);
    readTempAndHumidity();  // Update sensor readings
    analogWrite(PIN_FAN, standbyFanSpeed);  // Fan idle speed
  }

  if (enableGusts && currentDay >= 30) {
    int maxGusts = map(currentDay, 30, 90, 1, 6);
    unsigned long currentTimeMillis = millis();
    if (currentTimeMillis - lastGustTime > 86400000 / maxGusts) {  // 86400000 ms in a day
      // Perform a gust
      analogWrite(PIN_FAN, 255);
      delay(5000);
      analogWrite(PIN_FAN, 90);
      delay(5000);
      analogWrite(PIN_FAN, 255);
      delay(5000);
      analogWrite(PIN_FAN, 90);  // Back to idle speed
      lastGustTime = currentTimeMillis;
      gustCount++;
      if (gustCount >= maxGusts) {
        gustCount = 0;
      }
    }
  }
  if (enableWatering) {
    if (currentHour == wateringHourMorning && (millis() - lastWateringMorning) > 86400000) {
      // Water at user-defined morning hour
      waterPlants();
      lastWateringMorning = millis();
    } else if (currentHour == wateringHourEvening && (millis() - lastWateringEvening) > 86400000) {
      // Water at user-defined evening hour
      waterPlants();
      lastWateringEvening = millis();
    }
  }
  sendDataToServer(temperature, airHumidity, currentTime);
  delay(10000);
}
WiFiClient client; // Global WiFiClient instance

bool connectToServer(const char* server, int port) {
  int retries = 3;
  while (!client.connected() && retries--) {
    if (!client.connect(server, port)) {
      Serial.println("Connection failed. Retrying...");
      delay(1000);
    }
  }
  return client.connected();
}

bool canStartWatering() {
  if (connectToServer("44.235.86.3", 5000)) {
    Serial.println("Connected to the server.");
    client.println("GET /status HTTP/1.1");
    client.println("Host: 44.235.86.3");
    client.println("Connection: close");
    client.println();

    String payload = "";
    while (client.connected()) {
      if (client.available()) {
        String line = client.readStringUntil('\n');
        Serial.println("Received: " + line);
        payload += line;
      }
    }

    if (payload.indexOf("\"status\":\"free\"") != -1) {
      return true;
    }
  } else {
    Serial.println("Failed to connect to the server.");
  }
  return false;
}
void waterPlants() {
  static unsigned long stateStartTime = 0;
  unsigned long currentMillis = millis();
  Serial.println("attempt to water");

  switch (currentWateringState) {
    case STATE_IDLE:
      if (canStartWatering()) {
        if (client.connect("44.235.86.3", 5000)) {
          Serial.println("Attempting to start watering...");
          client.println("POST /startWatering HTTP/1.1");
          client.println("Host: 44.235.86.3");
          client.println("Connection: close");
          client.println();
          
          String payload = "";
          while (client.connected()) {
            if (client.available()) {
              String line = client.readStringUntil('\n');
              Serial.println("Received: " + line);
              payload += line;
            }
          }
          client.stop();
        }
        currentWateringState = STATE_OPEN_VALVE;
        stateStartTime = currentMillis;
      }
      break;

    case STATE_OPEN_VALVE:
      digitalWrite(PIN_VALVE, HIGH);
      if (currentMillis - stateStartTime > 2000) {
        currentWateringState = STATE_PUMP_ON;
        stateStartTime = currentMillis;
        Serial.println("valve on");
      }
      break;

    case STATE_PUMP_ON:
      digitalWrite(PIN_WATER_PUMP, HIGH);
      if (currentMillis - stateStartTime > 20000) {
        currentWateringState = STATE_PUMP_OFF;
        stateStartTime = currentMillis;
        Serial.println("watered successfully");
      }
      break;

    case STATE_PUMP_OFF:
      digitalWrite(PIN_WATER_PUMP, LOW);
      if (currentMillis - stateStartTime > 3000) {
        currentWateringState = STATE_CLOSE_VALVE;
        stateStartTime = currentMillis;
      }
      break;

    case STATE_CLOSE_VALVE:
      digitalWrite(PIN_VALVE, LOW);
      if (client.connect("44.235.86.3", 5000)) {
        client.println("POST /stopWatering HTTP/1.1");
        client.println("Host: 44.235.86.3");
        client.println("Connection: close");
        client.println();
        client.stop();
      }
      currentWateringState = STATE_IDLE;
      break;
  }
}
void waterBasedOnHumidityN(float airHumidity, float targetHumidity) {
  static unsigned long stateStartTimeHumidity = 0;
  unsigned long currentMillis = millis();

  if (airHumidity < targetHumidity) {
    switch (currentWateringHumidityState) {
      case STATE_IDLE:
        
      currentWateringHumidityState = STATE_OPEN_VALVE;
      stateStartTimeHumidity = currentMillis;
        break;

      case STATE_OPEN_VALVE:
        digitalWrite(PIN_VALVE, HIGH); // Open the water valve
        if (currentMillis - stateStartTimeHumidity > 100) { // Wait for 2 seconds
          currentWateringHumidityState = STATE_PUMP_ON;
          stateStartTimeHumidity = currentMillis;
        }
        break;

      case STATE_PUMP_ON:
        digitalWrite(PIN_WATER_PUMP, HIGH); // Start the water pump
        if (currentMillis - stateStartTimeHumidity > 8000) { // Run pump for 5 seconds
          currentWateringHumidityState = STATE_PUMP_OFF;
          stateStartTimeHumidity = currentMillis;
        }
        break;

      case STATE_PUMP_OFF:
        digitalWrite(PIN_WATER_PUMP, LOW); // Stop the water pump
        if (currentMillis - stateStartTimeHumidity > 100) { // Wait for 3 seconds
          currentWateringHumidityState = STATE_CLOSE_VALVE;
          stateStartTimeHumidity = currentMillis;
        }
        break;

      case STATE_CLOSE_VALVE:
        digitalWrite(PIN_VALVE, LOW); // Close the water valve
        currentWateringHumidityState = STATE_IDLE; // Return to idle state
        break;
    }
  }
}

void waterBasedOnHumidity(float airHumidity, float targetHumidity) {
  static unsigned long stateStartTimeHumidity = 0;
  unsigned long currentMillis = millis();

   if (airHumidity < targetHumidity) {
    switch (currentWateringHumidityState) {
      case STATE_IDLE:
        if (canStartWatering()) {
          if (client.connect("44.235.86.3", 5000)) {
            client.println("POST /startWatering HTTP/1.1");
            client.println("Host: 44.235.86.3");
            client.println("Connection: close");
            client.println();
          }
          currentWateringHumidityState = STATE_OPEN_VALVE;
          stateStartTimeHumidity = currentMillis;
        }
        break;

      case STATE_OPEN_VALVE:
        digitalWrite(PIN_VALVE, HIGH);
        if (currentMillis - stateStartTimeHumidity > 2000) {
          currentWateringHumidityState = STATE_PUMP_ON;
          stateStartTimeHumidity = currentMillis;
        }
        break;

      case STATE_PUMP_ON:
        digitalWrite(PIN_WATER_PUMP, HIGH);
        if (currentMillis - stateStartTimeHumidity > 5000) {
          currentWateringHumidityState = STATE_PUMP_OFF;
          stateStartTimeHumidity = currentMillis;
        }
        break;

      case STATE_PUMP_OFF:
        digitalWrite(PIN_WATER_PUMP, LOW);
        if (currentMillis - stateStartTimeHumidity > 3000) {
          currentWateringHumidityState = STATE_CLOSE_VALVE;
          stateStartTimeHumidity = currentMillis;
        }
        break;

      case STATE_CLOSE_VALVE:
        digitalWrite(PIN_VALVE, LOW);
        if (client.connect("44.235.86.3", 5000)) {
          client.println("POST /stopWatering HTTP/1.1");
          client.println("Host: 44.235.86.3");
          client.println("Connection: close");
          client.println();
        }
        currentWateringHumidityState = STATE_IDLE;
        break;
    }
  }
}
void sendDataToServer(float temperature, float humidity, RTCTime currentTime) {
  String server = "44.235.86.3";
  int port = 6000;
  
  // Retrieve the board ID
  String boardID = getBoardID();  // This will be "Reactor_0", "Reactor_UV_1", "Reactor_UV_2", "Reactor_UV_3", or "UNKNOWN_REACTOR"

  // Convert data to JSON
  String payload = "{";
  payload += "\"id\":\"" + boardID + "\",";
  payload += "\"temperature\":" + String(temperature) + ",";
  payload += "\"humidity\":" + String(humidity) + ",";
  payload += "\"time\":\"" + String(currentTime.getHour()) + ":" + String(currentTime.getMinutes()) + ":" + String(currentTime.getSeconds()) + "\"";
  payload += "}";

  Serial.println(payload);

  if (connectToServer(server.c_str(), port)) {
    client.println("POST /sendData HTTP/1.1");
    client.println("Host: " + server);
    client.println("Connection: close");
    client.println("Content-Type: application/json");
    client.println("Content-Length: " + String(payload.length()));
    client.println();
    client.println(payload);
    client.stop();
  } else {
    Serial.println("Failed to connect to the server of time and conditions.");
  }
}
void synchronizeWithNTP() {
    Serial.println("Synchronizing with NTP server...");

    Udp.begin(localPort);
    sendNTPpacket(timeServer);
    delay(1000);  // Consider using a non-blocking delay in the future

    if (Udp.parsePacket()) {
        Serial.println("NTP packet received");
        Udp.read(packetBuffer, NTP_PACKET_SIZE);

        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
        unsigned long secsSince1900 = highWord << 16 | lowWord;
        const unsigned long seventyYears = 2208988800UL;
        unsigned long epoch = secsSince1900 - seventyYears;
        long epochLocal = epoch + 3600L * timeZone;

        if (isDST) {
            epochLocal += 3600L;
        }

        int Second = epochLocal % 60;
        int Minute = (epochLocal % 3600) / 60;
        int Hour = (epochLocal % 86400L) / 3600L;

        String epochDay, epochMonth;
        int Day, Month, Year;
        calculateDate(Day, Month, Year, epochDay, epochMonth, epoch);
        
        RTCTime startTime(Day, Month::SEPTEMBER, Year, Hour, Minute, Second, DayOfWeek::FRIDAY, SaveLight::SAVING_TIME_ACTIVE);    
        RTC.setTime(startTime);
    } else {
        Serial.println("Failed to receive NTP packet");
    }
}

String getBoardID() {
  WiFi.macAddress(mac);
  
  // Reactor_0 MAC address
  byte REACTOR_0_MAC[6] = {0x2C, 0xA5, 0xCF, 0x75, 0x54, 0xDC};
  
  bool isReactor0 = true;
  for (int i = 0; i < 6; i++) {
    if (mac[i] != REACTOR_0_MAC[i]) {
      isReactor0 = false;
      break;
    }
  }

  if (isReactor0) {
    return "Reactor_0";
  }

  // Reactor_UV_1 MAC address
  byte REACTOR_UV_1_MAC[6] = {0xAC, 0xB8, 0xC3, 0x75, 0x54, 0xDC};
  
  bool isReactorUV1 = true;
  for (int i = 0; i < 6; i++) {
    if (mac[i] != REACTOR_UV_1_MAC[i]) {
      isReactorUV1 = false;
      break;
    }
  }

  if (isReactorUV1) {
    return "Reactor_UV_1";
  }

  // Reactor_UV_2 MAC address
  byte REACTOR_UV_2_MAC[6] = {0xD8, 0x55, 0xC5, 0x75, 0x54, 0xDC};
  
  bool isReactorUV2 = true;
  for (int i = 0; i < 6; i++) {
    if (mac[i] != REACTOR_UV_2_MAC[i]) {
      isReactorUV2 = false;
      break;
    }
  }

  if (isReactorUV2) {
    return "Reactor_UV_2";
  }

  // Reactor_UV_3 MAC address
  byte REACTOR_UV_3_MAC[6] = {0xA0, 0xE1, 0xD0, 0x75, 0x54, 0xDC};
  
  bool isReactorUV3 = true;
  for (int i = 0; i < 6; i++) {
    if (mac[i] != REACTOR_UV_3_MAC[i]) {
      isReactorUV3 = false;
      break;
    }
  }

  if (isReactorUV3) {
    return "Reactor_UV_3";
  }
  
  return "UNKNOWN_REACTOR";  // Default if no match
}
void stopPumpAndValve() {
    digitalWrite(PIN_VALVE, LOW); // Close the valve
    digitalWrite(PIN_WATER_PUMP, LOW); // Stop the pump
    isPumpAndValveActive = false;
}
void updatePumpAndValve() {
    if (isPumpAndValveActive && millis() - pumpAndValveStartTime >= pumpAndValveDuration) {
        stopPumpAndValve();
    }
}


