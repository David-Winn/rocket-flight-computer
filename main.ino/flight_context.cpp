#include "flight_context.hpp"
#include "flight_state.hpp"
#include <Arduino_LSM6DS3.h>

FlightContext::FlightContext() {
  lastTickMicros = micros();
  stateEntryTime = 0;
  fileName[0] = '\0';
}


void FlightContext::initHardware() {
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(LAUNCHBUTTON, INPUT);

  Serial.begin(115200);
  delay(100);


  if (!SD.begin(CS_SD)) {
    Serial.println(F("Error with SD"));
    while (1) {
      digitalWrite(LED0, LOW);
    }
  } else {
    digitalWrite(LED0, HIGH);
  }

  delay(1000);

  char testName[20];
  int fileNum = 1;
  
  strcpy(testName, "data.txt");
  if (!SD.exists(testName)) {
    strcpy(fileName, testName);
  } else {
    do {
      snprintf(testName, sizeof(testName), "data%d.txt", fileNum);
      fileNum++;
    } while (SD.exists(testName) && fileNum < 1000);
    
    strcpy(fileName, testName);
    Serial.print(F("File: "));
    Serial.println(fileName);
  }

  File file = SD.open(fileName, FILE_WRITE);
  if (!file) {
    Serial.println(F("File create error"));
  }
  file.close();

  if (!IMU.begin()) {
    Serial.println(F("IMU failed"));
    while (1) {
      digitalWrite(LED1, LOW);
    }
  }

  Serial.print(F("Accel rate: "));
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(F(" Hz"));

  if (!IMU.accelerationAvailable()) {
    Serial.println(F("Accel error"));
    while (1) {
      digitalWrite(LED1, HIGH);
      delay(100);
      digitalWrite(LED1, LOW);
      delay(100);
    }
  } else {
    Serial.println(F("Accel OK"));
    digitalWrite(LED1, HIGH);
  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(xA, yA, zA);
    IMU.readGyroscope(xG, yG, zG);
    xAO = -xA;
    yAO = -yA;
    zAO = -zA;
    xGO = -xG;
    yGO = -yG;
    zGO = -zG;
  }

  lastTickMicros = micros();
  stateEntryTime = millis();
}

void FlightContext::readSensors() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(xA, yA, zA);
    IMU.readGyroscope(xG, yG, zG);
    xA -= xAO;
    yA -= yAO;
    zA -= zAO;
    xG -= xGO;
    yG -= yGO;
    zG -= zGO;
  }
}

void FlightContext::writeTelemetry() {
  if (!collecting) {
    digitalWrite(LED3, LOW);
    return;
  }

  File file = SD.open(fileName, FILE_WRITE);

  if (!file) {
    digitalWrite(LED0, LOW);
    return;
  }

  file.print(xA);
  file.print(',');
  file.print(yA);
  file.print(',');
  file.print(zA);
  file.print(',');
  file.print(xG);
  file.print(',');
  file.print(yG);
  file.print(',');
  file.print(zG);
  file.print(',');
  file.println(millis());
  file.close();

  digitalWrite(LED3, HIGH);
}

void FlightContext::tick() {
  static unsigned int timeLast = micros();
  const int TICK_LENGTH = 10000;
  int uTickLength = (timeLast + TICK_LENGTH - micros() - 15000);
  delayMicroseconds(uTickLength);
  delayMicroseconds(15000);
  timeLast = micros();
}

void FlightContext::setState(FlightState* newState) {
  if (currentState == newState) {
    return;
  }
  if (currentState) {
    currentState->exit(*this);
  }
  currentState = newState;
  stateEntryTime = millis();
  if (currentState) {
    currentState->enter(*this);
  }
}

void FlightContext::updateState() {
  if (currentState) {
    currentState->update(*this);
  }
}

unsigned long FlightContext::timeSinceStateEntryMs() const {
  return millis() - stateEntryTime;
}