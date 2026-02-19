#include "flight_context.hpp"
#include "flight_state.hpp"
#include <Arduino_LSM6DS3.h>

FlightContext::FlightContext() {
  lastTickMicros = micros();
  stateEntryTime = 0;
  fileName[0] = '\0';
  logFileName[0] = '\0';

  // Initialize failsafe
  launchTime = 0;
  failsafeArmed = false;
  failsafeTriggered = false;
}


void FlightContext::initHardware() {
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(LAUNCHBUTTON, INPUT);
  pinMode(PARACHUTE_PIN, OUTPUT);

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

  // Create matching log file for event logging
  {
    int logNum = 1;
    char testLog[20];
    strcpy(testLog, "log.txt");
    if (!SD.exists(testLog)) {
      strcpy(logFileName, testLog);
    } else {
      do {
        snprintf(testLog, sizeof(testLog), "log%d.txt", logNum);
        logNum++;
      } while (SD.exists(testLog) && logNum < 1000);
      strcpy(logFileName, testLog);
    }
    Serial.print(F("Log file: "));
    Serial.println(logFileName);

    File lf = SD.open(logFileName, FILE_WRITE);
    if (lf) {
      lf.println(F("=== Flight Event Log ==="));
      lf.close();
    }
  }

  if (!IMU.begin()) {
    Serial.println(F("IMU failed"));
    while (1) {
      digitalWrite(LED1, LOW);
    }
  }

  delay(100); // Allow time for first accel reading 

  Serial.print(F("Accel rate: "));
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(F(" Hz"));

  // Wait for first accelerometer reading (up to 500ms)
  unsigned long imuWait = millis();
  while (!IMU.accelerationAvailable()) {
      if (millis() - imuWait > 500) {
          Serial.println(F("Accel error"));
          while (1) {
              digitalWrite(LED1, HIGH);
              delay(100);
              digitalWrite(LED1, LOW);
              delay(100);
          }
      }
      delay(10);
  }
Serial.println(F("Accel OK"));
digitalWrite(LED1, HIGH);


  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(xA, yA, zA);
    IMU.readGyroscope(xG, yG, zG);
    xAO = xA;
    yAO = yA;
    zAO = zA;
    xGO = xG;
    yGO = yG;
    zGO = zG;
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
  static unsigned long timeLast = micros();
  const int TICK_LENGTH = 10000;
  long uTickLength = (long)(timeLast + TICK_LENGTH - micros());
  if (uTickLength > 0) {
    delayMicroseconds(uTickLength);
  }
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

// Parachute failsafe
void FlightContext::armFailsafe() {
  launchTime = millis();
  failsafeArmed = true;
  failsafeTriggered = false;
  Serial.println(F("[EVENT] FAILSAFE ARMED"));

  logBegin();
  logPrintln(F("[EVENT] FAILSAFE ARMED"));
  logEnd();
}

void FlightContext::checkFailsafe() {
  if (failsafeArmed && !failsafeTriggered) {
    unsigned long elapsed = millis() - launchTime;
    
    if (elapsed >= FAILSAFE_TIMEOUT) {
        Serial.println(F("* * * * * * * * * * * * * * * * * * * * *"));
        Serial.println(F("[EVENT] FAILSAFE TRIGGERED -> PARACHUTE DEPLOYMENT!"));
        Serial.println(F("* * * * * * * * * * * * * * * * * * * * *"));

        logBegin();
        logPrintln(F("* * * * * * * * * * * * * * * * * * * * *"));
        logPrintln(F("[EVENT] FAILSAFE TRIGGERED -> PARACHUTE DEPLOYMENT!"));
        logPrintln(F("* * * * * * * * * * * * * * * * * * * * *"));
        logEnd();

        digitalWrite(PARACHUTE_PIN, HIGH);
        delay(1000); 
        digitalWrite(PARACHUTE_PIN, LOW);
        
       failsafeTriggered = true;
       failsafeArmed = false;
    }
  }
}

// ---- Event logging helpers ----

void FlightContext::logBegin() {
  _logFile = SD.open(logFileName, FILE_WRITE);
}

void FlightContext::logEnd() {
  if (_logFile) {
    _logFile.flush();
    _logFile.close();
  }
}

void FlightContext::logPrint(const __FlashStringHelper* msg) {
  if (_logFile) _logFile.print(msg);
}

void FlightContext::logPrint(const char* msg) {
  if (_logFile) _logFile.print(msg);
}

void FlightContext::logPrint(float val, int decimals) {
  if (_logFile) _logFile.print(val, decimals);
}

void FlightContext::logPrint(unsigned long val) {
  if (_logFile) _logFile.print(val);
}

void FlightContext::logPrintln(const __FlashStringHelper* msg) {
  if (_logFile) _logFile.println(msg);
}

void FlightContext::logPrintln(const char* msg) {
  if (_logFile) _logFile.println(msg);
}

void FlightContext::logPrintln() {
  if (_logFile) _logFile.println();
}