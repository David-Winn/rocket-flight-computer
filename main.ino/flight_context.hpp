#pragma once
#include <Arduino.h>
#include <SD.h>
#include "pins.hpp"
#include "flight_state.hpp"

class FlightContext {
public:
    FlightContext();

    /* Sensor data */
    float xA, yA, zA;
    float xG, yG, zG;

    /* Offsets */
    float xAO, yAO, zAO;
    float xGO, yGO, zGO;

    char logFileName[20];
    File _logFile;

    char fileName[20]; 
    
    bool collecting = true;

    /* State machine */
    FlightState* currentState;
    unsigned long stateEntryTime;

    // state pointers
    class PrelaunchState* prelaunchState;
    class BoostState*     boostState;
    class CoastState*     coastState;
    class ApogeeState*    apogeeState;
    class DescentState*   descentState;
    class LandedState*    landedState;

    /* Failsafe members*/
    static const unsigned long FAILSAFE_TIMEOUT = 20000; // seconds
    unsigned long launchTime;
    bool failsafeArmed;
    bool failsafeTriggered;

    /* Hardware logic */
    void initHardware();
    void readSensors();
    void writeTelemetry();
    void tick();

    /* State machine control */
    void setState(FlightState* newState);
    void updateState();
    unsigned long timeSinceStateEntryMs() const;
    /* Failsafe moethds */
    void armFailsafe();
    void checkFailsafe();
    void logBegin();

    /* Logging */
    void logEnd();
    void logPrint(const __FlashStringHelper* msg);
    void logPrint(const char* msg);
    void logPrint(float val, int decimals = 2);
    void logPrint(unsigned long val);
    void logPrintln(const __FlashStringHelper* msg);
    void logPrintln(const char* msg);
    void logPrintln();

private:
    unsigned long lastTickMicros;
};