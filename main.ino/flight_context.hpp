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

    /* Hardware logic */
    void initHardware();
    void readSensors();
    void writeTelemetry();
    void tick();

    /* State machine control */
    void setState(FlightState* newState);
    void updateState();
    unsigned long timeSinceStateEntryMs() const;

private:
    unsigned long lastTickMicros;
};