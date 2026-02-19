#include "descent_state.hpp"
#include "coast_state.hpp"
#include "flight_context.hpp"
#include "landed_state.hpp"
#include <Arduino.h>


// Parachute deployment threshold
static constexpr float DEPLOY_DISTANCE = 0;


// Velocity must be very low and acceleration near gravity
static constexpr float LANDING_VELOCITY_THRESHOLD = 1.0f;  // m/s
static constexpr float LANDING_ACCEL_THRESHOLD = 10.0f;    // m/s^2 (close to 1g)
static constexpr int LANDING_CONSECUTIVE = 10;             


void DescentState::enter(FlightContext& ctx) {
    Serial.println(F("[STATE] Descent enter"));
    ctx.logBegin();
    ctx.logPrintln(F("[STATE] Descent enter"));
    ctx.logEnd();
    lastAlt = ctx.zA; 

    parachuteDeployed = false;
    stableCount = 0;
    
    // Initialize distance algorithm at start of descent
    distanceAlg.reset();
    lastUpdateMicros = micros();
}

void DescentState::update(FlightContext& ctx) {

    // Calculate time delta since last update
    unsigned long currentMicros = micros();
    float dtSeconds = (currentMicros - lastUpdateMicros) / 1000000.0f;
    lastUpdateMicros = currentMicros;
    
    // Update distance algorithm with current acceleration
    distanceAlg.update(ctx.zA, dtSeconds);
    
    float distanceFallen = distanceAlg.getDistance();
    float velocity = distanceAlg.getVelocity();


    // Deploy parachute after falling DEPLOY_DISTANCE meters
    if (!parachuteDeployed && distanceFallen >= DEPLOY_DISTANCE) {

        digitalWrite(PARACHUTE_PIN, HIGH);
        delay(100);
        digitalWrite(PARACHUTE_PIN, LOW);

        Serial.println(F("========================================"));
        Serial.println(F("[EVENT] PARACHUTE DEPLOYMENT!"));
        Serial.print(F("[INFO] Distance fallen: "));
        Serial.print(distanceFallen);
        Serial.println(F("m"));
        Serial.print(F("[INFO] Velocity: "));
        Serial.print(velocity);
        Serial.println(F("m/s"));
        Serial.println(F("========================================"));

        ctx.logBegin();
        ctx.logPrintln(F("========================================"));
        ctx.logPrintln(F("[EVENT] PARACHUTE DEPLOYMENT!"));
        ctx.logPrint(F("[INFO] Distance fallen: "));
        ctx.logPrint(distanceFallen, 2);
        ctx.logPrintln(F("m"));
        ctx.logPrint(F("[INFO] Velocity: "));
        ctx.logPrint(velocity, 2);
        ctx.logPrintln(F("m/s"));
        ctx.logPrintln(F("========================================"));
        ctx.logEnd();

        ctx.failsafeArmed = false;
        parachuteDeployed = true;
    }

    // Detect landing 
    float accelMagnitude = abs(ctx.zA);
    
    if (parachuteDeployed && velocity < LANDING_VELOCITY_THRESHOLD && accelMagnitude < LANDING_ACCEL_THRESHOLD) {
        stableCount++;

    } else {
        stableCount = 0;
    }

    if (stableCount >= LANDING_CONSECUTIVE) {

        Serial.print(F("[EVENT] Landing detected -> LandedState"));
        Serial.print(F(" | velocity: "));
        Serial.print(velocity, 2);
        Serial.print(F(" m/s | distance_fallen: "));
        Serial.print(distanceFallen, 2);
        Serial.println(F(" m"));

        ctx.logBegin();
        ctx.logPrint(F("[EVENT] Landing detected -> LandedState"));
        ctx.logPrint(F(" | velocity: "));
        ctx.logPrint(velocity, 2);
        ctx.logPrint(F(" m/s | distance_fallen: "));
        ctx.logPrint(distanceFallen, 2);
        ctx.logPrintln(F(" m"));
        ctx.logEnd();

        ctx.setState(ctx.landedState);
    }

}

void DescentState::exit(FlightContext& ctx) {
    Serial.println(F("[STATE] Descent exit"));
    ctx.logBegin();
    ctx.logPrintln(F("[STATE] Descent exit"));
    ctx.logEnd();
}