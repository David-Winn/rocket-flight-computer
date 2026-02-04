#include "descent_state.hpp"
#include "coast_state.hpp"
#include "flight_context.hpp"
#include "landed_state.hpp"
#include <Arduino.h>


// Parachute deployment threshold
static constexpr float DEPLOY_DISTANCE = 0;

// Time based parachute deployment failsafe
static constexpr unsigned long DEPLOY_FAILSAFE_MS = 15000;  // 15 seconds


// Velocity must be very low and acceleration near gravity
static constexpr float LANDING_VELOCITY_THRESHOLD = 1.0f;  // m/s
static constexpr float LANDING_ACCEL_THRESHOLD = 10f;    // m/s² (close to 1g)
static constexpr int LANDING_CONSECUTIVE = 10;             


void DescentState::enter(FlightContext& ctx) {
    Serial.println(F("[STATE] Descent enter"));
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

        digitalWrite(5, HIGH);
        delay(100);
        digitalWrite(5, LOW);

        Serial.println(F("========================================"));
        Serial.println(F("[EVENT] PARACHUTE DEPLOYMENT!"));
        Serial.print(F("[INFO] Distance fallen: "));
        Serial.print(distanceFallen);
        Serial.println(F("m"));
        Serial.print(F("[INFO] Velocity: "));
        Serial.print(velocity);
        Serial.println(F("m/s"));
        Serial.println(F("========================================"));

        parachuteDeployed = true;
    }

    // Time delay failsafe if distance calculation fails to deply parachute 
    unsigned long timeSinceDescent = millis() - entryMs;
    if (!parachuteDeployed && timeSinceDescent >= DEPLOY_FAILSAFE_MS) {

        digitalWrite(5, HIGH);
        delay(100);
        digitalWrite(5, LOW);

        Serial.println(F("========================================"));
        Serial.print(F("[EVENT] Parachute deployment FAILSAFE"));
        Serial.print(F(" | time: "));
        Serial.print(timeSinceDescent);
        Serial.print(F(" ms | distance_fallen: "));
        Serial.print(distanceFallen, 2);
        Serial.print(F(" m | velocity: "));
        Serial.print(velocity, 2);
        Serial.println(F(" m/s"));
        Serial.println(F("========================================"));
        
        parachuteDeployed = true;
    }

    Serial.println(F("[EVENT] Landing detected -> LandedState"));

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

        ctx.setState(ctx.landedState);
    }

}

void DescentState::exit(FlightContext& ctx) {
    Serial.println(F("[STATE] Descent exit"));
}

