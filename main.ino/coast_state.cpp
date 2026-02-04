#include "coast_state.hpp"
#include "apogee_state.hpp"
#include "flight_context.hpp"
#include "distance_alg.hpp"
#include <Arduino.h>

// Apogee detected when upward velocity becomes downward
static constexpr float VELOCITY_THRESHOLD = -0.5f;  // 
static constexpr int APOGEE_CONSECUTIVE = 3;  // Require some number of consecutive readings below threshold

void CoastState::enter(FlightContext& ctx) {
    Serial.println(F("[STATE] Coast enter"));
    
    // Initialize distance algorithm to track velocity
    distanceAlg.reset();
    lastUpdateMicros = micros();
    consecutiveNegativeVelocity = 0;
}

void CoastState::update(FlightContext& ctx) {
    // Calculate time delta since last update
    unsigned long currentMicros = micros();
    float dtSeconds = (currentMicros - lastUpdateMicros) / 1000000.0f;
    lastUpdateMicros = currentMicros;
    
    // Update distance algorithm to track velocity
    distanceAlg.update(ctx.zA, dtSeconds);
    float velocity = distanceAlg.getVelocity();

    // Detect when velocity becomes negative for transitioning from coast to descent 
    if (velocity < VELOCITY_THRESHOLD) {
        consecutiveNegativeVelocity++;
    } else {
        consecutiveNegativeVelocity = 0;
    }

    if (consecutiveNegativeVelocity >= APOGEE_CONSECUTIVE) {
        Serial.print(F("[EVENT] Apogee detected -> ApogeeState"));
        Serial.print(F(" | velocity: "));
        Serial.print(velocity, 2);
        Serial.println(F(" m/s"));
        ctx.setState(ctx.apogeeState);
    }
}

void CoastState::exit(FlightContext& ctx) {
    Serial.println(F("[STATE] Coast exit"));
}