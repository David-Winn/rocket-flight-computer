#include "coast_state.hpp"
#include "apogee_state.hpp"
#include "flight_context.hpp"
#include "descent_state.hpp"
#include "distance_alg.hpp"
#include <Arduino.h>


// Wait this long at apogee before transitioning to descent
// Gives time to stabilize and prepare for parachute deployment
static constexpr unsigned long APOGEE_DWELL_MS = 500;  // 500ms

void ApogeeState::enter(FlightContext& ctx) {
    Serial.println(F("[STATE] Apogee enter"));
    
    entryMs = millis();
    
    // Reset distance algorithm at apogee
    distanceAlg.reset();
    lastUpdateMicros = micros();

}

void ApogeeState::update(FlightContext& ctx) {

    unsigned long currentMicros = micros();
    float dtSeconds = (currentMicros - lastUpdateMicros) / 1000000.0f;
    lastUpdateMicros = currentMicros;
    
    // Update distance algorithm with current acceleration
    distanceAlg.update(ctx.zA, dtSeconds);
    
    float distanceFallen = distanceAlg.getDistance();
    float velocity = distanceAlg.getVelocity();

    Serial.println(F("[EVENT] Descent detected -> DescentState"));

        unsigned long timeAtApogee = millis() - entryMs;
    if (timeAtApogee >= APOGEE_DWELL_MS) {
        Serial.print(F("[EVENT] Apogee stabilized -> DescentState"));
        Serial.print(F(" | distance_fallen: "));
        Serial.print(distanceFallen, 2);
        Serial.print(F(" m | velocity: "));
        Serial.print(velocity, 2);
        Serial.println(F(" m/s"));
        ctx.setState(ctx.descentState);
    }

}

void ApogeeState::exit(FlightContext& ctx) {
    Serial.println(F("[STATE] Apogee state exit"));
}


