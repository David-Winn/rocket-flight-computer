#include "prelaunch_state.hpp"
#include "boost_state.hpp"
#include "flight_context.hpp"
#include <Arduino.h>

// Launch detected when vertical acceleration exceeds this threshold
static constexpr float LAUNCH_ACCEL_THRESHOLD = 15.0f;  // m/s² (roughly 1.5g)

void PrelaunchState::enter(FlightContext& ctx) {
    Serial.println(F("[STATE] Prelaunch enter"));
    consecutiveHighAccel = 0;
}

void PrelaunchState::update(FlightContext& ctx) {
    // Detect sustained upward acceleration
    if (ctx.zA < -LAUNCH_ACCEL_THRESHOLD) {
        consecutiveHighAccel++;
    } else {
        consecutiveHighAccel = 0;
    }

    // Launch confirmed after 3 consecutive readings above threshold 
    if (consecutiveHighAccel >= 3) {
        Serial.print(F("[EVENT] Launch detected -> BoostState"));
        Serial.print(F(" | accel: "));
        Serial.print(-ctx.zA, 2);
        Serial.println(F(" m/s²"));
        ctx.setState(ctx.boostState);
    }
}

void PrelaunchState::exit(FlightContext& ctx) {
    Serial.println(F("[STATE] Prelaunch exit"));
}