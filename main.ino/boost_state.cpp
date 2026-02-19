#include "boost_state.hpp"
#include "flight_context.hpp"
#include "coast_state.hpp"
#include <Arduino.h>

// Motor burnout detected when vertical acceleration falls below this threshold
static constexpr float BURNOUT_ACCEL_THRESHOLD = 5.0f; // m/s^2
static constexpr int BURNOUT_CONSECUTIVE = 5;          // Require 5 consecutive low readings 

int consecutiveLowAccel = 0;

void BoostState::enter(FlightContext &ctx)
{
    Serial.println(F("[STATE] Boost enter"));
    ctx.logBegin();
    ctx.logPrintln(F("[STATE] Boost enter"));
    ctx.logEnd();
    startZ = ctx.zA;
}

void BoostState::update(FlightContext &ctx)
{
    // Look for sustained low vertical acceleration 
    if (ctx.zA > -BURNOUT_ACCEL_THRESHOLD)
    {
        // Acceleration has dropped below threshold
        consecutiveLowAccel++;
    }
    else
    {
        // Still burning
        consecutiveLowAccel = 0;
    }

    if (consecutiveLowAccel >= BURNOUT_CONSECUTIVE)
    {
        Serial.print(F("[EVENT] Motor burnout -> CoastState"));
        Serial.print(F(" | accel: "));
        Serial.print(ctx.zA, 2);
        Serial.println(F(" m/s^2"));

        ctx.logBegin();
        ctx.logPrint(F("[EVENT] Motor burnout -> CoastState"));
        ctx.logPrint(F(" | accel: "));
        ctx.logPrint(ctx.zA, 2);
        ctx.logPrintln(F(" m/s^2"));
        ctx.logEnd();

        ctx.setState(ctx.coastState);
    }
}

void BoostState::exit(FlightContext &ctx)
{
    Serial.println(F("[STATE] Boost exit"));
    ctx.logBegin();
    ctx.logPrintln(F("[STATE] Boost exit"));
    ctx.logEnd();
}