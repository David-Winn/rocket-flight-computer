#include "landed_state.hpp"
#include "flight_context.hpp"
#include <Arduino.h>

void LandedState::enter(FlightContext& ctx) {
    Serial.println(F("[STATE] Landed enter"));
}

void LandedState::update(FlightContext& ctx) {
    // Do nothing; stay in landed state

}

void LandedState::exit(FlightContext& ctx) {
    Serial.println(F("[STATE] Landed exit "));
}
