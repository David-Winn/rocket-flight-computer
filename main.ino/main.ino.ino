#include <Arduino.h>
#include "pins.hpp"
#include "flight_machine.hpp"
#include "distance_alg.hpp"


FlightMachine fm;

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("Booting...");

    fm.ctx.initHardware(); // init IMU, SD, pins
    fm.start();            // wire states and set initial state

    Serial.println("Ready.");
}

void loop() {

    digitalWrite(LED2, HIGH);

    fm.ctx.readSensors();
    fm.ctx.writeTelemetry();

    fm.update();

    fm.ctx.tick();

    digitalWrite(LED2, LOW);
}

