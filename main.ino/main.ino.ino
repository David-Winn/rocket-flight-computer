#include <Arduino.h>
#include "pins.hpp"
#include "flight_machine.hpp"
#include "distance_alg.hpp"


FlightMachine fm;
  
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Booting...");

  fm.ctx.initHardware();  
  fm.start();             

  Serial.println("Ready.");

  // Log boot complete to SD
  fm.ctx.logBegin();
  fm.ctx.logPrintln(F("=== System Ready ==="));
  fm.ctx.logEnd();

  Serial.println(F("=== System Ready ==="));

}

void loop() {

    digitalWrite(LED2, HIGH);

    fm.ctx.readSensors();

    fm.ctx.checkFailsafe();

    fm.ctx.writeTelemetry();
    fm.update();
    fm.ctx.tick();

    digitalWrite(LED2, LOW);
}
