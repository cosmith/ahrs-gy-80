#include "GY80.h"
#include <Wire.h>

GY80 sensor = GY80();

void setup() {
    sensor.setup();
}

void loop() {
    sensor.getValues();
    delay(10);
}
