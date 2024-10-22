#include "odometer.hpp"
#include <iostream>

Odometer::Odometer(int pinA, int pinB) : pinA(pinA), pinB(pinB), rotation(0) {
    wiringPiSetupGpio(); // GPIOのセットアップ (BCMピン番号を使用)
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    wiringPiISR(pinA, INT_EDGE_RISING, &Odometer::getPulse, this);
}

void Odometer::getPulse(void* instance) {
    Odometer* self = static_cast<Odometer*>(instance);
    self->updateRotation();
}

void Odometer::updateRotation() {
    if (digitalRead(pinB) == HIGH) {
        rotation--;
    } else {
        rotation++;
    }
}

void Odometer::zeroSet() {
    rotation = 0;
}

int Odometer::getRotation() {
    return rotation;
}

