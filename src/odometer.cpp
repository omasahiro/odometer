#include "odometer.hpp"
#include <iostream>
#include <pigpiod_if2.h>

Odometer* Odometer::instance = nullptr; // 静的メンバーの初期化

Odometer::Odometer(int pi, int pinA, int pinB) : pi(pi), pinA(pinA), pinB(pinB), rotation(0) {
    instance = this; 
    set_mode(pi, pinA, PI_INPUT);
    set_mode(pi, pinB, PI_INPUT);
    
    // コールバックの設定
    callback(pi, pinA, RISING_EDGE, Odometer::staticGetPulse);
}

void Odometer::staticGetPulse(int pi, unsigned user_gpio, unsigned level, uint32_t tick) {
    if (instance) {
        instance->getPulse();
    }
}

void Odometer::getPulse() {
    updateRotation();
}

void Odometer::updateRotation() {
    if (gpio_read(pi, pinB) == 1) {
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
