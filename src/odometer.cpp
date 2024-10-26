#include "odometer.hpp"
#include <iostream>

Odometer* Odometer::instance = nullptr; // 静的メンバーの初期化

Odometer::Odometer(int pinA, int pinB) : pinA(pinA), pinB(pinB), rotation(0) {
    instance = this; // インスタンスを静的メンバーに設定
    wiringPiSetupGpio(); // GPIOのセットアップ (BCMピン番号を使用)
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    
    // コールバック用の静的メソッドを設定
    wiringPiISR(pinA, INT_EDGE_RISING, Odometer::staticGetPulse);
}

void Odometer::staticGetPulse() {
    if (instance) {
        instance->getPulse(); // インスタンスを使って非静的メソッドを呼び出す
    }
}

void Odometer::getPulse() {
    updateRotation();
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
