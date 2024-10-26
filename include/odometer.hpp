#ifndef ODOMETER_HPP
#define ODOMETER_HPP

#include <wiringPi.h>
#include <cstdint>

class Odometer {
private:
    int rotation;
    int pinA;
    int pinB;

public:
    static Odometer* instance; // 静的メンバーを追加
    Odometer(int pinA, int pinB);
    static void staticGetPulse(); // 静的メソッドを宣言
    void getPulse();
    void zeroSet();
    void updateRotation();
    int getRotation();
};

#endif // ODOMETER_HPP
