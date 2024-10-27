#ifndef ODOMETER_HPP
#define ODOMETER_HPP

#include <cstdint>
#include <pigpiod_if2.h>

class Odometer {
private:
    int rotation;
    int pinA;
    int pinB;
    int pi;  // pigpioのインスタンスIDを保持するためのメンバ変数

public:
    static Odometer* instance; 
    Odometer(int pi, int pinA, int pinB);  // piハンドルを受け取るコンストラクタに変更
    static void staticGetPulse(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
    void getPulse();
    void zeroSet();
    void updateRotation();
    int getRotation();
};

#endif // ODOMETER_HPP
