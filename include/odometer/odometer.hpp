#ifndef ODOMETER_HPP
#define ODOMETER_HPP

#include <wiringPi.h>
#include <cstdint>

class Odometer {
private:
    int rotation;
    int pinA;
    int pinB;
    static void getPulse(void* instance);

public:
    Odometer(int pinA, int pinB);
    void zeroSet();
    void updateRotation();
    int getRotation();
};

#endif // ODOMETER_HPP

