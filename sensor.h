#ifndef SENSOR_H_
#define SENSOR_H_

#include "mbed.h"

class Sensor : public I2C {

    private:

    public:
        Sensor(PinName SDA, PinName SCL) : mbed::I2C(SDA, SCL) {}

        uint8_t get_status(uint8_t);
        uint8_t get_blink_status(uint8_t);
        void start_blink_led(uint8_t);
        void stop_blink_led(uint8_t);
};
#endif //SENSOR_H_