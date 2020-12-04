#include "sensor.h"

uint8_t Sensor::get_status(uint8_t slave_addr)
{
    this->start();
    this->write(slave_addr);
    this->write(0x01);
    this->start();
    this->write(slave_addr | 0x01);
    uint8_t rvalue = this->read(0);
    this->stop();
    return rvalue;
}

uint8_t Sensor::get_blink_status(uint8_t slave_addr)
{
    this->start();
    this->write(slave_addr);
    this->write(0x02);
    this->start();
    this->write(slave_addr | 0x01);
    uint8_t rvalue = this->read(0);
    this->stop();
    return rvalue;
}

void Sensor::start_blink_led(uint8_t slave_addr)
{
    this->start();
    this->write(slave_addr);
    this->write(0x03);
    this->write(0x01);
    this->write(0xFF);
    this->stop();
}

void Sensor::stop_blink_led(uint8_t slave_addr)
{
    this->start();
    this->write(slave_addr);
    this->write(0x03);
    this->write(0x00);
    this->write(0xFF);
    this->stop();
} 