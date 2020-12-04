
#include "mbed.h"
#include <cstdint>
#include "sensor.h"

// Initialization Sensor I2C class using default K64F pins:
Sensor s(PTE25, PTE24);

//Initialize output for 7-seg display
#define segG PTC16
#define segF PTB18
#define segE PTC17
#define segD PTB19
#define segC PTB9
#define segB PTC1
#define segA PTA1
//Port output variable for 7-segment display
BusOut sseg_bus(segA, segB, segC, segD, segE, segF, segG);
//Array for the display of numbers 0-3
uint8_t sseg[4] = {0x3F, 0x06, 0x5B, 0x4F};

//number of sensors used
#define TOTAL_PARKING_SPACES 3

//Task ticker for task functions
Ticker tasktimer;

//Sensor slave addresses
#define SLAVE_ADDR_1A 0x1A << 1
#define SLAVE_ADDR_1B 0x1B << 1
#define SLAVE_ADDR_1C 0x1C << 1

//This variable is to synchronize tasks in the main loop to the Ticker tasktimer
volatile uint8_t timerflag = 0;

/*  This variable holds the current detection status of all sensors
    0 : No object detected
    1 : Object detected
*/ 
uint8_t sensor_status[3] = {0,0,0};

/*  This variable is set by individual sensor ISR1x functions
    it is used to tell the function: task_sensor_update() which sensors to check
    after a sensor has supplied an external interrupt to the K64F
*/
volatile uint8_t sensor_to_check[3] = {0,0,0};

/*
    This variable is set by any of the individual sensor ISR1x functions
    It denotes that atleast one sensor needs to be checked.
*/
volatile uint8_t sensor_update = 0;

//external interrupt inputs (from sensors)
InterruptIn sensor_1A(PTD0);
InterruptIn sensor_1B(PTD1);
InterruptIn sensor_1C(PTD1);

//sensor function declarations
void sensors_init(const uint8_t*);
void task_sensor_update(volatile uint8_t*, const uint8_t*);

void display_sseg(uint8_t);

//Sensor external interupt ISRs
void ISR1A()
{
    sensor_to_check[0] = 1;
    sensor_update = 1;
}
void ISR1B()
{
    sensor_to_check[1] = 1;
    sensor_update = 1;
}
void ISR1C()
{
    sensor_to_check[2] = 1;
    sensor_update = 1;
}

void timer_tick()
{
    timerflag = 1;
}

int main()
{
    s.frequency(400000);
    
    //Interupt Tasks (get update from sensor)
    sensor_1A.rise(&ISR1A);
    //sensor_1B.rise(&ISR1B);
    //sensor_1C.rise(&ISR1C);
    
    //Ticker for 50ms task executions
    tasktimer.attach(&timer_tick, 5ms);

    const uint8_t sensors[3] = {SLAVE_ADDR_1A, SLAVE_ADDR_1B, SLAVE_ADDR_1C};
    sensors_init(sensors);

    while (true) {
       
        while(!timerflag);
        if(sensor_update)
        {
            task_sensor_update(sensor_to_check, sensors);
            display_sseg(TOTAL_PARKING_SPACES - sensor_status[0] - sensor_status[1] - sensor_status[2]);
            sensor_update = 0;
        }
        timerflag = 0;
    }
}

void display_sseg(uint8_t digit)
{
    sseg_bus = sseg[digit];
}

void sensors_init(const uint8_t* sensor_addr)
{
    sensor_status[0] = s.get_status(sensor_addr[0]) ? 1 : 0;
    //sensor_status[1] = sensor.get_status(sensor_addr[1]) ? 1 : 0;
    //sensor_status[2] = sensor.get_status(sensor_addr[2]) ? 1 : 0;
    display_sseg(TOTAL_PARKING_SPACES - sensor_status[0] - sensor_status[1] - sensor_status[2]);
}

void task_sensor_update(volatile uint8_t* sensor_to_check, const uint8_t* sensor_addr)
{
    if (sensor_to_check[0])
    {
        sensor_status[0] = s.get_status(sensor_addr[0]) ? 1 : 0;
        sensor_to_check[0] = 0;
    }
        
    if (sensor_to_check[1])
    {
        sensor_status[1] = s.get_status(sensor_addr[1]) ? 1 : 0;
        sensor_to_check[1] = 0;
    }
        
    if (sensor_to_check[2])
    {
        sensor_status[2] = s.get_status(sensor_addr[2]) ? 1 : 0;
        sensor_to_check[2] = 0;
    }    
}