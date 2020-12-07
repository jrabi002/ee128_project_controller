
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

//Button Input
DigitalIn button(PTA2);
volatile uint8_t button_pressed = 0;

//number of sensors used
#define TOTAL_PARKING_SPACES 3

//Task ticker for task functions
Ticker tasktimer;

//Counters to blink sseg when no spot is available (25ms increments)
#define BLINK_COUNT 5
#define BLINK_TIMER 20
uint8_t blink_sseg_timer = BLINK_TIMER;
uint8_t blink_sseg_count = BLINK_COUNT;
uint8_t blink_sseg_on = 0;
uint8_t sseg_on = 1;


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
InterruptIn sensor_1C(PTD2);

//interrupt for button input
InterruptIn button_press(PTA2);

//sensor function declarations
void sensors_init(const uint8_t*);
void task_sensor_update(volatile uint8_t*, const uint8_t*);
void sseg_update(void);
void find_closest_parking_space(uint8_t*, const uint8_t*);
void blink_sseg_led(void);

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

void ISR_button()
{
    if (!button_pressed)
        button_pressed = 1;
}

int main()
{
    //Set I2C frequency
    s.frequency(400000);
    
    //Interupt Tasks (get update from sensor)
    sensor_1A.rise(&ISR1A);
    sensor_1B.rise(&ISR1B);
    sensor_1C.rise(&ISR1C);

    //Button configuration use internal pullup resistors
    //Inverse Logic
    button.mode(PullUp);
    
    //wait 1ms for pullup
    wait_us(1000);
    
    //Interupt Task for button
    button_press.fall(&ISR_button);
    
    //Ticker for 50ms task executions
    tasktimer.attach(&timer_tick, 25ms);

    const uint8_t sensors[3] = {SLAVE_ADDR_1A, SLAVE_ADDR_1B, SLAVE_ADDR_1C};
    
    //delay to wait for sensors to startup (1s)
    wait_us(1000000);

    sensors_init(sensors);

    while (true) {
       
        while(!timerflag);
        if(sensor_update)
        {
            task_sensor_update(sensor_to_check, sensors);
            sseg_update();
            sensor_update = 0;
        }
        if (button_pressed == 1) 
        {
            find_closest_parking_space(sensor_status, sensors);
            button_pressed = 0;
        }
        if (blink_sseg_on == 1)
            blink_sseg_led();
            
        timerflag = 0;
    }
}

void sseg_update(void)
{
    sseg_bus = sseg[TOTAL_PARKING_SPACES - sensor_status[0] - sensor_status[1] - sensor_status[2]];
}

void sensors_init(const uint8_t* sensor_addr)
{
    sensor_status[0] = s.get_status(sensor_addr[0]) ? 1 : 0;
    sensor_status[1] = s.get_status(sensor_addr[1]) ? 1 : 0;
    sensor_status[2] = s.get_status(sensor_addr[2]) ? 1 : 0;
    sseg_update();
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

void find_closest_parking_space(uint8_t* sensor_status, const uint8_t* sensor_addr)
{
    if (sensor_status[0] == 0)
        s.start_blink_led(sensor_addr[0]);
    else if (sensor_status[1] == 0)
        s.start_blink_led(sensor_addr[1]);
    else if (sensor_status[2] == 0)
        s.start_blink_led(sensor_addr[2]);
    else 
        blink_sseg_on = 1;
}

void blink_sseg_led(void)
{
    if (blink_sseg_timer > 0)
        blink_sseg_timer--;
    if (blink_sseg_timer == 0)
    {
        if (blink_sseg_count > 0)
            {
                sseg_on ^= 1;
                if (sseg_on)
                    sseg_update();
                else 
                    sseg_bus = 0;
            blink_sseg_count--;
            }
        blink_sseg_timer = BLINK_TIMER;
    }
    if (blink_sseg_count == 0)
    {
        blink_sseg_on = 0;
        blink_sseg_count = BLINK_COUNT;
        blink_sseg_timer = BLINK_TIMER;
        if (sseg_on == 0)
        {
            sseg_on = 1;
            sseg_update();
        }
    }
}