#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>

struct sensors_ dig;
uint8_t count = 0;                                      // line counter
uint8_t pL3 = 0, pR3 = 0, pL1 = 0, pR1 = 0;             // last rounds values
uint8_t cL3 = 0, cR3 = 0, cL1 = 0, cR1 = 0;             // current round values
uint16_t start = 0, stop = 0, lap = 0, out = 0, in = 0; // timestamps
char line[] = "line";

void countLines(void);
void followLine(void);
void missedLine(void);
void backInLine(void);

void zmain(void)
{
    motor_start(); // enable motor controller
    IR_Start(); //enable IR
    reflectance_start(); //enable reflectancing
    motor_forward(0, 0); // set speed to zero to stop motors
    reflectance_set_threshold(23500, 9000, 11000, 11000, 9000, 23500); //limit for each sensor, if below limit value is 0 and if above limit value is 1

    while (SW1_Read())
        ;

    reflectance_digital(&dig);

    while (!(dig.L3 || dig.R3)) // checks reflectane of L3 and R3 sensors
    {
        reflectance_digital(&dig);
        motor_forward(50, 0);
    }
    motor_forward(0, 0);
    print_mqtt("Zumo4/ready", "%s", line);

    IR_wait();
    reflectance_set_threshold(20000, 9000, 8000, 8000, 9000, 20000);
    start = xTaskGetTickCount();
    print_mqtt("Zumo4/start", "%d", start);
    reflectance_digital(&dig);

    while (count < 2)
    {
        pL3 = dig.L3, pR3 = dig.R3, pL1 = dig.L1, pR1 = dig.R1; //checks the value of sensors after previous loop
        reflectance_digital(&dig);
        cL3 = dig.L3, cR3 = dig.R3, cL1 = dig.L1, cR1 = dig.R1; // checks the value of sensors now
        countLines();
        followLine();
        missedLine();
        backInLine();
    }

    motor_forward(0, 0);
    stop = xTaskGetTickCount();
    print_mqtt("Zumo4/stop", "%d", stop);
    lap = stop - start;
    print_mqtt("Zumo4/time", "%d", lap);
    motor_stop();

    while (true)
    {
        vTaskDelay(200);
    }
}
//function to count the horizontal lines
void countLines(void)
{
    if (!(pL3 && pR3) && (cL3 && cR3)) //if the value has changed line will be counted
    {
        count++;
    }
}
//function to follow the vertical line
void followLine(void)
{
    if (!dig.L1) //line following
    {
        motor_turn(255, 0, 0);
    }
    else if (!dig.R1)
    {
        motor_turn(0, 255, 0);
    }
    else
    {
        motor_forward(255, 0);
    }
}
//function to check if outside the vertical line
void missedLine(void)
{
    if (!dig.L1 && !dig.R1)
    {
        if (pR1 == 1 && cR1 == 0)
        {
            out = xTaskGetTickCount();
            print_mqtt("Zumo4/miss", "%d", out);
        }
        else if (pL1 == 1 && cL1 == 0)
        {
            out = xTaskGetTickCount();
            print_mqtt("Zumo4/miss", "%d", out);
        }
    }
}
//function to check if the vertical line is found again
void backInLine(void)
{
    if (dig.L1 || dig.R1)
    {
        if (pL1 == 0 && pR1 == 0)
        {
            in = xTaskGetTickCount();
            print_mqtt("Zumo4/line", "%d", in);
        }
    }
}
