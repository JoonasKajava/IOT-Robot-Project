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
#include <stdlib.h>

#define black true
#define white false

struct sensors_ dig;
int speed = 140;

void init();
void findNext(bool color);
void stayInside();
void avoidEdge();
void avoidObstacle();

void tankTurn();

void zmain(void)
{
    init();

    // Drive to the edge of the ring
    findNext(black);
    print_mqtt("Zumo4/ready", "%s", "zumo");
    IR_wait();
    int start = xTaskGetTickCount();
    print_mqtt("Zumo4/start", "%d", start);
    // Enter the ring
    findNext(white);

    stayInside();
    motor_forward(0,0);
    motor_stop();
    int stop = xTaskGetTickCount();
    print_mqtt("Zumo4/stop", "%d", stop);
    print_mqtt("Zumo4/time", "%d", stop - start);
    while (true)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
}

void init()
{
    srand(time(NULL));
    motor_start();
    motor_forward(0, 0);
    IR_Start();
    Ultra_Start();
    reflectance_start();
    reflectance_set_threshold(16000, 16000, 16000, 16000, 16000, 16000);
}

// white = false, black = true
void findNext(bool color)
{
    reflectance_digital(&dig);
    motor_forward(speed, 1);
    while (dig.L3 != color || dig.R3 != color)
        reflectance_digital(&dig);
    motor_forward(0, 0);
}

void stayInside()
{
    while (SW1_Read())
    {
        motor_forward(speed, 1);
        avoidEdge();
        avoidObstacle();
    }
}

void avoidEdge()
{
    reflectance_digital(&dig);
    if (dig.L3 || dig.R3)
    {
        motor_backward(speed, 10);
        tankTurn();
    }
}

void avoidObstacle()
{
    if (Ultra_GetDistance() < 5)
    {
        print_mqtt("Zumo4/obstacle", "%d", xTaskGetTickCount());
        tankTurn();
    }
}

void tankTurn()
{
    int random = rand() % 300 + 100;
    SetMotors(1, 0, speed, speed, random);
}