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

static int speed = 255; // static speed
static int turn_speed = 255; //static turn speed
struct sensors_ sens; 
int x_cord; //tracks the X coordiante
int y_cord; //tracks the Y coordinate
int direction; //0 if facing up, 1 if facing right, 2 if facing down, 3 if facing left

void intersection();
    
void init_settings(){
    //Does the Initial settings on startup
    y_cord = 0;
    x_cord = 0;
    direction = 0;
    Ultra_Start();
    IR_Start();
    IR_flush();
    motor_start();
    motor_forward(0,0);
    reflectance_start();
    reflectance_set_threshold(9000, 9000, 18000, 18000, 9000, 9000);
}

void corrected_forward(){
    //when called in a loop,
    //this function runs the bot forward on a line while correcting for misalignment
    int corr_speed = speed/2;
    while(true){
        reflectance_digital(&sens);
        if(!sens.L1){
            SetMotors(0,0,speed,corr_speed,0);
            break;
        }
        if(!sens.R1){
            SetMotors(0,0,corr_speed,speed,0);
            break;
        }
        if(sens.R1 && sens.L1){
            motor_forward(speed,0);
            break;
        }
        break;
    }
}

void turn_90(int lr){
    //does a tank turn until it sees the next line on the grid.
    //It should turn the bot 90 degrees if the bot is aligned correctly.
    //if lr == 1 turn right if 0 turn left
    bool onLine = true;
    while(true){
        SetMotors(!lr,lr,turn_speed,turn_speed,0);
        reflectance_digital(&sens);   
        if(!onLine && sens.L1 && sens.R1){
            break;
        }
        if(onLine && !sens.L1 && !sens.R1) onLine = false;
    }
    motor_forward(0,0);
    //keeps track of the facing direction
    if(lr){
        if(direction == 3) direction = 0;
        else direction += 1;
    }
    if(!lr){
        if(direction == 0) direction = 3;
        else direction -= 1;
    }
}

void uTurn(){
    //Does a tank turn until it sees and skips over one line and stop on the next one.
    //should do a complete U turn if the bot is aligned correctly
    int count = 0;
    bool onLine = true;
    while(count < 2){
        SetMotors(0,1,turn_speed,turn_speed,0);
        reflectance_digital(&sens);   
        if(!onLine && sens.L1 && sens.R1){
            count += 1;
            onLine = true;
        }
        if(onLine && !sens.L1 && !sens.R1) onLine = false;
    }
    motor_forward(0,0);
    //Tracks the direction that the bot is facing correctly.
    if(direction == 0) direction = 2;
    else if(direction == 1) direction = 3;
    else if(direction == 2) direction = 0;
    else if(direction == 3) direction = 1;
}

void trackPos(){
    //keeps track of the coordinates of the bot.
    // this is called at an intersection and coordinates are added according to the facing direction.
    if(direction == 0)y_cord += 1;
    if(direction == 1)x_cord += 1;
    if(direction == 2)y_cord -= 1;
    if(direction == 3)x_cord -= 1;
}

 


void zmain(void){
    TickType_t runTime = 0;
    bool onLine = false;
    init_settings(); //initializes settings
    while(SW1_Read() == 1) vTaskDelay(10);//waits for button press
    //runs to the next intersection after button press.
    while(!sens.L3 && !sens.R3){
        motor_forward(40,0);
        reflectance_digital(&sens);
    }
    motor_forward(0,0);
    onLine = true;
    print_mqtt("Zumo001/ready","maze");
    IR_flush();
    IR_wait();//waits for an IR signal.
    TickType_t Ttime = xTaskGetTickCount();// gets starting time.
    print_mqtt("Zumo001/start","%d",(int)Ttime);
    //runs to the first intersection.
    while(true){
        reflectance_digital(&sens);
        if(!onLine && ((sens.L3 && sens.L2) || (sens.R2 && sens.R3))){
            break;
        }
        if(onLine && ((!sens.L3 && !sens.L2)||(!sens.R2 && !sens.R3))) onLine = false;
        motor_forward(200,0);
    }
    motor_forward(200,100);//aligns the bot correctly on x = 0 y = 0
    motor_forward(0,0);
    
    //runs the bot forward until it sees an intersection,
    //then calls intersection(); to move forward accordingly.
    while(true){
        reflectance_digital(&sens);
        if(x_cord == 0 && y_cord == 13)break;//once the bot is on coordinates(0,13) the loops stops
        if(!onLine && ((sens.L3 && sens.L2) || (sens.R2 && sens.R3))){
            intersection();
            onLine = true;
        }
        if(onLine && ((!sens.L3 && !sens.L2)||(!sens.R2 && !sens.R3))) onLine = false;
        corrected_forward();
    }
    //runs forward until the finnish line ends.
    while(sens.L1 || sens.R1){
        reflectance_digital(&sens);
        corrected_forward();
    }
    motor_forward(0,0);
    motor_stop();
    runTime = xTaskGetTickCount();//gets the total run time.
    print_mqtt("Zumo001/stop","%d",(int) runTime);//prints total run time
    print_mqtt("Zumo001/time","%d",(int)(runTime-Ttime));//prints start to stop time.
    while(true) vTaskDelay(200);//infinite loop
}

void intersection(){
    motor_forward(100,10);
    motor_forward(0,0);
    trackPos();//adds +1 to the right coordinate.
    int left = 0; // 0 if the bot can't turn left, 1 if it can.
    int right = 0;// 0 if the bot can't turn right, 1 if it can.
    int forward = 0;// 0 if the bot can't continue forward, 1 if it can.
    
    reflectance_digital(&sens);
    //checks if there is a line on the left and/or the right side
    if(sens.L3) left = 1;
    if(sens.R3) right = 1;
    
    motor_forward(200,115);//moving forward to check if line continues forward
    motor_forward(0,0);
    reflectance_digital(&sens);
    
    //checks if the line continues forward and if there's no obstacles ahead.
    if((sens.L1 || sens.R1) && Ultra_GetDistance() > 9) forward = 1;
    
    //switch case based on the facing direction. 0 = up, 1 = right 2 = down, 3 = left
    switch(direction){
        
        case 0: //when facing forward
            if(forward)break;
            if(x_cord <= 0 && left){
                turn_90(0);
                if(Ultra_GetDistance() < 9){
                    uTurn();
                }
                break;
            }
            if(x_cord > 0  && right){
                turn_90(1);
                if(Ultra_GetDistance() < 9){
                    uTurn();
                }
                break;
            }
            if(x_cord <= 0 && right){
                turn_90(1);
                break;
            }
            if(x_cord >= 0 && left){
                turn_90(0);
                break;
            }
            break;
                
        case 1://when facing right
            if(left && forward){
                turn_90(0);
                if(Ultra_GetDistance() < 9){
                    turn_90(1);
                }
                break;
            }
            if(left && !forward){
                turn_90(0);
                if(Ultra_GetDistance() < 9){
                    turn_90(0);
                }
                break;
            }
            break;
        
        case 2://when facing down.
            if(x_cord < 0 && right){
                turn_90(1);
                if(Ultra_GetDistance() < 9){
                    uTurn();
                }
                break;
            }
            break;
          
        case 3://when facing left
            if(right && forward){
                turn_90(1);
                if(Ultra_GetDistance() < 9){
                    turn_90(0);
                }
                break;
            }
            if(right && !forward){
                turn_90(1);
                if(Ultra_GetDistance() < 9){
                    turn_90(1);
                }
                break;
            }
            break;
    }
    print_mqtt("Zumo001/position", "%d %d",x_cord,y_cord);// prints the coordinates of the intersection.
}   