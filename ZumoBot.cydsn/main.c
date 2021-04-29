/**
* @mainpage ZumoBot Project
* @brief    You can make your own ZumoBot with various sensors.
* @details  <br><br>
    <p>
    <B>General</B><br>
    You will use Pololu Zumo Shields for your robot project with CY8CKIT-059(PSoC 5LP) from Cypress semiconductor.This 
    library has basic methods of various sensors and communications so that you can make what you want with them. <br> 
    <br><br>
    </p>
    
    <p>
    <B>Sensors</B><br>
    &nbsp;Included: <br>
        &nbsp;&nbsp;&nbsp;&nbsp;LSM303D: Accelerometer & Magnetometer<br>
        &nbsp;&nbsp;&nbsp;&nbsp;L3GD20H: Gyroscope<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Reflectance sensor<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Motors
    &nbsp;Wii nunchuck<br>
    &nbsp;TSOP-2236: IR Receiver<br>
    &nbsp;HC-SR04: Ultrasonic sensor<br>
    &nbsp;APDS-9301: Ambient light sensor<br>
    &nbsp;IR LED <br><br><br>
    </p>
    
    <p>
    <B>Communication</B><br>
    I2C, UART, Serial<br>
    </p>
*/

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
/**
 * @file    main.c
 * @brief   
 * @details  ** Enable global interrupt since Zumo library uses interrupts. **<br>&nbsp;&nbsp;&nbsp;CyGlobalIntEnable;<br>
*/

#if 0
//Maze
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
 
    
#endif

#if 1
// Hello World!
void zmain(void)
{
    printf("\nHello, World!\n");

    while(true)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif

#if 0
// Name and age
void zmain(void)
{
    char name[32];
    int age;
    
    
    printf("\n\n");
    
    printf("Enter your name: ");
    //fflush(stdout);
    scanf("%s", name);
    printf("Enter your age: ");
    //fflush(stdout);
    scanf("%d", &age);
    
    printf("You are [%s], age = %d\n", name, age);

    while(true)
    {
        BatteryLed_Write(!SW1_Read());
        vTaskDelay(100);
    }
 }   
#endif


#if 0
//battery level//
void zmain(void)
{
    ADC_Battery_Start();        

    int16 adcresult =0;
    float volts = 0.0;

    printf("\nBoot\n");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed

    while(true)
    {
        char msg[80];
        ADC_Battery_StartConvert(); // start sampling
        if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT)) {   // wait for ADC converted value
            adcresult = ADC_Battery_GetResult16(); // get the ADC value (0 - 4095)
            // convert value to Volts
            // you need to implement the conversion
            
            // Print both ADC results and converted value
            printf("%d %f\r\n",adcresult, volts);
        }
        vTaskDelay(500);
    }
 }   
#endif

#if 0 

//Tick Timer Example
void zmain(void) 
{
	TickType_t Ttime = xTaskGetTickCount();
	TickType_t PreviousTtime = 0;

	while(true) 
	{
		while(SW1_Read()) vTaskDelay(1); // loop until user presses button
		Ttime = xTaskGetTickCount(); // take button press time
		/*Print out the time between button presses in seconds. int cast used to suppress warning messages*/
		printf("The amount of time between button presses is: %d.%d seconds\n", (int)(Ttime-PreviousTtime)/1000%60, (int)(Ttime-PreviousTtime)%1000);
		while(!SW1_Read())vTaskDelay(1); // loop while user is pressing the button
		PreviousTtime = Ttime; // remember previous press time
	}
	
}

#endif

#if 0
// button
void zmain(void)
{
    while(true) {
        printf("Press button within 5 seconds!\n");
	    TickType_t Ttime = xTaskGetTickCount(); // take start time
        bool timeout = false;
        while(SW1_Read() == 1) {
            if(xTaskGetTickCount() - Ttime > 5000U) { // too long time since start
                timeout = true;
                break;
            }
            vTaskDelay(10);
        }
        if(timeout) {
            printf("You didn't press the button\n");
        }
        else {
            printf("Good work\n");
            while(SW1_Read() == 0) vTaskDelay(10); // wait until button is released
        }
    }
}
#endif

#if 0
// button
void zmain(void)
{
    printf("\nBoot\n");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed
    
    bool led = false;
    
    while(true)
    {
        // toggle led state when button is pressed
        if(SW1_Read() == 0) {
            led = !led;
            BatteryLed_Write(led);
            if(led) printf("Led is ON\n");
            else printf("Led is OFF\n");
            Beep(1000, 150);
            while(SW1_Read() == 0) vTaskDelay(10); // wait while button is being pressed
        }        
    }
 }   
#endif


#if 0
//ultrasonic sensor//
void zmain(void)
{
    Ultra_Start();                          // Ultra Sonic Start function
    
    while(true) {
        int d = Ultra_GetDistance();
        // Print the detected distance (centimeters)
        printf("distance = %d\r\n", d);
        vTaskDelay(200);
    }
}   
#endif

#if 0
//IR receiverm - how to wait for IR remote commands
void zmain(void)
{
    IR_Start();
    
    printf("\n\nIR test\n");
    
    IR_flush(); // clear IR receive buffer
    printf("Buffer cleared\n");
    
    bool led = false;
    // Toggle led when IR signal is received
    while(true)
    {
        IR_wait();  // wait for IR command
        led = !led;
        BatteryLed_Write(led);
        if(led) printf("Led is ON\n");
        else printf("Led is OFF\n");
    }    
 }   
#endif



#if 0
//IR receiver - read raw data
// RAW data is used when you know how your remote modulates data and you want to be able detect 
// which button was actually pressed. Typical remote control protocols requires a protocol specific
// state machine to decode button presses. Writing such a state machine is not trivial and requires
// that you have the specification of your remotes modulation and communication protocol    
void zmain(void)
{
    IR_Start();
    
    uint32_t IR_val; 
    
    printf("\n\nIR test\n");
    
    IR_flush(); // clear IR receive buffer
    printf("Buffer cleared\n");
    
    // print received IR pulses and their lengths
    while(true)
    {
        if(IR_get(&IR_val, portMAX_DELAY)) {
            int l = IR_val & IR_SIGNAL_MASK; // get pulse length
            int b = 0;
            if((IR_val & IR_SIGNAL_HIGH) != 0) b = 1; // get pulse state (0/1)
            printf("%d %d\r\n",b, l);
        }
    }    
 }   
#endif


#if 0
//reflectance
void zmain(void)
{
    struct sensors_ ref;
    struct sensors_ dig;

    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000); // set center sensor threshold to 11000 and others to 9000
    

    while(true)
    {
        // read raw sensor values
        reflectance_read(&ref);
        // print out each period of reflectance sensors
        printf("%5d %5d %5d %5d %5d %5d\r\n", ref.L3, ref.L2, ref.L1, ref.R1, ref.R2, ref.R3);       
        
        // read digital values that are based on threshold. 0 = white, 1 = black
        // when blackness value is over threshold the sensors reads 1, otherwise 0
        reflectance_digital(&dig); 
        //print out 0 or 1 according to results of reflectance period
        printf("%5d %5d %5d %5d %5d %5d \r\n", dig.L3, dig.L2, dig.L1, dig.R1, dig.R2, dig.R3);        
        
        vTaskDelay(200);
    }
}   
#endif


#if 0
//motor
void zmain(void)
{
    motor_start();              // enable motor controller
    motor_forward(0,0);         // set speed to zero to stop motors

    vTaskDelay(3000);
    
    motor_forward(100,2000);     // moving forward
    motor_turn(200,50,2000);     // turn
    motor_turn(50,200,2000);     // turn
    motor_backward(100,2000);    // moving backward
     
    motor_forward(0,0);         // stop motors

    motor_stop();               // disable motor controller
    
    while(true)
    {
        vTaskDelay(100);
    }
}
#endif

#if 0
/* Example of how to use te Accelerometer!!!*/
void zmain(void)
{
    struct accData_ data;
    
    printf("Accelerometer test...\n");

    if(!LSM303D_Start()){
        printf("LSM303D failed to initialize!!! Program is Ending!!!\n");
        vTaskSuspend(NULL);
    }
    else {
        printf("Device Ok...\n");
    }
    
    while(true)
    {
        LSM303D_Read_Acc(&data);
        printf("%8d %8d %8d\n",data.accX, data.accY, data.accZ);
        vTaskDelay(50);
    }
 }   
#endif    

#if 0
// MQTT test
void zmain(void)
{
    int ctr = 0;

    printf("\nBoot\n");
    send_mqtt("Zumo01/debug", "Boot");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 

    while(true)
    {
        printf("Ctr: %d, Button: %d\n", ctr, SW1_Read());
        print_mqtt("Zumo01/debug", "Ctr: %d, Button: %d", ctr, SW1_Read());

        vTaskDelay(1000);
        ctr++;
    }
 }   
#endif


#if 0
void zmain(void)
{    
    struct accData_ data;
    struct sensors_ ref;
    struct sensors_ dig;
    
    printf("MQTT and sensor test...\n");

    if(!LSM303D_Start()){
        printf("LSM303D failed to initialize!!! Program is Ending!!!\n");
        vTaskSuspend(NULL);
    }
    else {
        printf("Accelerometer Ok...\n");
    }
    
    int ctr = 0;
    reflectance_start();
    while(true)
    {
        LSM303D_Read_Acc(&data);
        // send data when we detect a hit and at 10 second intervals
        if(data.accX > 1500 || ++ctr > 1000) {
            printf("Acc: %8d %8d %8d\n",data.accX, data.accY, data.accZ);
            print_mqtt("Zumo01/acc", "%d,%d,%d", data.accX, data.accY, data.accZ);
            reflectance_read(&ref);
            printf("Ref: %8d %8d %8d %8d %8d %8d\n", ref.L3, ref.L2, ref.L1, ref.R1, ref.R2, ref.R3);       
            print_mqtt("Zumo01/ref", "%d,%d,%d,%d,%d,%d", ref.L3, ref.L2, ref.L1, ref.R1, ref.R2, ref.R3);
            reflectance_digital(&dig);
            printf("Dig: %8d %8d %8d %8d %8d %8d\n", dig.L3, dig.L2, dig.L1, dig.R1, dig.R2, dig.R3);
            print_mqtt("Zumo01/dig", "%d,%d,%d,%d,%d,%d", dig.L3, dig.L2, dig.L1, dig.R1, dig.R2, dig.R3);
            ctr = 0;
        }
        vTaskDelay(10);
    }
 }   

#endif

#if 0
void zmain(void)
{    
    RTC_Start(); // start real time clock
    
    RTC_TIME_DATE now;

    // set current time
    now.Hour = 12;
    now.Min = 34;
    now.Sec = 56;
    now.DayOfMonth = 25;
    now.Month = 9;
    now.Year = 2018;
    RTC_WriteTime(&now); // write the time to real time clock

    while(true)
    {
        if(SW1_Read() == 0) {
            // read the current time
            RTC_DisableInt(); /* Disable Interrupt of RTC Component */
            now = *RTC_ReadTime(); /* copy the current time to a local variable */
            RTC_EnableInt(); /* Enable Interrupt of RTC Component */

            // print the current time
            printf("%2d:%02d.%02d\n", now.Hour, now.Min, now.Sec);
            
            // wait until button is released
            while(SW1_Read() == 0) vTaskDelay(50);
        }
        vTaskDelay(50);
    }
 }   
#endif

/* [] END OF FILE */
