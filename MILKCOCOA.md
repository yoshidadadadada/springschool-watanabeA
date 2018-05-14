# springschool-watanabeA

/*
Warning!
  When exporting and using it, increase the following stack size.
  
  [EthernetInterface/lwip/lwipopts.h]---------
  #define TCPIP_THREAD_STACKSIZE      1024
  ->
  #define TCPIP_THREAD_STACKSIZE      2048
  --------------------------------------------
*/
/*
This works with the following library.
  mbed-rtos : revision 115
*/

#include <kernel.h>
#include "kernel_cfg.h"
#include "app.h"
#include "app_config.h"
#include "GR_PEACH_WlanBP3595.h"
#include "mbed.h"
#include "Milkcocoa.h"
#include "Zumo.h"

int num_v=7;

GR_PEACH_WlanBP3595 wlan;
//Serial pc(USBTX, USBRX);

Zumo zumo;

// Wi-Fi SSID
#define WLAN_SSID                       ("SPWH_H32_E42E74")
// Wi-Fi?p?X???[?h
#define WLAN_PSK                        ("springschool2018")
// APP_ID
#define MILKCOCOA_APP_ID                "onjgergp19"

#define MILKCOCOA_DATASTORE             "zumo/sensor"
#define MILKCOCOA_MESSAGE_DATASTORE     "zumo/message"
#define MILKCOCOA_PUSH_DATASTORE        "zumo/remotecontrol"
#define MILKCOCOA_SENSOR_DATASTORE      "zumo/sensor2"
#define MILKCOCOA_SERVERPORT            1883
const int MAX_SENSOR_VALUE = 1024;
const int IR_THRESHOLD = 200;
void readAnalogIrValue(unsigned int *values);

extern void onpush(MQTT::MessageData& md);
void reace(int x,int y,int z);
void goal();
int basespeed = 50;
//void readAnalogIrValue(unsigned int *values);

const char MQTT_SERVER[]  = MILKCOCOA_APP_ID ".mlkcca.com";
const char MQTT_CLIENTID[] = __TIME__ MILKCOCOA_APP_ID;

DigitalOut  red_led(LED1);              // On: error
DigitalOut  green_led(LED2);            // On: WLAN has been connected

DigitalInOut ir_pins[] = {
    DigitalInOut(D5),
    DigitalInOut(A2),
    DigitalInOut(A0),
    DigitalInOut(D11),
    DigitalInOut(A3),
    DigitalInOut(D4)
};
DigitalOut emitter(A4);
Timer timer;
PwmOut pwm_right(P8_14);
PwmOut pwm_pan(P5_3);
PwmOut pwm_tilt(P3_8);
DigitalOut led(LED1);
Serial pc(USBTX, USBRX);

DigitalOut led_r(LED1);
DigitalOut led_g(LED2);
DigitalOut led_b(LED3);

// zumo??@??ID
const char zumoId[] = "zumo123456789";
// ?????N??
int initFlag = 0;
// ?X?s?[?h
int baseSpeed = 50;
int leftPower = 0;
int rightPower = 0;
// ?C???^?[?o??????b??
int intervalSecLimit = 10;
// ?C???^?[?o???^?C?}?[
Timer intervalTimer;
// ?N??????????v??
Timer exeTimer;

int sensor_L;
int sensor_C;
int sensor_R;


unsigned int irvalues[6];

int Speed_Right;
int Speed_Left;
int s = 0;
void ledStatus(int r,int g,int b){
    led_r = r;
    led_g = g;
    led_b = b;
}

/** Main function
 *
 */

void judeg(){
	IrBitField_T irbits;

     //   tankDrive(baseSpeed,baseSpeed);


		zumo.readIr(irbits);

		if (irbits.left && irbits.center && irbits.right) {
			goal();
		}
		else{
			reace(irbits.left , irbits.center , irbits.right);
		}
}


void task_main(intptr_t exinf) {
    pc.baud(9600);
    // start
    intervalTimer.start();
    exeTimer.start();
    ledStatus(1,1,1);

    

	IrBitField_T irbits;

    pc.printf("[[Milkcocoa mbed version]]\n\r");
    
    // connecting
    ledStatus(0,1,0);
    
    pc.printf("--- WLAN CONNECT START ---\n\r");
    
    MQTTBP3595 *ipstack = new MQTTBP3595(WLAN_SSID, WLAN_PSK);
    
    MClient *client = new MClient(ipstack);
    
    Milkcocoa *milkcocoa = new Milkcocoa(client, MQTT_SERVER, MILKCOCOA_SERVERPORT, MILKCOCOA_APP_ID, MQTT_CLIENTID);

    pc.printf("--- Milkcocoa start ---\n\r");
    
    milkcocoa->connect();
    pc.printf("milkcocoa base connected\n\r");
    
    // connected
    ledStatus(0,0,1);
    pc.printf(".");
    wait(1.0);
    ledStatus(0,0,0);
    pc.printf(".");
    wait(1.0);
    ledStatus(0,0,1);
    pc.printf(".");
    wait(1.0);
    ledStatus(0,0,0);
    pc.printf(".");
    wait(1.0);
    ledStatus(0,0,1);
    wait(1.0);  // 5?b???x????????????????????
    
    pc.printf("\n\r");
    pc.printf("milkcocoa onpush event connected\n\r");
    pc.printf("push -> topic:");
    pc.printf(MILKCOCOA_PUSH_DATASTORE);
    pc.printf("\n\r");
    int milkcocoa_ok = milkcocoa->on(MILKCOCOA_PUSH_DATASTORE, "push", onpush);
    pc.printf("%d\n\r",milkcocoa_ok);
    
    // connect status
    ledStatus(0,milkcocoa_ok,1);


   	while(1){
        // onpush???m??????????K?v??????
        milkcocoa->loop();
        // ????N?????????b?Z?[?W?????
        if( initFlag == 0 ){
            initFlag = 1;
            DataElement msg_elem = DataElement();
            pc.printf("--- ZUMO_START ---\n\r");
            msg_elem.setValue("status", 1);
            msg_elem.setValue("onpush", milkcocoa_ok );
            msg_elem.setValue("et", (int)exeTimer.read() );
            milkcocoa->push(MILKCOCOA_MESSAGE_DATASTORE, msg_elem);
        }
        // intervalSecLimit ?w??b?????a??f?[?^????
//*************************************************************************************************************
        if( intervalTimer.read() > intervalSecLimit ){
            pc.printf("--- ZUMO_SEND ---\n\r");
            DataElement push_elem = DataElement();
//            DataElement sensor_elem = DataElement();


            zumo.readAnalogIrValue(irvalues);
            zumo.readIr(irbits);
           // readMotor(Speed_Left,Speed_Right);
            push_elem.setValue("time", (int)exeTimer.read() );
            // right_sensor
            push_elem.setValue("r",irbits.right);
            // center_sensor
            push_elem.setValue("c",irbits.center);
            // left_sensor
            push_elem.setValue("l",irbits.left);
            // right_motor_data
            push_elem.setValue("R",rightPower);
            // left_moter_data
  //          push_elem.setValue("L",leftPower);

            push_elem.setValue("M",baseSpeed);

        	milkcocoa->push(MILKCOCOA_DATASTORE, push_elem);
            intervalTimer.reset();
        }


   		if(s==1){
   		  		judeg();
   		}

       dly_tsk(30);
   	}
}

void reace(int x,int y,int z){
		if (x) {
			// turn strong right
	        leftPower = -1;
	        rightPower = 1;
	        zumo.driveTank(baseSpeed * leftPower,baseSpeed * rightPower);
		    led_r = 0;
		    led_g = 0;
		    led_b = 0;
		}
		else if (z) {
			// turn strong left
		    leftPower = 1;
		    rightPower = -1;
		    zumo.driveTank(baseSpeed * leftPower,baseSpeed * rightPower);
		    led_r = 0;
		    led_g = 0;
		    led_b = 0;
		} else if (y) {
			// move forward
	        leftPower = 1;
	        rightPower = 1;
	        zumo.driveTank(baseSpeed * leftPower,baseSpeed * rightPower);
		    led_r = 0;
		    led_g = 0;
		    led_b = 0;
		}else if (z&&y) {
			///turn left
	        leftPower = -1;
	        rightPower = 1;
	        zumo.driveTank(baseSpeed * leftPower,baseSpeed * rightPower);
		    led_r = 0;
		    led_g = 0;
		    led_b = 0;

		}else{
			//turn left
	        leftPower = 1;
	        rightPower = -1;
	        zumo.driveTank(baseSpeed * leftPower,baseSpeed * rightPower);
		    led_r = 0;
		    led_g = 0;
		    led_b = 0;

		}
	}


void goal(){
	// on the end line. stop.
    leftPower = 0;
    rightPower = 0;
    zumo.driveTank(baseSpeed * leftPower,baseSpeed * rightPower);
	led_r = 1;
	led_g = 1;
	led_b = 1;
	s = 0;
}



void onpush(MQTT::MessageData& md)
{
    MQTT::Message &message = md.message;
    DataElement de = DataElement((char*)message.payload);
    pc.printf("\n\r");
    pc.printf("[onpush]\n\r");
    pc.printf((char*)message.payload);
    pc.printf("\n\r");
    const char *value = de.getString("v");
    pc.printf("%d\n\r",de.getInt("v"));
    pc.printf(value);
    pc.printf("\n\r");
    
    int v = de.getInt("v");

    if( v == 0 ){
        pc.printf("mode 0 \n\r");
        leftPower = 0;
        rightPower = 0;
        zumo.driveTank(baseSpeed * leftPower,baseSpeed * rightPower);
    } else if( v == 1 ){
        pc.printf("mode 1 \n\r");
        leftPower = 1;
        rightPower = 1;
        zumo.driveTank(baseSpeed * leftPower,baseSpeed * rightPower);
    } else if( v == 2 ){
        pc.printf("mode 2 \n\r");
        leftPower = -1;
        rightPower = -1;
        zumo.driveTank(baseSpeed * leftPower,baseSpeed * rightPower);
    } else if( v == 3 ){
        pc.printf("mode 3 \n\r");
        leftPower = 1;
        rightPower = -1;
        zumo.driveTank(baseSpeed * leftPower,baseSpeed * rightPower);
    } else if( v == 4 ){
        pc.printf("mode 4 \n\r");
        leftPower = -1;
        rightPower = 1;
        zumo.driveTank(baseSpeed * leftPower,baseSpeed * rightPower);
    } else if( v == 5 ){
        pc.printf("mode 5 \n\r");
        baseSpeed += 10;
        zumo.driveTank(baseSpeed * leftPower,baseSpeed * rightPower);
    } else if( v == 6 ){
        pc.printf("mode 6 \n\r");
        baseSpeed -= 10;
        zumo.driveTank(baseSpeed * leftPower,baseSpeed * rightPower);
    } else if( v == 7 ){
    	s =1;
    	return;
    } else if( v == 8 ){
        
    } else if( v == 9 ){
        
    } else if( v == 90 ){
        
    } else if( v == 91 ){
        
    } else if( v == 92 ){
        
    }
}



