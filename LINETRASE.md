# springschool-watanabeA
#include <kernel.h>
#include "kernel_cfg.h"
#include "camtank.h"
#include "app.h"
#include "HTTPServer.h"
#include "mbed_rpc.h"
#include "SDFileSystem.h"
#include "i2c_setting.h"
#include "Zumo.h"
#include "mbed.h"

#include "GR_PEACH_Camera.h"
#include "GR_PEACH_WlanBP3595.h"

GR_PEACH_Camera camera;

Zumo zumo;

GR_PEACH_WlanBP3595 wlan;
Serial pc(USBTX,USBRX);

DigitalOut usb1en(P3_8);

#if (SD_SPICH == 2)
SDFileSystem sdfs(P8_5, P8_6, P8_3, P8_4, "sdroot"); // mosi miso sclk cs name
#elif (SD_SPICH == 1)
SDFileSystem sdfs(P4_6, P4_7, P4_4, P4_5, "sdroot"); // mosi miso sclk cs name
#endif


#include "app_config.h"

#include "Milkcocoa.h"


#define MILKCOCOA_SERVERPORT  1883
const int MAX_SENSOR_VALUE = 1024;
const int IR_THRESHOLD = 200;

extern void onpush(MQTT::MessageData& md);

DigitalOut dir_left(D8);
DigitalOut dir_right(D7);

PwmOut pwm_left(P5_0);      //TIOC0
PwmOut pwm_right(P8_14);    //TIOC2

static DigitalInOut ir_pins[] =
		{ DigitalInOut(D5), DigitalInOut(A2), DigitalInOut(A0), DigitalInOut(
				D11), DigitalInOut(A3), DigitalInOut(D4) };
static DigitalOut emitter(A4);
static Timer timer;

DigitalOut led_r(LED1);
DigitalOut led_g(LED2);
DigitalOut led_b(LED3);

int basespeed = 50;
int leftPower = 50;
int rightPower = 50;

static int constrain(int input, int min, int max) {
	if (input < min) {
		return min;
	} else if (max < input) {
		return max;
	} else {
		return input;
	}
}

void driveTank(int left, int right) {
	int _lmotor = constrain(left, -255, 255);
	int _rmotor = constrain(right, -255, 255);

	if (_lmotor < 0) {
		dir_left = 1;
		_lmotor = -_lmotor;
	} else {
		dir_left = 0;
	}
	if (_rmotor < 0) {
		dir_right = 1;
		_rmotor = -_rmotor;
	} else {
		dir_right = 0;
	}
	pwm_left.write(_lmotor / 255.0f);
	pwm_right.write(_rmotor / 255.0f);
}

void reace(int x,int y,int z){
		basespeed = 50;
		if (x) {
			// turn strong right
			driveTank(-1.0 * basespeed, basespeed);
		    led_r = 0;
		    led_g = 0;
		    led_b = 0;
		} else if (z) {
			// turn strong left
			driveTank(basespeed, -1.0 * basespeed);
		    led_r = 0;
		    led_g = 0;
		    led_b = 0;
		} else if (y) {
			// move forward
			driveTank(basespeed, basespeed);
		    led_r = 0;
		    led_g = 0;
		    led_b = 0;
		}else if (z&&y) {
			///turn left
			driveTank(basespeed*0,basespeed);
		    led_r = 0;
		    led_g = 0;
		    led_b = 0;

		}else if (x&&y) {
			//turn left
			driveTank(basespeed,basespeed*0);
		    led_r = 0;
		    led_g = 0;
		    led_b = 0;

		}else {
			//turn right
			driveTank(basespeed, 0.1 * basespeed);
		    led_r = 0;
		    led_g = 0;
		    led_b = 0;
		}
	}

void goal(){
	basespeed = 0;
			// on the end line. stop.
			driveTank(basespeed, basespeed);
	led_r = 1;
	led_g = 1;
	led_b = 1;

}



void Zumo::readAnalogIrValue(unsigned int *values) {
	int i;
	emitter = 1;
	wait_us(200);
	for (i = 0; i < 6; i++) {
		values[i] = MAX_SENSOR_VALUE - 1;
		ir_pins[i].output();
		ir_pins[i] = 1;
	}
	for (i = 0; i < 6; i++) {
		ir_pins[i].input();
		ir_pins[i] = 0;
	}
	timer.start();
	timer.reset();
	unsigned int time = 0;
	while (timer.read_us() < MAX_SENSOR_VALUE) {
		time = timer.read_us();
		for (i = 0; i < 6; i++) {
			if (ir_pins[i] == 0 && time < values[i]) {
				values[i] = time;
			}
		}
	}
	timer.stop();
	emitter = 0;
	wait_us(200);
}

void Zumo::readIr(IrBitField_T &irbits) {
	unsigned int irvalues[6];
	readAnalogIrValue(irvalues);
	irbits.right = (irvalues[0] > IR_THRESHOLD);
	irbits.center = (irvalues[3] > IR_THRESHOLD);
	irbits.left = (irvalues[5] > IR_THRESHOLD);
}

void task_main(intptr_t exinf) {
	pc.baud(9600);
	IrBitField_T irbits;
	unsigned int irValues[6];
	while (1) {
		zumo.readAnalogIrValue(irValues);
		pc.printf("%d %d %d %d %d %d \r\n",irValues[0],irValues[1],irValues[2],irValues[3],irValues[4],irValues[5]);
		zumo.readIr(irbits);
		if (irbits.left && irbits.center && irbits.right) {
			goal();
		}
		else{
			reace(irbits.left , irbits.center , irbits.right);
		}

		dly_tsk(100);

	}
}

static char i2c_setting_str_buf[I2C_SETTING_STR_BUF_SIZE];

static void TerminalWrite(Arguments* arg, Reply* r) {
	if ((arg != NULL) && (r != NULL)) {
		for (int i = 0; i < arg->argc; i++) {
			if (arg->argv[i] != NULL) {
				printf("%s", arg->argv[i]);
			}
		}
		printf("\n");
		r->putData<const char*>("ok");
	}
}

static void SetI2CfromWeb(Arguments* arg, Reply* r) {
	int result = 0;

	if (arg != NULL) {
		if (arg->argc >= 2) {
			if ((arg->argv[0] != NULL) && (arg->argv[1] != NULL)) {
				sprintf(i2c_setting_str_buf, "%s,%s", arg->argv[0], arg->argv[1]);
				result = 1;
			}
		} else if (arg->argc == 1) {
			if (arg->argv[0] != NULL) {
				sprintf(i2c_setting_str_buf, "%s", arg->argv[0]);
				result = 1;
			}
		} else {
			 /*Do nothing*/
		}
		/* command analysis and execute */

		if (result != 0) {
			if (i2c_setting_exe(i2c_setting_str_buf) != false) {
				r->putData<const char*>(i2c_setting_str_buf);
			}
		}
	}
}

void SaveJpeg(Arguments* arg, Reply* r) {
	static int count = 0;
	int size;
	const char *p_data;
	char filename[128];
	FILE * fp;

	sprintf(filename, "/sdroot/DCIM/100PEACH/CAM%05d.jpg", count);

	size = snapshot_req(&p_data);
	fp = fopen(filename, "w");
	if(fp){
		fwrite(p_data, size, 1, fp);
		fclose(fp);
		printf("save_jpeg[%s}\r\n", filename);
	}
	else
	{
		printf("save_jpeg fopen error\r\n");
	}
	count++;
	return;
}
