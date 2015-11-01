
#include <RASLib/inc/common.h>
#include <RASLib/inc/servo.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/adc.h>
#include <RASLib/inc/time.h>
#include <RASLib/inc/linesensor.h>

/*
Team Name: Average Joes

Team Members:
Nancy Xu
Joshua Guenther
Matthew J-Union
Sarah Muschinske

Team Mentors:
Noor Haibara
Abigail Johnson

Important Notes:
ADC is only supported on a limited number of pins in hardware
The following pins support ADC:
PIN_B4, PIN_B5, PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_E0, PIN_E1, PIN_E2, PIN_E3, PIN_E4, PIN_E5
*/

// Global variables
static tMotor *leftMotor;
static tMotor *rightMotor;
static tADC *ir[4];
static tLineSensor *ls;

static tBoolean initialized = false;

float irDistances[4];
float irValues[4];
float line[8];

int x;
double prevErr;
double intErr;
double diffErr;
double err;
double out;
double leftset;
double rightset;
double prevLinerr;
double intLinerr;
double diffLinerr;
double wlerr;
double wrerr;
double llerr;
double lrerr;
double outlin;
float prevLerr = 0;
float prevRerr = 0;
int done;

//Constants
static const float PROPK = 0.25;
static const float INTK = 0;
static const float DIFFK =  0.015;
static const float PROPKL = 0.1;
static const float INTKL = 0;
static const float DIFFKL = 0.01;

// Function Prototypes
void left(void);
void right(void);
void updsens(void);
void linefollow(void);
void zigzag(void);



void init(void) {
  ir[0] = InitializeADC(PIN_D0); //Right Back IR Sensor
  ir[1] = InitializeADC(PIN_B4); //Right Front IR Sensor
  ir[2] = InitializeADC(PIN_E4); //Left Front IR Sensor
  ir[3] = InitializeADC(PIN_E3); //Left Back  IR Sensor
  leftMotor = InitializeServoMotor(PIN_B0, false);
  rightMotor = InitializeServoMotor(PIN_B5, false);
  SetMotor(leftMotor, 0.0f);
  SetMotor(rightMotor, 0.0f);
  //(sensor pin, launchpad pin): (8, PF4) (7, PD7) (6, PD6) (5, PC7) (4, PC6) (3, PC5) (2, PC4) (1, PB3) (LEDon, PF3) (Vcc, VBUS) (GND, GND).
  //use 8 I/O pins to initialize a GPIO line sensor
	ls = InitializeGPIOLineSensor(PIN_B3, PIN_C4, PIN_C5, PIN_C6, PIN_C7, PIN_D6, PIN_D7, PIN_F4);
}

/* Convert the value of the IR sensor from a signal to a distance in inches
	Approximated through quantitative data from the IR sensor
  {(inches, IR sensor value), (13, 0.249), (10, 0.32), (7, 0.47), (4, 0.739) (2, 0.94)}*/
float irToInches(float x){
  return 42.9f - 212.3f*x +483.2f*x*x - 503.3f*x*x*x +191.7f*x*x*x*x;
}



void updsens(void){
    	//put the values of the line sensor into the 'line' array
   LineSensorReadArray(ls, line);
    //LineSensorReadContinuouslyUS(tLineSensor *ls, tTime us)
    	//Right back IR Sensor
	irValues[0] = ADCRead(ir[0]);
	irDistances[0] = irToInches(irValues[0]);
	//Right front IR Sensor
	irValues[1] = ADCRead(ir[1]);
	irDistances[1] = irToInches(irValues[1]);
	//Left front IR Sensor
	irValues[2] = ADCRead(ir[2]);
	irDistances[2] = irToInches(irValues[2]);
	//Left back IR Sensor
	irValues[3] = ADCRead(ir[3]);
	irDistances[3] = irToInches(irValues[3]);
    	Printf("Right Side:  (value, inches) = (%.5f, %.5f)\n", irValues[0], irDistances[0]);
    	Printf("Right Front: (value, inches) = (%.5f, %.5f)\n", irValues[1], irDistances[1]);
    	Printf("Left Side:   (value, inches) = (%.5f, %.5f)\n", irValues[2], irDistances[2]);
    	Printf("Left Front:  (value, inches) = (%.5f, %.5f)\n", irValues[3], irDistances[3]);
    	//Printf("line sensor: %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", line[0], line[1], line[2], line[3], line[4], line[5], line[6], line[7]);

    	Printf("\n\n\r");
    	return;}

void findflag(void){
	while(1){
   		SetMotor(leftMotor, 1);
   		SetMotor(rightMotor, -1);
   		Wait(3.f);
    	SetMotor(leftMotor, 0.1);
    	SetMotor(rightMotor, -0.1);
    	updsens();
    	if(irDistances[0] < 20){
        	SetMotor(leftMotor, -1);
        	SetMotor(rightMotor, -1);
        	Wait(1.f);
        	SetMotor(leftMotor, 1);
          SetMotor(rightMotor, -1);
        	Wait(5.f);
        	done = 1;
        	return;
      }
    	else if(irDistances[1] < 20){
        	SetMotor(leftMotor, -1);
        	SetMotor(rightMotor, -1);
        	Wait(0.5f);
         	SetMotor(leftMotor, 1);
          SetMotor(rightMotor, -1);
          Wait(5.f);
        	done = 1;
          return;
      }
    	else if(irDistances[2] < 20){
        	SetMotor(leftMotor, 1);
        	SetMotor(rightMotor, 1);
        	Wait(1.f);
         	SetMotor(leftMotor, 1);
                    	SetMotor(rightMotor, -1);
                    	Wait(5.f);
        	done = 1;
                    	return;
        	}
    	else if (irDistances[3] < 20){
        	SetMotor(leftMotor, 1);
        	SetMotor(rightMotor, 1);
        	Wait(0.5f);
         	SetMotor(leftMotor, 1);
          SetMotor(rightMotor, -1);
          Wait(5.f);
        	done = 1;
          return;
        	}
    	}
	return;
	}

void rightS2(void){
  while(1){
    //go foward
   	SetMotor(rightMotor, -1.0);
   	SetMotor(leftMotor, 1.0);
   	//turn backup and left once wall is seen
   	updsens();
   	if(irDistances[1] && irDistances[2] < 10){
   	  //backup
   	  SetMotor(rightMotor, .7);
   	  SetMotor(leftMotor, -.7);
   	  Wait(.75);
   	  //left
   	  SetMotor(rightMotor, -.7);
   	  SetMotor(leftMotor, -.7);
   	  Wait(1.3);
   	}
  }
}

void rightS(void){
  int i = 0;
  while(1){
      //go foward
   	  if(i == 1){
   		   break;}
   	  SetMotor(rightMotor, -1.0);
   	  SetMotor(leftMotor, 1.0);
   	  //turn right once wall is seen
   	  updsens();
   	  if( irDistances[1] && irDistances[2] < 10){
        i = 1;
        //backup
   		  SetMotor(rightMotor, .7);
        SetMotor(leftMotor, -.7);
        Wait(.75);
        //right
        SetMotor(rightMotor, .7);
        SetMotor(leftMotor, .7);
        Wait(1.25);
   	  }
    }
    SetPin(PIN_F3, !led_on);
    i = 0;
    while(1){
   	 //go foward
   	 SetMotor(rightMotor, -1.0);
   	 SetMotor(leftMotor, 1.0);
   	 //turn right once wall is seen
   	 updsens();
   	 if( irDistances[1] && irDistances[2] < 10){
   		 //backup
   		 SetMotor(rightMotor, .7);
   		 SetMotor(leftMotor, -.7);
   		 Wait(.75);
   		 //left
   		 SetMotor(rightMotor, -.7);
   		 SetMotor(leftMotor, -.7);
   		 Wait(1.25);
   		 }
   	 }
    return;
}

void leftS(void){
    int i = 0;
    while(1){
   	 //go foward
   	 if (i == 1)
   		 break;
   	 SetMotor(rightMotor, -1.0);
   	 SetMotor(leftMotor, 1.0);
   	 //turn right once wall is seen
   	 updsens();
   	 if ( irDistances[1] && irDistances[2] < 10){
   		 i = 1;
   		 //backup
   		 SetMotor(rightMotor, .7);
   		 SetMotor(leftMotor, -.7);
   		 Wait(.75);
   		 //left
   		 SetMotor(rightMotor, -.7);
   		 SetMotor(leftMotor, -.7);
   		 Wait(1.25);
   	 }
    }
    while(1){
   	 //go foward
   	 SetMotor(rightMotor, -1.0);
   	 SetMotor(leftMotor, 1.0);
   	 //turn right once wall is seen
   	 updsens();
   	 if( irDistances[1] && irDistances[2] < 10){
   		 //backup
   		 SetMotor(rightMotor, .7);
   		 SetMotor(leftMotor, -.7);
   		 Wait(.75);
   		 //right
   		 SetMotor(rightMotor, .7);
   		 SetMotor(leftMotor, .7);
   		 Wait(1.25);
   		 //rightS2();
   	 }
    }
    return;
}



tBoolean led_on;
/*Method used to blink an LEd.
This was first written to complete the first checkpoint.*/
void blink(void) {
  SetPin(PIN_F1, led_on);
  SetPin(PIN_F3, !led_on);
  led_on = !led_on;
}

/*Method to make the robot move in a riz-zag pattern.
  This was used to fill the requirements for one of the checkpoints.*/
void zigzag(void){
  SetMotor(rightMotor, 1);
  SetMotor(leftMotor, 1);
  Wait(0.7f);
  SetMotor(rightMotor, -1);
  SetMotor(leftMotor, 1);
  Wait(2.f);
  while (1){
    SetMotor(rightMotor, 1);
    SetMotor(leftMotor, 1);
    Wait(1.5f);
    SetMotor(rightMotor, -1);
    SetMotor(leftMotor, 1);
    Wait(2.f);
    SetMotor(rightMotor, -1);
    SetMotor(leftMotor, -1);
    Wait(1.5f);
    SetMotor(rightMotor, -1);
    SetMotor(leftMotor, 1);
    Wait(2.f);
    return;
  }
}

int main(void) {
	Printf("Beginning of Program\n");
	init();
	Printf("initialized\n");

  //Continuous call to get a LED to blink
  //CallEvery(blink, 0, 0.25f);

  //Choose between leftS, rightS, or rightS2 before match.
    leftS(); //
    //rightS(); //
    //rightS2(); //

   	findflag();

	//linefollow(); // We were not able to implement line following in time.

	// Make sure the motors are off before exiting
	SetMotor(leftMotor, 0.0f);
	SetMotor(rightMotor, 0.0f);
	Printf("End of Program\n");
	return 0;
}
