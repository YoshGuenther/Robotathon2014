
#include <RASLib/inc/common.h>
#include <RASLib/inc/servo.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/adc.h>
#include <RASLib/inc/linesensor.h>

static tMotor *leftMotor;
static tMotor *rightMotor;
static tADC *ir[4];
static tLineSensor *gls;
static tBoolean initialized = false;

float irDistances[4];
float irValues[4];
float irWallDistance = 15;
float rightPos = 0; 
float leftPos = 0;

int x;
int count;
int irIndex = 0;

/*ADC is only supported on a limited number of pins in hardware
The following pins are supported:
PIN_B4, PIN_B5, PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_E0, PIN_E1, PIN_E2, PIN_E3, PIN_E4, PIN_E5 */

void init(void) {
    // don't initialize this if we've already done so
    if(initialized){
        // make sure the motors are off
        SetMotor(leftMotor, 0.0f);
        SetMotor(rightMotor, 0.0f);
        return;}
    initialized = true;
				
		ir[0] = InitializeADC(PIN_D0); //Right Side IR Sensor
		ir[1] = InitializeADC(PIN_B4); //Right Front IR Sensor
		ir[2] = InitializeADC(PIN_E4); //Left Side IR Sensor
		ir[3] = InitializeADC(PIN_E3); //Left Front IR Sensor
				
    //use 8 I/O pins to initialize a GPIO line sensor
    //gls = InitializeGPIOLineSensor(PIN_, PIN_, PIN_, PIN_, PIN_D PIN_E PIN_, PIN_);

		leftMotor = InitializeServoMotor(PIN_B0, false);
		rightMotor = InitializeServoMotor(PIN_B5, false);
		SetMotor(leftMotor, 0.0f);
		SetMotor(rightMotor, 0.0f);}

		
/*convert the value of the IR sensor to a distance in inches
	Approximated through quantitative data from the IR sensor
  {(inches, IR sensor value), (13, 0.249), (10, 0.32), (7, 0.47), (4, 0.739) (2, 0.94)}*/
float irToInches(float x){ 
return 42.9f - 212.3f*x +483.2f*x*x - 503.3f*x*x*x +191.7f*x*x*x*x;}


// The 'main' function is the entry point of the program
int main(void) {
    // Initialization code can go here
		Printf("Beginning of Program\n");
	  init();   
		Printf("initialized\n");
	
    while (1) {
			//Right side IR Sensor
				irValues[0] = ADCRead(ir[0]); 				
				irDistances[0] = irToInches(irValues[0]); 
			//Right front IR Sensor
				irValues[1] = ADCRead(ir[1]);  	
				irDistances[1] = irToInches(irValues[1]); 
			//Left side IR Sensor
				irValues[2] = ADCRead(ir[2]);					
				irDistances[2] = irToInches(irValues[2]); 
			//Left side IR Sensor
				irValues[3] = ADCRead(ir[3]);			
				irDistances[3] = irToInches(irValues[3]); 
				
				Printf("Right Side: (value, inches) = (%.5f, %.5f)\n", irValues[0], irDistances[0]);
				Printf("Right Front: (value, inches) = (%.5f, %.5f)\n", irValues[1], irDistances[1]);
				Printf("Left Side: (value, inches) = (%.5f, %.5f)\n", irValues[2], irDistances[2]);	
				Printf("Left Front: (value, inches) = (%.5f, %.5f)\n", irValues[3], irDistances[3]);
				Printf("\n\n\r");
			
			 /*float line[8];
				//put the values of the line sensor into the 'line' array 
        LineSensorReadArray(gls, line);*/

			
			if(irDistances[0] < irWallDistance && irDistances[1] < irWallDistance){
				//if the right and front right sensors are blocked, turn left
				leftPos = -1;
				rightPos = 1;}
			else if(irDistances[2] < irWallDistance && irDistances[3] < irWallDistance){
				//if the left and front left sensors are blocked, turn right
				leftPos = 1;
				rightPos = -1;}
			else if(irDistances[3] < irWallDistance || irDistances[1] < irWallDistance){
				//if both of the front sensors are blocked....
				if(irDistances[0] > irDistances[2]){
						//if the distance on the right is greater than the left, turn right
						leftPos = 1;
						rightPos = -1;}
				else{
						//if the distance on the right is less than the left, turn left
						leftPos = -1;
						rightPos = 1;}}
			else{
					//if none of the censors are blocked, go forward
					leftPos = 1;
					rightPos = 1;} 
						
			SetMotor(leftMotor, leftPos);
			SetMotor(rightMotor, -rightPos);
			Wait(0.125f);}
		
		// make sure the motors are off before exiting the demo 
    SetMotor(leftMotor, 0.0f);
    SetMotor(rightMotor, 0.0f);
    Printf("End of Program\n");
	}
	
	


