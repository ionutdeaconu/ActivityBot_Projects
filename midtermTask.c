/*
 * Created by Ionut Deaconu, Rija Rizvi and Samantha Han
 *
 * The program makes an ActivityBot robot go down a narrow passage, 
 * stopping at the end, rotating and retracing its steps.
 *
 * The robot uses IR sensors to keep distance from the walls and 
 * PING sensors to stop at a dead-end.
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "abdrive.h"
#include "simpletext.h"
#include "simpletools.h"
#include "ping.h"
 
//struct for linked list of all the position
struct position 
{
  struct position *next; //pointer to the next item in the list
  int leftTicks; //the number of left ticks the robot has moved
  int rightTicks; //the number of right ticks the robot has moved
};  

//sd_mount(DO, CLF, DI, CS);
//FILE* fp;

//define the distance from the PING sensor
int PINGdistance;
//initialise the values of the SD card pins
int DO = 22, CLK = 23, DI = 24, CS = 25;
//initialise the PID constants
double kp = 1;
double kd = 0.4;
double proportional;
double derivative;

double irLeft = 0;
double irRight = 0;
double error;
double errorPrior = 0;
//Variables for PID controller
double leftSpeed = 40; //TO_DO adjust for right speed
double rightSpeed = 40;

int ticksLeftSoFar, ticksRightSoFar, ticksLeftNow, ticksRightNow;
int errorLeft = 0, errorRight = 0;
int leftWheel, rightWheel;

//define the previous node of the singly-linked list
struct position *previous = NULL;

//function to insert a new position in the linked list
//at the beginning at the list
void insertPosition(int left, int right)
{
  //allocate requested memory and returns pointer to the new position
  struct position *newPosition = (struct position*) malloc(sizeof(struct position));
  
  newPosition->leftTicks = left;
  newPosition->rightTicks = right;
  
  newPosition->next = previous;
  previous = newPosition;
} 

//Variables for system time
long lastTime;
long nowTime;
struct timeval tv;
struct timeval now;
int dt = 100000; //in time interval of 1/3 of a second 

//Variables for position
int currentX;
int currentY;

int cumLeft = 0;
int cumRight = 0;
float totalAngle = 0.0;

double robotW = 33.0; //in ticks
double cumDist = 0.0;

//Returns the distance to the left wall
double leftDist ()
{
	irLeft = 0;
	for(int dacVal = 0; dacVal < 160; dacVal += 10)  
  {                                               
    dac_ctr(26, 0, dacVal);                       
    freqout(11, 1, 38000);                        
    irLeft += input(10);
  }
	return irLeft;
}

//Returns the distance to the right wall
double rightDist ()
{
	irRight = 0;
	for(int dacVal = 0; dacVal < 160; dacVal += 8) // increment dacVal by 16 if we want to increase for higher resolution and have range from 0 to 10.
  {                                               
    dac_ctr(27, 1, dacVal);                       
    freqout(1, 1, 38000);                        
    irRight += input(2);
  }
	return irRight;
}

//function to return the output of the PID controller
double navigate(int iterationTime)
{
	error = leftDist() - rightDist();

	double output = 0;
	double actualTime = (iterationTime * 1.0) / 1000000.0;
	
	proportional = error;
	derivative = error - errorPrior;
	output = kp * proportional + kd * derivative;
	
	errorPrior = error;
	return output;
}

void cumulative(int l, int r)
{
	double incremental = (l - r) / robotW;
	totalAngle += incremental;
	double rm = (l + r) / 2.0;
	double dx = rm - rm * cos(incremental);
	double dy = rm * sin(incremental);
	double offsetDist = sqrt(dx * dx + dy * dy);
	cumDist += offsetDist;
}

int main(int argc, const char* argv[])
{  
  low(26);                                      
  low(27);
	gettimeofday (&tv, NULL);
	lastTime = tv.tv_usec; //gets microseconds
	
  PINGdistance = ping_cm(8);
	while(PINGdistance > 20)
	{
		//fp = fopen("ticks.txt", "w");
		gettimeofday(&now, NULL);
		nowTime = now.tv_usec;
		if(nowTime - lastTime >= dt)
		{

			double offset = navigate(dt);
			leftSpeed = 40 - offset; 
			rightSpeed = 40 + offset; 
			drive_getTicks(&leftWheel, &rightWheel);

			ticksLeftNow = leftWheel - ticksLeftSoFar;
			ticksRightNow = rightWheel - ticksRightSoFar;
			
			
			//cumulative(ticksLeftNow, ticksRightNow);
			insertPosition(ticksLeftNow, ticksRightNow);
			
			ticksLeftSoFar = leftWheel;
			ticksRightSoFar = rightWheel;
			//check if the distance measured by the PING sensor is 10
			//if so, stop and turn around

			drive_speed(leftSpeed, rightSpeed);	
		}
		PINGdistance = ping_cm(8);
	}

	
	drive_speed(0, 0);
	//stop so that the TAs can measure the distance between the wall and the robot
	//pause(5000);
	
	//turn off sensors
	
	//dead-reckoning in number of ticks travelled and in radian
	cumulative(leftWheel, rightWheel);
	/*
	*****comment out to get distance travelled in mm and actual angle value
	cumDist = cumDist * 3.25;
	totalAngle = (totalAngle * 180.0) / M_PI;
	*/
	//printf("distance %lf angle: %lf \n", cumDist, totalAngle);

	//turn around 180 degrees
  drive_goto(-10, -10);
	drive_goto(50, -52);

	drive_getTicks(&ticksLeftSoFar, &ticksRightSoFar);

  drive_speed(previous->rightTicks * 10, previous->leftTicks * 10);
  //going back

	struct position *temp;
	//loops through the linked list to make the robot traverse back its steps
  for (temp = previous; temp->next != NULL; temp = temp->next)
 	{
    drive_getTicks(&ticksLeftNow, &ticksRightNow);

    ticksLeftSoFar = ticksLeftNow - ticksLeftSoFar;
    ticksRightSoFar = ticksRightNow - ticksRightSoFar;
   
    errorLeft = temp->next->rightTicks - ticksLeftSoFar;
    errorRight = temp->next->leftTicks - ticksRightSoFar;

    ticksLeftSoFar = ticksLeftNow;
    ticksRightSoFar = ticksRightNow;

    drive_speed((temp->rightTicks) * 10 + errorRight, (temp->leftTicks) * 10 + errorLeft);
    pause(100);
 	} 
	return 0;
}