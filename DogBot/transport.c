/*************************************************************************
* Title:    Code for PDI control of motors using Pololu Libraries
* Author:   Phillip Stevens
* Target:   DogBot

* Source:   Heavily drawn from: http://www.barello.net/Papers/Motion_Control/index.htm
* And Source: http://geology.heroy.smu.edu/~dpa-www/robo/Encoder/imu_odo/index_IE.htm#sec4
**************************************************************************/

#include <stdlib.h>
#include <math.h>
#include <inttypes.h>

#include <avr/io.h> 
#include <avr/pgmspace.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Pololu include files. */
#include <pololu/orangutan.h>

/* Transport include file. */
#include "transport.h"

/* Dogbot include file.  This is temporary; just needed for the LCD header at the moment. */
#include "dogbot.h"

/*
    Globals/Statics etc.
*/

MotorInfo leftMotor, rightMotor;    // Global information on the state of each drive motor.


// Initialized data (PID tuning parameters)

const int Kp 			PROGMEM = 256;      // Proportional gain
const int Kd 			PROGMEM =  16;      // Derivative gain
const int Ki 			PROGMEM =   1;      // Integral gain
const int Ko 			PROGMEM =  16;      // Output factor
const int Acceleration  PROGMEM =   8;      // Acceleration (cycles to reach full velocity)

void MotorInit(void);           // Initialize Motion Control data
int  DoPID(MotorInfo *p);       // Do the PID Calculation based for the Motor pointed to by p. 
void DoMotion(MotorInfo *p);    // Called at the loop rate to add "velocity" to the setpoint effecting motion for the Motor pointed to by p.

void TaskTransport(void *pvParameters); // Transport (Motor & Odometry) Control


/*---------------------------------------------------------*/
// Initialize Motion Control data

void MotorInit(void)
{
    // Initialize Motion Control data
 
    leftMotor.Kp = rightMotor.Kp =                      pgm_read_word(&Kp);
    leftMotor.Kd = rightMotor.Kd =                      pgm_read_word(&Kd);
    leftMotor.Ki = rightMotor.Ki =                      pgm_read_word(&Ki);
    leftMotor.Ko = rightMotor.Ko =                      pgm_read_word(&Ko);
    leftMotor.Acceleration = rightMotor.Acceleration =  pgm_read_word(&Acceleration); 
    
}

/*---------------------------------------------------------*/
// Do the PID Calculation based for the Motor pointed to by p.

int DoPID(MotorInfo *p)
{
    int error;
    int output;

    error = p->EncoderSetpoint - p->Encoder;

    // Derivative error is the delta Perror

    output = (p->Kp*error + p->Kd*(error - p->prevErr) + p->Ki*p->iError)/p->Ko;
    p->prevErr = error;

    // Accumulate Integral error *or* Limit output.
    // Stop accumulating when output saturates
    
    if (output >= MAXOUTPUT)
        output = MAXOUTPUT;
    else if (output <= -MAXOUTPUT)
        output = -MAXOUTPUT;
    else
        p->iError += error;
        
    return ( output);
}

/*---------------------------------------------------------
 void DoMotion(MotorInfo *p)

 Called at the loop rate to add "Velocity" to the Encoder Setpoint thus
 effecting a motion.

 Velocity is ramped up and down by "Acceleration" proportional to VelocitySetpoint
 Velocity will take Acceleration cycles to reach VelocitySetpoint.

-*/

void DoMotion(MotorInfo *p)
{
    if (p->Velocity < p->VelocitySetpoint) // velocity calculated in Encoder TICKS per period
    {
        p->Velocity += p->VelocitySetpoint / p->Acceleration; // Acceleration is divided. 8 implies 8 cycles to full velocity.
        if (p->Velocity > p->VelocitySetpoint)
            p->Velocity = p->VelocitySetpoint;
    }
    else 
    {
        p->Velocity -= p->Acceleration;
        if (p->Velocity < p->VelocitySetpoint)
            p->Velocity = p->VelocitySetpoint;
    }
    p->EncoderSetpoint = p->Velocity;
}



/*---------- The Transport Task -------------------*/

void TaskTransport(void *pvParameters) // Manage Transport (Motors & Odometry)
{
    (void) pvParameters;;

    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialized with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();
	
	xTransportVector xRequestedVector; // create a structure to receive the path request.
	
	int leftEncoder;        // left odometry measure (TICKS)
	int rightEncoder;       // right odometry measure (TICKS)
	
    float deltaDist;        // distance calculated since last cycle in mm
    float localTheta;       // accumulated local theta (radians)  

    float remainingX;      // our distance from destination cartesian position in x direction in mm
    float remainingY;      // our distance from destination cartesian position in y direction in mm
    
    float finalTheta;       // required poise (radians) once we are completed
    
    float targetDistance;	// distance to the target in mm
    float targetBearing;     // just which direction we should be headed (radians)
    float headingError;      // the difference between targetBearing and localTheta (radians)
    float prevHeadingError;	 // the heading error on the last cycle
    
    const float   KpH = 1.5;		   // proportional value for heading    
    const float   KdH = 0.1;           // derivative value for heading

        
    int     speed;             // what should be our speed relative to the MAXOUTPUT
    float   speedProportion;   // proportional speed to manage direction
    
    
    
    MotorInit();
    
   
	
    while(1)
    {

        if( xTransportQueue != 0 )
        {
        /* Block on the Transport queue to wait for data to arrive. */
            if( xQueueReceive( xTransportQueue, &xRequestedVector, portMAX_DELAY) )
            {        
                /*
                I need to know the X & Y coordinates, speed and the preferred path.
                This means that I don't have to maintain a global X & Y coordinate.
                The Navigation task will look after that process. I just need a local X & Y coordinate.
                xRequestedVector
                int    X;       // distance to be travelled in cm.
                int    Y;       // distance to be travelled in cm.
                int    Poise:   // initial poise in absolute degrees.
                char   Speed;   // speed relative to 100%



                Here the robot is heading in the X direction when starting, unless another poise is given.
                Turning left adds Y direction, turning right subtracts Y direction.
                
                The basic proposition is quite simple.  For a two wheel differential drive 
                robot the total distance the robot has traveled is:

                        distance = (right_encoder + left_encoder) / 2.0;

                and the heading of the robot, it's rotation about it's center in radians, is

                        theta = (right_encoder - left_encoder) / WHEEL_BASE;

                where WHEEL_BASE is the distance between the two wheels.

                With these two quantities, a bit of trig can give you the robot's position in
                X and Y, as follows:

                    X_position = distance * cos(theta);
                    Y_position = distance * sin(theta);

                You can also multiply theta * (180.0/M_PI) to get the heading in degrees.
                    
                */
                
            	remainingX = (float)-xRequestedVector.X * 10.0;
            	remainingY = (float)-xRequestedVector.Y * 10.0;
            	localTheta = (float)xRequestedVector.initialPoise * M_PI/180.0; // establish the provided initial Poise in Radians
            	finalTheta = (float)xRequestedVector.finalPoise * M_PI/180.0; // establish the provided final Poise in Radians
            	speed      = xRequestedVector.Speed * MAXSPEED / 1000 *2;        // It is in encoder TICKS per period.
            			// Here for 100ms periods (Max 24 TICKS)
            			// (with display cruft goes to 200ms or 48 TICKS so multiply by 2)
                
                while( 1) //(fabs((double)remainingX) > 10.0) || (fabs((double)remainingY) > 10.0) ) //|| (fabs(finalTheta - localTheta) > M_PI/16) ) // we have not yet reached our target
                {

                    rightEncoder = svp_get_counts_and_reset_ab(); // get the encoder counts from the SVP aux processor, and reset count to 0         
                    leftEncoder = svp_get_counts_and_reset_cd(); 
                    
                    rightMotor.Encoder  = rightEncoder; // drop the encoder counts into the motor for PID control
                    leftMotor.Encoder   = leftEncoder;

                    // Do the odometery calculations
                    deltaDist    = (((float)rightEncoder * RIGHT_WHEEL_TICK + (float)leftEncoder * LEFT_WHEEL_TICK)/2.0); // in millimetres
                    localTheta  += (((float)rightEncoder * RIGHT_WHEEL_TICK - (float)leftEncoder * LEFT_WHEEL_TICK)/WHEEL_BASE); // in radians

                    if (localTheta > 2.0*M_PI)
                        localTheta -= 2.0*M_PI;       // Keep theta between 0<->2PI radians
                    else if (localTheta < 0.0)
                        localTheta += 2.0*M_PI;

                    remainingX += deltaDist * (float)cos((double)localTheta); // add the delta distance in mm travelled
                    remainingY += deltaDist * (float)sin((double)localTheta); // to our current location.
                    
                    targetDistance = sqrt((remainingX*remainingX)+(remainingY*remainingY)); // this is the remaining distance to the target
    
                	/* calculate the targetBearing, and make sure were not doing divide-by-zero! */
                	if (remainingX > 0.0)
                	    targetBearing =  - (float)atan((double)(remainingY/remainingX));
                	else if (remainingX < 0.0)
                	    targetBearing =    (float)atan((double)(remainingY/remainingX));
                	    
                    /*  (-ve) is too port, (+ve) is too starboard */
                	headingError = targetBearing - localTheta;
                	
                	if (headingError > M_PI)
                	    headingError -= 2.0*M_PI;
                    else if (headingError < -M_PI)
                        headingError += 2.0*M_PI;
                        
              
                    // here the headingError influences the speed (p->VelocitySetpoint) of each motor.
                    // also the xRequestedVector.Speed (as a proportioning of nominal maximum speed) is respected.
                        
                    if (targetDistance >= 50.0) // some distance from the target
                    {
                        if (headingError > M_PI/2.0)
                        {
                                rightMotor.VelocitySetpoint =  speed /2;
                                leftMotor.VelocitySetpoint  = -speed /2;
                        }
                        else if (headingError < -M_PI/2.0)
                        {
                                rightMotor.VelocitySetpoint = -speed /2;
                                leftMotor.VelocitySetpoint  =  speed /2;		                            
                        }
                        else
                        {
                            // Derivative error is the delta Perror
							speedProportion = KpH*headingError + KdH*(headingError - prevHeadingError);
							prevHeadingError = headingError;

							// Limit output
							// Stop accumulating when output saturates
							if (speedProportion >= 1)
								speedProportion = 1;
							else if (speedProportion <= -1)
								speedProportion = -1;
								
                            rightMotor.VelocitySetpoint =  speed * (1 + speedProportion); // here we want to go forward
                            leftMotor.VelocitySetpoint  =  speed * (1 - speedProportion); // at about the right speed			
                        
                        }
                    }
                    else if ( (targetDistance < 50.0) && (fabs((double)(finalTheta - localTheta)) > M_PI/8) ) // on target but wrong heading
                    {
                        if (headingError > M_PI/2.0)
                        {
                                rightMotor.VelocitySetpoint =  speed /4;
                                leftMotor.VelocitySetpoint  = -speed /4;
                        }
                        else if (headingError < -M_PI/2.0)
                        {
                                rightMotor.VelocitySetpoint = -speed /4;
                                leftMotor.VelocitySetpoint  =  speed /4;		                            
                        }
                        else
                        {
                            // Derivative error is the delta Perror
							speedProportion = KpH*headingError + KdH*(headingError - prevHeadingError);
							prevHeadingError = headingError;

							// Limit output
							// Stop accumulating when output saturates
							if (speedProportion >= 1)
								speedProportion = 1;
							else if (speedProportion <= -1)
								speedProportion = -1;
								
                            rightMotor.VelocitySetpoint =  speed * (+ speedProportion); // here we rotate about a point
                            leftMotor.VelocitySetpoint  =  speed * (- speedProportion);	// with 0 forward speed			
                        
                        }
                    } 
                    else if ( (targetDistance < 50.0)  && (fabs((double)(finalTheta - localTheta)) <= M_PI/8) ) // on target and final heading
                    {
                        rightMotor.VelocitySetpoint = 0;
                        leftMotor.VelocitySetpoint  = 0;		                    
                    }
 
  // here we set the speed for the if he heading stuff is not in use above.

    //                rightMotor.VelocitySetpoint = speed;
     //               leftMotor.VelocitySetpoint  = speed;  
  
                    
                    DoMotion(&rightMotor);
                    DoMotion(&leftMotor);
                    
                    set_motors( DoPID(&rightMotor), DoPID(&leftMotor) ); // PID Calculations and Set motor speed.


    /*------------This is testing code cruft:---------------------------- */ 

                    if( xLCDSemaphore != NULL )
                    {
                        // See if we can obtain the semaphore.  If the semaphore is not available
                        // wait 10 ticks to see if it becomes free.	
                        if( xSemaphoreTake( xLCDSemaphore, ( portTickType ) 10 ) == pdTRUE )
                        {
                            // We were able to obtain the semaphore and can now access the
                            // shared resource.
                            
                            set_digital_output( IO_D1, 1);               // red LED on to show process  
                            vTaskDelay( 50 / portTICK_RATE_MS ); 
                            set_digital_output( IO_D1, 0);               // red LED off to show process
                            vTaskDelay( 50 / portTICK_RATE_MS );

                //            lcd_goto_xy(2, 0);              // go to the first character of the first LCD line
                //           print_long((long)requiredX);
                //            lcd_goto_xy(10, 0);              // go to the ninth character of the first LCD line
                //            print_long((long)requiredY);     

                            lcd_goto_xy(0, 0);              // Go to start of first line
                            print("                ");      // 16 spaces to clear the screen
                            lcd_goto_xy(0, 0);              // go to the first character of the first LCD line
                            print_long( (long) (remainingX / 10.0) ); print("x"); // x in cm ( leftMotor.EncoderSetpoint)
                            lcd_goto_xy(6, 0);              // go to the seventh character of the first LCD line
                            print_long( (long) (remainingY / 10.0) ); print("y"); // y in cm ( rightMotor.EncoderSetpoint)
                            lcd_goto_xy(13, 0);              // go to the forteenth character of the first LCD line
                            print_long( (long) (headingError * (180.0/M_PI)) );   // theta in degrees
                            
                            xSemaphoreGive( xLCDSemaphore );   
                        }
                                  
                    }

  /*-------------------- end test cruft --------------------------------*/


                    vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_RATE_MS ) );
                                
                }                
            }      
        }
        xSemaphoreGive( xAtDestinationSemaphore );
        taskYIELD();  
    }
}



