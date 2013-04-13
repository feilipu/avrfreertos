#ifndef CHEADER_TRANSPORT
# define CHEADER_TRANSPORT

// Here is the *real* dogbot transport hardware!

// 85cm is 308 left wheel & 303 right wheel clicks.
// Therefore wheel diameter is 42.510780

#define ENCODER_TICKS         48                     // Number of clicks/revolution

#define LEFT_WHEEL_TICK      2.7597                  // Each wheel has slightly different TICK
#define RIGHT_WHEEL_TICK     2.8053                  // millimeters per TICK

#define WHEEL_DIAMETER  42.510780
#define WHEEL_BASE      83.0

#define MAXOUTPUT 255                           // maximum motor output
#define MAXSPEED 240                            // maximum number of encoder counts per second

//#define DIST_PER_TICK   (LEFT_WHEEL_TICK + RIGHT_WHEEL_TICK)/2.0 // average millimeters per tick


/* structure to manage the motor information; one each for left and right motors */
typedef struct
{
    int EncoderSetpoint;    // Expected DeltaEncoder count per cycle
    int Encoder;            // Encoder count at current iteration
    int VelocitySetpoint;   // speed in Encoder counts per cycle at which we intend to travel
    int Velocity;           // speed in Encoder counts per cycle currently
    int Acceleration;       // acceleration is added fraction of velocity, until we reach intended Setpoint
      
    int Kp;
    int Kd;
    int Ki;
    int Ko; // Output Scale
    
    int prevErr;
    int iError;
} MotorInfo, * pMotorInfo;

extern MotorInfo leftMotor, rightMotor;


/* structure to pass the transport task control parameters */
typedef struct
{
    int    X;               // distance to be travelled in cm.
    int    Y;               // distance to be travelled in cm.
    int    initialPoise;    // initial poise in Degrees. 0 = facing along X axis. 90 = facing Y axis.
    int    finalPoise;      // final poise in Degrees.
    int    Speed;           // speed relative to 100%
} xTransportVector, * pTransportVector;

/*
 * Declare a variable of type xQueueHandle.
 * This queue will have the Transport values written to it.
 */
extern xQueueHandle xTransportQueue;

/* Create a Semaphore mutex flag for EMERGENCY STOP. */
extern xSemaphoreHandle xEmergencyStopSemaphore;

/* Create a Semaphore mutex flag for ARRIVAL. */
extern xSemaphoreHandle xAtDestinationSemaphore;

extern void TaskTransport(void *pvParameters); // Transport (Motor & Odometry) Control

#endif //CHEADER_TRANSPORT

