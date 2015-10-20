
#ifndef FlyControl.c 
#define FlyControl.c

// Update inteval (in mS) for the flywheel control loop
#define FW_LOOP_SPEED              25

// Maximum power we want to send to the flywheel motors
#define FW_MAX_POWER              127

// encoder counts per revolution depending on motor
#define MOTOR_TPR_269           240.448
#define MOTOR_TPR_393R          261.333
#define MOTOR_TPR_393S          392
#define MOTOR_TPR_393T          627.2
#define MOTOR_TPR_QUAD          360.0

// Structure to gather all the flywheel ralated data
typedef struct _fw_controller {
    long            counter;                ///< loop counter used for debug

    // encoder tick per revolution
    float           ticks_per_rev;          ///< encoder ticks per revolution

    // Encoder
    long            e_current;              ///< current encoder count
    long            e_last;                 ///< current encoder count

    // velocity measurement
    float           v_current;              ///< current velocity in rpm
    long            v_time;                 ///< Time of last velocity calculation

    // TBH control algorithm variables
    long            target;                 ///< target velocity
    long            current;                ///< current velocity
    long            last;                   ///< last velocity
    float           error;                  ///< error between actual and target velocities
    float           last_error;             ///< error last time update called
    float           gain;                   ///< gain
    float           drive;                  ///< final drive out of TBH (0.0 to 1.0)
    float           drive_at_zero;          ///< drive at last zero crossing
    long            first_cross;            ///< flag indicating first zero crossing
    float           drive_approx;           ///< estimated open loop drive 

    // final motor drive
    long            motor_drive;            ///< final motor control value
    } fw_controller;

typedef struct fw_motors{
	tMotor[4]* m;
} fw_motors;

fw_motors = _FlyWheel;
void initFlyWheel(fw_motors* initMotors){
	fw_motors = *initMotors;
}

void _updateFlyWheel(int power){
	fw_motors.m[0]* = power;
	fw_motors.m[1]* = power;
	fw_motors.m[2]* = power;
	fw_motors.m[3]* = power;
}

task FlyWheelControl(){
	while(true){
		if(vexRT[Btn5U]){
			_updateFlyWheel(int power)
		}
		delay(FW_LOOP_SPEED);
	}
}


#endif