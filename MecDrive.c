#ifndef MecDrive.c
#define MecDrive.c
#define JOYSTICK_DEADZONE  13
#include "Utils.c"
#include "PIDController.h"
#include "GyroLib.c"
#include "Tracker.c"
#include "Constants.h"

const float LEFT_RIGHT_OFFSET = 1;
const float AUTO_MAX_POW = 80;
typedef struct {
	tMotor fl;
	tMotor fr;
	tMotor bl;
	tMotor br;
	tMotor ml;
	tMotor mr;
	tSensors encLeft;
	tSensors encRight;
	tSensors gyro;
	PID master;
	PID slave;
	PID gyroPID;
	float netDistance;
	bool pidEnabled;
} DriveBase;
DriveBase mec;

//zero drive encoders
void zeroDriveEncoders(){
	SensorValue[mec.encLeft] = 0;
	SensorValue[mec.encRight] = 0;
}


//sets drive output to output through linear map
void _setLeftDrivePow(int power){
	if(power == 0){
		motor[mec.fl] = 0;
		motor[mec.bl] = 0;
		motor[mec.ml] = 0;
		return;
	}
//	motor[mec.fl] = power;
//	motor[mec.bl] = power;
//	motor[mec.ml] = power;
	setLinMotorPow(mec.fl,power);
	setLinMotorPow(mec.bl,power);
	setLinMotorPow(mec.ml,power);
}

void _setRightDrivePow(int power){
	if(power == 0){
		motor[mec.fr] = 0;
		motor[mec.br] = 0;
		motor[mec.mr] = 0;
		return;
	}
	//motor[mec.fr] = power;
	//motor[mec.br] = power;
	//motor[mec.mr] = power;
	setLinMotorPow(mec.br,power);
	setLinMotorPow(mec.fr,power);
	setLinMotorPow(mec.mr,power);
}



void mec_GyroTurnAbs(int degrees, bool escapable){
	pidInit(mec.gyroPID, 1.9333,1.5,0.3,0,1270);
	pidInit(mec.slave, 	0.45,0,0,0,1270);
	pidReset(mec.gyroPID);
	pidReset(mec.slave);
	mec.pidEnabled = false;
	degrees = degrees % 360;
	//this might break things
	//zeroDriveEncoders();
	bool targetReached = false;
	int setPoint = degrees;

	long timeInit = nPgmTime;
	long atTargetTime = nPgmTime;

	float errorThreshold = 0.6; //tolerance of 0.2 degrees

	float initTicksL = SensorValue[mec.encLeft];
	float initTicksR = SensorValue[mec.encRight];

	int integralLimit = 15 / mec.gyroPID.kI;
	int escapeThresh = 30;
	while(!targetReached){

		//deal with input values
		float error = setPoint - GyroGetAngle();
		writeDebugStreamLine("%f", GyroGetAngle());
		writeDebugStreamLine("init error: %f", error);
		if(abs(error) > abs( setPoint - (GyroGetAngle() + 360))){
				error = setPoint - (GyroGetAngle() + 360) ;
		}
		else if(abs(error) > abs( setPoint - (GyroGetAngle() - 360))){
				error = setpoint - (GyroGetAngle() - 360);
		}


		float slaveErr = (-(SensorValue[mec.encLeft] - initTicksL)) + (SensorValue[mec.encRight] - initTicksR);
		float slaveOut = pidExecute(mec.slave, slaveErr);
		float driveOut = pidExecute(mec.gyroPID,error);

		if(sgn(mec.gyroPID.error) != sgn(mec.gyroPID.lastError)){
			mec.gyroPID.errorSum = 0;
		}

		if(abs(mec.gyroPID.errorSum) > integralLimit){
			mec.gyroPID.errorSum = integralLimit * mec.gyroPID.errorSum/abs(mec.gyroPID.errorSum);
		}

		//we need to add a special case in case driveOut is saturated
	//	if(abs(driveOut + slaveOut) > 127){
	// 		//we are saturated adjust both outputs
	//		float n = abs(driveOut + slaveOut) / 120;
	//		driveOut = driveOut / n;
	//		slaveOut = slaveOut / n;
	//	}
		int minVal = 15;
		if(abs(error) > errorThreshold){
			atTargetTime = nPgmTime;
			driveOut += minVal * driveOut / abs(driveOut);
		}

		if(abs(driveOut) > 100){
			driveOut = 100 * driveOut / abs(driveOut);
		}
		_setLeftDrivePow(driveOut + slaveOut);
		_setRightDrivePow(-(driveOut - slaveOut));
		printPIDDebug(mec.slave);

		if(nPgmTime - atTargetTime > 250){
			targetReached = true;
			_setLeftDrivePow(0);
			_setRightDrivePow(0);
			break;
		}

		if(escapable){
			if(abs(vexRT[Ch2]) > escapeThresh || abs(vexRT[Ch3]) > escapeThresh || abs(vexRT[Ch1]) > escapeThresh
				|| abs(vexRT[Ch4]) > escapeThresh){
				targetReached = true;
			_setLeftDrivePow(0);
			_setRightDrivePow(0);
				return;
			}
		}
	}
	mec.pidEnabled = true;
}

void mec_GyroTurnAbs(float degrees){
	mec_GyroTurnAbs(degrees, false);
}



void mec_GyroTurnRel(int degrees){
	if(degrees < 0){
		degrees += 360;
	}
	if(degrees > 360){
		degrees -= 360;
	}
	mec_GyroTurnAbs(GyroGetAngle() + degrees);
}

void enableGyro(){
	//GyroInit(mec.gyro, xAccel, yAccel);
}



//returns speed of drive in feet/s
float mec_getLeftVelocity(float dTime, int initEnc){
	if(dTime != 0){
	return ((SensorValue[mec.encLeft] - initEnc) / dTime) / TICKS_PER_FEET;
}
return 0;
}

float mec_getRightVelocity(float dTime, int initEnc){
	if(dTime != 0){
	return ((SensorValue[mec.encRight] - initEnc) / dTime) / TICKS_PER_FEET;
}
return 0;
}


float util_calculatePredVel(float initSpeed, float a, float d){
	return sqrt(initSpeed * initSpeed + 2 * a * d);
}

void mec_tmpDriveInches(float setpoint, float maxAccel, float maxSpeed){
	//first generate path to the middle of the setPoint
	pidInit(mec.master,80,0,0,0,1300);
	pidInit(mec.slave,80,0,0,0,1300);
	float kV = 80;
	float kA = 80;
	float timeToMax = maxSpeed / maxAccel;
	float dToMax = 0.5 * maxAccel * timeToMax * timeToMax;

	bool atTarget = false;

	long initTime = nPgmTime;
	int leftenc_init = SensorValue[mec.encLeft];
	int masterInit = SensorValue[mec.encLeft];
	int rightenc_init = SensorValue[mec.encRight];
	float maxReachedVel = 0;
	while(!atTarget){
		//velocity in feet/s
		float pred_vel = 0;
		float pred_accl = 0;

		float curr_velocity = mec_getLeftVelocity(nPgmTime - initTime, leftenc_init);
		leftenc_init = SensorValue[mec.encLeft];
		float curr_slave_vel = mec_getRightVelocity(nPgmTime - initTime, rightenc_init);
		rightenc_init = SensorValue[mec.encRight];
		initTime = nPgmTime;
		float curr_distance = (SensorValue[mec.encLeft] - masterInit) / TICKS_PER_FEET;



		if(curr_distance < setpoint / 2){
			//if we are before halfway go full accel till max speed
			if(abs(curr_velocity) >= abs(maxSpeed)){
				pred_accl = 0;
				pred_vel = maxSpeed;
				maxReachedVel = maxSpeed;
			}
			else{
				pred_accl = maxAccel;
				pred_vel = util_calculatePredVel(0,maxAccel,curr_distance);
				maxReachedVel = curr_velocity;
			}
		} else{
			//if dToMax is greater than the setpoint than we deccel right away
			if(setpoint/2 - dToMax < 0){
				pred_vel = util_calculatePredVel(maxReachedVel,-maxAccel,curr_distance - ((setpoint/2)));
				pred_accl = -maxAccel;
			}
			else if(curr_distance > dToMax + setPoint/2){
				//time to wait is equal to
				pred_vel = util_calculatePredVel(maxReachedVel,-maxAccel,curr_distance - (dToMax + (setpoint/2)));
			}
			else{
				pred_vel = maxSpeed;
				pred_accl = 0;
			}
		}

		//next step is to calculate the velocity we should be traveling at and the acceleration we should be at

		int driveOut = pidExecute(mec.master,pred_vel - curr_velocity) + kV * pred_vel + kA * pred_accl;
		int slaveOut = pidExecute(mec.slave,curr_velocity - curr_slave_vel);
		_setLeftDrivePow(driveOut + slaveOut);
		_setRightDrivePow(driveOut - slaveOut);
			writeDebugStreamLine("%f %f %f %f", curr_distance,pred_vel,curr_velocity, pred_accl);
		wait1Msec(100);
	}
}

void mec_tmpDriveInches2(float setpoint, float maxAccel, float maxSpeed){
	float tend = maxSpeed / maxAccel;
	long initTime = nPgmTime;
	float x;
	float v;
	while(initTime + tend > nPgmTime){
		x = 0.5 * maxAccel * tend * tend;
		v = maxAccel * tend;
	}
	initTime += tend;
	while(initTime + (setpoint/maxSpeed) > nPgmTime){
		x = 0.5 * maxSpeed * maxSpeed / maxAccel + maxSpeed * ((initTime + (nPgmTime - initTime) - maxSpeed / maxAccel);
		v = maxSpeed;
	}
}

void mec_driveInches(float inches, int maxSpeed,int expiryms){
	//def constants
	pidInit(mec.master, 0.5,0.3,0.2,0,1270);
	pidInit(mec.slave, 0.3,0,0,0,1270);
	pidReset(mec.master);
	pidReset(mec.slave);

	int integralLimit = 40 / mec.master.ki;

	//turn off pid task
	mec.pidEnabled = false;
	bool targetReached = false;

	float setPoint = inches * TICKS_PER_INCHES;
	long timeInit = nPgmTime;
	long atTargetTime = nPgmTime;
	long expTime = nPgmTime + expiryms;
	float errorThreshold = 0.5 * TICKS_PER_INCHES; //tolerance of 0.2 inches

	int initDriveL = SensorValue[mec.encLeft];
	int initDriveR = SensorValue[mec.encRight];
	while(!targetReached){
		//left encoder is master
		float error = setPoint - (SensorValue[mec.encLeft] - initDriveL);
		float slaveErr = (SensorValue[mec.encLeft] - initDriveL) + (SensorValue[mec.encRight] - initDriveR);

		if(sgn(mec.master.error) != sgn(mec.master.errorSum)){
				mec.master.errorSum = 0;
		}
		if(abs(mec.master.errorSum) > integralLimit){
			mec.master.errorSum = mec.master.errorSum > 0 ?  integralLimit : -integralLimit;
		}
		float driveOut = pidExecute(mec.master,error);
		printPIDDebug(mec.master);
		float slaveOut = pidExecute(mec.slave, slaveErr);

		//we need to add a special case in case driveOut is saturated
		if(abs(driveOut) > maxSpeed){
			driveOut = maxSpeed * (driveOut/abs(driveOut));
		}
		_setLeftDrivePow(driveOut - slaveOut);
		_setRightDrivePow(driveOut + slaveOut);

		if(abs(error) > errorThreshold){
			atTargetTime = nPgmTime;
		}
		if(nPgmTime - atTargetTime > 150){
			targetReached = true;
			_setLeftDrivePow(0);
			_setRightDrivePow(0);
			break;
		}
		if(nPgmTime >= expTime){
			writeDebugStreamLine("TARGET NOT REACHED");
			break;
		}
	}
	mec.pidEnabled = true;
}

void mec_driveInches(float inches){
	mec_driveInches(inches,70,99999);
}


//prints encoder values for left and right
void printDriveEncoders(){
	writeDebugStreamLine("LEFT ENCODER : %f RIGHT ENCODER: %f",
	SensorValue[mec.encLeft], SensorValue[mec.encRight]);
}

//prints encoder values for left and right
void printPIDDriveDebug(){
	writeDebugStreamLine("LEFT ENCODER : %f RIGHT ENCODERmecf",
		SensorValue[mec.encLeft], SensorValue[mec.encRight]);
	printPIDDebug(mec.master);
	writeDebugStreamLine("SLAVE")
	printPIDDebug(mec.slave);
}

void printGyroPIDDebug(){
	writeDebugStreamLine("LEFT ENCODER : %f RIGHT ENCODER: %f",
		SensorValue[mec.encLeft], SensorValue[mec.encRight]);
	printPIDDebug(mec.gyroPID);
}

void _mecDrive(){
		int x1 = vexRT[Ch4];
		int y1 = vexRT[Ch3];
		int x2 = vexRT[Ch1];
		int y2 = vexRT[Ch2];

		x1 = threshold(x1,JOYSTICK_DEADZONE);
		y1 = threshold(y1,JOYSTICK_DEADZONE);
		x2 = threshold(x2,JOYSTICK_DEADZONE);
		y2 = threshold(y2,JOYSTICK_DEADZONE);

	//	writeDebugStreamLine("%f", x1);

		int oldSign = x1;
		if(x1!=0){


			x1 = ((x1 * x1 * x1/abs(x1)) / (127)) / 2;

			if(abs(y2) > 80){
				x1 = oldSign > 0 ? x1 + 55 : x1 - 55;
			}
			else{
				if(abs(x1) < 60){
					x1 = oldSign > 0 ? x1 + 20 : x1 - 20;
				}
				else{
					x1 = oldSign > 0 ? x1 + 60 : x1 - 60;
				}
			}

			if(abs(y1) > 60){
				y1 = y1 > 0 ? 30 : -30;
				x1 = 0;
			}
			else{
				y1 = 0;
			}
		}
		float leftOut = y2 + x1 + y1;
		float rightOut = y2 - x1 - y1;
		// if(abs(leftOut) > 127){
		// 	float n = abs(leftOut) / 120;
		// 	leftOut = leftOut / n;
		// 	rightOut = rightOut / n;
		// }
		// if(abs(rightOut) > 127){
		// 	float n = abs(rightOut) / 120;
		// 	leftOut = leftOut / n;
		// 	rightOut = rightOut / n;
		// }

	//writeDebugStreamLine("%f", x1);
	//float heading = atan(y/x);
	_setLeftDrivePow(leftOut);
	_setRightDrivePow(rightOut);
}


void faceNet(){
	mec_GyroTurnAbs(trk_GetNetAngle(), true);
}

task _PIDmecDrive(){
	zeroDriveEncoders();
	startTask(trk_tsk_Track,25);
	trk_initEncoders(mec.encLeft, mec.encRight);
	mec.pidEnabled = true;
	bool running = true;
	while(running){
		while(mec.pidEnabled){
			if(vexRT[Btn7L] == 1){
				faceNet();
				mec.pidEnabled = true;
			}
			_mecDrive();
			wait1Msec(20);
		}
		wait1Msec(50);
	}
}



void initMecDrive(DriveBase db){
	mec.fl = db.fl;
	mec.fr = db.fr;
	mec.ml = db.ml;
	mec.mr = db.mr;
	mec.bl = db.bl;
	mec.br = db.br;
	mec.encLeft = db.encLeft;
	mec.encRight = db.encRight;
	mec.master = db.master;
	mec.slave = db.slave;
	mec.gyro = db.gyro;
	//	writeDebugStreamLine("PRINTING %d %d %d %d", db.fl, db.fr, db.bl, db.br);
	//	startTask(_mecDrive);
	//PID, kp, ki, kd, epsilon, slew)
	//may want to tweak these for teleop
	//pidInit(mec.master, 	1.4111,0 ,0,0,1270);

}

void mec_StartTeleop(){
	startTask(_PIDmecDrive, 9);
}


void stopPIDDrive(){
	stopTask(_PIDmecDrive);
}
#endif
