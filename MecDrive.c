
#ifndef MecDrive.c
#define MecDrive.c
#define JOYSTICK_DEADZONE  10
#include "Utils.c"
#include "PIDController.h"

//constants
const unsigned int TICK_PER_DEGREE = 3.3;
const unsigned int TICKS_PER_INCHES = ;
const unsigned int TICKS_PER_CENTIMETERS = ;

//PID constants


typedef struct {
	tMotor fl;
	tMotor fr;
	tMotor bl;
	tMotor br;
	tSensor encLeft;
	tSensor encRight;
	PID master;
	PID slave;
} DriveBase;

DriveBase mec;


//sets drive output to output through linear map 
void _setLeftDrivePow(int pow){
	setLinMotorPow(mec.fl,pow);
	setLinMotorPow(mec.bl,pow);
}

void _setRightDrivePow(){
	setLinMotorPow(mec.br,pow);
	setLinMotorPow(mec.fr,pow);
}

void turnDegrees(int degrees){
	zeroDriveEncoders();
	bool targetReached = false;
	int setPoint = degrees * TICKS_PER_DEGREE;
	long timeInit = nPgmTime;
	long atTargetTime = nPgmTime;
	int errorThreshold = 2 * TICKS_PER_DEGREE; //tolerance of 2 degrees
	zeroDriveEncoders();
	while(!targetReached){
		//left encoder is master
		float error = setPoint - mec.encLeft;
		float slaveErr = mec.encLeft - abs(mec.encRight);

		float driveOut = pidExecute(mec.master,error)
		float slaveOut = pidExecute(mec.slave, slaveErr);
		//we need to add a special case in case driveOut is saturated
		if(abs(driveOut + slaveOut) > 127){
			//we are saturated adjust both outputs 
			float slaveRatio = driveOut / slaveOut;
		
			driveOut = 100 * (driveOut / abs(driveOut));
			slaveOut = 100 / slaveRatio * ((driveOut + slaveOut)/abs(driveOut + slaveOut));
			
		}
		_setLeftDrivePow(driveOut);
		_setRightDrivePow(-(driveOut + slaveOut));

		if(abs(error) > errorThreshold){
			atTargetTime = nPgmTime;
		}
		if(nPgmTime - atTargetTime > 350){
			targetReached = true;
			_setLeftDrivePow(0);
			_setRightDrivePow(0);
			break;
		}
	}
}

void driveInches(float inches){
	zeroDriveEncoders();
	bool targetReached = false;
	int setPoint = inches * TICKS_PER_DEGREE;
	long timeInit = nPgmTime;
	long atTargetTime = nPgmTime;
	int errorThreshold = 2 * TICKS_PER_DEGREE; //tolerance of 2 degrees
	zeroDriveEncoders();
	while(!targetReached){
		//left encoder is master
		float error = setPoint - mec.encLeft;
		float slaveErr = mec.encRight - mec.encLeft;

		float driveOut = pidExecute(mec.master,error)
		float slaveOut = pidExecute(mec.slave, slaveErr);
		//we need to add a special case in case driveOut is saturated
		if(abs(driveOut + slaveOut) > 127){
			//we are saturated adjust both outputs 
			float slaveRatio = driveOut / slaveOut;
		
			driveOut = 100 * (driveOut / abs(driveOut));
			slaveOut = 100 / slaveRatio * ((driveOut + slaveOut)/abs(driveOut + slaveOut));
			
		}
		_setLeftDrivePow(driveOut);
		_setRightDrivePow((driveOut + slaveOut));

		if(abs(error) > errorThreshold){
			atTargetTime = nPgmTime;
		}
		if(nPgmTime - atTargetTime > 350){
			targetReached = true;
			_setLeftDrivePow(0);
			_setRightDrivePow(0);
			break;
		}
	}
}

void zeroDriveEncoders(){
	SensorValue[mec.encLeft] = 0;
	SensorValue[mec.encRight] = 0;
}



void _mecDrive(){
		int x1 = vexRT[Ch4];
		int y1 = vexRT[Ch3];
		int x2 = vexRT[Ch1];
		int y2 = vexRT[Ch2];
		int deadzone = 10;
		x1 = threshold(x1,deadzone);
		y1 = threshold(y1,deadzone);
		x2 = threshold(x2,deadzone);
		y2 = threshold(y2,deadzone);
	//	writeDebugStreamLine("%f", x1);
		if(y2!=0){y2 = (y2 * y2 * (y2/abs(y2))/(127);}
		if(x1!=0){x1 = (x1 * x1 * (x1/abs(x1))/127;}
		if(abs(y1) > 15){
				y1 = y1 > 0 ? 40 : -40;
		}
	//	if(x2!=0){x2 = (x2 * (x2/abs(x2))/127;}
	//	writeDebugStreamLine("%f", x1);
	//float heading = atan(y/x);
	_setLeftDrivePow(y2 + x1 + y1);
	_setRightDrivePow(y2 - x1 - y1);
}


task _PIDmecDrive(){
	zeroDriveEncoders();
	bool pidEnabled = true;
	int deadzone = 10;
	float maxSpeed = 50; //wild guess, 50cm/s 
	long initTime = nPgmTime;

	int initTicksL = 0;
	int initTicksR = 0;
	while(pidEnabled){

		//eed to calculate currentVel 
		int x1 = vexRT[Ch4];
		int y1 = vexRT[Ch3];
		int x2 = vexRT[Ch1];
		int y2 = vexRT[Ch2];
		x1 = threshold(x1,deadzone);
		y1 = threshold(y1,deadzone);
		x2 = threshold(x2,deadzone);
		y2 = threshold(y2,deadzone);

		long dTime = nPgmTime - initTime;
		initTime = nPgmTime;
		//find the vel of our robot
		float currentSpeedL = ((initTicksL - SensorValue[mec.encLeft]) * TICKS_PER_CENTIMETERS) / dTime; 
		float currentSpeedR = ((initTicksR - SensorValue[mec.encRight]) * TICKS_PER_CENTIMETERS) / dTime; 
		initTicksL = SensorValue[mec.encLeft];
		initTicksR = SensorValue[mec.encRight];

		float speedConstant = maxSpeed / 127;
		float targetSpeedL = (y2 + x1 + y1) * speedConstant;
		float targetSpeedR = (y2 - x1 - y1) * speedConstant;
		float ratio =  targetSpeedR / targetSpeedL;
		//master is left encoder
		float driveOutL = pidExecute(mec.master,targetSpeedL - currentSpeedL);
		float driveOutR = pidExecute(mec.master,targetSpeedR - currentSpeedR);
		float slaveOut = pidExecute(mec.slave, (ratio * currentSpeedL) - currentSpeedR);

		_setLeftDrivePow(driveOutL);
		_setRightDrivePow(driveOutR + slaveOut)

		delay(20);		
		//if target left and right speed are the same, we'll want to drive straight
	}
}


void engagePIDDrive(){
	startTask(_PIDmecDrive);
}

void initMecDrive(DriveBase db){
	mec.fl = db.fl;
	mec.fr = db.fr;
	mec.bl = db.bl;
	mec.br = db.br;
	mec.encLeft = db.encLeft;
	mec.encRight = db.encRight;
	mec.master = mec.master;
	mec.slave = mec.slave;
	//	writeDebugStreamLine("PRINTING %d %d %d %d", db.fl, db.fr, db.bl, db.br);
	//	startTask(_mecDrive);
	//PID, kp, ki, kd, epsilon, slew)
	pidInit(mec.master, 0,0,0,20,1270);
	pidInit(mec.slave, 0,0,0,20,1270);
}

void stopMecDrive(){
	stopTask(_PIDmecDrive);
}
#endif