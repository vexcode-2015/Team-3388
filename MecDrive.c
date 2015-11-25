
#ifndef MecDrive.c
#define MecDrive.c
#define JOYSTICK_DEADZONE  10
#include "Utils.c"
#include "PIDController.h"
#include "SmartMotorLib.c"
#include "GyroLib.c"
#include "Tracker.c"
#include "Constants.h"

//constants

//ticks per centimeter = 2 pi (r = 4 * 2.54 cm) / 360*


const float LEFT_RIGHT_OFFSET = 1;


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
	bool smartDrive;
	float netDistance;
	bool pidEnabled;
} DriveBase;

DriveBase mec;

//zero's drive encoders
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
	motor[mec.fl] = power;
	motor[mec.bl] = power;
	motor[mec.ml] = power;
	//setLinMotorPow(mec.fl,power);
	//setLinMotorPow(mec.bl,power);
	//setLinMotorPow(mec.ml,power);
}

void _setRightDrivePow(int power){
	if(power == 0){
		motor[mec.fr] = 0;
		motor[mec.br] = 0;
		motor[mec.mr] = 0;
		return;
	}
	motor[mec.fr] = power ;
	motor[mec.br] = power ;
	motor[mec.mr] = power ;
	//setLinMotorPow(mec.br,power);
	//setLinMotorPow(mec.fr,power);
	//setLinMotorPow(mec.mr,power);
}

void turnDegrees(int degrees){
	zeroDriveEncoders();
	bool targetReached = false;
	float setPoint = degrees * TICKS_PER_DEGREE;
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
		//we need to add a special case in case driveOut + slaveOut is saturated
		//the important part is to keep the ratio's the same.
		if(abs(driveOut + slaveOut) > 127){
			float n = abs(driveOut + slaveOut) / 120;
			driveOut = driveOut / n;
			slaveOut = slaveOut / n;
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


void gyroTurnDegreesAbs(int degrees){
	pidInit(mec.slave, 	0.2,0,0,0,1270);
	mec.pidEnabled = false;
	degrees = degrees % 360;
	zeroDriveEncoders();
	bool targetReached = false;
	int setPoint = degrees;
	long timeInit = nPgmTime;
	long atTargetTime = nPgmTime;
	int errorThreshold = 2; //tolerance of 2 degrees
	zeroDriveEncoders();
	float initTicksL = 0;
	float initTicksR = 0;
	initTicksL = SensorValue[mec.encLeft];
	initTicksR = SensorValue[mec.encRight];
	while(!targetReached){
		//left encoder is master
		printPIDDebug(mec.gyroPID);
		float error = setPoint - GyroGetAngle();
		if(abs(error) > abs(setPoint - (GyroGetAngle() + 360))){
				error = setPoint - (GyroGetAngle() + 360);
		}
		float slaveErr = (-(SensorValue[mec.encLeft] - initTicksL)) + (SensorValue[mec.encRight] - initTicksR);
		float driveOut = pidExecute(mec.gyroPID,error)
		float slaveOut = pidExecute(mec.slave, slaveErr);
		//we need to add a special case in case driveOut is saturated
		if(abs(driveOut + slaveOut) > 127){
			//we are saturated adjust both outputs
			float n = abs(driveOut + slaveOut) / 120;
			driveOut = driveOut / n;
			slaveOut = slaveOut / n;
		}
		if(abs(driveOut) > 60){
			driveOut = 60 * (driveOut/abs(driveOut));
		}
		else if((abs(driveOut) < 24 && driveOut != 0) && abs(error) > 1){
			if(driveOut != 0){
				driveOut = 20 * (driveOut/abs(driveOut));
			}
		}
		if(abs(error) < 1){
			mec.gyroPID.errorSum = 0;
		}
		_setLeftDrivePow(driveOut + slaveOut);
		_setRightDrivePow(-(driveOut - slaveOut));
		printPIDDebug(mec.slave);
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
	mec.pidEnabled = true;
}


void gyroTurnDegreesRel(int degrees){
	gyroTurnDegreesAbs(GyroGetAngle() + degrees);
}

void enableGyro(){
	GyroInit(mec.gyro);
}




void driveInches(float inches, int maxSpeed){
	zeroDriveEncoders();
	pidInit(mec.slave, 0.6,0,0.1,0,1270);
	mec.pidEnabled = false;
	bool targetReached = false;
	float setPoint = inches * TICKS_PER_INCHES;
	long timeInit = nPgmTime;
	long atTargetTime = nPgmTime;
	float errorThreshold = 0.2 * TICKS_PER_INCHES; //tolerance of 0.2 inches

	int initDriveL = SensorValue[mec.encLeft];
	int initDriveR = SensorValue[mec.encRight];
	while(!targetReached){
		//left encoder is master
		float error = setPoint - (SensorValue[mec.encLeft] - initDriveL);
		float slaveErr = (SensorValue[mec.encLeft] - initDriveL) + (SensorValue[mec.encRight] - initDriveR);
		if(abs(mec.master.error) > 100){
				mec.master.errorSum = 0;
		}
		if(sgn(mec.master.error) != sgn(mec.master.errorSum)){
				mec.master.errorSum = 0;
		}
		float driveOut = pidExecute(mec.master,error);
		printPIDDebug(mec.master);
		float slaveOut = pidExecute(mec.slave, slaveErr);

		//we need to add a special case in case driveOut is saturated
		/**
		if(abs(driveOut + slaveOut) > 127){
			//we are saturated adjust both outputs
			float n = abs(driveOut + slaveOut) / 120;
			driveOut = driveOut / n;
			slaveOut = slaveOut / n;
		}
		**/
		if(abs(driveOut) > maxSpeed){
			driveOut = maxSpeed * (driveOut/abs(driveOut));
		}
		if(abs(driveOut) < 15 && driveOut != 0){
			driveOut = 15 * driveOut/abs(driveOut);
		}
		_setLeftDrivePow(driveOut - slaveOut);
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
	mec.pidEnabled = true;
}

void driveInches(float inches){
	driveInches(inches,127);
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

			x1 = ((x1 * x1 * x1/(abs(x1)) / (127)) / 2;
			if(abs(y2) > 80){
				x1 = oldSign > 0 ? x1 + 55 : x1 - 55;
			}
			else{
				x1 = oldSign > 0 ? x1 + 25 : x1 - 25;
			}
			}
		float leftOut = y2 + x1;
		float rightOut = y2 - x1;
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
	//	gyroTurnDegreesAbs(trk_GetNetAngle());
}

task _PIDmecDrive(){
	zeroDriveEncoders();
	mec.pidEnabled = true;
	bool running = true;
	while(running){
		while(mec.pidEnabled){
		//	if(vexRT[Btn7L] == 1){
		//		faceNet();
		//	}
		_mecDrive();
		wait1Msec(20);
		}
		wait1Msec(50);
	}
}


void engagePIDDrive(){
	startTask(_PIDmecDrive, 9);
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
	mec.smartDrive = false;
	//	writeDebugStreamLine("PRINTING %d %d %d %d", db.fl, db.fr, db.bl, db.br);
	//	startTask(_mecDrive);
	//PID, kp, ki, kd, epsilon, slew)
	//may want to tweak these for teleop
	pidInit(mec.master, 0.18,0,0.02,0,1270);//pidInit(mec.master, 	1.4111,0 ,0,0,1270);
	pidInit(mec.gyroPID, 0.7333,0,0.02,1,1270);
	engagePIDDrive();
}

void stopPIDDrive(){
	stopTask(_PIDmecDrive);
}
#endif
