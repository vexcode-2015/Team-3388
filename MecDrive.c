#ifndef MecDrive.c
#define MecDrive.c
#define JOYSTICK_DEADZONE  10
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
	bool smartDrive;
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
	motor[mec.fr] = power;
	motor[mec.br] = power;
	motor[mec.mr] = power;
	//setLinMotorPow(mec.br,power);
	//setLinMotorPow(mec.fr,power);
	//setLinMotorPow(mec.mr,power);
}

void mec_GyroTurnAbs(int degrees){
	GyroResetAngle();
	pidInit(mec.gyroPID, 1,0.1,0.2,1,1270);
	pidInit(mec.slave, 	0.2,0,0,0,1270);
	mec.pidEnabled = false;
	degrees = degrees % 360;
	//this might break things
	//zeroDriveEncoders();
	bool targetReached = false;
	int setPoint = degrees;

	long timeInit = nPgmTime;
	long atTargetTime = nPgmTime;

	int errorThreshold = 2; //tolerance of 2 degrees

	float initTicksL = SensorValue[mec.encLeft];
	float initTicksR = SensorValue[mec.encRight];

	int integralLimit;
	if(mec.gyroPID.kI != 0){
		integralLimit = 40 / mec.gyroPID.kI;
	}

	while(!targetReached){
		//left encoder is master

		float error = setPoint - GyroGetAngle();
		if(abs(error) > abs((setPoint+360) - (GyroGetAngle()))){
				error = (setPoint+360) - (GyroGetAngle() );
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


void mec_GyroTurnRel(int degrees){
	if(degrees < 0){
		degrees += 360;
	}
	if(degrees > 360){
		degrees -= 360;
	}
	mec_GyroTurnAbs(degrees);
}

void enableGyro(){
	GyroInit(mec.gyro);
}




void driveInches(float inches, int maxSpeed){
	//def constants
	pidInit(mec.master, 0.4,0,0,0,1270);
	pidInit(mec.slave, 0.05,0,0,0,1270);
	pidReset(mec.master);
	pidReset(mec.slave);

	int integralLimit = 0;
	if( mec.master.ki != 0){
	 integralLimit = 30 / mec.master.ki;
	}
	//turn off pid task
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
	driveInches(inches,70);
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
				x1 = oldSign > 0 ? x1 + 25 : x1 - 25;
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
	//pidInit(mec.master, 	1.4111,0 ,0,0,1270);

}

void mec_StartTeleop(){
	startTask(_PIDmecDrive, 9);
}


void stopPIDDrive(){
	stopTask(_PIDmecDrive);
}
#endif
