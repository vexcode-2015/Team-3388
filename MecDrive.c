#ifndef MecDrive.c
#define MecDrive.c
#define JOYSTICK_DEADZONE  5
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

float _getLeftEnc(){
	return -SensorValue[mec.encLeft];
}

float _getRightEnc(){
	return -SensorValue[mec.encRight];
}


float GYRO_KP = 2.4;//2;
float GYRO_KI = 2;//3;
float GYRO_KD = 0.25; //0.34
float GYRO_INTLIM = 1270;
float GYRO_ERROR_THRESH = 0.7;


void mec_GyroTurnAbs(int degrees, bool escapable){
	pidInit(mec.gyroPID, GYRO_KP,GYRO_KI,GYRO_KD,0,GYRO_INTLIM);
	pidInit(mec.slave, 	0.3,0,0,0,1270);
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

	float errorThreshold = 0.5; //tolerance of 0.2 degrees

	float initTicksL = _getLeftEnc();
	float initTicksR = _getRightEnc();

	int integralLimit = 15 / mec.gyroPID.kI;
	int escapeThresh = 30;
	while(!targetReached){

		//deal with input values
		float error = setPoint - GyroGetAngle();
		if(abs(error) > abs( setPoint - (GyroGetAngle() + 360))){
				error = setPoint - (GyroGetAngle() + 360) ;
		}
		else if(abs(error) > abs( setPoint - (GyroGetAngle() - 360))){
				error = setpoint - (GyroGetAngle() - 360);
		}


		float slaveErr = (-(_getLeftEnc() - initTicksL)) - (_getRightEnc() - initTicksR);
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
		int minVal = 13;
		if(abs(error) > errorThreshold){
			atTargetTime = nPgmTime;
			if(abs(driveOut) < minVal){
				if(driveOut != 0){
					driveOut += minVal * driveOut / abs(driveOut);
				}
			}
		}




		if(abs(driveOut) > 100){
			driveOut = 100 * driveOut / abs(driveOut);
		}
		_setLeftDrivePow((driveOut + slaveOut));
		_setRightDrivePow(-(driveOut - slaveOut));



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




//returns speed of drive in feet/s
float mec_getLeftVelocity(float dTime, int initEnc){
	if(dTime != 0){
	return ((_getLeftEnc() - initEnc) / dTime) / TICKS_PER_FEET;
}
return 0;
}

float mec_getRightVelocity(float dTime, int initEnc){
	if(dTime != 0){
	return ((_getRightEnc() - initEnc) / dTime) / TICKS_PER_FEET;
}
return 0;
}


float util_calculatePredVel(float initSpeed, float a, float d){
	return sqrt(initSpeed * initSpeed + 2 * a * d);
}



const int DRIVE_SAT_TIME = 200;

float _DRIVE_KP = 0.3;
float _DRIVE_KI = 0.2;
float _DRIVE_KD = 0.16;

float _SLAVE_KP = 0.3;

void mec_driveInches(float inches, int maxSpeed,int expiryms){
	//def constants
	pidInit(mec.master, _DRIVE_KP,_DRIVE_KI,_DRIVE_KD,0,1270);
	pidInit(mec.slave, _SLAVE_KP,0,0,0,1270);
	pidReset(mec.master);
	pidReset(mec.slave);

	int integralLimit = 25 / mec.master.ki;

	//turn off pid task
	mec.pidEnabled = false;
	bool targetReached = false;

	float setPoint = inches * TICKS_PER_INCHES;
	long timeInit = nPgmTime;
	long atTargetTime = nPgmTime;
	long expTime = nPgmTime + expiryms;
	float errorThreshold = 0.7 * TICKS_PER_INCHES; //tolerance of 0.2 inches

	int initDriveL = _getLeftEnc();
	int initDriveR = _getRightEnc();
	while(!targetReached){
		//left encoder is master
		float error = setPoint - ( _getLeftEnc() - initDriveL);
		float slaveErr = (_getLeftEnc() - initDriveL) - (_getRightEnc()- initDriveR);

		if(sgn(mec.master.error) != sgn(mec.master.errorSum)){
				mec.master.errorSum = 0;
		}
		if(abs(mec.master.errorSum) > integralLimit){
			mec.master.errorSum = mec.master.errorSum > 0 ?  integralLimit : -integralLimit;
		}
		float driveOut = pidExecute(mec.master,error);

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
		if(nPgmTime - atTargetTime > DRIVE_SAT_TIME){
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
	mec_driveInches(inches,110,99999);
}

void mec_driveInchesTwoStage(float distance, float slowDist, int maxSpeed, int secondMax, int expiryms){
	pidInit(mec.master, _DRIVE_KP,_DRIVE_KI,_DRIVE_KD,0,1270);
	pidInit(mec.slave, _SLAVE_KP,0,0,0,1270);
	pidReset(mec.master);
	pidReset(mec.slave);

	int integralLimit = 40 / mec.master.ki;

	//turn off pid task
	mec.pidEnabled = false;
	bool targetReached = false;

	float setPoint = distance * TICKS_PER_INCHES;
	long timeInit = nPgmTime;
	long atTargetTime = nPgmTime;
	long expTime = nPgmTime + expiryms;
	float errorThreshold = 0.2 * TICKS_PER_INCHES; //tolerance of 0.2 inches

	int initDriveL = _getLeftEnc();
	int initDriveR = _getRightEnc();
	while(!targetReached){
		//left encoder is master
		float error = setPoint - ( _getLeftEnc() - initDriveL);
		float slaveErr = (_getLeftEnc() - initDriveL) - (_getRightEnc()- initDriveR);

		if(sgn(mec.master.error) != sgn(mec.master.errorSum)){
				mec.master.errorSum = 0;
		}
		if(abs(mec.master.errorSum) > integralLimit){
			mec.master.errorSum = mec.master.errorSum > 0 ?  integralLimit : -integralLimit;
		}
		float driveOut = pidExecute(mec.master,error);

		float slaveOut = pidExecute(mec.slave, slaveErr);

		//we need to add a special case in case driveOut is saturated


		if((distance - slowDist) * TICKS_PER_INCHES > (_getLeftEnc() - initDriveL)){

			if(abs(driveOut) > secondMax){
				driveOut = secondMax * driveOut / abs(driveOut);
			}
		}

		if(abs(driveOut) > maxSpeed){

			driveOut = maxSpeed * (driveOut/abs(driveOut));
		}
		_setLeftDrivePow(driveOut - slaveOut);
		_setRightDrivePow(driveOut + slaveOut);

		if(abs(error) > errorThreshold){
			atTargetTime = nPgmTime;
		}
		if(nPgmTime - atTargetTime > DRIVE_SAT_TIME){
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
		int y1 = 0;//vexRT[Ch3];
		int x2 = vexRT[Ch1];
		int y2 = vexRT[Ch2];

		x1 = threshold(x1,JOYSTICK_DEADZONE);
		y1 = threshold(y1,JOYSTICK_DEADZONE);
		x2 = threshold(x2,JOYSTICK_DEADZONE);
		y2 = threshold(y2,JOYSTICK_DEADZONE);

	//	writeDebugStreamLine("%f", x1);

		int oldSign = x1;
		if(x1!=0){


			x1 = (x1 * x1 * x1 ) / (127 * 127);//((x1 * x1 * x1 / abs(x1) ) / (127));

			if(abs(y1) > 40){
				x1 = 20 * y1 / abs(y1);
				y1 = 0;
			}
			else{
				y1 = 0;
			}
		/**
			if(abs(y2) > 80){
				x1 = oldSign > 0 ? x1 + 55 : x1 - 55;
			}
			else{
				if(abs(x1) < 70){
					x1 = oldSign > 0 ? x1 + 13 : x1 - 13;
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
			} **/
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
	//startTask(trk_tsk_Track,25);
	trk_initEncoders(mec.encLeft, mec.encRight);
	mec.pidEnabled = true;
	bool running = true;
	while(running){
		while(mec.pidEnabled){
			if(vexRT[Btn7L] == 1){
			//	faceNet();
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


void mec_StopTeleop(){
	stopTask(_PIDmecDrive);
}
#endif
