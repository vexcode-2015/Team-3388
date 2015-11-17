
#ifndef MecDrive.c
#define MecDrive.c
#define JOYSTICK_DEADZONE  10
#include "Utils.c"
#include "PIDController.h"
#include "SmartMotorLib.c"

//constants

//ticks per centimeter = 2 pi (r = 4 * 2.54 cm) / 360*
const unsigned float TICKS_PER_CENTIMETERS = 0.177326562;
const unsigned float TICKS_PER_INCHES = TICKS_PER_CENTIMETERS /2.54;
const unsigned float TICKS_PER_METER = TICKS_PER_CENTIMETERS / 100;

const unsigned float NET_DISTANCE_METERS = 4.8;
//naive calculation for this one ((2 * pi * (9in * 2.54cm)) /360*) /0.177236 cm / tick
const unsigned float TICK_PER_DEGREE = 2.2499859;

const unsigned float MAX_SPEED_MS = 3;
//PID constants
typedef struct {
	tMotor fl;
	tMotor fr;
	tMotor bl;
	tMotor br;
	tMotor ml;
	tMotor mr;
	tSensor encLeft;
	tSensor encRight;
	tSensor gyro;
	PID master;
	PID slave;
	PID gyroPID;
	bool smartDrive;
	float netDistance;
} DriveBase;

DriveBase mec;


void _initSmartMotorDrive(){
	SmartMotorInit();
	SmartMotorsSetEncoderGearing(mec.fl,5);
	SmartMotorPtcMonitorEnable();
	SmartMotorLinkMotors(mec.fl, mec.bl);
	SmartMotorLinkMotors(mec.fl, mec.ml);
	SmartMotorLinkMotors(mec.fr, mec.br);
	SmartMotorLinkMotors(mec.fr, mec.mr);
	SmartMotorRun();
	mec.smartDrive = true;
}


//sets drive output to output through linear map 
void _setLeftDrivePow(int pow){
	if(mec.smartDrive){
		setSmartPow(mec.fl,pow);
		setSmartPow(mec.bl,pow);
		setSmartPow(mec.ml,pow);
	}
	setLinMotorPow(mec.fl,pow);
	setLinMotorPow(mec.bl,pow);
	setLinMotorPow(mec.ml,pow);
}

void _setRightDrivePow(){
	if(mec.smartDrive){
		setSmartPow(mec.br,pow);
		setSmartPow(mec.fr,pow);
		setSmartPow(mec.mr,pow);
	}
	setLinMotorPow(mec.br,pow);
	setLinMotorPow(mec.fr,pow);
	setLinMotorPow(mec.mr,pow);
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

void gyroTurnDegreesRel(int degrees){
	gyroTurnDegreesAbs(GyroGetAngle() + degrees);
}

void gyroTurnDegreesAbs(int degrees){
	zeroDriveEncoders();
	pidInit(mec.gyroPID,0,0,0,0,0);
	bool targetReached = false;
	int setPoint = degrees;
	long timeInit = nPgmTime;
	long atTargetTime = nPgmTime;
	int errorThreshold = 2; //tolerance of 2 degrees
	zeroDriveEncoders();
	while(!targetReached){
		//left encoder is master
		float error = setPoint - GyroGetAngle();
		float slaveErr = mec.encLeft - abs(mec.encRight);

		float driveOut = pidExecute(gyroPID,error)
		float slaveOut = pidExecute(mec.slave, slaveErr);
		//we need to add a special case in case driveOut is saturated
		if(abs(driveOut + slaveOut) > 127){
			//we are saturated adjust both outputs 
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

void enableGyro(){
	GyroInit(mec.gyro);
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
			float n = abs(driveOut + slaveOut) / 120;
			driveOut = driveOut / n;
			slaveOut = slaveOut / n;
			
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

//zero's drive encoders
void zeroDriveEncoders(){
	SensorValue[mec.encLeft] = 0;
	SensorValue[mec.encRight] = 0;
}

//prints encoder values for left and right
void printDriveEncoders(){
	writeDebugStreamLine("LEFT ENCODER : %f RIGHT ENCODER: %f",
	 SensorValue[fly.encLeft], SensorValue[fly.encRight]);
}

//prints encoder values for left and right
void printPIDDriveDebug(){
	writeDebugStreamLine("LEFT ENCODER : %f RIGHT ENCODER: %f",
		SensorValue[fly.encLeft], SensorValue[fly.encRight]);
	printPIDDebug(mec.master);
	writeDebugStreamLine("SLAVE")
	printPIDDebug(mec.slave);
}

void printGyroPIDDebug(){
	writeDebugStreamLine("LEFT ENCODER : %f RIGHT ENCODER: %f",
		SensorValue[fly.encLeft], SensorValue[fly.encRight]);
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

void faceNet(float netX, float netY, float currX, float currY){
	float dY = netY - currY;
	float dX = netX - currX;
	gyroTurnDegreesAbs(radiansToDegrees(atan(dY/dX)));
}

void setNetDistance(float netX, float netY, float currX, float currY){
	float dY = netY - currY;
	float dX = netX - currX;
	mec.netDistance = sqrt(dY * dY + dX * dX);
}

float getNetDistance(){
	return mec.netDistance;
}

task _PIDmecDrive(){
	zeroDriveEncoders();
	bool pidEnabled = true;
	int deadzone = 10;
	float maxSpeed = 50; //wild guess, 50cm/s 
	long initTime = nPgmTime;

	int initTicksL = 0;
	int initTicksR = 0;
	float x = 0;
	float y = 0;

	float netX = 0;
	float netY = * TICKS_PER_CENTIMETERS;


	pidReset(mec.slave);
	pidReset(mec.master);
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

		if(vexRT[Btn7L] == 1){
			faceNet(netX, netY, x, y);
		}
		setNetDistance(netX, netY, x, y);
		long dTime = nPgmTime - initTime;
		initTime = nPgmTime;
		//find the vel of our robot
		float currentSpeedL = ((initTicksL - SensorValue[mec.encLeft]) * TICKS_PER_CENTIMETERS) / dTime; 
		float currentSpeedR = ((initTicksR - SensorValue[mec.encRight]) * TICKS_PER_CENTIMETERS) / dTime; 
		initTicksL = SensorValue[mec.encLeft];
		initTicksR = SensorValue[mec.encRight];

		x += ((currentSpeedL + currentSpeedR) / 2) * dTime * cosDegrees(GyroGetAngle());
		y += ((currentSpeedL + currentSpeedR) / 2) * dTime * sinDegrees(GyroGetAngle());

		float speedConstant = MAX_SPEED_MS / 1.27;
		float targetSpeedL = (y2 + x1 + y1) * speedConstant;
		float targetSpeedR = (y2 - x1 - y1) * speedConstant;
		float ratio =  targetSpeedR / targetSpeedL;
		//master is left encoder
		float driveOutL = pidExecute(mec.master,targetSpeedL - currentSpeedL);
		float driveOutR = pidExecute(mec.master,targetSpeedR - currentSpeedR);
		float slaveOut = pidExecute(mec.slave, (ratio * currentSpeedL) - currentSpeedR);

		_setLeftDrivePow(motor[mec.fl] + driveOutL);
		_setRightDrivePow(motor[mec.fr] + driveOutR + slaveOut)

		delay(20);		
		//if target left and right speed are the same, we'll want to drive straight
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
	pidInit(mec.master, 	1.4111,0,0,0,1270);
	pidInit(mec.slave, 		0.1,0,0,0,1270);
	pidInit(mec.gyroPID, 	1.6666,0,0,2,1270);
	engagePIDDrive();
}

void stopPIDDrive(){
	stopTask(_PIDmecDrive);
}
#endif