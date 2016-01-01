#ifndef Tracker.c
#define Tracker.c

#include "Constants.h"
#include "GyroLib.c"
struct Position{
	float x;
	float y;
	float netX;
	float netY;
	float netDistance;
	tSensors leftEnc;
	tSensors rightEnc;
};

Position _RobotPos;

void trk_initEncoders(tSensors left, tSensors right){
	_RobotPos.leftEnc = left;
	_RobotPos.rightEnc = right;
	_RobotPos.netY = 150.7;
	_RobotPos.netX = 0;
}

float trk_GetRobotX(){
	return _RobotPos.x;
}

float trk_GetRobotY(){
	return _RobotPos.y;
}

void trk_setNetDistance(float netX, float netY, float currX, float currY){
	float dY = netY - currY;
	float dX = netX - currX;
	_RobotPos.netDistance = sqrt(dY * dY + dX * dX);
}

void trk_zeroNetAtGoal(){
	_RobotPos.netX = 0;
	_RobotPos.netY = 36;
}


float trk_GetNetAngle(){
	float dY = _RobotPos.netY - _RobotPos.y;
	float dX = _RobotPos.netX - _RobotPos.x;
	if(dX == 0){
		return 0;
	}
	return -1 * (((atan2(dY,dX) / PI) * 180) - 90);
}

float trk_getNetDistance(){
	float dY = _RobotPos.netY - _RobotPos.y;
	float dX = _RobotPos.netX - _RobotPos.x;
	_RobotPos.netDistance = sqrt(dY * dY + dX * dX);
	return _RobotPos.netDistance;
}

task trk_tsk_Track()
{
	long initTime = nPgmTime;
	int initTicksL = SensorValue[_RobotPos.leftEnc];
	int initTicksR = SensorValue[_RobotPos.rightEnc];
	float currentSpeedL;
	float currentSpeedR;
	float dTime;
	int dCounter = 0;
	while(true){
		dTime = (nPgmTime - initTime);
		initTime = nPgmTime;
		//find the vel of our robot

		if(dTime != 0){
			 currentSpeedL = -1 * ((initTicksL - SensorValue[_RobotPos.leftEnc]) / TICKS_PER_INCHES) / dTime;
			 currentSpeedR = ((initTicksR - SensorValue[_RobotPos.rightEnc]) / TICKS_PER_INCHES) / dTime;
			 currentSpeedL *= 1000;
			 currentSpeedR *= 1000;
		}
		float currentHeading = GyroGetAngle();
		if( dCounter > 15){
		//writeDebugStreamLine("LEFT : %f  RIGHT : %f", currentSpeedL, currentSpeedR);
		//writeDebugStreamLine("X : %f  Y : %f H:", 	_RobotPos.x , 	_RobotPos.y, currentHeading );
			dCounter = 0;
		}
		dCounter++;
		initTicksL = SensorValue[_RobotPos.leftEnc];
		initTicksR = SensorValue[_RobotPos.rightEnc];


		_RobotPos.y += ((currentSpeedL + currentSpeedR) / 2) * dTime * cosDegrees(currentHeading) / 1000;
		_RobotPos.x += ((currentSpeedL + currentSpeedR) / 2) * dTime * sinDegrees(currentHeading) / 1000;
		wait1Msec(20);
	}
}
#endif
