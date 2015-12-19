#ifndef Tracker.c
#define Tracker.c

#include "Constants.h"

 struct Position{
	float x;
	float y;
	float netDistance;
	tSensors leftEnc;
	tSensors rightEnc;
};

Position _RobotPos;
float GetRobotX(){

}

float GetRobotY(){

}

float netX = 0;
float netY = 434.45 * TICKS_PER_CENTIMETERS;

void setNetDistance(float netX, float netY, float currX, float currY){
	float dY = netY - currY;
	float dX = netX - currX;
	_RobotPos.netDistance = sqrt(dY * dY + dX * dX);
}


float trk_GetNetAngle(){
	float dY = netY - _RobotPos.y;
	float dX = netX - _RobotPos.x;
	return radiansToDegrees(atan(dY/dX));
}


float getNetDistance(){
	return _RobotPos.netDistance;
}

task Track()
{
	bool pidEnabled = true;
	int deadzone = 10;
	long initTime = nPgmTime;
	int initTicksL = 0;
	int initTicksR = 0;
	float currentSpeedL;
	float currentSpeedR;
	float dTime;
	while(true){
		dTime = nPgmTime - initTime;
		initTime = nPgmTime;
		//find the vel of our robot

		if(dTime != 0){
			 currentSpeedL = ((initTicksL - SensorValue[_RobotPos.leftEnc]) * TICKS_PER_CENTIMETERS) / dTime;
			 currentSpeedR = ((initTicksR - SensorValue[_RobotPos.rightEnc]) * TICKS_PER_CENTIMETERS) / dTime;
		}
		initTicksL = SensorValue[_RobotPos.leftEnc];
		initTicksR = SensorValue[_RobotPos.rightEnc];

		_RobotPos.x += ((currentSpeedL + currentSpeedR) / 2) * dTime * cosDegrees(GyroGetAngle());
		_RobotPos.y += ((currentSpeedL + currentSpeedR) / 2) * dTime * sinDegrees(GyroGetAngle());
		wait1Msec(20);
	}
}
#endif
