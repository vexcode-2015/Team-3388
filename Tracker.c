#ifndef Tracker.c
#define Tracker.c

#include "Constants.h"

 struct Position{
	float x;
	float y;
	float netDistance;
	tSensors leftEnc;
	tSensors rightEnc;
} ;

Position _RobotPos;
float GetRobotX(){

}

float GetRobotY(){

}

void setNetDistance(float netX, float netY, float currX, float currY){
	float dY = netY - currY;
	float dX = netX - currX;
	_RobotPos.netDistance = sqrt(dY * dY + dX * dX);
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
	float x = 0;
	float y = 0;

	float netX = 0;
	float netY = 400 * TICKS_PER_CENTIMETERS;
	float speedConstant = MAX_SPEED_MS / 1.80;

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

		//x += ((currentSpeedL + currentSpeedR) / 2) * dTime * cosDegrees(GyroGetAngle());
		//y += ((currentSpeedL + currentSpeedR) / 2) * dTime * sinDegrees(GyroGetAngle());
	}
}
#endif
