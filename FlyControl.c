
#ifndef FlyControl.c
#define FlyControl.c
#define FW_LOOP_SPEED	20


#include "Utils.c"
#include "PIDController.h"
#include "MecDrive.c"

int SKILLS_SHORT_RPM = 2130;
int SKILLS_SHORT_POW = 45;
int SKILLS_RPM = 1950; //1950
int SKILLS_POW = 50;
int SHORT_RPM = 1800;//1690;
int SHORT_POW = 30;
int MED_RPM = 2100;
int MED_POW = 45;
int LONG_RPM  = 2560;//2480;//2680;//2950;
int HIGH_POW = 59;//75;


typedef struct {
	tMotor f1;
	tMotor f2;
	tSensors enc;
	tSensors sonar;
	PID flyPID;
	int setPoint;
	float currSpeed;
	int pred;
} Fw_Controller;
Fw_Controller _fly;


void _updateFlyWheelLin(int power){
	if(power > 127){
		power = 127;
	}
	setLinMotorPow(_fly.f1, power);
	setLinMotorPow(_fly.f2, power);
}

void _updateFlyWheel(int power){
	_updateFlyWheelLin(power);
}

long	nSysTime_last;
long	encoder_counts;
long	encoder_counts_last;

float FwCalculateSpeed()
{
	int			delta_ms;
	int			delta_enc;
	encoder_counts = -SensorValue[_fly.enc] ;
	delta_ms = nSysTime - nSysTime_last;
	nSysTime_last = nSysTime;
	delta_enc = (encoder_counts - encoder_counts_last);
	encoder_counts_last = encoder_counts;
	float newSpeed = (1000.0 / delta_ms) * delta_enc;
	_fly.currSpeed = (_fly.currSpeed * 0.6) + newSpeed * 0.4;
	return _fly.currSpeed;
}

void setFlyRpm(int rpm){
	_fly.setPoint = rpm;
	pidReset(_fly.flyPID);
}

void setFlyRpm(int rpm, int pred){
	setFlyRpm(rpm);
	_fly.pred = pred;
}

void fw_ButtonControl(){
	if(vexRT[Btn8D]){
		if(_fly.pred == 0){
				setFlyRpm(SKILLS_SHORT_RPM);
				_fly.pred = SKILLS_SHORT_POW;
			while(vexRT[Btn8D]){
				wait1Msec(20);
			}
		}
		else{
			setFlyRpm(0);
			_fly.pred = 0;
			while(vexRT[Btn8D]){
					wait1Msec(20);
			}
		}
	}
	if(vexRT[Btn8L]){
		setFlyRpm(SHORT_RPM);
		_fly.pred = SHORT_POW;
	}
	if(vexRT[Btn8R]){
		setFlyRpm(MED_RPM);
		_fly.pred = MED_POW;
	}
	if(vexRT[Btn8U]){
		setFlyRpm(LONG_RPM);
		_fly.pred = HIGH_POW;
	}
}

void fw_shortSpeed(){
	setFlyRpm(SHORT_RPM);
	_fly.pred = SHORT_POW;
}

void fw_fullCourtSpeed(){
	setFlyRpm(LONG_RPM);
	_fly.pred = HIGH_POW;
}

void fw_skillSpeed(){
	setFlyRpm(SKILLS_RPM);
	_fly.pred = SKILLS_POW;
}

void fw_skillsShortSpeed(){
	setFlyRpm(SKILLS_SHORT_RPM);
	_fly.pred = SKILLS_SHORT_POW;
}

void fw_midSpeed(){
	setFlyRpm(MED_RPM);
	_fly.pred = MED_POW;
}

bool deployLift = false;
task flw_tsk_FeedForwardCntrl(){
	pidReset(_fly.flyPID);
	pidInit(_fly.flyPID, 0.6, 0.05, 0, 0, 999999);

	int integralLimit;
	if(_fly.flyPID.kI == 0){
		integralLimit = 0;
	} else{
		integralLimit = 13.0 / _fly.flyPID.kI;
	}

	float output = 0;
	float initTime = nPgmTime;
	while(true){
		fw_ButtonControl();
		//
		//we do not want to zero our error sum when we cross
		if(abs(_fly.flyPID.errorSum) > integralLimit){
			_fly.flyPID.errorSum = integralLimit * _fly.flyPID.errorSum/(abs(_fly.flyPID.errorSum));
		}

		_fly.currSpeed =  FwCalculateSpeed();
		float outVal = pidExecute(_fly.flyPID, _fly.setPoint - _fly.currSpeed);
		float dTime = nPgmTime - initTime;
		initTime = nPgmTime;



		playTone(_fly.currSpeed/2,2);

		output = _fly.pred + outVal;
		if(output < 0){
			output = 0;
		}
		if(output > 127){
			output = 127;
		}
		if((_fly.currSpeed <= 50 && _fly.setPoint == 0) && vexRT[Btn7L] == 1){
			output = -90;
			_updateFlyWheel(output);
			wait1Msec(4000);
			_updateFlyWheel(0);
			wait1Msec(10000);
		}
		_updateFlyWheel(output);
		delay(FW_LOOP_SPEED);
	}
}

void initFlyWheel(Fw_Controller* initMotors){
	_fly.currSpeed = 0;
	_fly = *initMotors;
	_fly.f1 = initMotors->f1;
	_fly.f2 = initMotors->f2;
	_fly.enc = initMotors->enc;
}

void fw_stopFlyControl(){
	stopTask(flw_tsk_FeedForwardCntrl);
}

void fw_startFlyControl(){
	startTask(flw_tsk_FeedForwardCntrl,24);
}

#endif
