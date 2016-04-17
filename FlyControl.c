
#ifndef FlyControl.c
#define FlyControl.c
//old 20
#define FW_LOOP_SPEED	20


#include "Utils.c"
#include "PIDController.h"
#include "MecDrive.c"

int SKILLS_SHORT_RPM = 2100;
int SKILLS_SHORT_POW = 45;
int SKILLS_RPM = 1950; //1950
int SKILLS_POW = 50;
int SHORT_RPM = 1800;//1690;
int SHORT_POW = 45;
int MED_RPM = 2100;
int MED_POW = 55;
int LONG_RPM  = 2540;//2480;//2680;//2950;
int HIGH_POW = 65;//75;


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
	_fly.currSpeed = (_fly.currSpeed * 0.7) + newSpeed * 0.3;
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

/**void spinFlyWheelAuto(){
	float rpm = get_required_rpm(trk_getNetDistance());

	_setFlyWheel(rpm);
	if(rpm < MED_RPM){

		_fly.pred = MED_POW;
		if(rpm < LOW_RPM){
			_fly.pred = LOW_POW;
		}
	}
	else{
		_fly.pred = HIGH_POW;
	}

}**/



void fw_ButtonControl(){
	//btnControl
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
	 if(vexRT[Btn7R]){
	 	//spinFlyWheelAuto();
	 	//pidReset(_fly.flyPID);
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
	//TRY: fairly good fast recovery
	//prev P 0.
	pidInit(_fly.flyPID, 1.0, 0.05, 0, 0, 9999);
	//pidInit(_fly.flyPID, 0.5, 0.1, 0, 0, 9999);

	//pidInit(_fly.flyPID, 0.15, 0.05, 0, 100, 9999);

	//Good results at low rpms
	//pidInit(_fly.flyPID, 0.1, 0.01, 0, 100, 9999);

	//dislike slow
	//pidInit(_fly.flyPID, 0.2, 0.001, 0, 100, 9999);

	int integralLimit = 24 / _fly.flyPID.kI;
	float output = 0;
	float initTime = nPgmTime;
	//basically for this we need a guess motor power
	while(true){

		fw_ButtonControl();
		//printPIDDebug(_fly.flyPID);
		//we do not want to zero our error sum when we cross
		if(abs(_fly.flyPID.errorSum) > integralLimit){
			_fly.flyPID.errorSum = integralLimit * _fly.flyPID.errorSum/(abs(_fly.flyPID.errorSum));
		}

		_fly.currSpeed =  FwCalculateSpeed();
		float outVal = pidExecute(_fly.flyPID, _fly.setPoint - _fly.currSpeed);
		float dTime = nPgmTime - initTime;
		initTime = nPgmTime;

		//filter output
		float coeff = dTime/FW_LOOP_SPEED;
		if(coeff < 1){
			coeff = 1;
		}


		playTone(_fly.currSpeed/2,2);
		output = _fly.pred + ((outVal) * (coeff));


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


task flw_task_bangbang(){
	while(true){

		fw_ButtonControl();
		_fly.currSpeed =  FwCalculateSpeed();
		float error = _fly.setPoint - _fly.currSpeed;
		writeDebugStreamLine("%f",error);

		float outVal = 0;
		if(abs(error) > 10){
			outVal = error > 0 ? 127 : 0;
		}


		float output = outVal;

		if(output < 0){
			output = 0;
		}
		if(output > 127){
			output = 127;
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
	//writeDebugStreamLine("FLY %d %d %d %d", _fly.f1,_fly.f2, _fly.f3, _fly.f4);



	//startTask(flw_task_bangbang,20);
	//startTask(flw_task_PIDCntrl, 20);
}

void fw_stopFlyControl(){
	stopTask(flw_tsk_FeedForwardCntrl);
}

void fw_startFlyControl(){
	startTask(flw_tsk_FeedForwardCntrl,24);
}

#endif
