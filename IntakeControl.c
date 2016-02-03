#ifndef IntakeControl.c
#define IntakeControl.c

#include "FlyControl.c"
const int shootDelay = 400;

 typedef struct IntakeController{
	 tMotor liftIntake;
	 tMotor outIntake;
	 tSensors liftSensor;
	 tSensors outerSensor;
	 tSensors enc;
	 int __intakeController.ballCount;
 };


IntakeController _intakeController;
const int intakeDriveTicks = 370;
void driveIntake(int ticks){
	int dTicks = 0;
	int iTicks = -SensorValue[ _intakeController.enc ];
	while(abs(dTicks) < abs(ticks)){
		//writeDebugStreamLine("%f   %f",SensorValue[_intakeController.enc], dTicks);
		motor[ _intakeController.outIntake ] = ticks > 0 ? 127 : -127;
		motor[ _intakeController.liftIntake ] = ticks > 0 ? 127 : -127;
		dTicks = (-SensorValue[_intakeController.enc]) - iTicks;
		wait1Msec(20);
	}
	motor[ _intakeController.liftIntake ] =  0 ;
}

void driveIntake(){
	driveIntake(intakeDriveTicks);
}

void IntakeInit(tMotor lift, tMotor outer, tSensors liftSense, tSensors outSense, tSensors encoder){
	_intakeController.liftIntake = lift;
	_intakeController.outIntake = outer;
	_intakeController.liftSensor = liftSense;
	_intakeController.outerSensor = outSense;
	_intakeController.enc = encoder;
}


const int _ballThresh = 2900;
bool ballAtLift(){
	return SensorValue[_intakeController.liftSensor] < _ballThresh;
}



const int ultraThresh = 10;
bool ballAtOuter(){
	return SensorValue[_intakeController.outerSensor] <= ultraThresh;
}

int __intakeController.ballCount = 0;
int outToggle = 1;

void resetBallCount(){
	_intakeController.ballCount = 0;
}

void incrementBallCount(){
	_intakeController.ballCount++;
}


void autoIntake(){
	if(ballAtLift() && _intakeController.ballCount <= 1){
		wait1Msec(200);
			driveIntake(intakeDriveTicks);
			_intakeController.ballCount++;
			wait1Msec(200);
			//motor[_intakeController.outIntake] = 127;
	}
	else if((ballAtLift() && (_intakeController.ballCount == 2)) && !ballAtOuter()){
			_intakeController.ballCount = 3;
			wait1Msec(200);
			//motor[_intakeController.outIntake] = 127;
	}
	else if(ballAtLift() && ballAtOuter() && _intakeController.ballCount != 4){
		_intakeController.ballCount = 4;
		wait1Msec(200);
	}

	if(_intakeController.ballCount == 4){
		if(!ballAtOuter()){
			_intakeController.ballCount--;
		}
		else{
			motor[_intakeController.outIntake] = 0;
		}
		wait1Msec(200);
	}
	else{
		//motor[_intakeController.outIntake] = 127;
	}
}


void autoShoot(){
	motor[_intakeController.outIntake] = 127;
	for(int i = 0; i<_intakeController.ballCount; i++){
		driveIntake(intakeDriveTicks);
		wait1Msec(shootDelay);
	}
	_intakeController.ballCount = 0;
}


void _decrementBallCount(){
	if( _intakeController.ballCount >= 2){
		_intakeController.ballCount--;
	}
}


void ink_adjustFire(int threshold){
	int temp = _fly.pred;
	if(abs(_fly.flyPID.error) < threshold){
				driveIntake(intakeDriveTicks);
				if(_intakeController.ballCount == 1){
						driveIntake(intakeDriveTicks);
				}
				if(_intakeController.ballCount != 0){
					_intakeController.ballCount--;
				}
				_fly.pred = 127;

	}
	autoIntake();
	wait1Msec(200);
	_fly.pred = temp;
}


void ink_fireWhenReady(int threshold){
		//ink_adjustFire(threshold);
		if(abs(_fly.flyPID.error) < threshold){
				wait1Msec(30);
				if(abs(_fly.flyPID.error) < threshold){
					writeDebugStreamLine("shot error %d",_fly.flyPID.error);


				driveIntake(intakeDriveTicks);
				if(_intakeController.ballCount == 1){
						driveIntake(intakeDriveTicks);
				}
				if(_intakeController.ballCount != 0){
					_intakeController.ballCount--;
				}
	}
}
	autoIntake();
}

void ink_waitUntilFire(int errorThresh){
	bool fired = false;
	while(!fired){
		if(abs(_fly.flyPID.error) < errorThresh){
			driveIntake(intakeDriveTicks);
			fired = true;
		}
		wait1Msec(100);
	}
}


task intakeControl(){
	long initTime = nSysTime;

	while(true){
		autoIntake();

		//autoshooter
		if(vexRT[Btn7L] == 1){
			//empty everything

		}

		if(vexRT[Btn5D] == 1){

			//override in case of a jam
			_intakeController.ballCount = 0;
			motor[_intakeController.liftIntake] = 127;
			motor[_intakeController.outIntake] = 127;
			while(vexRT[Btn5D] == 1){
				wait1Msec(50);
			}

		}
		else if(vexRT[Btn5U] == 1){
			if(outToggle == 1){
				outToggle = 0;
			}
			else{
				outToggle = 1;
				motor[_intakeController.outIntake] = 127;
			}
			while(vexRT[Btn5U] == 1)
			{
				wait1Msec(20);
			}
		}
		else if(vexRT[Btn6D] == 1){
			//override
			while(vexRT[Btn6D] == 1){
				ink_fireWhenReady(20);
				wait1Msec(20);
			}
		}
		else if(vexRT[Btn6U] == 1){
			_decrementBallCount();

			driveIntake(-intakeDriveTicks);
		}
		else if(vexRT[Btn7U] == 1){
			_intakeController.ballCount = 0;
			//override
			motor[_intakeController.liftIntake] = -127;
			motor[_intakeController.outIntake] = -127;
			while(vexRT[Btn7U] == 1){
				wait1Msec(50);
			}
		}
		else{
			motor[_intakeController.liftIntake] = 0;
			//	writeDebugStreamLine("%f", outToggle);
			if(outToggle == 1 && _intakeController.ballCount != 4){
				motor[_intakeController.outIntake] = 127;
			}
			else{
				motor[_intakeController.outIntake] = 0;
			}
		}
		while(vexRT[Btn7D] == 1){

			driveIntake(intakeDriveTicks);
			_intakeController.ballCount = 0;
			wait1Msec(shootDelay);
		}
			while(vexRT[Btn7R] == 1){
				ink_fireWhenReady(30);
				wait1Msec(20);
		}


		wait1Msec(20);
	}
}

void autonomousShoot(){
	driveIntake();
	wait1Msec(900);
	driveIntake();
	wait1Msec(900);
	driveIntake();
	wait1Msec(900);
	driveIntake();
	wait1Msec(900);
	_intakeController.ballCount = 0;
}

void fw_skillsShoot(){
	driveIntake();
	wait1Msec(1200);
	return;
}


#endif
