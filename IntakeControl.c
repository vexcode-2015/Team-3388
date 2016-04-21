#ifndef IntakeControl.c
#define IntakeControl.c

#include "FlyControl.c"
#include "MecDrive.c"
const int shootDelay = 400;

 typedef struct IntakeController{
	 tMotor liftIntake;
	 tMotor outIntake;
	 tSensors liftSensor;
	 tSensors outerSensor;
	 tSensors topSensor;
	 tSensors enc;
	 int __intakeController.ballCount;
 };


IntakeController _intakeController;

const int _ballThresh = 2970;
bool ballAtLift(){
	return SensorValue[_intakeController.liftSensor] < _ballThresh;
}


bool ink_ballAtTop(){
	return SensorValue[_intakeController.topSensor] < _ballThresh;
}

const int intakeDriveTicks = 340;//360


void driveIntake(int ticks, bool slow, bool shoot = true){
	int dTicks = 0;
	int iTicks = -SensorValue[ _intakeController.enc];
	float kP = 0.1;
	while(abs(dTicks) < abs(ticks) && vexRT[Btn7U] == 0){
		if(!shoot){
				if(ink_ballAtTop()){
					break;
				}
		}
	writeDebugStreamLine("%f   %f",SensorValue[_intakeController.enc], dTicks);
			dTicks = (-SensorValue[_intakeController.enc]) - iTicks;
	if(!slow){
		motor[ _intakeController.outIntake ] = ticks > 0 ? 127 : -127;
		motor[ _intakeController.liftIntake ] = ticks > 0 ? 127 : -127;
	}
	else{
		motor[ _intakeController.outIntake ] = ticks > 0 ? 127 : -127;
		motor[ _intakeController.liftIntake ] = kP * (ticks - dTicks) + 90;//    ticks > 0 ? 100 : -100;
	}

		wait1Msec(20);
	}
	motor[ _intakeController.liftIntake ] =  0 ;
}

void driveIntake(int ticks){
	driveIntake(ticks,false);
}


void driveIntake(){
	driveIntake(intakeDriveTicks);
}

void ink_driveBack(){
	driveIntake(-intakeDriveTicks);
}

void IntakeInit(tMotor lift, tMotor outer, tSensors liftSense, tSensors outSense, tSensors encoder, tSensors top){
	_intakeController.liftIntake = lift;
	_intakeController.outIntake = outer;
	_intakeController.liftSensor = liftSense;
	_intakeController.outerSensor = outSense;
	_intakeController.enc = encoder;
	_intakeController.topSensor = top;
}




const int ultraThresh = 29;
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

	if(ballAtLift() && _intakeController.ballCount == 0){
			wait1Msec(50);
			driveIntake(intakeDriveTicks,true,false);
			_intakeController.ballCount++;
			//motor[_intakeController.outIntake] = 127;
	} else if(ballAtLift() && _intakeController.ballCount == 1){
			wait1Msec(50);
			driveIntake(100 + intakeDriveTicks,true,false);
			_intakeController.ballCount++;
	}
	else if((ballAtLift() && (_intakeController.ballCount == 2)) && !ballAtOuter()){
			_intakeController.ballCount = 3;
			wait1Msec(50);
			//motor[_intakeController.outIntake] = 127;
	}
	else if(ballAtLift() && ballAtOuter() && _intakeController.ballCount != 4){
		_intakeController.ballCount = 4;
		wait1Msec(30);
	}

	if(_intakeController.ballCount == 4){
		if(!ballAtOuter()){
			wait1Msec(50);
			if(!ballAtOuter()){
				_intakeController.ballCount--;

			}
		}

			motor[_intakeController.outIntake] = 0;

		wait1Msec(30);
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

	driveIntake(intakeDriveTicks,true,false);
	if(abs(_fly.flyPID.error) < threshold){
				wait1Msec(30);
		if(abs(_fly.flyPID.error) < threshold){
				//writeDebugStreamLine("shot error %d",_fly.flyPID.error);
					driveIntake(100,true);

				if(_intakeController.ballCount == 1){
						//driveIntake(intakeDriveTicks,true);
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
		driveIntake(intakeDriveTicks,true,false);
		if(abs(_fly.flyPID.error) < errorThresh){
			driveIntake(200,true);
			fired = true;
		}
		wait1Msec(50);
	}
}


void ink_spitOut(){
	ink_driveBack();
	if(ballCount >= 1){
		ballCount--;
	}
	//wait1Msec(100);
}

void ink_set(int pow){
		motor[_intakeController.liftIntake] = pow;
		motor[_intakeController.outIntake] = pow;

}

task intakeControl(){
	long initTime = nSysTime;

	while(true){
		autoIntake();

		//autoshooter
		if(vexRT[Btn7L] == 1){
			//ink_spitOut();

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
				ink_fireWhenReady(60);
				wait1Msec(20);
			}
		}
		else if(vexRT[Btn6U] == 1){
			motor[_intakeController.outIntake] = -127;
			while(vexRT[Btn6U] == 1){
				wait1Msec(50);
			}
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
			if(!ink_ballAtTop()){
					motor[_intakeController.outIntake] = 127;
					motor[_intakeController.liftIntake] = 100;
			} else{
					motor[_intakeController.outIntake] = 0;
						motor[_intakeController.liftIntake] = 0;
			}
			if(abs(_fly.flyPID.error) < 30){
				driveIntake(150);
				wait1Msec(200);
			}
			wait1Msec(20);
		}
			while(vexRT[Btn7R] == 1){
				ink_fireWhenReady(30);
				wait1Msec(200);
		}
		wait1Msec(20);
	}
}

void ink_startTask(int balls){
	outToggle  = 1;
	startTask(intakeControl);
	ink_set(127);
	_intakeController.ballCount = balls;
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



float ink_runShotErr = 30;
task ink_autoShoot(){
	while(true){
		writeDebugStreamLine("curr drive err %f",mec_getDriveErr() *  TICKS_PER_INCHES);
		if( abs(mec_getDriveErr() /  TICKS_PER_INCHES)  < ink_runShotErr){
				ink_set(127);
		} else{
				ink_set(0);
  	}
		wait1Msec(50);
	}
}

void ink_startRunningShot(float thresh){
	ink_runShotErr = thresh;
	startTask(ink_autoShoot);
}

void ink_stopRunningShot(){
	stopTask(ink_autoShoot);
}



#endif
