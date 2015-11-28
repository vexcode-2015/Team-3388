#ifndef IntakeControl.c
#define IntakeControl.c


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
const int intakeDriveTicks = 380;
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

bool ballAtOuter(){
	return SensorValue[_intakeController.outerSensor] < _ballThresh;
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
			wait1Msec(500);
			driveIntake(intakeDriveTicks);
			_intakeController.ballCount++;
			//motor[_intakeController.outIntake] = 127;
	}
	else if((ballAtLift() && (_intakeController.ballCount == 2)) && !ballAtOuter()){
			_intakeController.ballCount = 3;
			//motor[_intakeController.outIntake] = 127;
	}
	else if(ballAtLift() && ballAtOuter() && _intakeController.ballCount != 4){
		_intakeController.ballCount = 4;
	}

	if(_intakeController.ballCount == 4){
		motor[_intakeController.outIntake] = 0;
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



task intakeControl(){
	long initTime = nSysTime;

	while(true){
		autoIntake();

		//autoshooter
		if(vexRT[Btn7L] == 1){
			//empty everything
			autoShoot();
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
			_intakeController.ballCount = 0;
			//override
			driveIntake(intakeDriveTicks);
		}
		else if(vexRT[Btn6U] == 1){
			_intakeController.ballCount = 0;
			//override
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
		wait1Msec(20);
	}
}

void autonomousShoot(){
	StopTask(intakeControl);
	driveIntake();
	wait1Msec(shootDelay);
	driveIntake();
	wait1Msec(shootDelay);
	driveIntake();
	wait1Msec(shootDelay);
	driveIntake();
	wait1Msec(shootDelay);
	StartTask(intakeControl);
}
#endif
