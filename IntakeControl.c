#ifndef IntakeControl.c
#define IntakeControl.c

 typedef struct IntakeController{
	 tMotor liftIntake;
	 tMotor outIntake;
	 tSensors liftSensor;
	 tSensors outerSensor;
	 tSensors enc;
	 int ballCount;
 };


IntakeController _intakeController;

const int intakeDriveTicks = 400;
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

	int ballCount = 0;
task intakeControl(){
	long initTime = nSysTime;
	int outToggle = 1;

	while(true){

		if(ballAtLift() && ballCount <= 1){
				wait1Msec(500);
				driveIntake(intakeDriveTicks);
				ballCount++;
				motor[_intakeController.outIntake] = 127;
		}
		else if((ballAtLift() && (ballCount == 2)) && !ballAtOuter()){
				ballCount = 3;
				motor[_intakeController.outIntake] = 127;
		}
		else if(ballAtLift() && ballAtOuter() && ballCount != 4){
			ballCount = 4;
		}

		if(ballCount == 4){
			motor[_intakeController.outIntake] = 0;
		}

		if(vexRT[Btn7L] == 1){
			//empty everything
			motor[_intakeController.outIntake] = 127;
			for(int i = 0; i<ballCount; i++){
				driveIntake(intakeDriveTicks);
				wait1Msec(200);
			}
			ballCount = 0;
		}


		if(vexRT[Btn5D] == 1){
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
			driveIntake(intakeDriveTicks);
		}
		else if(vexRT[Btn6U] == 1){
			driveIntake(-intakeDriveTicks);
		}
		else if(vexRT[Btn7U] == 1){
			motor[_intakeController.liftIntake] = -127;
			motor[_intakeController.outIntake] = -127;
		}
		else{
			motor[_intakeController.liftIntake] = 0;
		//	writeDebugStreamLine("%f", outToggle);
			if(outToggle == 1 && ballCount != 4){
				motor[_intakeController.outIntake] = 127;
			}
			else{
				motor[_intakeController.outIntake] = 0;
			}
		}
		while(vexRT[Btn7D] == 1){
			writeDebugStreamLine("Fire");
			driveIntake(intakeDriveTicks);
			wait1Msec(200);
		}
		wait1Msec(20);
	}
}
#endif
