#ifndef 
#define IntakeControl.c

void driveIntake(int ticks){
	int dTicks = 0;
	int iTicks = -SensorValue[ encIntake ];
	while(abs(dTicks) < abs(ticks)){
		writeDebugStreamLine("%f   %f",SensorValue[encIntake], dTicks);
		motor[ mIntake2 ] = ticks > 0 ? 127 : -127;
		motor[ mIntake ] = ticks > 0 ? 127 : -127;
		dTicks = (-SensorValue[encIntake]) - iTicks;
		wait1Msec(20);
	}
	motor[ mIntake ] =  0 ;
}


task intakeControl(){
	long initTime = nSysTime;
	int outToggle = 1;
	while(true){
		if(vexRT[Btn5D] == 1){
			writeDebugStreamLine("Fire");
			motor[mIntake] = 127;
			motor[mIntake2] = 127;
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
				motor[mIntake2] = 127;
			}
			while(vexRT[Btn5U] == 1)
			{
				wait1Msec(20);
			}
		}
		else if(vexRT[Btn6D] == 1){
			driveIntake(400);
		}
		else if(vexRT[Btn6U] == 1){
			driveIntake(-400);
		}
		else if(vexRT[Btn7U] == 1){
			motor[mIntake] = -127;
			motor[mIntake2] = -127;
		}
		else if(vexRT[Btn7L] == 1){
			driveIntake(400);
			wait1Msec(200);
			driveIntake(400);
			wait1Msec(200);
		}
		else{
			motor[mIntake] = 0;
		//	writeDebugStreamLine("%f", outToggle);
			if(outToggle == 1){
				motor[mIntake2] = 127;
			}
			else{
				motor[mIntake2] = 0;
			}
		}
		while(vexRT[Btn7D] == 1){
			writeDebugStreamLine("Fire");
			driveIntake(400);
			wait1Msec(200);
		}
		wait1Msec(20);
	}
}



#endif