#pragma config(UART_Usage, UART1, uartVEXLCD, baudRate19200, IOPins, None, None)
#pragma config(UART_Usage, UART2, uartNotUsed, baudRate4800, IOPins, None, None)
#pragma config(Sensor, in1,    gyroDrive,      sensorNone)
#pragma config(Sensor, in3,    lfIntake,       sensorLineFollower)
#pragma config(Sensor, in4,    lfOuter,        sensorLineFollower)
#pragma config(Sensor, dgtl1,  encLeftDr,      sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  encFlywheel,    sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  encIntake,      sensorQuadEncoder)
#pragma config(Sensor, dgtl10, statusLightRed, sensorDigitalOut)
#pragma config(Sensor, dgtl11, encRightDr,     sensorQuadEncoder)
#pragma config(Motor,  port1,           mIntake,       tmotorVex393HighSpeed_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           mFly1,         tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port3,           mFly2,         tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           mDrMr,         tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           mDrMl,         tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port6,           mDrBl,         tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           mDrFl,         tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           mDrBr,         tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           mDrFr,         tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          mIntake2,      tmotorVex393TurboSpeed_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "MecDrive.c"
#include "FlyControl.c"
#include "Utils.c"
#include "IntakeControl.c"
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)
#include "Vex_Competition_Includes.c"

Fw_Controller fly;
DriveBase dr;
int autoNum = 0;
void pre_auton()
{
	bStopTasksBetweenModes = false;
	dr.fl = mDrFl; dr.fr = mDrFr; dr.bl = mDrBl;	dr.br = mDrBr; dr.ml = mDrMl; dr.mr = mDrMr;
	dr.gyro = gyroDrive; dr.encLeft = encLeftDr; dr.encRight = encRightDr;

	IntakeInit(mIntake, mIntake2, lfIntake, lfOuter, encIntake);
	initMecDrive(dr);

	fly.f1 = mFly1; fly.f2 = mFly2;
	fly.enc = encFlywheel;
	autoNum = 0;
	//autoNum = readAutoNum();
	if (!(nVexRCReceiveState & vrDisabled)) {
		break;
	}
	
	printCalibratingGyro();
	wait1Msec(1500)
	enableGyro();
	wait1Msec(3000);

	bool redSide = true;

	while((nVexRCReceiveState & vrDisabled)){
		printBatteryToLCD();
	}
}

task autonomous()
{
	//startTask(Track, 8);
	GyroResetAngle();
	fw_fullCourtSpeed();
	wait1Msec(2500);
	autonomousShoot();
	wait1Msec(700);
	if(redSide){
		gyroTurnDegreesRel(160.43494);
	}
	else{
		gyroTurnDegreesRel(-160.43494);
	}
	wait1Msec(500);
	driveInches(-30);
	driveInches(-6, 30);
	wait1Msec(500);
	faceNet();
	autonomousShoot();
	//driveInches(-12);
	//turnDegrees(-90);
	/**
	initFlyWheel(fly);
	setFlyWheel(LONG_RPM, LONG_PRED);
	wait1Msec(3000);
	driveIntake(400);
	wait1Msec(3000);
	writeDebugStreamLine("%f",SensorValue[encIntake]);
	driveIntake(400);
	wait1Msec(3000);
	writeDebugStreamLine("%f",SensorValue[encIntake]);
	driveIntake(400);
	wait1Msec(3000);
	driveIntake(400);
	switch(autoNum){
		case 0:{
			break;
		}
		case 1:{
			break;
		}
		case 2:{
			break;
		}
	}**/
}


//calibrate
void calibrate(){
	coeff = 0;
	for(int i = 1; i<100; i++){
		coeff = 0.001 + 0.0002 * i;
		writeDebugStreamLine("Testing Coeff %f", coeff);
		setFlyWheel(LONG_RPM,LONG_PRED);
		long pTime = nSysTime;
		long nTime = nSysTime;
		while(abs(pTime - nSysTime) < 4000){
			if(abs(error) > 100){
				long sTime = nSysTime;
				while(abs(error) > 10 && (abs(pTime - nSysTime) < 4000)){
					wait1Msec(50);
				}
				writeDebugStreamLine("%f, coeff %f  %f SETTLE TIME %f",error, coeff, Y, nSysTime - sTime);
			}
		}
		setFlyWheel(0,0);
		while(abs(error) > 10){
				//writeDebugStreamLine("zeroing");
			wait1Msec(500);
		}
	}
}


void driveTesting(){
		if(vexRT[Btn7D] == 1){
			printDriveEncoders();
		}

		if(vexRT[Btn7L] == 1){
			zeroDriveEncoders();
		}
}


task usercontrol()
{
	setStatusLight(statusLightRed);
	GyroResetAngle();
	initFlyWheel(fly);
	initMecDrive(dr);
	startTask(intakeControl, 3);

	while(true)
	{
	//	_mecDrive();
		//driveTesting();

	//	writeDebugStreamLine("%f", _fly.flyPID.kP);

		//writeDebugStreamLine("%f, coeff %f  %f SETTLE TIME %f", curr, coeff, Y, nSysTime);
		//writeDebugStreamLine("%f", GyroGetAngle());
	//printPIDDebug(_fly.flyPID);
	//writeDebugStreamLine("%f", motor[mFly1]);
	printPIDDebug(mec.master);
	writeDebugStreamLine("%f", __intakeController.ballCount);
//	writeDebugStreamLine("%f : %f", curr, motor[mFly1]);

//	writeDebugStreamLine("%d", motor[mFly1]);

	//writeDebugStreamLine("%f",motor[_fly.f1]);

		//	writeDebugStreamLine("%f, %f  %f",error, nAvgBatteryLevel, Y);
		//writeDebugStreamLine("_fly.currSpeed%f, set %f", FwCalculateSpeed(), _setRPM);
		wait1Msec(50);
	}
}
