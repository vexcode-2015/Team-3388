#pragma config(UART_Usage, UART1, uartVEXLCD, baudRate19200, IOPins, None, None)
#pragma config(UART_Usage, UART2, uartNotUsed, baudRate4800, IOPins, None, None)
#pragma config(Sensor, in1,    gyroDrive,      sensorAnalog)
#pragma config(Sensor, in2,    lfIntake,       sensorLineFollower)
#pragma config(Sensor, in3,    potColour,      sensorPotentiometer)
#pragma config(Sensor, in5,    lfTop,        sensorPotentiometer)
#pragma config(Sensor, in4,    potSwitcher,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  encLeftDr,      sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  encFlywheel,    sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  encIntake,      sensorQuadEncoder)
#pragma config(Sensor, dgtl9,  ultraIntake,    sensorSONAR_cm)
#pragma config(Sensor, dgtl11, encRightDr,     sensorQuadEncoder)
#pragma config(Motor,  port1,           mIntake,       tmotorVex393HighSpeed_HBridge, openLoop)
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
#include "Auto.c"
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)
#include "Vex_Competition_Includes.c"

Fw_Controller fly;
DriveBase dr;

void motorTest(){
	for(int i = 0; i<10; i++){
		motor[i] = 50;
		wait1Msec(500);
		motor[i] = 0;
	}
}

void pre_auton()
{
	bStopTasksBetweenModes = false;

	//drive
	//motors
	dr.fl = mDrFl; dr.fr = mDrFr; dr.bl = mDrBl;
	dr.br = mDrBr; dr.ml = mDrMl; dr.mr = mDrMr;
	//sensors
	dr.encLeft = encLeftDr; dr.encRight = encRightDr;
	dr.gyro = gyroDrive;

	//intake init
	IntakeInit(mIntake, mIntake2, lfIntake, ultraIntake, encIntake, lfTop);
	initMecDrive(dr);

	//flywheel init
	fly.f1 = mFly1; fly.f2 = mFly2;
	fly.enc = encFlywheel;
	initFlyWheel(fly);

	//calibrate gyro
	GyroInit(in1);
	playTone(440, 50);
	playTone(440/2, 50);
	playTone(440*2, 50);
	wait1Msec(3000);

}

task autonomous()
{
	stopTask(intakeControl);
	stopTask(usercontrol);
	mec_StopTeleop();
	fw_startFlyControl();
	ink_set(0);


	GyroZeroAbs();


	int colourThresh = 2000;
	bool isRed = false;

	if(SensorValue[potColour] > colourThresh ){
		writeDebugStreamLine("auto red detected");
		isRed = true;
	}
	else{
		writeDebugStreamLine("auto blue detected");
	}

	int selection = utl_getPotSet(SensorValue[potSwitcher]);
	writeDebugStreamLine("selection = %f", selection);
	if(selection == 1){
		auto_rout_outsideHerdMid(isRed);
	}
	else if (selection == 2){
		auto_rout_outsideRunShot(isRed);
	}
	else if(selection == 3){
		//backwards towards mid stacks
		auto_rout_challengeMidOutside(isRed);
	}
	else if(selection == 4){
		auto_rout_shoot4(isRed);
	}
	else if(selection == 5){
		//shoot
		auto_rout_insideChallengeMid(isRed,true);
	}
	else if(selection == 6){
		//don't shoot
		auto_rout_insideChallengeMid(isRed,false);
	}
	else if(selection == 7){
		auto_rout_test();
	}
}

void utl_fw_printRecovery(){
	long init = nPgmTime;
	if(abs(_fly.flyPID.error) > 50){
			while((abs(_fly.flyPID.error) > 50)){
				wait1Msec(20);
			}
		writeDebugStreamLine("SETTLE TIME %f", nPgmTime - init);
	}
}

task usercontrol ()
{
	long initTime = nPgmTime;



	stopTask(autonomous);

	fw_stopFlyControl();
	ink_stopRunningShot();
	fw_startFlyControl();
	GyroResetAngle();

	mec_StartTeleop();
	startTask(intakeControl,6);



	while(true)
	{
		utl_fw_printRecovery();

	//	_mecDrive();
	//driveTesting();

	//writeDebugStreamLine("%f", _fly.flyPID.kP);

	//writeDebugStreamLine("%f, \coeff %f  %f SETTLE TIME %f", curr, coeff, Y, nSysTime);
	//writeDebugStreamLine("%f", GyroGetAngle());

	//printPIDDebug(mec.slave);
	//writeDebugStreamLine("GYRO ANGLE : %f", GyroGetAngle());

	//printPIDDebug(mec.master);
	//	writeDebugStreamLine("%f", _intakeController.ballCount);
	//utl_fw_printRecovery();

	 //printPIDDebug(_fly.flyPID);
	//writeDebugStreamLine("%f, %f", _fly.pred, motor[mFly1]);

	//	writeDebugStreamLine("%f, %f  %f",error, nAvgBatteryLevel, Y);
	//writeDebugStreamLine("_fly.currSpeed%f, set %f", FwCalculateSpeed(), _setRPM);
		wait1Msec(50);
	}
}
