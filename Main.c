#pragma config(UART_Usage, UART1, uartVEXLCD, baudRate19200, IOPins, None, None)
#pragma config(UART_Usage, UART2, uartNotUsed, baudRate4800, IOPins, None, None)
#pragma config(Sensor, in1,    gyroDrive,      sensorAnalog)
#pragma config(Sensor, in2,    lfIntake,       sensorLineFollower)
#pragma config(Sensor, in3,    potColour,      sensorPotentiometer)
#pragma config(Sensor, in4,    potTile,        sensorPotentiometer)
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
int autoNum = 0;

void motorTest(){
	for(int i = 0; i<10; i++){
		motor[i] = 50;
		wait1Msec(500);
		motor[i] = 0;
	}
}

void pre_auton()
{

	//bStopTasksBetweenModes = false;
	dr.fl = mDrFl; dr.fr = mDrFr; dr.bl = mDrBl;	dr.br = mDrBr; dr.ml = mDrMl; dr.mr = mDrMr;
	dr.gyro = gyroDrive; dr.encLeft = encLeftDr; dr.encRight = encRightDr;

	IntakeInit(mIntake, mIntake2, lfIntake, ultraIntake, encIntake);
	initMecDrive(dr);

	fly.f1 = mFly1; fly.f2 = mFly2;
	fly.enc = encFlywheel;
	initFlyWheel(fly);

	autoNum = 0;
	//autoNum = readAutoNum();




	printCalibratingGyro();
	GyroInit(in1);
	bStopTasksBetweenModes = false;
	playTone(440, 50);
	wait1Msec(1500);



//	while((nVexRCReceiveState & vrDisabled)){
//		printBatteryToLCD();
//	}
}
	bool redSide = true;

task autonomous()
{

	stopTask(usercontrol);
	mec_StopTeleop();
	fw_startFlyControl();
	GyroZeroAbs();
	//auto_rout_mid3stack();


	int colourThresh = 2000;

	bool isRed = false;
	if(SensorValue[potColour] > colourThresh ){
		writeDebugStreamLine("auto red detected");
		isRed = true;
	}
	else{
			writeDebugStreamLine("auto blue detected");
	}

	int tileThresh = 2000;
	bool isOutside = true;
	if(SensorValue[potTile] < tileThresh){
			writeDebugStreamLine("auto inside detected");
		isOutside = false;
	}
	else{
		writeDebugStreamLine("auto outside detected");
	}

	if(SensorValue[potColour] > 1000 && SensorValue[potColour] < 3000){
		auto_rout_skillsShort();
	}else{
	//	auto_rout_midShootTwo(isRed);
		auto_rout_threestacks();
		//auto_rout_getMidBalls(isOutside,isRed);
	}
	//\mec_tmpDriveInches(1,0.2,1); **/
}


void driveTesting(){
		if(vexRT[Btn7D] == 1){
			printDriveEncoders();
		}

		if(vexRT[Btn7L] == 1){
			zeroDriveEncoders();
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
	fw_stopFlyControl();
	fw_startFlyControl();
	stopTask(autonomous);
	//GyroResetAngle();

	mec_StartTeleop();
	startTask(intakeControl,6);


	 while(true)
	{
	//	_mecDrive();
	//driveTesting();

	//writeDebugStreamLine("%f", _fly.flyPID.kP);

	//writeDebugStreamLine("%f, \coeff %f  %f SETTLE TIME %f", curr, coeff, Y, nSysTime);
	//writeDebugStreamLine("%f", GyroGetAngle());
	//printPIDDebug(mec.slave);
	//writeDebugStreamLine("GYRO ANGLE : %f", GyroGetAngle());

	//printPIDDebug(mec.master);
	//	writeDebugStreamLine("%f", _intakeController.ballCount);

	//writeDebugStreamLine("%f     %f", motor[mIntake], _fly.pred);
	//utl_fw_printRecovery();

	//printPIDDebug(_fly.flyPID);


		//	writeDebugStreamLine("%f, %f  %f",error, nAvgBatteryLevel, Y);
		//writeDebugStreamLine("_fly.currSpeed%f, set %f", FwCalculateSpeed(), _setRPM);
		wait1Msec(500);
	}
}
