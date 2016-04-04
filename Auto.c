#ifndef Auto.c
#define Auto.c

#include "MecDrive.c"
#include "IntakeControl.c"
#include "FlyControl.c"

void auto_rout_mid3stack(bool isRed){
	int mult = isRed ? -1 : 1;
	//drive forward
	fw_fullCourtSpeed();

	_fly.pred = 57;

	ink_set(0);
	_intakeController.ballCount = 4;
	wait1Msec(1000);
	for(int i = 0; i<5; i++){
		ink_waitUntilFire(30);
		wait1Msec(100);
	}

	mec_driveInches(15,100,3000);

	//shoot first 4 balls
	_intakeController.ballCount = 0;
	startTask(intakeControl);
	//turn toward first stack
	mec_GyroTurnRel(-130 * mult);
	mec_driveInches(-21,15,9999);
	mec_driveInches(-30,127,500);

	mec_GyroTurnRel(-180 * mult);
	stopTask(intakeControl);
	ink_set(-127);
	wait1Msec(2000);

	_intakeController.ballCount = 0;
	startTask(intakeControl);
	//face the wall
	mec_GyroTurnRel(-90 * mult);
	//grab second stack
	mec_driveInches(-15,127,1000);
	mec_driveInches(5,127,500);
	mec_driveInches(-10,40,1000);
	mec_driveInches(10);
	mec_GyroTurnRel(90);
	ink_set(-127);
	wait1Msec(2000);

}

void auto_rout_challengedMiddle(bool isRed){
	int mult = isRed ? -1 : 1;
	fw_fullCourtSpeed();
	mec_driveInches(20,80,9000);
	ink_waitUntilFire(100);
	ink_set(127);
	wait1Msec(800);
	mec_GyroTurnAbs(-200 * mult);
	mec_driveInches(-100);
}



void auto_rout_threestacks(){
	setFlyRpm(SHORT_RPM + 200);
	_fly.pred = SHORT_POW + 10;
	startTask(intakeControl);

	mec_driveInchesTwoStage(-105,40,100,30,9999);
	mec_GyroTurnAbs(140);
	for(int i = 0; i<6; i++){
		ink_waitUntilFire(9000);
	}
	setFlyRpm(SHORT_RPM);
	mec_GyroTurnAbs(-10);
	mec_driveInches(-30);
	mec_GyroTurnAbs(130);
	for(int i = 0; i<6; i++){
		ink_waitUntilFire(9000);
	}
	mec_GyroTurnAbs(30);

	mec_driveInches(-5,30,2000);
}


void auto_rout_skills(){

	fw_skillSpeed();
	writeDebugStreamLine("running skills");
	///fw_fullCourtSpeed();

	mec_GyroTurnAbs(16);
	writeDebugStreamLine("%f", GyroGetAngle());
	long initTime = nPgmTime;
	wait1Msec(3000);
	while(nPgmTime < initTime + 17000){
		writeDebugStreamLine("%f", GyroGetAngle());
		ink_set(1);
	}
	startTask(intakeControl);

	_intakeController.ballCount = 0;
	writeDebugStreamLine("before 105 %f", GyroGetAngle());
	mec_GyroTurnAbs(105);

	writeDebugStreamLine("before 105 %f", GyroGetAngle());
	mec_driveInchesTwoStage(-38,-30,120,36,99999);
	writeDebugStreamLine("3. %f", GyroGetAngle());


	mec_GyroTurnAbs(75);

	mec_driveInches(-30);

	mec_GyroTurnAbs(0);
	mec_driveInches(-20,120,1000);
	mec_driveInches(8);
	mec_GyroTurnRel(-14);
	stopTask(intakeControl);
	while(true){
		ink_waitUntilFire(100);
	}

//	fw_skillSpeed();
	stopTask(intakeControl);
	while(true){
		ink_fireWhenReady(50);
	}
	//mec_tmpDriveInches(1,0.2,1);


}



void auto_rout_getMidBalls(bool isRed,bool isOut){
		GyroZeroAbs();

		int mod = isRed ? 1 : -1;
		mod = isOut ? mod * 1 : mod * -1;
		setFlyRpm(2330);
		_fly.pred = 50;

		_intakeController.ballCount = 4;
		mec_driveInches(20);

		stopTask(intakeControl);
		for(int i = 0; i<6; i++){
			ink_waitUntilFire(20);
		}

		_intakeController.ballCount = 0;
		setFlyRpm(2290);
		_fly.pred = 50;

		startTask(intakeControl,9);
		if(isRed){
			mec_GyroTurnRel((140) * mod);
		}
		else{
			mec_GyroTurnRel(140 * mod);
		}
		mec_driveInches(-16,35,9999);
		mec_GyroTurnAbs((mod * 3));

		for(int i = 0; i<6; i++){
			ink_waitUntilFire(20);
		}

}

void auto_rout_skillsShort(){
	GyroZeroAbs();
	fw_skillsShortSpeed();
	//mec_GyroTurnRel(3);

	writeDebugStreamLine("%f", GyroGetAngle());
	long initTime = nPgmTime;
	wait1Msec(3000);
	while(nPgmTime < initTime + 15000){
		ink_set(127);
		wait1Msec(20);
	}
	ink_set(0);
	mec_GyroTurnAbs(90);
	startTask(intakeControl);
	mec_driveInches(10,100,1000);
	mec_driveInches(-127,100,5500);
	mec_driveInches(5);
	mec_GyroTurnRel(-93);

	stopTask(intakeControl);
	while(true){
		ink_set(127);
		wait1Msec(20):
	}
	//mec_tmpDriveInches(1,0.2,1);


}




void auto_facingnetOutsideAuto(bool isRed){

	//fw_skillSpeed();

	int flip = isRed ? -1 : 1;
	startTask(intakeControl);
	mec_driveInches(12);
	mec_GyroTurnRel(130 * flip);
	mec_driveInches(-28);

//	setFlyRpm(2230);
//	_fly.pred = 60;
	mec_GyroTurnRel(-90 * flip);
	mec_driveInches(-15,25,1800);
	mec_driveInches(5,50,500);
	mec_driveInches(-15,120,1000);
	mec_driveInches(5,50,1000);
	mec_GyroTurnAbs(6 * flip);

	stopTask(intakeControl);
	_intakeController.ballCount = 4;
	while(true){
		ink_fireWhenReady(50);
		wait1Msec(20);
	}
}


//initial rel in line with corner
void auto_rout_facingStack(bool isRed){

	int flip = isRed ? -1 : 1;

	_intakeController.ballCount = 4;


	setFlyRpm(2210);
	_fly.pred = 68;

	mec_GyroTurnAbs(8.2 * flip);
	mec_driveInches(33.5);

	startTask(intakeControl);
	mec_GyroTurnAbs((90 - 30) * flip);
	for(int i = 0; i<4; i++){
		ink_waitUntilFire(20);
	}
	setFlyRpm(2230);
	_fly.pred = 68;
	_intakeController.ballCount = 0;
	mec_GyroTurnAbs((90) * flip);
	mec_driveInches(-20,35,1800);
	mec_driveInches(10,35,500);
	mec_driveInches(-10,35,1000);
	mec_driveInches(5);
	mec_GyroTurnAbs((90 - 30) * flip);
	stopTask(intakeControl);
	_intakeController.ballCount = 4;
	while(true){
		ink_waitUntilFire(20);
	}
	_intakeController.ballCount = 0;
}

void auto_rout_midShootTwo(bool isRed){
		setFlyRpm(2210);
		_fly.pred = 50;
		startTask(intakeControl);
		int mod = isRed ? -1 : 1;

		mec_driveInchesTwoStage(-45,-30,100,30,9999);
		mec_GyroTurnRel(140 * mod);

		for(int i = 0; i<4; i++){
			ink_waitUntilFire(300);
		}

		mec_GyroTurnRel(-40 * mod);
		mec_driveInches(-20,30,2000);
		mec_driveInches(10);
		mec_GyroTurnRel(40 * mod);
}



void autoTesting(){
	mec_driveInches(20);
	mec_driveInches(-20);
	mec_driveInches(10);
	mec_driveInches(10);
	mec_driveInches(-10);
	mec_driveInches(-10);

}


#endif
