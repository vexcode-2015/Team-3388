#ifndef Auto.c
#define Auto.c

#include "MecDrive.c"
#include "IntakeControl.c"


void auto_rout_skills(){

	fw_skillSpeed();
	writeDebugStreamLine("running skills");
	///fw_fullCourtSpeed();

	mec_GyroTurnAbs(16);
	writeDebugStreamLine("%f", GyroGetAngle());
	long initTime = nPgmTime;
	while(nPgmTime < initTime + 22000){
		writeDebugStreamLine("%f", GyroGetAngle());
		ink_waitUntilFire(100);
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
	mec_driveInches(8);
	fw_skillsShortSpeed();
	mec_GyroTurnAbs(7);

	writeDebugStreamLine("%f", GyroGetAngle());
	long initTime = nPgmTime;
	while(nPgmTime < initTime + 15000){
		ink_waitUntilFire(100);
		wait1Msec(20);
	}
	mec_GyroTurnAbs(0);
	mec_driveInches(-20);
	mec_GyroTurnAbs(90);

	startTask(intakeControl);
	mec_driveInchesTwoStage(-130,-30,100,60,5999);
	mec_driveInches(7);
	mec_GyroTurnAbs(0);
	mec_driveInches(20);
	mec_GyroTurnRel(-7);

	stopTask(intakeControl);
	while(true){
		ink_fireWhenReady(100);
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
