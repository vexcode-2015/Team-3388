#ifndef Auto.c
#define Auto.c

#include "MecDrive.c"
#include "IntakeControl.c"
#include "FlyControl.c"

void auto_rout_test(){
	mec_driveInches(20,100,3000);
	mec_driveInches(-20,100,3000);
}

void runshot(){
	fw_shortSpeed();
	ink_startRunningShot(45);
	mec_driveInches(120,120,5000);
	ink_stopRunningShot();
	ink_set(127);
}

//working, can be run from any tile
void auto_rout_shoot4(bool isRed){
	fw_fullCourtSpeed();
	int mod = isRed ? 1 : -1;
	stopTask(intakeControl);
	ink_set(0);
	for(int i = 0; i<6; i++){
		ink_waitUntilFire(30);
	}
	fw_shortSpeed();
	ink_startTask(0);
	mec_driveInches(80,120,3000);
	mec_GyroTurnRel(-190 * mod);
	mec_driveInches(-60, 120,2000);
	mec_driveInches(10,120,1500);
	mec_GyroTurnRel(180);
	ink_set(127);
}

//working, run from the inside tile
void auto_rout_insideChallengeMid(bool isRed, bool shoot){
	int mod = 1;
	if(isRed){
		mod = -1;
	}
	ink_set(0);
	ink_startTask(0);
	fw_midSpeed();
	mec_driveInches(-25,105,5000);
	mec_driveInches(-16,105,5000);
	mec_GyroTurnRel(180 * mod);
	stopTask(intakeControl);
	ink_set(-127);


	mec_driveInches(-4.0,90,2000);
	wait1Msec(300);
	mec_GyroTurnRel(-60 * mod);
	ink_set(0);
	ink_startTask(0);
	mec_driveInches(-8,80,2000);
	mec_driveInches(-20,30,1500);
	mec_driveInches(57,110,4000);
	mec_GyroTurnRel(45 * mod);
	stopTask(intakeControl);
	if(shoot){
		ink_set(127);
		wait1Msec(1000);
	}
	ink_set(-127);
	mec_driveInches(-30,120,3000);
	mec_driveInches(80,127,3000);
}

//OUTSIDE AUTOS
void auto_rout_outsideRunShot(bool isRed){
	int mod = isRed ? 1 : -1;
	fw_shortSpeed();
	setFlyRpm(SHORT_RPM + 90, SHORT_POW + 5);
	ink_startRunningShot(45);
	mec_driveInches(100,100,4000);
	ink_stopRunningShot();
	ink_set(127);
	wait1Msec(700);
	ink_set(0);
	ink_startTask(0);
	mec_GyroTurnRel(-90 * mod);
	mec_driveInches(-28,100,2000);
	mec_GyroTurnRel(80 * mod);
	stopTask(intakeControl);
	ink_set(-127);
	mec_driveInches(-95,100,3000);
	mec_GyroTurnRel(45 * mod);

	//mec_driveInches(80,80,3000);
}

void auto_rout_outsideShoot(bool isRed){
	setFlyRpm(SHORT_RPM + 300, SHORT_POW + 7);
	ink_startTask(0);
	mec_driveInches(-55,120,5000);
	mec_driveInches(-20,120,5000);
	mec_GyroTurnRel(180 + 30);
	stopTask(intakeControl);
	ink_set(127);
	wait1Msec(1000);
	setFlyRpm(SHORT_RPM + 100, SHORT_POW + 5);
	ink_startTask(0);
	mec_GyroTurnRel(180 - 15);
	mec_driveInches(-20,60,3000);
	mec_GyroTurnRel(180 + 15);
	stopTask(intakeControl);
	ink_set(127);
	wait1Msec(1200);
	mec_GyroTurnRel(90 + 15);
	mec_driveInches(-20,30,2000);
	mec_driveInches(10,90,1000);
}

//positive = turn right
void auto_rout_outsideHerdMid(bool isRed){
	int mod = 1;
	if(isRed){
		mod = -1;
	}
	mec_driveInches(-115,120,5000);
	mec_GyroTurnRel(-90 * mod);
	mec_driveInches(-8,127,3000);
	mec_GyroTurnRel(-80 * mod);
	ink_set(-127);
	mec_driveInches(-100,127,4000);
	mec_GyroTurnRel(-45 * mod);
	mec_driveInches(-10,127,2000):
	mec_GyroTurnRel(90 * mod);
}

//charges middle stacks
void auto_rout_challengeMidOutside(bool isRed){
	int mod = isRed ? 1 : -1;
	setFlyRpm(1800,50);
	ink_set(127);
	mec_driveInches(-70,127,5000);
	ink_set(0);
	setFlyRpm(SHORT_RPM + 100, SHORT_POW + 10);
	ink_startTask(0);
	mec_driveInches(-30,120,5000);
	mec_GyroTurnRel(180 - 33 * mod);
	stopTask(intakeControl);
	ink_set(127);
	wait1Msec(1000);
	ink_set(0);
	ink_startTask(0);
	mec_GyroTurnRel(-110 * mod);
	mec_driveInches(-15,127,3000);
	mec_driveInches(-10,30,1500);
	mec_driveInches(8,127,3000);

	mec_GyroTurnRel(110 * mod);
	stopTask(intakeControl);
	ink_set(127);
	wait1Msec(800);
	ink_startTask(0);
	mec_GyroTurnRel(90 * mod);
	mec_driveInches(-20,80,3000);
	mec_driveInches(-10,100,2000);
	mec_GyroTurnRel(-45 * mod);
	stopTask(intakeControl);
	ink_set(127);
	wait1Msec(800);
}

void auto_rout_outsideHerdMidShoot(bool isRed){
	int mod = 1;
	if(isRed){
		mod = -1;
	}
	fw_shortSpeed();
	ink_startTask(0);
	if(!isRed){
		mec_driveInches(-90,115,7000,0.97);
	} else{
		mec_driveInches(-90,115,7000,1.03);
	}
	setFlyRpm(SHORT_RPM + 60, SHORT_POW + 5);
	mec_driveInches(-22,60,3000);
	mec_GyroTurnRel(-110 * mod);
	mec_driveInches(-25,127,2000);
	mec_GyroTurnRel(-60 * mod);
	stopTask(intakeControl);
	ink_set(127);
	mec_driveInches(-20,40,2000);
	wait1Msec(800);
	ink_set(0);
	mec_GyroTurnRel(-30 * mod);

	ink_set(-127);
	mec_driveInches(-80,110,4000);
	mec_GyroTurnRel(-90 * mod);
	mec_driveInches(-5,127,500);
}

//SKILLS
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
}

#endif
