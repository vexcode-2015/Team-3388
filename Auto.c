#ifndef Auto.c
#define Auto.c

#include "MecDrive.c"
#include "IntakeControl.c"
#include "FlyControl.c"

void auto_rout_test(){
	while(true){
	mec_GyroTurnRel(10);
	wait1Msec(2000);
	mec_GyroTurnRel(180);

	wait1Msec(2000);
	mec_GyroTurnRel(45);

	wait1Msec(2000);
	}

}


void auto_rout_shoot4(bool isRed){
	fw_fullCourtSpeed();
	for(int i = 0; i<6; i++){
		ink_waitUntilFire(30);
	}
	mec_GyroTurnRel(-180);
	mec_driveInches(-40,120,3000);
}



void auto_rout_challengeCornerStack(bool isRed){
	ink_startTask(0);
	mec_driveInches(-35,110,5000);
	mec_GyroTurnRel(-50);
	mec_driveInches(-35,110,5000);
	mec_GyroTurnRel(50 + 180);
	ink_set(-127);
	mec_driveInches(-40,70,5000);
	mec_GyroTurnRel(45);
	mec_driveInches(-5,100,1500);
	mec_GyroTurnRel(90);
	mec_driveInches(45,120,5000);
}


void auto_rout_insideChallengeMid(bool isRed, bool shoot){
	int mod = 1;
	if(isRed){
		mod = -1;
	}
	ink_startTask(0);
	fw_midSpeed();
	mec_driveInches(-20,100,5000);
	mec_driveInches(-20,50,5000);
	mec_GyroTurnRel(180 * mod);
	stopTask(intakeControl);
	ink_set(-127);
	mec_driveInches(-5,110,2000);
	mec_GyroTurnRel(-70 * mod);
	ink_startTask(0);
	mec_driveInches(-11,110,4000);
	mec_driveInches(-20,30,1300);
//	mec_driveInches(5,100,500);
//	mec_driveInches(-30,60,700);
	mec_driveInches(60,90,4000);
	mec_GyroTurnRel(46 * mod);
	stopTask(intakeControl);
	if(shoot){
	ink_set(127);
	wait1Msec(1500);
	}
	ink_set(-127);
	mec_driveInches(-30,120,3000);
	mec_driveInches(80,127,3000);
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


void auto_rout_outsideHerdMid(bool isRed){
	int mod = 1;
	if(isRed){
		mod = -1;
	}
	mec_driveInches(-100,100,5000);
	mec_GyroTurnRel(-90 * mod);
	mec_driveInches(-15,100,3000);
	mec_GyroTurnRel(-90 * mod);
	ink_set(-127);
	mec_driveInches(-70,100,5000);
	mec_GyroTurnRel(-90 * mod);
	mec_driveInches(-10,100,2000):
	mec_GyroTurnRel(90 * mod);

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


void auto_rout_otusideHerdGrabStack(bool isRed){
	fw_shortSpeed();
	startTask(intakeControl);
	_intakeController.ballCount = 0;
	mec_driveInches(-124,127,4000);
	mec_driveInches(7,80,1000);
	mec_GyroTurnRel(180);
	stopTask(intakeControl);
	ink_set(127);
	wait1Msec(800);
	mec_GyroTurnRel(90);
	mec_driveInches(-28,110,2000);

	mec_GyroTurnRel(-85);
	ink_set(-127);
	mec_driveInches(-90,110,3500);
	mec_GyroTurnRel(-60);
	mec_driveInches(-10,127,1000);
}


void auto_rout_outsideHerdMidShoot4(bool isRed){
	fw_shortSpeed();
	mec_driveInches(125,127,5000);
	mec_driveInches(-5);
	ink_set(127);
	wait1Msec(800);
	mec_GyroTurnRel(90);
	mec_driveInches(-23,110,2000);

	mec_GyroTurnRel(-90);
	ink_set(-127);
	mec_driveInches(-100,110,5000);
	mec_GyroTurnRel(-90);
	mec_driveInches(-5,127,500);
}


void auto_rout_challengedMiddle(bool isRed){
	int mult = isRed ? -1 : 1;
	fw_shortSpeed();
		ink_set(127);
	mec_driveInches(-100,120,5000);

}



void auto_rout_herdMid(bool isRed){
	mec_driveInches(-40);
	mec_GyroTurnRel(-50);
	startTask(intakeControl);
	mec_driveInches(-25);
	mec_GyroTurnRel(180 + 55);
	stopTask(intakeControl);
	ink_set(-127);
	mec_driveInches(-40,127,1600);
	mec_GyroTurnRel(-70);
	mec_GyroTurnRel(70);
	ink_set(0);
	_intakeController.ballCount = 0;
		startTask(intakeControl);
	mec_driveInches(-30,127,1000);


	mec_driveInches(65);

	mec_GyroTurnRel(-45);
	stopTask(intakeControl);
	ink_set(-127);
	mec_driveInches(-35);
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
