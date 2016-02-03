#ifndef Auto.c
#define Auto.c

#include "MecDrive.c"
#include "IntakeControl.c"

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
	}
}


//initial rel in line with corner
void auto_rout_facingStack(bool isRed){

	int flip = isRed ? -1 : 1;

	_intakeController.ballCount = 4;


	setFlyRpm(2170);
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


void autoTesting(){
	mec_driveInches(20);
	mec_driveInches(-20);
	mec_driveInches(10);
	mec_driveInches(10);
	mec_driveInches(-10);
	mec_driveInches(-10);

}


#endif
