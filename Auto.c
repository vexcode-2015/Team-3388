#ifndef Auto.c
#define Auto.c

void auto_facingnetOutsideAuto(){

	//fw_skillSpeed();


	startTask(intakeControl);
	mec_driveInches(12);
	mec_GyroTurnRel(130);
	mec_driveInches(-28);

	setFlyRpm(2230);
	_fly.pred = 60;
	mec_GyroTurnRel(-90);
	mec_driveInches(-15,25,1800);
	mec_driveInches(5,50,500);
	mec_driveInches(-15,90,1000);
	mec_driveInches(5,50,1000);
	mec_GyroTurnAbs(6);

	stopTask(intakeControl);
	_intakeController.ballCount = 4;
	while(true){
		ink_fireWhenReady(50);
	}
}


//initial rel in line with corner
void auto_rout_facingStack(){

	//fw_skillSpeed();



	setFlyRpm(2280);
	_fly.pred = 68;
	mec_GyroTurnRel(15);
	mec_driveInches(33.5);
	int shootAngle = 60;
	mec_GyroTurnAbs(shootAngle);
	//shoot initial 4
	for(int i = 0; i<4; i++){
		ink_waitUntilFire(50);
	}
	startTask(intakeControl);
	mec_GyroTurnAbs(90);
	mec_driveInches(-20,20,3000);
	mec_driveInches(5,20,700);
	mec_driveInches(-5,20,700);
	mec_driveInches(5);
	mec_GyroTurnAbs(90);
	mec_driveInches(12);
	mec_GyroTurnAbs(shootAngle);

	stopTask(intakeControl);
	_intakeController.ballCount = 4;
	while(true){
		ink_fireWhenReady(50);
	}
}
#endif
