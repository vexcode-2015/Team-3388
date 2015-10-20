#ifndef MecDrive.c
#define MecDrive.c
#define JOYSTICK_DEADZONE  10

 typedef struct {
	int fl;
	int fr;
	int bl;
	int br;
} DriveBase ;

DriveBase mec;
void initMecDrive( DriveBase* db){
	mec = *db;
}

void mecDrive(){
	float x;
	float y;
	float speed;
	float turnSpeed;
	if(abs(vexRT[Ch4]) > JOYSTICK_DEADZONE){
		x = vexRT[Ch4];
	}
	if(abs(vexRT[Ch3]) > JOYSTICK_DEADZONE){
		y = vexRT[Ch3];
	}
	if(abs(vexRT[Ch2]) > JOYSTICK_DEADZONE){
	 	speed = vexRT[Ch2]/127;
	}
	if(abs(vexRT[Ch1]) > JOYSTICK_DEADZONE){
		turnSpeed = vexRT[Ch1]/127;
	}

	float heading = atan(y/x);

	motor[mec.fl] = 127 * (speed * sin(heading + (PI/4))) + turnSpeed;
	motor[mec.fr] = 127 * (speed * cos(heading + (PI/4))) - turnSpeed;
	motor[mec.bl] = 127 * (speed * cos(heading + (PI/4))) + turnSpeed;
	motor[mec.br] = 127 * (speed * sin(heading + (PI/4))) - turnSpeed;
}
#endif
