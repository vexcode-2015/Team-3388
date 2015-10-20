

#ifndef MecDrive.c
#define MecDrive.c
#define JOYSTICK_DEADZONE  10

typedef struct DriveBase{
	tMotor* fl;
	tMotor* fr;
	tMotor* bl;
	tMotor* br;
} DriveBase;

DriveBase mec;
void initMec( DriveBase* db){
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
	if(abs(vexRT[Ch2]) > JOYSTICK_DEADZONE){
		turnSpeed = vexRT[Ch1]/127;
	}

	float heading = atan(y/x);

	*mec.fl = 127 * (speed * sin(heading + (PI/4)) + turnSpeed;
	*mec.fr = 127 * (speed * cos(heading + (PI/4)) - turnSpeed;
	*mec.bl = 127 * (speed * cos(heading + (PI/4)) + turnSpeed;
	*mec.br = 127 * (speed * sin(heading + (PI/4)) - turnSpeed;
}
#endif