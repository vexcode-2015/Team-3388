

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
	if(abs(x) > JOYSTICK_DEADZONE){
		float x = vexRT[Ch4];
	}
	if(abs(y) > JOYSTICK_DEADZONE){
		float y = vexRT[Ch3];
	}
	float heading = atan(y/x);
	float speed = vexRT[Ch2]/127;
	float turnSpeed = vexRT[Ch3]/127;
	*mec.fl = 127 * (speed * sin(heading + (PI/4)) + turnSpeed;
	*mec.fr = 127 * (speed * cos(heading + (PI/4)) - turnSpeed;
	*mec.bl = 127 * (speed * cos(heading + (PI/4)) + turnSpeed;
	*mec.br = 127 * (speed * sin(heading + (PI/4)) - turnSpeed;
}
#endif