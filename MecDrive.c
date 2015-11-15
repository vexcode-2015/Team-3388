#ifndef MecDrive.c
#define MecDrive.c
#define JOYSTICK_DEADZONE  10
#include "Utils.c"
typedef struct {
	tMotor fl;
	tMotor fr;
	tMotor bl;
	tMotor br;
} DriveBase ;

DriveBase mec;
void _mecDrive(){

		int x1 = vexRT[Ch4];
		int y1 = vexRT[Ch3];
		int x2 = vexRT[Ch1];
		int y2 = vexRT[Ch2];
		int deadzone = 10;
		x1 = threshold(x1,deadzone);
		y1 = threshold(y1,deadzone);
		x2 = threshold(x2,deadzone);
		y2 = threshold(y2,deadzone);
	//	writeDebugStreamLine("%f", x1);
		if(y2!=0){y2 = (y2 * y2 * (y2/abs(y2))/(127);}
		if(x1!=0){x1 = (x1 * x1 * (x1/abs(x1))/127;}
		if(abs(y1) > 15){
				y1 = y1 > 0 ? 40 : -40;
		}
	//	if(x2!=0){x2 = (x2 * (x2/abs(x2))/127;}
	//	writeDebugStreamLine("%f", x1);
	//float heading = atan(y/x);
	motor[mec.fr] = y2 - x1 - x2 - y1;
	motor[mec.br] = y2 - x1 + x2 - y1;
	motor[mec.fl] = y2 + x1 + x2 + y1;
	motor[mec.bl] = y2 + x1 - x2 + y1;

}



void initMecDrive( DriveBase db){
	mec.fl = db.fl;
	mec.fr = db.fr;
	mec.bl = db.bl;
	mec.br = db.br;
//	writeDebugStreamLine("PRINTING %d %d %d %d", db.fl, db.fr, db.bl, db.br);
//	startTask(_mecDrive);
}

void stopMecDrive(){
//	stopTask(_mecDrive);
}

#endif
