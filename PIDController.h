#ifndef "PIDController.h"
#define "PIDController.h"

//Borrowing heavily from QCC2's PID code 

const short PID_LOOP_DELAY = 10;
typedef struct {
	float kP, kI, kD;
	int error, lastError;
	float errorSum;
	float output, lastOutput;
} PID;

void pidInit(PID &pid, float kP, float kI, float kD, float epsilon, float slewRate){
	pid.kP = kP;
	pid.kI = kI;
	pid.kD = kD;
	pid.epsilon = epsilon;

}

void pidFilteredOut(PID &pid){

}


float pidExecute(PID &pid, float error){
	pid.lastError = pid.error;
	pid.error = error;

	pid.dT = (nPgmTime - pid.timer)/1000; //delta time in seconds
	pid.timer = nPgmTime;

	delay(10);

	float rate;
	if(abs(pid.dT) > 0){
		rate = (pid.error - pid.lastError)/pid.dT;
	}
	else{
		rate = 0;
	}

	if(abs(error) > pid.epsilon){
		pid.errorSum += error*pid.dT;
	}

	pid.output = error * pid.kP 
	+ pid.errorSum * pid.kI
	+ rate * pid.kD;

	return pidFilteredOutput(pid);
}


#endif
