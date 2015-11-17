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

void printPIDDebug(PID &pid){
	writeDebugStreamLine("Error %f, Error sum %f, output %f, lastoutput %f ",
		 pid.error, pid.errorSum, pid.output, pid.lastOutput);
}


void pidInit(PID &pid, float kP, float kI, float kD, float epsilon, float slewRate){
	pid.kP = kP;
	pid.kI = kI;
	pid.kD = kD;
	pid.epsilon = epsilon;
	pid.slewRate = slewRate;

}

void pidFilteredOut(PID &pid){
	float filteredOut = pid.output;
	if(pid.dT != 0)
	{
		if(abs(pid.output - pid.lastOutput)/pid.dT > pid.slewRate)
			filteredOut = pid.lastOutput + pid.slewRate*pid.dT * (pid.output/abs(pid.output));
		else
			filteredOut = pid.output;
	}
	if(abs(filteredOut) > 127)
		filteredOut = 127 * filteredOut/abs(filteredOut);

	pid.lastOutput = filteredOut;
	return filteredOut;
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


void pidReset(PID &pid)
{
	pid.errorSum = 0;
	pid.dT = 0;
	pid.lastOutput = 0;
	pid.lastError = 0;
}

#endif
