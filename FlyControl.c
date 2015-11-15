
#ifndef FlyControl.c
#define FlyControl.c

#define LONG_RPM  2970
#define LONG_PRED  72
// Update inteval (in mS) for the flywheel control loop
#define FW_LOOP_SPEED		30

//typedef struct _fw_controller {
// int a;
//} fw_controller;

typedef struct {
	tMotor f1;
	tMotor f2;
	tMotor f3;
	tMotor f4;
	tSensors enc;
} fw_motors;


fw_motors _fly;
void _updateFlyWheel(int power){
	motor[_fly.f1] = power;
	motor[_fly.f2] = power;
	motor[_fly.f3] = power;
	motor[_fly.f4] = power;
}


// velocity measurement
long						nSysTime_last;					///< Time of last velocity calculation

// Encoder
long						encoder_counts;					///< current encoder count
long						encoder_counts_last;		///< current encoder count

float
FwCalculateSpeed()
{
	int			delta_ms;
	int			delta_enc;

	// Get current encoder value
	encoder_counts = SensorValue[_fly.enc] ;

	// This is just used so we don't need to know how often we are called
	// how many mS since we were last here
	delta_ms = nSysTime - nSysTime_last;
	nSysTime_last = nSysTime;

	// Change in encoder count
	delta_enc = (encoder_counts - encoder_counts_last);

	// save last position
	encoder_counts_last = encoder_counts;

	// Calculate velocity in rpm
	return (1000.0 / delta_ms) * delta_enc ;
}





float curr;
int _setRPM;
float error;
float coeff = 0.007;//0.002;//0.007;//0.0015500;//0.250000;
float Y = 0;
int predictedVal = 0;
bool firstCross = false;

void setFlyWheel(int rpm, int pred){
			_setRPM = rpm;
			firstCross = true;
			predictedVal = pred;
}

		float p = 0.05;

task PIDFlyControl(){
float errorOT = 0;
	while(true){
		if(vexRT[Btn8D]){
			setFlyWheel(0,0);
		}
		if(vexRT[Btn8L]){
			setFlyWheel(860,30);
		}
		if(vexRT[Btn8R]){
			setFlyWheel(1050,40);
		}
		if(vexRT[Btn8U]){
			setFlyWheel(LONG_RPM,LONG_PRED);
		}

		curr = FwCalculateSpeed();
		error = _setRPM - curr;
		int prevError = error;

		//	writeDebugStreamLine("POW %d",Y);

		float i = 0;//0.003;
		float d = 0.008;
		errorOT += error * FW_LOOP_SPEED;
		float errorD = (error - prevError) / FW_LOOP_SPEED;
		if (sgn(error)!=sgn(prevError)){
			errorOT = 0;
		}
		int pidResult = p * error + i * errorOT + errorD * d;
		if(pidResult < 0){
			_updateFlyWheel(0);
		}
		else{
			_updateFlyWheel(predictedVal + p * error + i * errorOT + errorD * d);
		}
				wait1Msec(FW_LOOP_SPEED);
					writeDebugStreamLine("%f, coeff %f  %f SETTLE TIME %f",error, coeff, Y, nSysTime);
	}
}


task
FlyWheelControl(){
	float tbh = 0;
	double prevError = 0;

	float errorOT = 0;
	while(true){
		if(vexRT[Btn8D]){
			setFlyWheel(0,0);
		}
		if(vexRT[Btn8L]){
			setFlyWheel(860,30);
		}
		if(vexRT[Btn8R]){
			setFlyWheel(1050,40);
		}
		if(vexRT[Btn8U]){
			setFlyWheel(LONG_RPM,LONG_PRED);
		}

		curr = FwCalculateSpeed();
		error = _setRPM - curr;

		Y += coeff*error;														// integrate the output;
		if (Y>127) Y=127; else if (Y<0) Y=0;		// clamp the output to 0..+1;
			if (sgn(error)!=sgn(prevError)){
			if(firstCross){
				Y = predictedVal;
				firstCross = false;
				tbh = Y;
			}// if zero crossing,
			else{
				Y = 0.5*(Y+tbh);								// then Take Back Half
				tbh = Y;
			}
		}
		prevError = error;
		_updateFlyWheel(Y);
		wait1Msec(FW_LOOP_SPEED);


		//	writeDebugStreamLine("POW %d",Y);
	/**
		float p = 0.4;
		float i = 0;//0.003;
		float d = 0;
		errorOT += error * FW_LOOP_SPEED;
		float errorD = (error - prevError) / FW_LOOP_SPEED;
		if (sgn(error)!=sgn(prevError)){
			errorOT = 0;
		}
		int pidResult = p * error + i * errorOT + errorD * d;
		if(pidResult < 0){
			_updateFlyWheel(0);
		}
		else{
			_updateFlyWheel(p * error + i * errorOT + errorD * d);
		}
		**/
	}
}

void initFlyWheel(fw_motors* initMotors){
	_fly = *initMotors;
	_fly.f1 = initMotors->f1;
	_fly.f2 = initMotors->f2;
	_fly.f3 = initMotors->f3;
	_fly.f4 = initMotors->f4;
	_fly.enc = initMotors->enc;
//	writeDebugStreamLine("FLY %d %d %d %d", _fly.f1,_fly.f2, _fly.f3, _fly.f4);
	//startTask(FlyWheelControl);
	startTask(PIDFlyControl);
}

#endif
