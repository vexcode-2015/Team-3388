
#ifndef FlyControl.c
#define FlyControl.c

#define LONG_RPM  		3040
#define LONG_PRED  		75
#define FW_LOOP_SPEED	30


#include "Utils.c"
#include "PIDController.h"
#include "MecDrive.c"
typedef struct {
	tMotor f1;
	tMotor f2;
	tSensors enc;
	tSensors sonar;
	PID flyPID;
} fw_motors;




fw_motors _fly;

void sensorSpeedGen(){

}

void distanceToRpm(){

}


static double launcher_data[] = {
	5.0229428804899,
	5.0435769408536,
	5.0906590677139,
	5.1520620357203,
	5.2114515169648,
	5.2889577101548,
	5.3709554894395,
	5.4560471879482,
	5.543248013936,
	5.6318501823473,
	5.7213367599409,
	5.8228468519621,
	5.9015299760192,
	6.0032378159464,
	6.0817789596611,
	6.1830196594275,
	6.2723878860324,
	6.3613051292507,
	6.4611724459771,
	6.5375858836638,
	6.6363162117154,
	6.7344334366707,
	6.8090882197654,
	6.9059605491874,
	7.0021921861554,
	7.0863911309814,
	7.1699670124247,
	7.2642959719632,
	7.3466279787682,
	7.4283508543565,
	7.520818743353,
	7.6126753002962,
	7.7039290827876,
	7.7832671891817,
	7.8620373471777,
	7.9515542241805,
	8.0405055415951,
	8.1289009571972
};
//distance in meters
double get_required_rpm(double distance) {
	int index = (distance - 1.2192) / 0.1016;

	if(index >= 0 && index < 38) {
		// v = r * RPM * 0.10472
		return launcher_data[index] / (12.7 * 0.10472);
	}
	return -1;
}

float getRpmFromDrive(){

}


void _updateFlyWheelLin(int power){
	if(power > 127){
		power = 127;
	}
	setLinMotorPow(_fly.f1, power);
	setLinMotorPow(_fly.f2, power);
	//setLinMotorPow(_fly.f3, power);
	//setLinMotorPow(_fly.f4, power);
}
void _updateFlyWheel(int power){
	/**
	motor[_fly.f1] = power;
	motor[_fly.f2] = power;
	motor[_fly.f3] = power;
	motor[_fly.f4] = power; **/
	_updateFlyWheelLin(power);
}


long						nSysTime_last;
long						encoder_counts;					///< current encoder count
long						encoder_counts_last;		///< current encoder count
float FwCalculateSpeed()
{
	int			delta_ms;
	int			delta_enc;
	encoder_counts = -SensorValue[_fly.enc] ;
	delta_ms = nSysTime - nSysTime_last;
	nSysTime_last = nSysTime;
	delta_enc = (encoder_counts - encoder_counts_last);
	encoder_counts_last = encoder_counts;
	return (1000.0 / delta_ms) * delta_enc;
}

float curr;
int _setRPM;
float error;
float coeff = 0.002;//0.002;//0.007;//0.0015500;//0.250000;
float Y = 0;
int predictedVal = 0;
bool firstCross = false;

void setFlyWheel(int rpm, int pred){
	_setRPM = rpm;
	firstCross = true;
	predictedVal = pred;
}



//use take back half to spin us up
float spinUp(int rpm){
	_setRPM = rpm;
	float tbh = 0;
	double prevError = 0;
	float errorOT = 0;
	long steadyTimer = nPgmTime;
	while(true){
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
		if(abs(error) > 100){
			steadyTimer = nPgmTime;
		}
		if((nPgmTime - steadyTimer) > 1000){
			return Y;
		}
	}
}


void spinFlyWheelAuto(){
	//_setFlyWheel(get_required_rpm(getNetDistance()), 80);
	//spinUp(get_required_rpm(getNetDistance()));
}



task PIDFlyControl(){
float errorOT = 0;
float setPoint = 0;
	pidReset(_fly.flyPID);
	pidInit(_fly.flyPID, 0.05, 0, 0.006, 100, 9999);
	float output = 0;
	while(true){
		if(vexRT[Btn8D]){
			setPoint = 0;
			pidReset(_fly.flyPID);
		}
		if(vexRT[Btn8L]){
			setPoint = 1980;
			pidReset(_fly.flyPID);
		}
		if(vexRT[Btn8R]){
			setPoint = 2450;
			pidReset(_fly.flyPID);
		}
		if(vexRT[Btn8U]){
			setFlyWheel(LONG_RPM,LONG_PRED);
			//setPoint = spinUp(LONG_RPM);
			pidReset(_fly.flyPID);
			setPoint = LONG_RPM;
			writeDebugStreamLine("Engaged PID");
		}
		if(vexRT[Btn7R]){
			spinFlyWheelAuto();
			pidReset(_fly.flyPID);
			writeDebugStreamLine("Engaged PID");

		}
		curr = FwCalculateSpeed();
		float outVal = pidExecute(_fly.flyPID, setPoint - curr);

		if(abs(_setRPM - curr) < 100){
			statusLightSet(1);
		}
		else{
			statusLightSet(0);
		}
	// writeDebugStreamLine("OUTVAL = %f", outVal);
		output += (outVal);
		if(output < 0){
			output = 0;
		}
		if(output > 127){
			output = 127;
		}
		_updateFlyWheel(output);

		delay(FW_LOOP_SPEED);
	}
}


task FlyWheelControl(){
	float tbh = 0;
	double prevError = 0;

	float errorOT = 0;
	while(true){
		if(vexRT[Btn8D]){
			setFlyWheel(0,0);
		}
		if(vexRT[Btn8L]){
			setFlyWheel(2060,40);
		}
		if(vexRT[Btn8R]){
			setFlyWheel(2550,60);
		}
		if(vexRT[Btn8U]){
			setFlyWheel(LONG_RPM,LONG_PRED);
		}

		curr = FwCalculateSpeed();
		error = _setRPM - curr;

		Y += coeff*error;
		if (Y>127) Y=127; else if (Y<0) Y=0;
			if (sgn(error)!=sgn(prevError)){
			if(firstCross){
				Y = predictedVal;
				firstCross = false;
				tbh = Y;
			}// if zero crossing,
			else{
				Y = 0.5*(Y+tbh);
				tbh = Y;
			}
		}
		prevError = error;
		_updateFlyWheel(Y);
		wait1Msec(FW_LOOP_SPEED );
	}
}

void initFlyWheel(fw_motors* initMotors){
	_fly = *initMotors;
	_fly.f1 = initMotors->f1;
	_fly.f2 = initMotors->f2;
	_fly.enc = initMotors->enc;
	pidInit(_fly.flyPID, 0.15, 0, 0.001, 100, 9999);
	//writeDebugStreamLine("FLY %d %d %d %d", _fly.f1,_fly.f2, _fly.f3, _fly.f4);
	//startTask(FlyWheelControl);
	startTask(PIDFlyControl, 20);
}

#endif
