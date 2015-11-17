
#ifndef FlyControl.c
#define FlyControl.c

#define LONG_RPM  		2970
#define LONG_PRED  		72
#define FW_LOOP_SPEED	30


#include "Utils.c"

typedef struct {
	tMotor f1;
	tMotor f2;
	tMotor f3;
	tMotor f4;
	tSensors enc;
	tSensors sonar;
	PID flyPID;
} fw_motors;


fw_motors _fly;

void sensorSpeedGen(){
	return distanceToRpm(SensorValue[_fly.sonar]);
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

void spinFlyWheelAuto(){
	setFlyWheel(get_required_rpm(getNetDistance()), 80);
	spinUp(get_required_rpm(getNetDistance()));
}


void _updateFlyWheel(int power){
	/**
	motor[_fly.f1] = power;
	motor[_fly.f2] = power;
	motor[_fly.f3] = power;
	motor[_fly.f4] = power; **/
	_updateFlyWheelLin(power);
}

void _updateFlyWheelLin(){
	setMotorPow(_fly.f1, power);
	setMotorPow(_fly.f2, power);
	setMotorPow(_fly.f3, power);
	setMotorPow(_fly.f4, power);
}

long						nSysTime_last;
long						encoder_counts;					///< current encoder count
long						encoder_counts_last;		///< current encoder count
float FwCalculateSpeed()
{
	int			delta_ms;
	int			delta_enc;
	encoder_counts = SensorValue[_fly.enc] ;
	delta_ms = nSysTime - nSysTime_last;
	nSysTime_last = nSysTime;
	delta_enc = (encoder_counts - encoder_counts_last);
	encoder_counts_last = encoder_counts;
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
		if((nPgmTime - steadyTimer) > 200){
			return Y;
		}
	}
}

task PIDFlyControl(){
float errorOT = 0;
float setPoint = 0;
	while(true){
		if(vexRT[Btn8D]){
			setFlyWheel(0,0);
			setPoint = spinUp(0);
			pidReset(fly.flyPID);
		}
		if(vexRT[Btn8L]){
			setFlyWheel(1050,30);
			setPoint = spinUp(1050);
			pidReset(fly.flyPID);
		}
		if(vexRT[Btn8R]){
			setFlyWheel(1050,40);
			setPoint = spinUp(1050);
			pidReset(fly.flyPID);
		}
		if(vexRT[Btn8U]){
			setFlyWheel(LONG_RPM,LONG_PRED);
			setPoint = spinUp(LONG_RPM);
			pidReset(fly.flyPID);
			writeDebugStreamLine("Engaged PID");
		}
		if(vexRT[Btn7R]){
			spinFlyWheelAuto();
			pidReset(fly.flyPID);
			writeDebugStreamLine("Engaged PID");
		}
		curr = FwCalculateSpeed();
		float outVal = pidExecute(fly.flyPID, _setRPM - curr);
		if(outVal < 0){
			outVal = 0;
		}
		_updateFlyWheel(Y + outVal);
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
	pidInit(_fly.pid,0.02,0,0.01,100,1270);
//	writeDebugStreamLine("FLY %d %d %d %d", _fly.f1,_fly.f2, _fly.f3, _fly.f4);
	//startTask(FlyWheelControl);
	startTask(PIDFlyControl);
}

#endif