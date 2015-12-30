
#ifndef FlyControl.c
#define FlyControl.c

#define FW_LOOP_SPEED	100


#include "Utils.c"
#include "PIDController.h"
#include "MecDrive.c"


int SKILLS_RPM = 1950; //1950
int SKILLS_POW = 55;
const int SHORT_RPM = 1620;
const int SHORT_POW = 35;
const int MED_RPM = 1950;
const int MED_POW = 45;
const int LONG_RPM  = 2520;//2680;//2950;
const int HIGH_POW = 65;//75;


typedef struct {
	tMotor f1;
	tMotor f2;
	tSensors enc;
	tSensors sonar;
	PID flyPID;
	int setPoint;
	float currSpeed;
	int pred;
} Fw_Controller;
Fw_Controller _fly;


// static double launcher_data[] = {
// 	5.0229428804899,
// 	5.0435769408536,
// 	5.0906590677139,
// 	5.1520620357203,
// 	5.2114515169648,
// 	5.2889577101548,
// 	5.3709554894395,
// 	5.4560471879482,
// 	5.543248013936,
// 	5.6318501823473,
// 	5.7213367599409,
// 	5.8228468519621,
// 	5.9015299760192,
// 	6.0032378159464,
// 	6.0817789596611,
// 	6.1830196594275,
// 	6.2723878860324,
// 	6.3613051292507,
// 	6.4611724459771,
// 	6.5375858836638,
// 	6.6363162117154,
// 	6.7344334366707,
// 	6.8090882197654,
// 	6.9059605491874,
// 	7.0021921861554,
// 	7.0863911309814,
// 	7.1699670124247,
// 	7.2642959719632,
// 	7.3466279787682,
// 	7.4283508543565,
// 	7.520818743353,
// 	7.6126753002962,
// 	7.7039290827876,
// 	7.7832671891817,
// 	7.8620373471777,
// 	7.9515542241805,
// 	8.0405055415951,
// 	8.1289009571972
// };

// //distance in meters
// double get_required_rpm(double distance) {
// 	int index = (distance - 1.2192) / 0.1016;
// 	if(index >= 0 && index < 38) {
// 		// v = r * RPM * 0.10472
// 		return launcher_data[index] / (12.7 * 0.10472);
// 	}
// 	return -1;
// }

void _updateFlyWheelLin(int power){
	if(power > 127){
		power = 127;
	}
	setLinMotorPow(_fly.f1, power);
	setLinMotorPow(_fly.f2, power);
}

void _updateFlyWheel(int power){
	_updateFlyWheelLin(power);
}


long	nSysTime_last;
long	encoder_counts;
long	encoder_counts_last;
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


void setFlyRpm(int rpm){
	_fly.setPoint = rpm;
	pidReset(_fly.flyPID);
}

void spinFlyWheelAuto(){
	//_setFlyWheel(get_required_rpm(getNetDistance()), 80);
	//spinUp(get_required_rpm(getNetDistance()));
}



void fw_ButtonControl(){
	//btnControl
	if(vexRT[Btn8D]){
		if(_fly.pred == 0){
				setFlyRpm(SKILLS_RPM);
				_fly.pred = SKILLS_POW;
			while(vexRT[Btn8D]){
			}
		}
		else{
			setFlyRpm(0);
			_fly.pred = 0;
			while(vexRT[Btn8D]){
			}
		}

	}
	if(vexRT[Btn8L]){
		setFlyRpm(SHORT_RPM);
		_fly.pred = SHORT_POW;
	}
	if(vexRT[Btn8R]){
		setFlyRpm(MED_RPM);
		_fly.pred = MED_POW;
	}
	if(vexRT[Btn8U]){
		setFlyRpm(LONG_RPM);
		_fly.pred = HIGH_POW;
	}
	// if(vexRT[Btn7R]){
	// 	spinFlyWheelAuto();
	// 	pidReset(_fly.flyPID);
	// }
}


void fw_fullCourtSpeed(){
	setFlyRpm(LONG_RPM);
	_fly.pred = HIGH_POW;
}

void fw_skillSpeed(){
	setFlyRpm(SKILLS_RPM);
	_fly.pred = SKILLS_POW;
}

void fw_midSpeed(){
	setFlyRpm(MED_RPM);
	_fly.pred = MED_POW;
}


task flw_task_PIDCntrl(){
	pidReset(_fly.flyPID);
	//pidInit(_fly.flyPID, 0.2, 0, 0.006, 100, 9999);
	pidInit(_fly.flyPID, 0.15, 0, 0, 100, 9999);
	//pidInit(_fly.flyPID, 0.1, 0, 0.006, 9999, 9999);
	float lastRpm1 = 0;
	float lastRpm2 = 0;
	float lastRpm3 = 0;
	float lastRpm4 = 0;

	float output = 0;
	float initTime = nPgmTime;
	while(true){
		fw_ButtonControl();

		lastRpm4 = lastRpm3;
		lastRpm3 = lastRpm2;
		lastRpm2 = lastRpm1;
		lastRpm1 = FwCalculateSpeed();

	//	float rpmAvg = (lastRpm4 * 1 + lastRpm3 * 2 + lastRpm2 * 8 + lastRpm1 * 16)/27;
	//	writeDebugStreamLine("rpmAvg %f", rpmAvg);
		_fly.currSpeed = lastRpm1 ;
		float outVal = pidExecute(_fly.flyPID, _fly.setPoint - _fly.currSpeed);
		float dTime = nPgmTime - initTime;
		initTime = nPgmTime;

		//filter output
		float coeff = dTime/FW_LOOP_SPEED;
		if(coeff < 1){
			coeff = 1;
		}
		output += (outVal) * (coeff);
		if(output < 0){
			output = 0;
		}
		if(output > 127){
			output = 127;
		}
		//writeDebugStreamLine("%f", output);
		_updateFlyWheel(output);
		delay(FW_LOOP_SPEED);
	}
}


task flw_tsk_FeedForwardCntrl(){
	pidReset(_fly.flyPID);
	//TRY: fairly good fast recovery
	//prev P 0.5
	pidInit(_fly.flyPID, 0.65 , 0.1, 0, 0, 9999);

	//pidInit(_fly.flyPID, 0.15, 0.05, 0, 100, 9999);

	//Good results at low rpms
	//pidInit(_fly.flyPID, 0.1, 0.01, 0, 100, 9999);

	//dislike slow
	//pidInit(_fly.flyPID, 0.2, 0.001, 0, 100, 9999);

	int integralLimit = 40 / _fly.flyPID.kI;
	float output = 0;
	float initTime = nPgmTime;
	//basically for this we need a guess motor power
	while(true){

		fw_ButtonControl();

		//we do not want to zero our error sum when we cross
		if(abs(_fly.flyPID.errorSum) > integralLimit){
			_fly.flyPID.errorSum = integralLimit * _fly.flyPID.errorSum/(abs(_fly.flyPID.errorSum));
		}

		_fly.currSpeed =  FwCalculateSpeed();
		float outVal = pidExecute(_fly.flyPID, _fly.setPoint - _fly.currSpeed);
		float dTime = nPgmTime - initTime;
		initTime = nPgmTime;

		//filter output
		float coeff = dTime/FW_LOOP_SPEED;
		if(coeff < 1){
			coeff = 1;
		}
		//
		output = _fly.pred + ((outVal) * (coeff));

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


task flw_task_bangbang(){
	while(true){

		fw_ButtonControl();
		_fly.currSpeed =  FwCalculateSpeed();
		float error = _fly.setPoint - _fly.currSpeed;
		writeDebugStreamLine("%f",error);

		float outVal = 0;
		if(abs(error) > 100){
			outVal = error > 0 ? 127 : 60;
		}
		else{
			outVal = error > 0 ? 80 : 70;
		}

		float output = outVal;

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




void initFlyWheel(Fw_Controller* initMotors){
	_fly = *initMotors;
	_fly.f1 = initMotors->f1;
	_fly.f2 = initMotors->f2;
	_fly.enc = initMotors->enc;
	//writeDebugStreamLine("FLY %d %d %d %d", _fly.f1,_fly.f2, _fly.f3, _fly.f4);
	startTask(flw_tsk_FeedForwardCntrl,20);
	//startTask(flw_task_bangbang,20);
	//startTask(flw_task_PIDCntrl, 20);
}

#endif
