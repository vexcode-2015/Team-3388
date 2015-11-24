
#ifndef FlyControl.c
#define FlyControl.c

#define FW_LOOP_SPEED	30


#include "Utils.c"
#include "PIDController.h"
#include "MecDrive.c"

const int SHORT_RPM = 1980;
const int MED_RPM = 2450;
const int LONG_RPM  = 3040



typedef struct {
	tMotor f1;
	tMotor f2;
	tSensors enc;
	tSensors sonar;
	PID flyPID;
	float setPoint;
	float currSpeed;
} Fw_Controller;
Fw_Controller _fly;


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
		setFlyRpm(0);
	}
	if(vexRT[Btn8L]){
		setFlyRpm(SHORT_RPM);
	}
	if(vexRT[Btn8R]){
		setFlyRpm(MED_RPM);
	}
	if(vexRT[Btn8U]){
		setFlyRpm(LONG_RPM);
	}
	// if(vexRT[Btn7R]){
	// 	spinFlyWheelAuto();
	// 	pidReset(_fly.flyPID);
	// }
}


void fw_fullCourtSpeed(){
	setFlyRpm(LONG_RPM);
}

task PIDFlyControl(){
	pidReset(_fly.flyPID);
	pidInit(_fly.flyPID, 0.05, 0, 0.006, 100, 9999);
	float output = 0;
	while(true){
		fw_ButtonControl();
		_fly.currSpeed = FwCalculateSpeed();
		float outVal = pidExecute(_fly.flyPID, _fly.setPoint - _fly.currSpeed);
	
		//filter output
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


void initFlyWheel(Fw_Controller* initMotors){
	_fly = *initMotors;
	_fly.f1 = initMotors->f1;
	_fly.f2 = initMotors->f2;
	_fly.enc = initMotors->enc;
	pidInit(_fly.flyPID, 0.05, 0, 0.006, 100, 9999);
	//writeDebugStreamLine("FLY %d %d %d %d", _fly.f1,_fly.f2, _fly.f3, _fly.f4);
	//startTask(FlyWheelControl);
	startTask(PIDFlyControl, 20);
}

#endif
