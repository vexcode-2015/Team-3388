
#ifndef FlyControl.c 
#define FlyControl.c

// Update inteval (in mS) for the flywheel control loop
#define FW_LOOP_SPEED              25

typedef struct _fw_controller {

} fw_controller;

typedef struct fw_motors{
	tMotor[4]* m;
} fw_motors;

fw_motors _flyWheelM;
void initFlyWheel(fw_motors* initMotors){
	_flyWheelM = *initMotors;
}

void _updateFlyWheel(int power){
	_flyWheelM.m[0]* = power;
	_flyWheelM.m[1]* = power;
	_flyWheelM.m[2]* = power;
	_flyWheelM.m[3]* = power;
}

task FlyWheelControl(){
	while(true){
		if(vexRT[Btn5U]){
			_updateFlyWheel(127)
		}
		if(vexRT[Btn5D]){
			_updateFlyWheel(100)
		}
		if(vexRT[Btn6D]){
			_updateFlyWheel(70)
		}
		if(vexRT[Btn6U]){
			_updateFlyWheel(90)
		}
		delay(FW_LOOP_SPEED);
	}
}
#endif
