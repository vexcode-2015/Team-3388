#ifndef Utils.c
#define Utils.c

const unsigned int _MotorMap[128] =
{
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
	0, 21, 21, 21, 22, 22, 22, 23, 24, 24,
 25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
 28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
 33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
 37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
 41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
 46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
 52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
 61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
 71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
 80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
 88, 89, 89, 90, 90,127,127,127
};

int threshold(int in, int deadzone){
	if(abs(in) < deadzone){
		return 0;
	}
	return in;
}

void setLinMotorPow(int index, int power){
	if(power == 0){
		motor[index] = 0;
		return;
	}
	if(abs(power) > 127){
		power = 127 * (power/abs(power));
	}
	motor[index] = power > 0 ? _MotorMap[abs(power)] : -_MotorMap[abs(power)];
}

void setLinMotorPow(tMotor index, int power, int deadzone){
	if(abs(power) > 127){
		power = power * (power/abs(power));
	}
	motor[index] = threshold(power > 0 ? _MotorMap[abs(power)] : -1 * _MotorMap[abs(power)], deadzone);
}

task utl_DebugHelperTask(){
	while(true){

		wait1Msec(30);
	}
}

float utl_getMin(float a, float b){
	if(a < b){
		return a;
	} else{
		return b;
	}
}

float utl_getMax(float a, float b){
	if(a > b){
		return a;
	} else{
		return b;
	}
}

int utl_getPotSet(int pot){
	//position 1
 	int errTol = 50;
 	int settings[7];
 	settings[0] = 4095;
 	settings[1] = 3600;
 	settings[2] = 2775;
 	settings[3] = 1978;
 	settings[4] = 1222;
 	settings[5] = 497;
 	settings[6] = 0;
	for(int i = 0; i<7; i++){
		if(abs(settings[i] - pot) < errTol){
			return i + 1;
		}
	}
}


#endif
