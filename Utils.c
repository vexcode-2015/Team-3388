#ifndef Utils.c
#define Utils.c

#include "SmartMotorLib.c"

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

int lightIndex = 0;
void setStatusLight(int light){
	lightIndex = light;
}


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
		return;
	}
	motor[index] = _MotorMap[abs(power)] * (power/abs(power)) ;
}

void setLinMotorPow(tMotor index, int power, int deadzone){
	if(abs(power) > 127){
		power = power * (power/abs(power));
	}
	motor[index] = threshold(power > 0 ? _MotorMap[abs(power)] : -1 * _MotorMap[abs(power)], deadzone);
}

void setSmartPow(tMotor index, int power){
	SetMotor(index,power > 0 ? _MotorMap[power] : -1 * _MotorMap[power]);
}

void statusLightSet(int status){
	SensorValue[lightIndex] = status;
}

void printCalibratingGyro(){
	clearLCDLine(0);																						// Clear line 1 (0) of the LCD
	clearLCDLine(1);
	displayLCDString(0, 0, "Calibrating gyro... ");
}

void printBatteryToLCD(){
	clearLCDLine(0);																						// Clear line 1 (0) of the LCD
	clearLCDLine(1);
	displayLCDString(0, 0, "Primary: ");
	string mainBattery;
	string backupBattery;
	sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
	displayNextLCDString(mainBattery);

	//Display the Backup battery voltage
	displayLCDString(1, 0, "Backup: ");
	sprintf(backupBattery, "%1.2f%c", BackupBatteryLevel/1000.0, 'V');		//Build the value to be displayed
	displayNextLCDString(backupBattery);

}

int readAutoNum(){
	int autoSelect = 0;
	while(nLCDButtons != 2){ //While center not pressed
		if(nLCDButtons == 0) {//No button pressed
			printBatteryToLCD();
			wait1Msec(10); //Do nothing
		}
		else{ //Some button was pressed
			if(nLCDButtons == 1){
				clearLCDLine(0);																						// Clear line 1 (0) of the LCD
				clearLCDLine(1);
				if(autoSelect != 2){
					 autoSelect = 2;
					displayLCDString(0, 0, "Blue Side Auto Selected");
				}
				else{
					autoSelect = 0;
					displayLCDString(0,0, "Center Auto Selected");
				}
			while((nLCDButtons == 1)){
				wait1Msec(50);
			}
		 }
		 else if(nLCDButtons == 4){
				clearLCDLine(0);																						// Clear line 1 (0) of the LCD
		clearLCDLine(1);
				if(autoSelect != 1){
					autoSelect = 1;
					displayLCDString(0, 0, "Red Side Auto Selected");
			}
		else{
					autoSelect = 0;
					displayLCDString(0,0, "Center Auto Selected");
				}
		while((nLCDButtons == 4)){
		wait1Msec(50);
		}
				//Increment if right press
			}
			//Update display
			while(nLCDButtons != 0){//Wait for release
				wait1Msec(10); //Wait for multitasking.
			}
		}
	}
}
#endif
