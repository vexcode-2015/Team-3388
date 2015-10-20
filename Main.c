
#include FlyControl.c
#include MecDrive.c
//#include "Vex_Competition_Includes.c"


// void pre_autonomous()
// {
// //Place pre-autonomous code here
// }
// task autonomous()
// {
// 	AutonomousCodePlaceholderForTesting();
// }
//task usercontrol()

task main()
{
	DriveBase dr = DriveBase(&motor[mDrFl],
		&motor[mDrFr],
		&motor[mDrBl],
		&motor[mDrFr]);
	initMecDrive(&dr);
	while(true)
	{
		mecDrive();
		delay(20);
	}
}