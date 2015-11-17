

// Stop recursive includes
#ifndef __GYROLIB__
#define __GYROLIB__

// Structure to hold global info for the gyro
typedef struct {
    tSensors port;
    bool     valid;
    float    angle;
    float    abs_angle;
    bool reset;
    } gyroData;

static  gyroData    theGyro = {in1, false, 0.0, 0.0};


void GyroResetAngle(){
    theGyro.reset = true;
}



void
GyroDebug( int displayLine )
{
    string str;

    if( theGyro.valid )
        {
        // display current value
        sprintf(str,"Gyro %5.1f   ", theGyro.angle );
        displayLCDString(displayLine, 0, str);
        }
    else
        displayLCDString(displayLine, 0, "Init Gyro.." );
}


task GyroTask()
{
    int     gyro_value;
    int     gyro_error = 0;
    int     lastDriftGyro = 0;

    float   angle;
    float   old_angle, delta_angle;

    long    nSysTimeOffset;

    // Gyro readings invalid
    theGyro.valid = false;

    // Cause the gyro to reinitialize (a theory anyway)
    SensorType[theGyro.port] = sensorNone;

    // Wait 1/2 sec
    wait10Msec(50);

    // Gyro should be motionless here
    SensorType[theGyro.port] = sensorGyro;

    // Wait 1/2 sec
    wait10Msec(50);

    // What is the current system timer
    nSysTimeOffset = nSysTime;

    // loop forever
    while(true)
        {
        if(theGyro.reset){
            SensorValue[theGyro.port] = 0;
            gyro_error = 0;
            lastDriftGyro = 0;
            old_angle = 0;
            angle = 0;
            delta_angle = 0;
            nSysTimeOffset = nSysTime;
            theGyro.reset = false;
        }
        // get current gyro value (deg * 10)
        gyro_value = SensorValue[theGyro.port];

        // Filter drift when not moving
        if( (nSysTime - nSysTimeOffset) > 250 )
            {
            if( abs( gyro_value - lastDriftGyro ) < 3 )
                gyro_error += (lastDriftGyro - gyro_value);

            lastDriftGyro = gyro_value;

            nSysTimeOffset = nSysTime;
            }

        // Create float angle, remove offset
        angle = (gyro_value + gyro_error)  / 10.0;

        // normalize into the range 0 - 360
        if( angle < 0 )
            angle += 360;

        // store in struct for others
        theGyro.angle = angle;

        // work out change from last time
        delta_angle = angle - old_angle;
        old_angle   = angle;

        // fix rollover
        if(delta_angle > 180)
          delta_angle -= 360;
        if(delta_angle < -180)
          delta_angle += 360;

        // store absolute angle
        theGyro.abs_angle = theGyro.abs_angle + delta_angle;

        // We can use the angle
        theGyro.valid = true;

        // Delay
        wait1Msec( 20 );
        }
}

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*      Initialize the Gyro on the given port                                  */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

void
GyroInit( tSensors port  )
{
    theGyro.port = port;

    StartTask( GyroTask );
}

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  Cause the gyro to be reinitialized by stopping and then restarting the     */
/*  polling task                                                               */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

void
GyroReinit()
{
    StopTask( GyroTask );
    StartTask( GyroTask );
}

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  Functions to get the public parameters                                     */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

float
GyroGetAngle()
{
    return( theGyro.angle );
}
float
GyroGetAbsAngle()
{
    return( theGyro.abs_angle );
}

bool
GyroGetValid()
{
    return( theGyro.valid );
}




#endif  //__GYROLIB__
