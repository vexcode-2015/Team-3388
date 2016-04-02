#ifndef Constants.h
#define Constants.h

const float TICKS_PER_CENTIMETERS = 13.8814729287;////5.639348377;
const float TICKS_PER_INCHES = 360 / (PI * 3.25);
const float TICKS_PER_FEET = TICKS_PER_INCHES  * 12;
const float TICKS_PER_METER = TICKS_PER_CENTIMETERS * 100;

const float NET_DISTANCE_METERS = 4.8;
//360 ticks per rotation / (2 * pi * r)
const float TICKS_PER_DEGREE = 17.62947;

const float MAX_SPEED_MS = 3;


#endif
