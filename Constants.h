#ifndef Constants.h
#define Constants.h

const float TICKS_PER_CENTIMETERS = 10.91535433;////5.639348377;
const float TICKS_PER_INCHES = TICKS_PER_CENTIMETERS  * 2.54;
const float TICKS_PER_FEET = TICKS_PER_INCHES  * 12;
const float TICKS_PER_METER = TICKS_PER_CENTIMETERS * 100;

const float NET_DISTANCE_METERS = 4.8;
//naive calculation for this one ((2 * pi * (9in * 2.54cm)) /360*) /0.177236 cm / tick
const float TICKS_PER_DEGREE = 2.2499859;

const float MAX_SPEED_MS = 3;


#endif
