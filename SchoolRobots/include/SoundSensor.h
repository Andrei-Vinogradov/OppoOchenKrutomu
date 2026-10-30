#ifndef SOUND_SENSOR_H
#define SOUND_SENSOR_H
#include <Arduino.h>

const int TRIG = 11;
const int ECHO = A5;

float readUltrasonicCm();

#endif