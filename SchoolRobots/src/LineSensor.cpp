#include "../include/LineSensor.h"

int readRaw(byte pin) 
{
    int sum = 0;
    for (int i = 0; i < 4; i++) sum += analogRead(pin);
    return sum / 4;
}
int readSensor(byte pin, int white, int black) 
{
    if (white == black) return 0;
    long value = map(readRaw(pin), white, black, 0, 1000);
    return constrain(value, 0L, 1000L);
}
int readLeft() 
{ 
    return readSensor(LEFT_SENSOR_PIN, LEFT_WHITE, LEFT_BLACK); 
}
int readRight() 
{ 
    return readSensor(RIGHT_SENSOR_PIN, RIGHT_WHITE, RIGHT_BLACK); 
}
bool isCross() 
{ 
    return readLeft() >= CROSS_THRESHOLD && readRight() >= CROSS_THRESHOLD; 
}
