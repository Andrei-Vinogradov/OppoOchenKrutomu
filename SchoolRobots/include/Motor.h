#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

// Motor Shield Plus: M1 = левый, M2 = правый
const int L_DIR = 4;
const int L_PWM = 5;
const int R_DIR = 7;
const int R_PWM = 6;
// Мотор крутится не в ту сторону — поменяйте false на true
const bool L_INVERT = true;
const bool R_INVERT = true;
const int SPEED = 65;

void setMotor(int dirPin, int pwmPin, bool invert, int speed);
// скорости от -255 (полный назад) до 255 (полный вперёд)
void drive(int left, int right);

#endif