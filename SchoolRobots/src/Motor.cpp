#include "../include/Motor.h"

void setMotor(int dirPin, int pwmPin, bool invert, int speed) 
{
  speed = constrain(speed, -255, 255);
  bool forward = speed >= 0;
  if (invert) forward = !forward;
  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, abs(speed));
}

// скорости от -255 (полный назад) до 255 (полный вперёд)
void drive(int left, int right) 
{
  setMotor(L_DIR, L_PWM, L_INVERT, left);
  setMotor(R_DIR, R_PWM, R_INVERT, right);
}