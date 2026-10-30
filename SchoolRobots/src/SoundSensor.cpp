#include "../include/SoundSensor.h"

float readUltrasonicCm() 
{
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long t = pulseIn(ECHO, HIGH, 12000);  // ждём не дольше 12 мс ≈ 2 м
  delay(12);
  if (t == 0) return -1;                          // эха нет
  return t / 58.0;
}