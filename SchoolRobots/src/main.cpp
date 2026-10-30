#include <Arduino.h>
#include "../include/SoundSensor.h"
#include "../include/Motor.h"
#include "../include/LineSensor.h"

float Kp = 1.5;
float Kd = 15;
int lastError = 0;
int valueSensor1;
int valueSensor2;

void setup()
{
  Serial.begin(9600);

  calibration();
  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);

  Serial.println("Все системы настроены...");
  
  delay(200);
}

void movement()
{
  valueSensor1 = readNorm(0);
  valueSensor2 = readNorm(1);
  
  // Игнорируем перекресток
  /*if (valueSensor1 >= BORDER_BLACK && valueSensor2 >= BORDER_BLACK)
  {
    drive(SPEED_FORWARD, SPEED_FORWARD);
    lastError = 0;

    return;
  }*/

  // Ошибка - величина отклонения от линии
  int error = valueSensor1 - valueSensor2;
  int derivative = error - lastError;
  // Корректировка - восстановительное значение для возврата на линию
  int U = Kp * error + Kd * derivative;
  // Ограничение корректировки (без неё - слишком резкие движения)
  U = constrain(U, -SPEED, SPEED); 
  
  int leftMotorSpeed = SPEED + U;
  int rightMotorSpeed = SPEED - U;
  
  drive(leftMotorSpeed, rightMotorSpeed);
  
  lastError = error;
}

void loop() 
{
  movement();
}

