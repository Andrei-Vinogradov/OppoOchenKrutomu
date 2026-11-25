#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H
#include <Arduino.h>

const byte LEFT_SENSOR_PIN = A0;
const byte RIGHT_SENSOR_PIN = A1;
const int LEFT_WHITE = 50;
const int LEFT_BLACK = 665;
const int RIGHT_WHITE = 80;
const int RIGHT_BLACK = 670;
const int CROSS_THRESHOLD = 850;
const bool CALIBRATION_MODE = false;
const bool DETECT_SIDE_T = false;
const float SIDE_T_MIN_CM = 1.5f;
const int SIDE_T_OTHER_MAX = 350;
const float MIN_CROSS_GAP_CM = 4.0f;

//  4 замера с аналогового пина датчика линии и возврат среднее
int readRaw(byte pin);
//  Масштабирование сырых данные датчика в диапазон от 0 (белый) до 1000 (черный).
int readSensor(byte pin, int white, int black);
// Чтение левого и правого датчика
int readLeft();
int readRight();
// Проверка находится ли робот на перекрёстке
bool isCross();

#endif
