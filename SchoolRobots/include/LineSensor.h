#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H
#include <Arduino.h>

const int N = 2;        // два датчика              
const int PINS[N] = {A0, A1};      // A0 — левый, A1 — правый
const int BORDER_BLACK = 111;
extern int minV[N];
extern int maxV[N];

void calibration();
// 0..1000: один край шкалы — белое, другой — чёрное (какой — проверьте)
int readNorm(int i);

#endif