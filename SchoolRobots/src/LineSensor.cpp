#include "../include/LineSensor.h"

int minV[N];
int maxV[N];

void calibration()
{
    for (int i = 0; i < N; i++) 
    { 
        minV[i] = 1023; 
        maxV[i] = 0; 
    }
    // 5 секунд водите робота поперёк линии руками
    unsigned long t0 = millis();
    while (millis() - t0 < 5000) 
    {
        for (int i = 0; i < N; i++) 
        {
            int v = analogRead(PINS[i]);
            minV[i] = min(minV[i], v);
            maxV[i] = max(maxV[i], v);
        }
    }
}

// 0..1000: один край шкалы — белое, другой — чёрное (какой — проверьте)
int readNorm(int i) 
{
  int v = analogRead(PINS[i]);
  return constrain(map(v, minV[i], maxV[i], 0, 1000), 0, 1000);
}