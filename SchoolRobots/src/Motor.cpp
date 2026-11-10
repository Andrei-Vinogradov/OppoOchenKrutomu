#include "../include/Motor.h"

float Kp = 0.016;
float Kd = 0.4;
float Kp_sync = 0.35;
float Kd_sync = 0.1;
float wheelBaseMm = 183.4;
float turnCorrection = 1.0;
float axisOffsetCm = -1;
volatile long leftTicks = 0;
volatile long rightTicks = 0;
int leftPower = 0;
int rightPower = 0;
int lastError = 0;
int lineLeft = 0;
int lineRight = 0;
long previousLeftTicks = 0;
long previousRightTicks = 0;
unsigned long leftMovedAt = 0;
unsigned long rightMovedAt = 0;
unsigned long stoppedAt = 0;

void setPowers(int left, int right)
{
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);
    digitalWrite(L_DIR, left >= 0 ? LOW : HIGH);
    digitalWrite(R_DIR, right >= 0 ? LOW : HIGH);
    analogWrite(L_PWM, abs(left));
    analogWrite(R_PWM, abs(right));
}
void smoothPowers(int left, int right) {
    leftPower += constrain(left - leftPower, -PWM_STEP, PWM_STEP);
    rightPower += constrain(right - rightPower, -PWM_STEP, PWM_STEP);
    setPowers(leftPower, rightPower);
    delay(5); // период твоего регулятора: при настройке Kd не менять
}
void stopMotors() {
    setPowers(0, 0); // L6206Q: EN = 0, свободный выбег, без активного тормоза EV3
    leftPower = rightPower = lastError = 0;
    stoppedAt = millis();
}
void waitForStop() {
    if (leftPower != 0 || rightPower != 0) return;
    unsigned long elapsed = millis() - stoppedAt;
    if (elapsed < STOP_PAUSE_MS) delay(STOP_PAUSE_MS - elapsed);
}
void stopWithError(const char *message) {
    stopMotors();
    //Serial.println(message);
    while (true) 
    {
        Serial.println(message);
        delay(2000); // не продолжаем маршрут после ошибки
    }
}
void checkEncoders(long left, long right, bool leftActive, bool rightActive) {
    unsigned long now = millis();
    if (left != previousLeftTicks) leftMovedAt = now;
    if (right != previousRightTicks) rightMovedAt = now;
    previousLeftTicks = left;
    previousRightTicks = right;
    if (leftActive && now - leftMovedAt >= STALL_TIME)
        stopWithError("STOP: left encoder / wheel stalled.");
    if (rightActive && now - rightMovedAt >= STALL_TIME)
        stopWithError("STOP: right encoder / wheel stalled.");
}
void countLeft() 
{ 
    leftTicks++; 
}
void countRight() 
{ 
    rightTicks++; 
}
void resetEncoders() {
    noInterrupts();
    leftTicks = rightTicks = 0;
    interrupts();
    previousLeftTicks = previousRightTicks = 0;
    leftMovedAt = rightMovedAt = millis();
}
void readTicks(long &left, long &right) {
    noInterrupts();
    left = leftTicks;
    right = rightTicks;
    interrupts();
}
long enc() {
    long left, right;
    readTicks(left, right);
    return (left + right) / 2;
}
long cmToTicks(float cm) {
    if (cm <= 0) return 0;
    return (long)(cm * 10.0f / MM_PER_TICK + 0.5f);
}
int profileSpeed(int speed, long passed, long total, long start, long brake) {
    int minimum = min(MIN_PWM, speed);
    int power = speed;
    start = constrain(start, 0L, total);
    brake = constrain(brake, 0L, total);
    if (start > 0 && passed < start)
        power = min(power, minimum + (long)(speed - minimum) * passed / start);
    long remaining = max(0L, total - passed);
    if (brake > 0 && remaining < brake)
        power = min(power, minimum + (long)(speed - minimum) * remaining / brake);
    return power;
}
long degreesToTicks(float degrees) {
    if (wheelBaseMm <= 0) stopWithError("STOP: set wheelBaseMm in millimeters.");
    if (turnCorrection <= 0) stopWithError("STOP: turnCorrection must be positive.");
    float distanceMm = 3.14159265f * wheelBaseMm * degrees / 360.0f;
    return (long)(distanceMm / MM_PER_TICK * turnCorrection + 0.5f);
}
