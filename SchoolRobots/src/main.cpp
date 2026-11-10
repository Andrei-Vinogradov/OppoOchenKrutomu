#include <Arduino.h>
#include "../include/Motor.h"
#include "../include/LineSensor.h"

// Считывание ошибки направления и корректировка моторов для удержания линии
void PD(int speed) 
{
    speed = constrain(speed, 0, 255);
    if (speed == 0) { stopMotors(); return; }
    lineLeft = readLeft();
    lineRight = readRight();
    int error = lineLeft - lineRight;
    if (abs(error) < 20) error = 0;
    int u = Kp * error + Kd * (error - lastError);
    lastError = error;
    u = constrain(u, -MAX_CORRECTION, MAX_CORRECTION);
    int minimum = min(MIN_PWM, speed);
    smoothPowers(constrain(speed - u, minimum, 255),
                 constrain(speed + u, minimum, 255));
}
// Базовое движение по линии
void lineDistance(int speed, long ticks, long start, long brake, bool stopAtEnd) {
    speed = constrain(speed, 0, 255);
    if (speed == 0 || ticks <= 0) { stopMotors(); return; }
    if (leftPower < 0 || rightPower < 0) stopMotors();
    waitForStop();
    resetEncoders();
    lastError = readLeft() - readRight();
    unsigned long started = millis();
    while (true) {
        long left, right;
        readTicks(left, right);
        checkEncoders(left, right, true, true);
        long passed = (left + right) / 2;
        if (passed >= ticks && left > 0 && right > 0) break;
        if (millis() - started >= MOVE_TIMEOUT) stopWithError("STOP: move timeout.");
        PD(profileSpeed(speed, passed, ticks, start, brake));
    }
    if (stopAtEnd) stopMotors();
}
// Езда по линии на заданное число тиков с резким стопом в конце.
void lineEncoder(int speed, long ticks) {
    lineDistance(speed, ticks, 0, 0, true);
}
// Езда по линии на заданное расстояние в сантиметрах с резким стопом в конце.
void lineCm(int speed, float cm) {
    lineEncoder(speed, cmToTicks(cm));
}
// Езда по линии в см с явным указанием участков плавного разгона и торможения
void line_encoder(int speed, float cm, float startCm, float brakeCm) {
    lineDistance(speed, cmToTicks(cm), cmToTicks(startCm), cmToTicks(brakeCm), true);
}
// Базовое движение без линии. Робот едет вслепую,
// но синхронизирует колеса между собой по энкодерам, чтобы его не уводило в сторону.
void syncTicks(int leftBase, int rightBase, long ticks, long start, long brake, unsigned long timeout){
    leftBase = constrain(leftBase, -255, 255);
    rightBase = constrain(rightBase, -255, 255);
    int leftAbs = abs(leftBase), rightAbs = abs(rightBase);
    int sum = leftAbs + rightAbs;
    if (sum == 0 || ticks <= 0) { stopMotors(); return; }
    bool reverseLeft = (leftPower > 0 && leftBase < 0) || (leftPower < 0 && leftBase > 0);
    bool reverseRight = (rightPower > 0 && rightBase < 0) || (rightPower < 0 && rightBase > 0);
    if (reverseLeft || reverseRight) stopMotors();
    waitForStop();
    resetEncoders();
    float scale = 2.0f * ticks / sum;
    long leftTarget = leftAbs ? max(1L, (long)(scale * leftAbs + 0.5f)) : 0;
    long rightTarget = rightAbs ? max(1L, (long)(scale * rightAbs + 0.5f)) : 0;
    int largest = max(leftAbs, rightAbs);
    long lastSyncError = 0;
    unsigned long started = millis();
    while (true) {
        long left, right;
        readTicks(left, right);
        bool leftDone = left >= leftTarget, rightDone = right >= rightTarget;
        if (leftDone && rightDone) break;
        checkEncoders(left, right, !leftDone, !rightDone);
        if (millis() - started >= timeout) stopWithError("STOP: move timeout.");
        long passed = (left + right) / 2;
        int l = profileSpeed(leftAbs, passed, ticks, start, brake);
        int r = profileSpeed(rightAbs, passed, ticks, start, brake);
        if (leftAbs && rightAbs) {
            long error = (long)(((float)left * rightAbs - (float)right * leftAbs) / largest);
            float correction = Kp_sync * error + Kd_sync * (error - lastSyncError);
            lastSyncError = error;
            // Ограничиваем ДО приведения к 16-битному int на Uno.
            int u = constrain(correction, -MAX_SYNC_CORRECTION, MAX_SYNC_CORRECTION);
            l = constrain(l - u, min(MIN_PWM, leftAbs), 255);
            r = constrain(r + u, min(MIN_PWM, rightAbs), 255);
        }
        l = leftBase < 0 ? -l : l;
        r = rightBase < 0 ? -r : r;
        if (leftDone) { l = 0; leftPower = 0; }
        if (rightDone) { r = 0; rightPower = 0; }
        smoothPowers(l, r);
    }
    stopMotors();
}
// Движение без линии в сантиметрах с поддержкой разгона/торможения.
void syncMove(int left, int right, float cm, float startCm, float brakeCm){
    syncTicks(left, right, cmToTicks(cm), cmToTicks(startCm), cmToTicks(brakeCm), MOVE_TIMEOUT);
}
// Езда прямо (или назад, если скорость отрицательная) без учета датчиков линии
void moveCm(int speed, float cm){
    syncMove(speed, speed, cm, 0, 0);
}
void crossing(int speed, int n, float afterCm, float startCm, int stopType) {
    speed = constrain(speed, 0, 255);
    if (speed == 0 || n <= 0) { stopMotors(); return; }
    if (stopType < 0 || stopType > 2) stopWithError("STOP: stopType must be 0, 1 or 2.");
    if (leftPower < 0 || rightPower < 0) stopMotors();
    waitForStop();
    if (startCm > 0)
        lineDistance(speed, cmToTicks(startCm), cmToTicks(startCm), 0, false);
    resetEncoders();
    int count = 0;
    bool wasCross = isCross(); // стартовый перекрёсток пропускаем
    long sideStart = -1;       // начало возможного бокового Т, в импульсах
    long lastCountedAt = -1;   // последний учтённый перекрёсток
    const long minCrossGap = cmToTicks(MIN_CROSS_GAP_CM);
    lastError = readLeft() - readRight();
    unsigned long started = millis();
    while (count < n) {
        long left, right;
        readTicks(left, right);
        checkEncoders(left, right, true, true);
        if (millis() - started >= MOVE_TIMEOUT) stopWithError("STOP: crossing not found.");
        PD(speed);
        // Используем ту же пару измерений, что и PD, без лишних analogRead.
        bool cross = lineLeft >= CROSS_THRESHOLD && lineRight >= CROSS_THRESHOLD;
        bool sideCross = false;
        if (DETECT_SIDE_T && !cross) {
            bool oneBlack = (lineLeft >= CROSS_THRESHOLD && lineRight <= SIDE_T_OTHER_MAX) ||
                            (lineRight >= CROSS_THRESHOLD && lineLeft <= SIDE_T_OTHER_MAX);
            if (oneBlack) {
                if (sideStart < 0) sideStart = (left + right) / 2;
                sideCross = (left + right) / 2 - sideStart >= cmToTicks(SIDE_T_MIN_CM);
            } else sideStart = -1;
        } else sideStart = -1;
        if (sideCross) cross = true;
        if (cross != wasCross) {
            delay(10);
            int checkL = readLeft(), checkR = readRight();
            bool confirmed = checkL >= CROSS_THRESHOLD && checkR >= CROSS_THRESHOLD;
            if (DETECT_SIDE_T && sideCross) {
                // За время подтверждения должен остаться чёрным тот же датчик.
                confirmed = (lineLeft >= CROSS_THRESHOLD && checkL >= CROSS_THRESHOLD && checkR <= SIDE_T_OTHER_MAX) ||
                            (lineRight >= CROSS_THRESHOLD && checkR >= CROSS_THRESHOLD && checkL <= SIDE_T_OTHER_MAX);
            }
            if (confirmed == cross) {
                if (cross) {
                    long position = (left + right) / 2;
                    if (lastCountedAt < 0 || position - lastCountedAt >= minCrossGap) {
                        count++;
                        lastCountedAt = position;
                    }
                }
                wasCross = cross;
            }
        }
    }
    if (stopType == 0) return; // после этого сразу должна идти следующая команда
    if (afterCm >= 0) {
        long ticks = cmToTicks(afterCm);
        long brake = stopType == 2 ? cmToTicks(min(afterCm, 5.0f)) : 0;
        if (ticks > 0) syncTicks(speed, speed, ticks, 0, brake, MOVE_TIMEOUT);
    } else {
        started = millis();
        while (AXIS_TIME > 0 && millis() - started < (unsigned long)AXIS_TIME) {
            long left, right;
            readTicks(left, right);
            checkEncoders(left, right, true, true);
            int power = speed;
            if (stopType == 2) {
                long remaining = max(0L, (long)AXIS_TIME - (long)(millis() - started));
                int minimum = min(MIN_PWM, speed);
                power = minimum + (long)(speed - minimum) * remaining / AXIS_TIME;
            }
            smoothPowers(power, power);
        }
    }
    stopMotors();
}
// Прежний вызов: до n-го перекрёстка, доезд до оси, стоп.
void lineCross(int speed, int n) {
    crossing(speed, n, axisOffsetCm, 0, 1);
}
// Тот же проезд, но доезд после обнаружения явно задан в сантиметрах.
void lineCross(int speed, int n, float afterCm) {
    crossing(speed, n, max(0.0f, afterCm), 0, 1);
}
// Вызов как в твоей библиотеке. Все расстояния — СМ, следующий перекрёсток.
void line_perekrestok(int speed, float afterCm, float startCm, int stopType) {
    crossing(speed, 1, max(0.0f, afterCm), startCm, stopType);
}
// На месте: плюс — НАЛЕВО, минус — НАПРАВО. Скорость положительная.
void turnTicks(int speed, long ticks) {
    speed = constrain(speed, 90, 255);
    stopMotors();
    if (speed == 0 || ticks == 0) return;
    waitForStop();
    long target = ticks > 0 ? ticks : -ticks;
    int left = ticks > 0 ? -speed : speed;
    // ИЗМЕНЕНИЯ. Вычисление фиксированного разгона и торможения.
    // 1.5f следует задать в качестве переменной и немного откалибровать
    long smoothTicks = cmToTicks(1.5f); 
    // Если поворот слишком маленький, разгон не должен превышать треть пути
    if (smoothTicks > target / 3) 
    {
        smoothTicks = target / 3;
    }
    //syncTicks(left, -left, target, 0 /*target / 1.1*/, 0 /*target / 1.1*/, TURN_TIMEOUT);
    syncTicks(left, -left, target, smoothTicks, smoothTicks, TURN_TIMEOUT);
    waitForStop();
}
// Поворот по энкодерам: плюс градусов НАЛЕВО, минус НАПРАВО.
void turn(int speed, int degrees) 
{
    if (speed <= 0 || degrees == 0) 
    { 
        stopMotors();
        return; 
    }
    long angle = degrees;
    if (angle < 0) angle = -angle;
    long ticks = degreesToTicks(angle);
    turnTicks(speed, degrees > 0 ? ticks : -ticks);
}
void turnToLine(int speed, int degrees) {
    if (speed <= 0 || degrees == 0) {
        stopMotors();
        return;
    }

    // 1. Основной поворот по энкодерам
    turn(speed, degrees);
    waitForStop();

    const int finePWM = 85;
    const int lineThreshold = 450;
    const int maxSteps = 3;
    const int pulseMs = 55;

    int originalSide = degrees > 0 ? 1 : -1;
    int lastSide = 0;

    // 2. Несколько небольших корректировок
    for (int i = 0; i < maxSteps; i++) {
        int l = readLeft();
        int r = readRight();

        int error = l - r;
        bool onLine = max(l, r) >= lineThreshold;

        // Уже достаточно точно стоим на линии
        if (onLine && abs(error) <= TURN_CENTER_ERROR) {
            stopMotors();
            return;
        }

        int side;

        if (!onLine) {
            // Линию пока не увидели:
            // продолжаем искать в сторону поворота
            side = originalSide;
        } else {
            // Направление коррекции по датчикам
            side = error > 0 ? 1 : -1;

            // Пересекли положение равновесия:
            // не начинаем раскачиваться обратно
            if (lastSide != 0 && side != lastSide) {
                stopMotors();
                return;
            }
        }

        lastSide = side;

        // 3. Плавный короткий импульс
        for (int k = 1; k <= 3; k++) {
            int p = finePWM * k / 3;
            setPowers(-side * p, side * p);
            delay(8);
        }

        delay(pulseMs);

        for (int k = 2; k >= 1; k--) {
            int p = finePWM * k / 3;
            setPowers(-side * p, side * p);
            delay(8);
        }

        stopMotors();
        waitForStop();
    }

    // Если не удалось точно выровняться,
    // прекращаем корректировки
    stopMotors();
    Serial.println("WARNING: turn alignment incomplete");
}
// Диагностика: белое = 0, чёрное = 1000. raw вписать в white/black наверху.
// Энкодеры проверяются вращением колёс рукой; моторы здесь выключены.
void printSensors() {
    stopMotors();
    long left, right;
    readTicks(left, right);
    Serial.print("L raw: "); Serial.print(readRaw(LEFT_SENSOR_PIN));
    Serial.print("  R raw: "); Serial.print(readRaw(RIGHT_SENSOR_PIN));
    Serial.print("  L norm: "); Serial.print(readLeft());
    Serial.print("  R norm: "); Serial.print(readRight());
    Serial.print("  L ticks: "); Serial.print(left);
    Serial.print("  R ticks: "); Serial.println(right);
    delay(100);
}

void setup() {
    Serial.begin(9600);
    pinMode(L_DIR, OUTPUT); pinMode(L_PWM, OUTPUT);
    pinMode(R_DIR, OUTPUT); pinMode(R_PWM, OUTPUT);
    pinMode(LEFT_SENSOR_PIN, INPUT); pinMode(RIGHT_SENSOR_PIN, INPUT);
    stopMotors();
    if (LEFT_WHITE == LEFT_BLACK || RIGHT_WHITE == RIGHT_BLACK)
        stopWithError("STOP: check sensor calibration.");
    pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), countLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), countRight, RISING);
    delay(1000);
}

void loop() {
    if (CALIBRATION_MODE) { printSensors(); return; }

    // Для теста
    lineCross(BASE_SPEED, 1 /*, 5.9*/);
    turnToLine(BASE_SPEED - 5, -90);
    lineCross(BASE_SPEED, 1 /*, 5.9*/);
    turnToLine(BASE_SPEED - 5, 90);
}
