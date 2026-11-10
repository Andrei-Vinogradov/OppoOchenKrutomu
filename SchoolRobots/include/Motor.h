#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

// Motor Shield Plus: M1 = левый, M2 = правый
const byte L_DIR = 4;
const byte L_PWM = 5;
const byte R_DIR = 7;
const byte R_PWM = 6;
// Пины энкодеров
const byte ENCODER_LEFT_PIN = 2;
const byte ENCODER_RIGHT_PIN = 3;
// Настройки езды
const int BASE_SPEED = 70;
const int MIN_PWM = 58;
const int MAX_CORRECTION = 25;
const int PWM_STEP = 1;

extern float Kp;
extern float Kd;
extern float Kp_sync;
extern float Kd_sync;
extern float axisOffsetCm;
extern volatile long leftTicks;
extern volatile long rightTicks;
extern int leftPower;
extern int rightPower;
extern int lastError;
extern int lineLeft;
extern int lineRight;
extern long previousLeftTicks;
extern long previousRightTicks;
extern unsigned long leftMovedAt;
extern unsigned long rightMovedAt;
extern unsigned long stoppedAt;

// ЗАМЕРИТЬ между серединами левого и правого колёс, мм
extern float wheelBaseMm;
// поправка угла после проверки на покрытии
extern float turnCorrection;

// Энкодер: один канал, только RISING; 11 импульсов * редуктор 45.
// 495 на оборот КОЛЕСА
const long TICKS_PER_WHEEL = 11L * 45;
const float WHEEL_DIAMETER_MM = 72.0;
const float MM_PER_TICK = 3.14159265f * WHEEL_DIAMETER_MM / TICKS_PER_WHEEL;
// Синхронизация колёс. Это отдельные коэффициенты, не Kp/Kd линии.
const int MAX_SYNC_CORRECTION = 15;
// Доезд после обнаружения перекрёстка.
const int AXIS_TIME = 300; // подобрать доезд до оси на роботе
// Поворот до линии: ширина окна поиска зависит от расположения датчиков.
// Начальное значение 25: искать за 25 градусов до заданного угла и до +25.
const int TURN_SEARCH_DEGREES = 25;
const int TURN_ALIGN_PWM = MIN_PWM + 5;
const int TURN_CENTER_ERROR = 170;
const unsigned long MOVE_TIMEOUT = 120000; // 2 мин для длинных проездов
const unsigned long TURN_TIMEOUT = 20000;
const unsigned long STALL_TIME = 1500; // нет импульсов движущегося колеса — стоп
const unsigned long STOP_PAUSE_MS = 100; // увеличить, если после PWM=0 выбег длится дольше

// Задать скорость и направление каждого мотора (ограничение +- 250)
void setPowers(int left, int right);
// Плавное изменение скорости моторов (шаг pwmStep)
void smoothPowers(int left, int right);
// Полная остановка моторов
void stopMotors();
// Небольшая пауза после выключения моторов
void waitForStop();
// Аварийная остановка моторов по причине ошибки
void stopWithError(const char *message);
// Проверка энкодеров. Если моторы включены,
// а тики с энкодеров не приходят дольше 1.5 сек, вызываем stopWithError
void checkEncoders(long left, long right, bool leftActive, bool rightActive);
// Одноканальные энкодеры считают путь без знака, в том числе при движении назад
void countLeft();
void countRight();
// Сброс энкодеров
void resetEncoders();
// Считывание значения энкодеров
void readTicks(long &left, long &right);
// Среднее арифметическое пройденных тиков левого и правого колеса
long enc();
// Перевод расстояния из сантиметров в импульсы энкодера
long cmToTicks(float cm);
// Перевод угол поворота робота в градусах в количество тиков
long degreesToTicks(float degrees);
// Рассчёт текущей скорости робота для создания профиля движения
int profileSpeed(int speed, long passed, long total, long start, long brake);

#endif
