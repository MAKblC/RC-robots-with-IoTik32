const int Pinv =  17;      // задаем номер для контакта
const int Pind =  16;      // задаем номер для контакта

float duration;
float turn;
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "TLC59108.h"
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x70);
float velo;
float dir;
#define HW_RESET_PIN 0 // Только програмнный сброс
#define I2C_ADDR TLC59108::I2C_ADDR::BASE
TLC59108 leds(I2C_ADDR + 7); // Без перемычек добавляется 3 бита адреса

void setup() {
  Serial.begin(115200);
  pinMode(Pind, INPUT);
  pinMode(Pinv, INPUT);
  Wire.begin();
  leds.init(HW_RESET_PIN);
  leds.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
  pwm.begin();
  // Частота (Гц)
  pwm.setPWMFreq(100);
  // Все порты выключены
  pwm.setPWM(8, 0, 4096);
  pwm.setPWM(9, 0, 4096);
  pwm.setPWM(10, 0, 4096);
  pwm.setPWM(11, 0, 4096);
  leds.setBrightness(2, 0xff);
}

void loop() {
  turn = pulseIn(Pind, HIGH, 100000);
  duration = pulseIn(Pinv, HIGH, 100000);
  if (turn == 0 or duration == 0) {
    motorA_setpower(0, false);
    motorB_setpower(0, true);
  }
  else {
    // Serial.println(turn);
    Serial.println(duration);
    // если отжат джойстик 2
    if (turn > 1550.00 and turn < 1630) {
      if (duration > 1400.00 and duration < 1550) {
        motorA_setpower(0, false);
        motorB_setpower(0, true);
      }
      else {
        velo = map(duration, 1500, 1800, 0, 100);
        velo = constrain(velo, -100, 100);
        motorA_setpower(velo, false);
        motorB_setpower(velo, true);
      }
    }
    // если отжат джойстик 1
    if (duration > 1400.00 and duration < 1550) {
      if (turn > 1550.00 and turn < 1630) {
        motorA_setpower(0, false);
        motorB_setpower(0, true);
      }
      else {
        dir = map(turn, 1100, 2000, -100, 100);
        dir = constrain(dir, -100, 100);
        motorA_setpower(dir, false);
        motorB_setpower((-1.00 * dir), true);
      }
    }
    // если задействованы оба джойстика
    if ((duration<1400 or duration>1550) and (turn<1550 or turn>1630)) {
      velo = map(duration, 1500.00, 1800.00, 0.00, 100.00);
      if (turn < 1550.00) {
        dir = map(turn, 1100.00, 1550.00, 0.00, -100.00);
        dir = constrain(dir, -100.00, 0.00);
        if (velo > 0) {
          motorA_setpower(velo, false);
          motorB_setpower(-1 * dir, true);
        }
        else {
          motorA_setpower(velo, false);
          motorB_setpower(dir, true);
        }
      }
      else {
        dir = map(turn, 1630.00, 2100.00, -100.00, 0.00);
        dir = constrain(dir, -1.00, 0.00);
        if (velo > 0) {
          motorA_setpower(-1 * dir, false);
          motorB_setpower(velo, true);
        }
        else {
          motorA_setpower(dir, false);
          motorB_setpower(velo, true);
        }
      }
    }
  }
}

void motorA_setpower(float pwr, bool invert)
{
  // Проверка, инвертирован ли мотор
  if (invert)
  {
    pwr = -pwr;
  }
  // Проверка диапазонов
  if (pwr < -100)
  {
    pwr = -100;
  }
  if (pwr > 100)
  {
    pwr = 100;
  }
  int pwmvalue = fabs(pwr) * 40.95;
  if (pwr < 0)
  {
    pwm.setPWM(10, 0, 4096);
    pwm.setPWM(11, 0, pwmvalue);
  }
  else
  {
    pwm.setPWM(11, 0, 4096);
    pwm.setPWM(10, 0, pwmvalue);
  }
}

// Мощность мотора "B" от -100% до +100% (от знака зависит направление вращения)
void motorB_setpower(float pwr, bool invert)
{
  // Проверка, инвертирован ли мотор
  if (invert)
  {
    pwr = -pwr;
  }
  // Проверка диапазонов
  if (pwr < -100)
  {
    pwr = -100;
  }
  if (pwr > 100)
  {
    pwr = 100;
  }
  int pwmvalue = fabs(pwr) * 40.95;
  if (pwr < 0)
  {
    pwm.setPWM(8, 0, 4096);
    pwm.setPWM(9, 0, pwmvalue);
  }
  else
  {
    pwm.setPWM(9, 0, 4096);
    pwm.setPWM(8, 0, pwmvalue);
  }
}
