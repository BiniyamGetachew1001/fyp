// ============================================================
//  motor_control.cpp
// ============================================================
#include <Arduino.h>
#include "config.h"
#include "motor_control.h"

// Currently-applied duty per wheel. The slew limiter eases these toward
// the commanded targets; the kickstart detector watches for 0 -> moving.
static int curL = 0;
static int curR = 0;

// writeMotors — the ONLY place analogWrite touches the pins. Forward-only
// wiring: a duty of 0 means "wheel off"; any >0 enables the direction pin.
static void writeMotors(int l, int r) {
  if (l < 0) l = 0;  if (l > MAX_SPEED) l = MAX_SPEED;
  if (r < 0) r = 0;  if (r > MAX_SPEED) r = MAX_SPEED;
  digitalWrite(MOT_L_IN1, l > 0 ? HIGH : LOW);
  digitalWrite(MOT_R_IN1, r > 0 ? HIGH : LOW);
  analogWrite(MOT_L_PWM, l);
  analogWrite(MOT_R_PWM, r);
  curL = l;
  curR = r;
}

void motorStop() {
  digitalWrite(MOT_L_IN1, LOW);
  digitalWrite(MOT_R_IN1, LOW);
  analogWrite(MOT_L_PWM, 0);
  analogWrite(MOT_R_PWM, 0);
  curL = 0;
  curR = 0;
}

void motorSetup() {
  pinMode(MOT_L_IN1, OUTPUT);
  pinMode(MOT_R_IN1, OUTPUT);
  pinMode(MOT_L_PWM, OUTPUT);
  pinMode(MOT_R_PWM, OUTPUT);
  motorStop();
}

// slewToward — limit how fast one wheel's duty may RISE per tick. Falling
// is instant. A nonzero target below MIN_SPEED is lifted to the floor so a
// wheel never buzzes in the dead stall band.
static int slewToward(int cur, int target) {
  if (target > 0 && target < MIN_SPEED) target = MIN_SPEED;
  if (target > cur) {
    int next = cur + SLEW_MAX_STEP;
    return (next < target) ? next : target;
  }
  return target;  // instant decrease
}

// motorTick — call every MOTOR_TICK_MS with the desired wheel duties.
//   1. KICKSTART: if a wheel is starting from rest (cur==0, target>0),
//      pulse it at KICK_PWM for KICK_MS to break static friction, so the
//      commanded cruise duty can stay low without stalling.
//   2. SLEW: ease each wheel one step toward its target (smooth accel).
void motorTick(int targetL, int targetR) {
  bool startingL = (targetL > 0 && curL == 0);
  bool startingR = (targetR > 0 && curR == 0);

  if (startingL || startingR) {
    writeMotors(startingL ? KICK_PWM : (targetL > 0 ? targetL : 0),
                startingR ? KICK_PWM : (targetR > 0 ? targetR : 0));
    delay(KICK_MS);
  }

  writeMotors(slewToward(curL, targetL), slewToward(curR, targetR));
}
