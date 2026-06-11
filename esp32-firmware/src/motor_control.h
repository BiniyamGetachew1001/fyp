// ============================================================
//  motor_control.h — low-level forward-only L298N driver
// ============================================================
//  The ONLY motor intelligence kept on the ESP: minimum-speed floor,
//  kickstart (static-friction breakaway) and slew-rate limiting.
//  Call motorTick(targetL, targetR) at a steady rate (MOTOR_TICK_MS);
//  it eases the wheels toward the commanded duties one slew step at a
//  time. motorStop() cuts instantly (failsafe / braking).
// ============================================================
#pragma once

void motorSetup();
void motorTick(int targetL, int targetR);  // slew + kickstart toward target
void motorStop();                          // instant halt
