// ============================================================
//  vision_link.h — UDP command receiver (the network listener)
// ============================================================
//  Spawns a FreeRTOS task that listens for MotorCmd datagrams from the
//  PC and publishes the latest commanded wheel duties. The executor
//  reads them via visionGetTargets(), which returns false when the link
//  has gone stale (failsafe -> caller should stop the motors).
// ============================================================
#pragma once
#include <stdint.h>

// Start the UDP listener task pinned to `core`. Call after Wi-Fi is up.
void visionLinkBegin(uint16_t port, BaseType_t core);

// Snapshot the latest command. Returns true and fills the targets if a
// valid packet arrived within CMD_TIMEOUT_MS; returns false if stale.
bool visionGetTargets(int* outLeft, int* outRight, uint8_t* outCmd, uint32_t* outSeq);
