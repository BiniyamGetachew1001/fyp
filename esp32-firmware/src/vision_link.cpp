// ============================================================
//  vision_link.cpp
// ============================================================
#include <Arduino.h>
#include <WiFiUdp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "vision_link.h"

static WiFiUDP      udp;
static uint16_t     listenPort = UDP_CMD_PORT;

// Latest decoded command, shared between the UDP task and the executor.
// Guarded by a short portMUX critical section (copy in/out is tiny).
static portMUX_TYPE cmdMux  = portMUX_INITIALIZER_UNLOCKED;
static volatile int      g_left   = 0;
static volatile int      g_right  = 0;
static volatile uint8_t  g_cmd    = CMD_STOP;
static volatile uint32_t g_seq    = 0;
static volatile uint32_t g_lastMs = 0;
static volatile bool     g_everRx = false;

static void udpTask(void* arg) {
  uint8_t buf[64];
  udp.begin(listenPort);
  Serial.printf("[NET] UDP command listener up on :%u (core %d)\n",
                listenPort, xPortGetCoreID());

  for (;;) {
    int n = udp.parsePacket();
    if (n <= 0) {
      vTaskDelay(pdMS_TO_TICKS(2));   // idle poll, yields the core
      continue;
    }

    int len = udp.read(buf, sizeof(buf));
    if (len != (int)sizeof(MotorCmd)) continue;       // wrong size -> drop

    MotorCmd pkt;
    memcpy(&pkt, buf, sizeof(pkt));
    if (pkt.magic != CMD_MAGIC || pkt.version != CMD_VERSION) continue;

    uint32_t nowMs = millis();
    portENTER_CRITICAL(&cmdMux);
    g_left   = pkt.left;
    g_right  = pkt.right;
    g_cmd    = pkt.cmd;
    g_seq    = pkt.seq;
    g_lastMs = nowMs;
    g_everRx = true;
    portEXIT_CRITICAL(&cmdMux);
  }
}

void visionLinkBegin(uint16_t port, BaseType_t core) {
  listenPort = port;
  xTaskCreatePinnedToCore(udpTask, "udpCmd", 4096, NULL, 2, NULL, core);
}

bool visionGetTargets(int* outLeft, int* outRight, uint8_t* outCmd, uint32_t* outSeq) {
  int      l, r;
  uint8_t  c;
  uint32_t seq, lastMs;
  bool     ever;

  portENTER_CRITICAL(&cmdMux);
  l = g_left; r = g_right; c = g_cmd; seq = g_seq; lastMs = g_lastMs; ever = g_everRx;
  portEXIT_CRITICAL(&cmdMux);

  if (outLeft) *outLeft = l;
  if (outRight) *outRight = r;
  if (outCmd) *outCmd = c;
  if (outSeq) *outSeq = seq;

  if (!ever) return false;                                  // nothing ever received
  if ((uint32_t)(millis() - lastMs) > CMD_TIMEOUT_MS) return false;  // stale -> failsafe
  return true;
}
