// ============================================================
//  ESP32-CAM — "The Doer"
//  Camera transmitter + motor executor. No vision, no state machine.
// ============================================================
//
//  Split architecture:
//    PC  ("Thinker") : MJPEG capture -> YOLO -> EMA -> state machine ->
//                      emits per-wheel motor commands (UDP).
//    ESP ("Doer")    : streams the camera, receives those commands, and
//                      drives the L298N with kickstart / slew / min-speed.
//
//  This file is just the wiring + the failsafe executor loop. The real
//  modules live in:
//    camera_stream.*  — sensor + MJPEG HTTP server (YOLO's frame source)
//    vision_link.*    — UDP listener task, latest-command + staleness
//    motor_control.*  — forward-only L298N w/ kickstart, slew, floor
//
//  ── PROTOCOL (see config.h) ────────────────────────────────
//    PC -> ESP : 10-byte UDP MotorCmd {magic,ver,cmd,left,right,seq}.
//    UDP is chosen for lowest control latency: every packet is the
//    latest truth (packet N+1 supersedes N), so there is nothing to
//    retransmit and no head-of-line blocking. A lost packet is replaced
//    by the next one milliseconds later. If the link goes silent for
//    CMD_TIMEOUT_MS the executor stops the motors (failsafe).
//
//  ── TASK / CORE LAYOUT ─────────────────────────────────────
//    Core 0 : udpTask (vision_link) + Wi-Fi/lwIP stack
//    Core 1 : motorExecutorTask + loop() (MJPEG server)
//  The blocking MJPEG handler can never starve motor control because the
//  executor is its own task, ticking every MOTOR_TICK_MS.
// ============================================================

#include <Arduino.h>
#include <WiFiMulti.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "camera_stream.h"
#include "vision_link.h"
#include "motor_control.h"

static WiFiMulti wifiMulti;

// ── Motor executor ─────────────────────────────────────────
//  Ticks at a steady rate independent of packet jitter. Reads the latest
//  commanded wheel duties; on a stale link it halts (failsafe). The PC
//  has already decided everything — here we only smooth it onto the iron.
static void motorExecutorTask(void* arg) {
  uint32_t lastSeq   = 0xFFFFFFFFu;
  bool     wasStale  = true;

  for (;;) {
    int     l = 0, r = 0;
    uint8_t cmd = CMD_STOP;
    uint32_t seq = 0;

    if (visionGetTargets(&l, &r, &cmd, &seq)) {
      motorTick(l, r);
      if (wasStale) {
        Serial.println("[EXEC] link live — accepting commands");
        wasStale = false;
      }
      if (seq != lastSeq) {
        static const char* TAG[] = {"STOP", "HOLD", "PIVOT_L", "PIVOT_R", "ADVANCE"};
        const char* t = (cmd <= CMD_ADVANCE) ? TAG[cmd] : "?";
        Serial.printf("[EXEC] seq=%lu cmd=%-7s -> L=%3d R=%3d\n",
                      (unsigned long)seq, t, l, r);
        lastSeq = seq;
      }
    } else {
      motorStop();   // failsafe: no fresh command -> halt
      if (!wasStale) {
        Serial.println("[EXEC] link STALE — motors stopped (failsafe)");
        wasStale = true;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(MOTOR_TICK_MS));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("=== ESP32-CAM 'Doer' — camera TX + UDP motor executor ===");
  Serial.printf("  MIN/MAX_SPEED = %d/%d  KICK = %d/%dms  SLEW = %d/tick(%dms)\n",
                MIN_SPEED, MAX_SPEED, KICK_PWM, KICK_MS, SLEW_MAX_STEP, MOTOR_TICK_MS);
  Serial.printf("  UDP cmd port = %d   FAILSAFE timeout = %dms\n",
                UDP_CMD_PORT, CMD_TIMEOUT_MS);
  Serial.println();

  if (!cameraInit()) {
    Serial.println("[ERROR] Camera init failed — halting");
    while (true) delay(1000);
  }
  Serial.println("[INIT] Camera OK");

  motorSetup();
  Serial.println("[INIT] Motors OK");

  // ── Wi-Fi ──
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("[WIFI] Scanning networks...");
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; ++i) {
    Serial.printf("  %d: %s (RSSI %d)\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
  }

  wifiMulti.addAP("seyay",         "123456780");
  wifiMulti.addAP("B.'s A35",      "abcd1234");
  wifiMulti.addAP("Qq", "22222222");
  // wifiMulti.addAP("qqq", "22222222");
    wifiMulti.addAP("Qq", "22222222");



  Serial.println("[WIFI] Connecting to strongest AP...");
  unsigned long start = millis();
  while (wifiMulti.run() != WL_CONNECTED && (millis() - start) < 25000) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    String ip = WiFi.localIP().toString();
    Serial.printf("[WIFI] Connected: %s  IP: %s\n", WiFi.SSID().c_str(), ip.c_str());
    Serial.printf("[WIFI] Stream URL  : http://%s/stream\n", ip.c_str());
    Serial.printf("[WIFI] Send cmds to: %s:%d  (UDP)\n", ip.c_str(), UDP_CMD_PORT);
    streamServerBegin();
    visionLinkBegin(UDP_CMD_PORT, 0);   // UDP listener on core 0
  } else {
    Serial.println("[WIFI] Connection failed — stream + command link disabled");
  }

  // Motor executor on core 1 (with loop()/MJPEG server). It keeps ticking
  // even while the stream handler is blocked serving a client.
  xTaskCreatePinnedToCore(motorExecutorTask, "motorExec", 4096, NULL, 2, NULL, 1);

  Serial.println();
  Serial.println("=== Running. Serial tags: [NET] link  [EXEC] executor/failsafe ===");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    streamServerHandle();
  }
  delay(1);   // yield
}
