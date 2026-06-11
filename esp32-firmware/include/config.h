// ============================================================
//  config.h — shared hardware map, tuning, and wire protocol
// ============================================================
//
//  This board is now a pure "Doer": it streams video and executes
//  motor commands sent by the PC ("Thinker"). ALL decision logic
//  (YOLO, EMA, state machine) lives on the PC. The only intelligence
//  retained here is LOW-LEVEL motor smoothing: minimum speed floor,
//  kickstart pulse, and slew-rate limiting — things that must run at
//  the hardware tick and cannot tolerate network latency.
// ============================================================
#pragma once
#include <stdint.h>

// ── AI Thinker ESP32-CAM camera pins ─────────────────────────
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ── Motor pins (L298N, forward-only: one direction pin per motor) ──
#define MOT_L_PWM    12   // ENA — Left  speed (PWM)
#define MOT_L_IN1    13   // IN1 — Left  forward enable
#define MOT_R_IN1    14   // IN3 — Right forward enable
#define MOT_R_PWM    15   // ENB — Right speed (PWM)

// ============================================================
//  Low-level motor refinements (retained on the ESP)
// ============================================================
// Kinetic floor: below this most TT motors stall once rolling. Any
// nonzero target is lifted to at least this. Breaking free from REST
// is handled separately by the kickstart.
#define MIN_SPEED        70
// Hard ceiling on applied duty.
#define MAX_SPEED        180
// Kickstart: one brief high pulse to break static friction when a wheel
// starts from a dead stop, then drop to the commanded cruise duty.
#define KICK_PWM         160
#define KICK_MS          60
// Slew: max duty INCREASE per motor tick (smooth acceleration). Falling
// is instant (braking always allowed).
#define SLEW_MAX_STEP    18
// Motor executor tick. Slew/kickstart run here, decoupled from the
// (jittery) network packet rate. ~40 Hz.
#define MOTOR_TICK_MS    25

// ============================================================
//  Vision/command link (UDP)
// ============================================================
#define UDP_CMD_PORT     3333
// FAILSAFE: if no valid command arrives within this window the executor
// stops the motors. Protects against PC crash / Wi-Fi drop / lost link.
#define CMD_TIMEOUT_MS   250

// PC -> ESP command packet. Authoritative fields are `left`/`right`
// (target duty 0..255 per wheel). `cmd` is an intent tag for telemetry.
// Fixed 10-byte little-endian layout — must match the Python sender.
#pragma pack(push, 1)
typedef struct {
  uint16_t magic;     // CMD_MAGIC
  uint8_t  version;   // CMD_VERSION
  uint8_t  cmd;       // CmdKind (telemetry/intent only)
  uint8_t  left;      // target left  duty 0..255
  uint8_t  right;     // target right duty 0..255
  uint32_t seq;       // PC monotonic counter
} MotorCmd;
#pragma pack(pop)

#define CMD_MAGIC     0x4332   // 'C2'
#define CMD_VERSION   1

enum CmdKind {
  CMD_STOP        = 0,   // SEARCH / STOPCLS  -> both wheels 0
  CMD_HOLD        = 1,   // WARMUP            -> both wheels 0 (filter settling)
  CMD_PIVOT_LEFT  = 2,   // TURN left         -> right wheel drives, left 0
  CMD_PIVOT_RIGHT = 3,   // TURN right        -> left wheel drives, right 0
  CMD_ADVANCE     = 4    // ADVANCE           -> both wheels forward
};

// ── Camera stream ──
#define STREAM_HTTP_PORT 80

// ============================================================
//  Static IP (so the stream URL / command target never changes)
// ============================================================
//  Set USE_STATIC_IP to 0 to fall back to DHCP. The address MUST be on
//  the SAME subnet as the AP you connect to, and SHOULD sit outside the
//  router's DHCP pool to avoid collisions. Values below match the
//  10.105.3.x / gateway .141 network ("B.'s A35 2" hotspot).
//  If you move to a different router/subnet, update these (or set 0).
#define USE_STATIC_IP    1
#define STATIC_IP_ADDR   10, 105, 3, 180
#define STATIC_GATEWAY   10, 105, 3, 141
#define STATIC_SUBNET    255, 255, 255, 0
#define STATIC_DNS       10, 105, 3, 141
