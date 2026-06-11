#!/usr/bin/env python3
# ============================================================
#  follower.py — "The Thinker"  (PC side)
# ============================================================
#  Captures the ESP32-CAM MJPEG stream, runs YOLO person detection,
#  smooths the target with an EMA filter, runs the full follow state
#  machine, and streams per-wheel motor commands down to the ESP32 over
#  UDP with minimum latency.
#
#  States  ->  command emitted to the ESP:
#    SEARCH  (no person)         -> STOP      (L=0,   R=0)
#    WARMUP  (just acquired)     -> HOLD      (L=0,   R=0)   EMA settling
#    TURN    (off-centre)        -> PIVOT L/R (one wheel drives, other 0)
#    ADVANCE (centred)           -> ADVANCE   (both wheels forward + trim)
#    STOPCLS (box too large)     -> STOP      (L=0,   R=0)
#
#  The ESP applies only kickstart / slew / min-speed to these duties and
#  halts if commands stop arriving (failsafe). All "thinking" is here.
#
#  Usage:
#    python follower.py --esp 192.168.1.50
#    python follower.py --esp 192.168.1.50 --url http://192.168.1.50/stream --show
#
#  See requirements.txt for dependencies.
# ============================================================

import argparse
import socket
import struct
import time
import sys

import cv2
import numpy as np
from ultralytics import YOLO

from telegram_notify import TelegramNotifier
import telegram_config

# ============================================================
#  Wire protocol — MUST match esp32-firmware/include/config.h
# ============================================================
CMD_MAGIC   = 0x4332          # 'C2'
CMD_VERSION = 1
# 10-byte little-endian: magic(H) version(B) cmd(B) left(B) right(B) seq(I)
CMD_STRUCT  = struct.Struct("<HBBBBI")

CMD_STOP        = 0
CMD_HOLD        = 1
CMD_PIVOT_LEFT  = 2
CMD_PIVOT_RIGHT = 3
CMD_ADVANCE     = 4
CMD_NAMES = {0: "STOP", 1: "HOLD", 2: "PIVOT_L", 3: "PIVOT_R", 4: "ADVANCE"}

# ============================================================
#  Tuning — the control "personality" now lives entirely on the PC
# ============================================================
# Per-wheel duties (0..255) the ESP receives. The ESP lifts anything
# nonzero to its own MIN_SPEED floor, so these are upper-level intents.
TURN_PWM        = 100     # single driven wheel during a pivot
ADVANCE_PWM     = 90      # both wheels while creeping forward
ADV_TRIM_MAX    = 12      # max heading trim added/subtracted while advancing
ADV_KP          = 16.0    # advance trim gain (trim = ADV_KP * error)

# EMA smoothing of the horizontal error (-1 left .. +1 right). Seeded to
# the first measurement on acquisition (no ramp-from-zero kick).
EMA_ALPHA       = 0.25

# Hysteresis "aligned" band on the smoothed error.
ALIGNED_ENTER   = 0.09    # become centred (ADVANCE) when |err| < this
ALIGNED_EXIT    = 0.16    # leave centred (TURN)    when |err| > this

# Edge guard: if the centroid is within this fraction of a frame edge,
# force a pivot back toward centre (never advance).
EDGE_MARGIN_FRAC = 0.09

# Distance gate: stop when the person's box covers more of the frame
# than this fraction (too close).
AREA_CLOSE_FRAC  = 0.40

WARMUP_FRAMES    = 2      # hold still N frames after acquisition (EMA settle)
LOST_FRAMES      = 3      # consecutive empty frames before declaring lost

# Detection
PERSON_CLASS_ID  = 0      # COCO "person"
CONF_THRESHOLD   = 0.40

# Control loop pacing target (also the command send rate to the ESP).
TARGET_HZ        = 30

# Swap left/right motors. The camera image is mirrored, so the robot was
# turning the wrong way; swapping the wheel outputs flips pivots AND the
# advance heading-trim together, consistently. Set False to disable.
SWAP_MOTORS      = True


# ============================================================
#  EMA filter
# ============================================================
class Ema:
    def __init__(self, alpha):
        self.alpha = alpha
        self.value = 0.0
        self.seeded = False

    def reset(self):
        self.value = 0.0
        self.seeded = False

    def update(self, raw):
        if not self.seeded:
            self.value = raw
            self.seeded = True
        else:
            self.value = self.alpha * raw + (1.0 - self.alpha) * self.value
        return self.value


# ============================================================
#  Follow state machine — decides per-wheel duties from the target
# ============================================================
class Follower:
    def __init__(self):
        self.ema = Ema(EMA_ALPHA)
        self.aligned = False
        self.warmup_remaining = WARMUP_FRAMES
        self.miss_count = 0
        self.fully_lost = True

    def _reset(self):
        self.ema.reset()
        self.aligned = False
        self.warmup_remaining = WARMUP_FRAMES

    def step(self, person):
        """person = (cx_norm, area_norm) or None. Returns (cmd, left, right)."""
        # ── No detection: brief gap pauses; sustained gap declares lost ──
        if person is None:
            self.miss_count += 1
            if self.miss_count >= LOST_FRAMES:
                if not self.fully_lost:
                    self._reset()
                    self.fully_lost = True
                return CMD_STOP, 0, 0          # SEARCH
            return CMD_HOLD, 0, 0              # brief dropout: hold, keep state

        cx_norm, area_norm = person
        self.miss_count = 0
        self.fully_lost = False

        # ── STOPCLS: too close ──
        if area_norm >= AREA_CLOSE_FRAC:
            self._reset()
            return CMD_STOP, 0, 0

        # ── error + EMA (seeded on first frame) ──
        raw_err = (cx_norm - 0.5) * 2.0       # -1 .. +1
        err = self.ema.update(raw_err)

        # ── WARMUP: hold still while the filter settles ──
        if self.warmup_remaining > 0:
            self.warmup_remaining -= 1
            return CMD_HOLD, 0, 0

        # ── EDGE GUARD: about to leave frame -> pivot back toward centre ──
        if cx_norm < EDGE_MARGIN_FRAC:
            self.aligned = False
            return CMD_PIVOT_LEFT, 0, TURN_PWM     # person far left -> yaw left
        if cx_norm > (1.0 - EDGE_MARGIN_FRAC):
            self.aligned = False
            return CMD_PIVOT_RIGHT, TURN_PWM, 0    # person far right -> yaw right

        # ── HYSTERESIS: TURN vs ADVANCE ──
        a = abs(err)
        if self.aligned:
            if a > ALIGNED_EXIT:
                self.aligned = False
        else:
            if a < ALIGNED_ENTER:
                self.aligned = True

        if self.aligned:
            # ADVANCE: creep straight with a small capped heading trim.
            trim = int(max(-ADV_TRIM_MAX, min(ADV_TRIM_MAX, ADV_KP * err)))
            left = max(0, min(255, ADVANCE_PWM + trim))   # err>0 (right) -> left faster
            right = max(0, min(255, ADVANCE_PWM - trim))
            return CMD_ADVANCE, left, right

        # TURN: pivot toward the person (one wheel drives, other stops).
        if err > 0:
            return CMD_PIVOT_RIGHT, TURN_PWM, 0    # person right -> drive LEFT wheel
        else:
            return CMD_PIVOT_LEFT, 0, TURN_PWM     # person left  -> drive RIGHT wheel


# ============================================================
#  UDP command sender
# ============================================================
class CommandLink:
    def __init__(self, esp_ip, port):
        self.addr = (esp_ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

    def send(self, cmd, left, right):
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        pkt = CMD_STRUCT.pack(CMD_MAGIC, CMD_VERSION, cmd,
                              int(left) & 0xFF, int(right) & 0xFF, self.seq)
        self.sock.sendto(pkt, self.addr)

    def stop(self):
        # Best-effort halt on shutdown.
        for _ in range(3):
            self.send(CMD_STOP, 0, 0)


# ============================================================
#  Vision: pick the most prominent person in a YOLO result
# ============================================================
def largest_person(result, frame_w, frame_h):
    """Return (cx_norm, area_norm) for the biggest person box, or None."""
    best_area = 0.0
    best = None
    boxes = result.boxes
    if boxes is None:
        return None
    for i in range(len(boxes)):
        cls = int(boxes.cls[i].item())
        conf = float(boxes.conf[i].item())
        if cls != PERSON_CLASS_ID or conf < CONF_THRESHOLD:
            continue
        x1, y1, x2, y2 = boxes.xyxy[i].tolist()
        area = (x2 - x1) * (y2 - y1)
        if area > best_area:
            best_area = area
            cx_norm = ((x1 + x2) * 0.5) / frame_w
            area_norm = area / (frame_w * frame_h)
            best = (cx_norm, area_norm)
    return best


# ============================================================
#  Main loop
# ============================================================
def main():
    ap = argparse.ArgumentParser(description="ESP32-CAM person-follower 'Thinker'")
    ap.add_argument("--esp", required=True, help="ESP32-CAM IP (for UDP commands)")
    ap.add_argument("--port", type=int, default=3333, help="ESP UDP command port")
    ap.add_argument("--url", default=None,
                    help="MJPEG stream URL (default http://<esp>/stream)")
    ap.add_argument("--model", default="yolov8n.pt", help="Ultralytics model")
    ap.add_argument("--imgsz", type=int, default=320, help="YOLO inference size")
    ap.add_argument("--show", action="store_true", help="Show annotated window")
    ap.add_argument("--telegram", action="store_true",
                    help="Send a snapshot + status to Telegram every interval")
    ap.add_argument("--tg-interval", type=float, default=5.0,
                    help="Seconds between Telegram notifications (default 5)")
    ap.add_argument("--chat-id", default=None,
                    help="Telegram chat id (default: auto-discover / env / config)")
    ap.add_argument("--source", default=None,
                    help="Video source override: webcam index (e.g. 0) or a file "
                         "path. Default = the ESP MJPEG stream. Use this to test "
                         "detection + Telegram WITHOUT the ESP32.")
    args = ap.parse_args()

    stream_url = args.url or f"http://{args.esp}/stream"

    # Choose the video source: webcam index, file, or the ESP stream.
    source = args.source if args.source is not None else stream_url
    if isinstance(source, str) and source.isdigit():
        source = int(source)

    print(f"[INIT] loading model {args.model} ...")
    model = YOLO(args.model)

    print(f"[INIT] opening source: {source!r} ...")
    cap = cv2.VideoCapture(source)
    # Keep the buffer at 1 so we always process the FRESHEST frame (low latency).
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    if not cap.isOpened():
        print(f"[ERROR] could not open source {source!r}.")
        print("        - ESP stream: is the board powered/streaming? open it in a browser.")
        print("        - To test without the ESP, run with:  --source 0   (laptop webcam)")
        sys.exit(1)
    print("[INIT] source opened OK")

    link = CommandLink(args.esp, args.port)
    follower = Follower()
    print(f"[INIT] sending UDP commands to {args.esp}:{args.port}")

    # ── Shared snapshot state for the Telegram thread ──
    #   GIL makes these single-reference reads/writes atomic; good enough.
    shared = {
        "frame": None,        # latest annotated BGR frame
        "fps": 0.0,
        "last_cmd": "STOP",
        "persons": 0,
        "last_frame_t": 0.0,  # wall time of last decoded frame
    }

    notifier = None
    if args.telegram:
        def build_snapshot():
            frame = shared["frame"]
            online = (time.time() - shared["last_frame_t"]) < 3.0 if shared["last_frame_t"] else False
            stamp = time.strftime("%Y-%m-%d %H:%M:%S")
            caption = (
                f"Robot status @ {stamp}\n"
                f"ESP: {args.esp}  ({'ONLINE' if online else 'OFFLINE / no stream'})\n"
                f"Stream FPS: {shared['fps']:.1f}\n"
                f"Persons in view: {shared['persons']}\n"
                f"Last command: {shared['last_cmd']}"
            )
            if frame is None:
                return None, caption
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                return None, caption
            return buf.tobytes(), caption

        chat = args.chat_id or telegram_config.CHAT_ID or None
        notifier = TelegramNotifier(telegram_config.BOT_TOKEN, chat_id=chat)
        print(f"[TG] person alerts ON — fire on detection, >= {args.tg_interval:.0f}s between alerts")
        # Startup ping: confirms the bot path works right now, before any detection.
        if notifier.send_message("Follower started — person alerts active."):
            print(f"[TG] startup message sent OK (chat_id={notifier.chat_id})")
        else:
            print("[TG] startup message FAILED — check token/chat_id "
                  "(send your bot a /start message once).")

    print("[RUN ] Ctrl-C to stop\n")

    period = 1.0 / TARGET_HZ
    last_fps_t = time.time()
    frames = 0
    last_tx = None            # last (cmd,left,right) sent to the ESP
    nofr = 0                  # consecutive failed frame reads
    last_nofr_log = 0.0

    try:
        while True:
            t0 = time.time()
            ok, frame = cap.read()
            if not ok or frame is None:
                # Stream hiccup: tell the robot to hold, keep the link alive.
                link.send(CMD_HOLD, 0, 0)
                nofr += 1
                if time.time() - last_nofr_log > 2.0:
                    print(f"[WARN] no frame from source ({nofr} consecutive) — "
                          f"stream down? Telegram alerts need frames to fire.")
                    last_nofr_log = time.time()
                time.sleep(0.05)
                continue
            if nofr:
                print(f"[INFO] frames resumed after {nofr} failures")
                nofr = 0

            h, w = frame.shape[:2]
            # Detect ONLY the person class; conf-filter at the model level.
            # result.plot() then draws person bounding boxes (and nothing else).
            result = model.predict(frame, imgsz=args.imgsz,
                                   classes=[PERSON_CLASS_ID], conf=CONF_THRESHOLD,
                                   verbose=False)[0]
            person = largest_person(result, w, h)

            cmd, left, right = follower.step(person)
            if SWAP_MOTORS:
                left, right = right, left
                if cmd == CMD_PIVOT_LEFT:
                    cmd = CMD_PIVOT_RIGHT
                elif cmd == CMD_PIVOT_RIGHT:
                    cmd = CMD_PIVOT_LEFT
            link.send(cmd, left, right)
            # Log every motion command we send to the ESP (only when it changes).
            if (cmd, left, right) != last_tx:
                print(f"[CMD->ESP {args.esp}:{args.port}] {CMD_NAMES[cmd]:7s} "
                      f"L={left:3d} R={right:3d}")
                last_tx = (cmd, left, right)

            # ── Update shared state for the Telegram thread ──
            shared["last_cmd"] = CMD_NAMES[cmd]
            shared["last_frame_t"] = time.time()
            if result.boxes is not None:
                shared["persons"] = int(sum(
                    1 for i in range(len(result.boxes))
                    if int(result.boxes.cls[i].item()) == PERSON_CLASS_ID
                    and float(result.boxes.conf[i].item()) >= CONF_THRESHOLD
                ))
            if args.show or args.telegram:
                vis = result.plot()
                cv2.putText(vis, f"{CMD_NAMES[cmd]}  L={left} R={right}", (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                shared["frame"] = vis

            # ── Telegram: fire the moment a person is detected, then this
            #    is rate-limited to one alert per --tg-interval (5s) seconds.
            if notifier is not None and shared["persons"] > 0:
                if notifier.maybe_send_async(build_snapshot, args.tg_interval):
                    print(f"[TG] person detected ({shared['persons']}) -> sending snapshot")

            # ── Telemetry ──
            frames += 1
            now = time.time()
            if now - last_fps_t >= 1.0:
                fps = frames / (now - last_fps_t)
                shared["fps"] = fps
                if person:
                    cxn, an = person
                    print(f"[{CMD_NAMES[cmd]:7s}] L={left:3d} R={right:3d} | "
                          f"cx={cxn:.2f} area={an:.2f} err_ema={follower.ema.value:+.2f} | {fps:4.1f} fps")
                else:
                    print(f"[{CMD_NAMES[cmd]:7s}] L={left:3d} R={right:3d} | no person | {fps:4.1f} fps")
                last_fps_t = now
                frames = 0

            # ── Optional visualisation (reuses the annotated frame) ──
            if args.show and shared["frame"] is not None:
                cv2.imshow("Thinker", shared["frame"])
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            # Pace the loop to TARGET_HZ (don't flood the ESP).
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)

    except KeyboardInterrupt:
        print("\n[STOP] interrupted")
    finally:
        if notifier is not None:
            notifier.stop()
        link.stop()                 # halt the robot
        cap.release()
        if args.show:
            cv2.destroyAllWindows()
        print("[STOP] sent STOP, released stream. Bye.")


if __name__ == "__main__":
    main()
