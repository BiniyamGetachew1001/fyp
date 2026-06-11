# ============================================================
#  telegram_notify.py — periodic Telegram snapshot + status
# ============================================================
#  A small, dependency-light (requests) notifier that sends a photo +
#  caption to a Telegram chat on a fixed interval, from its own daemon
#  thread so it never stalls the control loop.
#
#  Chat ID: if you don't pass one, it is auto-discovered from getUpdates
#  (the most recent message sent TO the bot). So the one-time setup is:
#  open Telegram, find your bot, and send it any message (e.g. /start).
# ============================================================
import threading
import time

import requests


class TelegramNotifier:
    API = "https://api.telegram.org/bot{token}/{method}"

    def __init__(self, token, chat_id=None, timeout=15):
        self.token = token
        self.chat_id = str(chat_id) if chat_id else None
        self.timeout = timeout
        self._thread = None
        self._stop = threading.Event()
        self._last_send = -1e9     # monotonic time of last send (force first)
        self._sending = False      # a send is currently in flight

    # ── low-level ──
    def _url(self, method):
        return self.API.format(token=self.token, method=method)

    def resolve_chat_id(self):
        """Return the chat id, auto-discovering it from getUpdates if needed."""
        if self.chat_id:
            return self.chat_id
        try:
            r = requests.get(self._url("getUpdates"), timeout=self.timeout)
            for upd in reversed(r.json().get("result", [])):
                msg = upd.get("message") or upd.get("channel_post")
                if msg and "chat" in msg:
                    self.chat_id = str(msg["chat"]["id"])
                    print(f"[TG] discovered chat_id = {self.chat_id}")
                    return self.chat_id
        except Exception as e:
            print(f"[TG] getUpdates failed: {e}")
        print("[TG] no chat_id yet — send your bot any message (e.g. /start) once.")
        return None

    def send_message(self, text):
        cid = self.resolve_chat_id()
        if not cid:
            return False
        try:
            r = requests.post(self._url("sendMessage"),
                              data={"chat_id": cid, "text": text},
                              timeout=self.timeout)
            return r.ok
        except Exception as e:
            print(f"[TG] sendMessage failed: {e}")
            return False

    def send_photo(self, jpeg_bytes, caption=""):
        cid = self.resolve_chat_id()
        if not cid:
            return False
        try:
            files = {"photo": ("snapshot.jpg", jpeg_bytes, "image/jpeg")}
            r = requests.post(self._url("sendPhoto"),
                              data={"chat_id": cid, "caption": caption},
                              files=files, timeout=self.timeout)
            if not r.ok:
                print(f"[TG] sendPhoto HTTP {r.status_code}: {r.text[:120]}")
            return r.ok
        except Exception as e:
            print(f"[TG] sendPhoto failed: {e}")
            return False

    # ── event-driven (detection-triggered) send with a cooldown ──
    def maybe_send_async(self, snapshot_fn, cooldown):
        """Call this whenever a person is detected. Sends a snapshot in a
        background thread IF at least `cooldown` seconds have passed since
        the last send and no send is currently in flight. Non-blocking, so
        it never stalls the control loop. Returns True if a send was fired."""
        now = time.monotonic()
        if self._sending or (now - self._last_send) < cooldown:
            return False
        self._last_send = now
        self._sending = True

        def run():
            try:
                jpeg, caption = snapshot_fn()
                if jpeg is not None:
                    ok = self.send_photo(jpeg, caption)
                    print(f"[TG] photo delivered: {ok}")
                else:
                    ok = self.send_message(caption)
                    print(f"[TG] text delivered (no frame): {ok}")
            except Exception as e:
                print(f"[TG] send error: {e}")
            finally:
                self._sending = False

        threading.Thread(target=run, daemon=True).start()
        return True

    # ── periodic worker (unused in detection-triggered mode) ──
    def start_periodic(self, interval, snapshot_fn):
        """snapshot_fn() -> (jpeg_bytes_or_None, caption_str). Sends every
        `interval` seconds (and once immediately). Photo if bytes present,
        else a text-only status message."""
        def run():
            # one immediate send to confirm the link, then on interval
            while not self._stop.is_set():
                try:
                    jpeg, caption = snapshot_fn()
                    if jpeg is not None:
                        self.send_photo(jpeg, caption)
                    else:
                        self.send_message(caption)
                except Exception as e:
                    print(f"[TG] periodic error: {e}")
                self._stop.wait(interval)

        self._thread = threading.Thread(target=run, daemon=True, name="telegram")
        self._thread.start()
        print(f"[TG] periodic notifier started ({interval:.0f}s interval)")

    def stop(self):
        self._stop.set()
