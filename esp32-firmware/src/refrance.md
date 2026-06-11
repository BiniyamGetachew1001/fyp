// ============================================================
//  ESP32-CAM Red Object Follower  —  STABLE CORRECTED VERSION
//  Board    : AI Thinker ESP32-CAM
//  Partition: Huge APP (3MB No OTA)
//  Baud     : 115200
// ============================================================

#include "esp_camera.h"
#include "Arduino.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "freertos/semphr.h"
#include "img_converters.h" 

// ---- WiFi credentials -----------------------------------
#define WIFI_SSID  "YourSSID"
#define WIFI_PASS  "YourPassword"

// ---- Camera pins (AI Thinker) ---------------------------
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM   0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM     5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

// ---- Motor pins -----------------------------------------
#define IN1 13
#define IN2 15
#define IN3 14
#define IN4  4
#define ENA 12
#define ENB 16

// ---- Speeds (3-6V TT motors on 7.4V supply) -------------
#define SPD_CLOSE   85
#define SPD_FAR    149
#define SPD_SEARCH  80

// ---- Red detection tuning --------------------------------
#define BLOB_MIN     150    // Lowered threshold to detect smaller/further red objects
#define DEAD_BAND_PCT   5
#define AREA_CLOSE  19200   // 25% of 320x240 equivalent 
#define AREA_STOP   35000   // ~45% of frame equivalent

// ---- Safety limits --------------------------------------
#define MAX_CONSECUTIVE_CAM_ERRORS 50  
#define MOTOR_RAMP_STEP            20  

static SemaphoreHandle_t camMutex = NULL;
static int continuousCamErrors = 0;

static int currentSpdL = 0;
static int currentSpdR = 0;

#define SCALED_WIDTH  80
#define SCALED_HEIGHT 60
static uint8_t rgb565StaticBuf[SCALED_WIDTH * SCALED_HEIGHT * 2];

// ---- JPEG Buffer Integrity Verification -----------------
bool isValidJPEG(camera_fb_t* fb) {
  if (!fb || fb->len < 4) return false;
  if (fb->buf[0] != 0xFF || fb->buf[1] != 0xD8) return false;
  if (fb->buf[fb->len - 2] != 0xFF || fb->buf[fb->len - 1] != 0xD9) return false;
  return true;
}

// ---- Soft-Start Motor Ramping ----------------------------
void writeMotorSpeeds(int targetL, int targetR) {
  if (targetL > currentSpdL) {
    currentSpdL = min(currentSpdL + MOTOR_RAMP_STEP, targetL);
  } else {
    currentSpdL = targetL; 
  }

  if (targetR > currentSpdR) {
    currentSpdR = min(currentSpdR + MOTOR_RAMP_STEP, targetR);
  } else {
    currentSpdR = targetR;
  }

  ledcWrite(ENA, constrain(currentSpdL, 0, 255));
  ledcWrite(ENB, constrain(currentSpdR, 0, 255));
}

void motorsOff() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  currentSpdL = 0;
  currentSpdR = 0;
  ledcWrite(ENA, 0);      
  ledcWrite(ENB, 0);
}

void goForward(int l, int r) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  writeMotorSpeeds(l, r);
}

void spinLeft(int spd) {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  writeMotorSpeeds(spd, spd);
}


// ---- Camera initialization ------------------------------
bool startCamera() {
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0       = Y2_GPIO_NUM;  cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2       = Y4_GPIO_NUM;  cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4       = Y6_GPIO_NUM;  cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6       = Y8_GPIO_NUM;  cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;
  
  cfg.xclk_freq_hz = 10000000; 
  cfg.pixel_format = PIXFORMAT_JPEG;   
  cfg.frame_size   = FRAMESIZE_QVGA;   
  cfg.jpeg_quality = 15; 
  cfg.fb_count     = 2;
  return esp_camera_init(&cfg) == ESP_OK;
}

// ---- Corrected Red detection ----------------------------
int detectRed(int* outArea) {
  *outArea = 0;

  if (camMutex == NULL) {
    return -1;
  }

  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(30)) != pdTRUE) {
    return -1; 
  }

  camera_fb_t* fb = esp_camera_fb_get();
  
  if (!fb || !isValidJPEG(fb)) {
    if (fb) {
      esp_camera_fb_return(fb);
    }
    xSemaphoreGive(camMutex);
    continuousCamErrors++;
    
    if (continuousCamErrors > MAX_CONSECUTIVE_CAM_ERRORS) {
      Serial.println("\n[CRITICAL] Camera hardware detached or unresponsive. Halting.");
      motorsOff();
      delay(1000);
      ESP.restart(); 
    }
    return -1;
  }

  continuousCamErrors = 0; 

  int outW = fb->width / 4;  
  int outH = fb->height / 4; 
  if (outW > SCALED_WIDTH || outH > SCALED_HEIGHT) {
    esp_camera_fb_return(fb);
    xSemaphoreGive(camMutex);
    return -1;
  }

  bool decoded = jpg2rgb565(fb->buf, fb->len, rgb565StaticBuf, JPG_SCALE_4X);

  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);

  if (!decoded) {
    return -1;
  }

  long sumX  = 0;
  int  count = 0;

  for (int y = 0; y < outH; y++) {
    for (int x = 0; x < outW; x++) {
      int i = (y * outW + x) * 2;
      
      // Reconstruct standard 16-bit little-endian value
      uint16_t pixel = rgb565StaticBuf[i] | (rgb565StaticBuf[i + 1] << 8);

      // Extract colors correctly
      uint8_t r = ((pixel >> 11) & 0x1F) << 3; 
      uint8_t g = ((pixel >> 5) & 0x3F) << 2;  
      uint8_t b = (pixel & 0x1F) << 3;         

      // Unified, relative red threshold
      bool isRed = (r > 90) && (r > (g + 45)) && (r > (b + 45));

      if (isRed) {
        sumX += x;
        count++;
      }
    }
  }

  int scaled_count = count * 16;
  *outArea = scaled_count;

  if (scaled_count < BLOB_MIN) return -1;
  
  return (int)((sumX / count) * 4);
}

// ---- MJPEG stream handler ---------------------------------
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CT =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUND = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART  =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t streamHandler(httpd_req_t* req) {
  char partBuf[64];
  httpd_resp_set_type(req, STREAM_CT);

  while (true) {
    if (camMutex == NULL) {
      vTaskDelay(pdMS_TO_TICKS(50));
      break;
    }

    if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(150)) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(15)); 
      continue;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    
    if (!fb || !isValidJPEG(fb)) {
      if (fb) {
        esp_camera_fb_return(fb);
      }
      xSemaphoreGive(camMutex);
      vTaskDelay(pdMS_TO_TICKS(15)); 
      continue;
    }

    esp_err_t res = httpd_resp_send_chunk(req, STREAM_BOUND, strlen(STREAM_BOUND));
    if (res == ESP_OK) {
      size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, partBuf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    }


    esp_camera_fb_return(fb);
    xSemaphoreGive(camMutex);

    if (res != ESP_OK) break;
    
    vTaskDelay(pdMS_TO_TICKS(15)); 
  }
  return ESP_OK;
}

// ---- HTML Handler ---------------------------------------
esp_err_t indexHandler(httpd_req_t* req) {
  const char* html =
    "<!DOCTYPE html><html><head>"
    "<title>ESP32-CAM Red Follower</title>"
    "<style>"
    "body{background:#111;color:#eee;font-family:sans-serif;"
    "text-align:center;padding:20px;margin:0;}"
    "h1{color:#e44;margin-bottom:8px;}"
    "img{width:min(640px,96vw);border:3px solid #e44;border-radius:8px;}"
    "p{color:#888;font-size:13px;}"
    "</style></head><body>"
    "<h1>Red Follower — Live View</h1>"
    "<img src='/stream' />"
    "<p>MJPEG stream | Red detection runs simultaneously on the robot.</p>"
    "</body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, strlen(html));
}

void startStreamServer() {
  httpd_config_t config  = HTTPD_DEFAULT_CONFIG();
  config.server_port     = 80;
  config.stack_size      = 8192;

  httpd_handle_t server  = NULL;
  if (httpd_start(&server, &config) != ESP_OK) return;

  httpd_uri_t idx = { "/",       HTTP_GET, indexHandler,  NULL };
  httpd_uri_t st  = { "/stream", HTTP_GET, streamHandler, NULL };
  httpd_register_uri_handler(server, &idx);
  httpd_register_uri_handler(server, &st);
}

// ---- SETUP ----------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== Red Follower + Live View ===");

  camMutex = xSemaphoreCreateMutex();
  if (camMutex == NULL) {
    Serial.println("[CRITICAL] Mutex initialization failed! Halting.");
    while (true) delay(1000);
  }

  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, HIGH); delay(500);
  digitalWrite(PWDN_GPIO_NUM, LOW);  delay(500);

  Serial.print("Camera... ");
  if (!startCamera()) {
    Serial.println("FAILED — check ribbon cable + Huge APP partition");
    while (true) delay(1000);
  }
  Serial.println("OK");

  Serial.printf("WiFi: connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nOpen browser: http://%s\n",
                  WiFi.localIP().toString().c_str());
    startStreamServer();
  } else {
    Serial.println("\nWiFi failed — stream disabled, tracking still works.");
  }

  // Motor pins config
  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);

  pinMode(ENA, OUTPUT); digitalWrite(ENA, LOW);
  pinMode(ENB, OUTPUT); digitalWrite(ENB, LOW);
  ledcAttachChannel(ENA, 1000, 8, 2);
  ledcAttachChannel(ENB, 1000, 8, 3);

  motorsOff();
  Serial.println("Motors OK. Tracking started.");
}

// ---- LOOP -----------------------------------------------
void loop() {
  int area = 0;
  int cx   = detectRed(&area);

  if (cx == -1) {
    Serial.println("[SEARCH] no red");
    spinLeft(SPD_SEARCH);
    delay(80);
    motorsOff();
  } else if (area >= AREA_STOP) {
    motorsOff();
    Serial.printf("[STOP]  area=%d\n", area);
  } else {
    int frameW   = 320;
    int deadBand = frameW * DEAD_BAND_PCT / 100;
    int error    = cx - (frameW / 2);
    int spd      = (area >= AREA_CLOSE) ? SPD_CLOSE : SPD_FAR;

    Serial.printf("[%s] cx=%d err=%d area=%d spd=%d\n",
                  (area >= AREA_CLOSE) ? "CLOSE" : "FAR",
                  cx, error, area, spd);

    if (abs(error) <= deadBand) {
      goForward(spd, spd);
    } else if (error < 0) {
      int corr = constrain(abs(error) * 2, 10, 80);
      goForward(max(spd - corr, 0), spd);
    } else {
      int corr = constrain(abs(error) * 2, 10, 80);
      goForward(spd, max(spd - corr, 0));
    }
  }

  delay(15);
}
Skip to main content We have updated our Terms of Service Robot Movement Troubleshooting And 
Code Analysis 50,130 tokens Of course. This is a fantastic idea because it combines the best of both 
worlds: the robot will drive autonomously, but you can also watch a live video feed from its "eye" 
in your web browser to debug and see what it's seeing. This requires merging the two pieces of code 
carefully. The main challenge is that the camera must be configured to output JPEG for the web 
stream, but the color tracking logic needs RGB to analyze pixels. We will prioritize the robot's 
function. Here is the fully merged and corrected code. It is designed to track red objects, and as a 
bonus, it will also stream the video. 
⚠️
 Critical Step: The Physical Wiring Change Because we are 
using the "AI Thinker ESP32-CAM" board setting (which is correct), PSRAM will be enabled. You 
MUST move the motor wire to avoid a conflict with Pin 16. Unplug the wire going to Pin 16 (ENB) 
on your ESP32-CAM. Plug that same wire into Pin 33. It's a safe, available pin. The code below 
already reflects this change (#define ENB 33). The Combined Color Tracking + WiFi Stream Code 
code C++ #include "esp_camera.h" #include "Arduino.h" #include <WiFi.h> #include 
<WebServer.h> // 
============================================================= // ESP32
CAM AUTONOMOUS RED TRACKING ROBOT WITH WIFI STREAM // 
============================================================= // ── 
Credentials (CHANGE THESE!) 
────────────────────────────────────────── const char* SSID = 
"YOUR_WIFI_SSID"; const char* PASSWORD = "YOUR_WIFI_PASSWORD"; // ── Motor Pin 
definitions (NOTE: ENB is now on Pin 33!) ────────────────── #define IN1 13 
#define IN2 14 #define IN3 15 #define IN4 2 #define ENA 12 #define ENB 33 // MOVED FROM 
PIN 16 TO AVOID PSRAM CONFLICT // ── Camera Pin config (AI-Thinker ESP32-CAM) 
────────────────────────────── #define PWDN_GPIO_NUM 32 #define 
RESET_GPIO_NUM -1 #define XCLK_GPIO_NUM 0 #define SIOD_GPIO_NUM 26 #define 
SIOC_GPIO_NUM 27 #define Y9_GPIO_NUM 35 #define Y8_GPIO_NUM 34 #define 
Y7_GPIO_NUM 39 #define Y6_GPIO_NUM 36 #define Y5_GPIO_NUM 21 #define 
Y4_GPIO_NUM 19 #define Y3_GPIO_NUM 18 #define Y2_GPIO_NUM 5 #define 
VSYNC_GPIO_NUM 25 #define HREF_GPIO_NUM 23 #define PCLK_GPIO_NUM 22 // ── 
Tuning Parameters 
───────────────────────────────────────────────────── // 
Speeds int forwardLeftSpeed = 105; int forwardRightSpeed = 85; int turningSpeed = 95; // Vision 
const int RED_MIN = 130; const int COLOR_MARGIN = 40; const int MIN_BLOB_SIZE = 30; 
WebServer server(80); // ── Camera init 
────────────────────────────────────────────────────────
─── bool initCamera() { camera_config_t config; config.ledc_channel = LEDC_CHANNEL_0; 
config.ledc_timer = LEDC_TIMER_0; config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = 
Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM; 
config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM; config.pin_d6 = 
Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM; config.pin_xclk = XCLK_GPIO_NUM; 
config.pin_pclk = PCLK_GPIO_NUM; config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href 
= HREF_GPIO_NUM; config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = 
SIOC_GPIO_NUM; config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = 
RESET_GPIO_NUM; config.xclk_freq_hz = 20000000; // CRITICAL: We use JPEG for streaming, 
and RGB565 for analysis. // We will switch formats on the fly in the loop. config.pixel_format = 
PIXFORMAT_JPEG; config.frame_size = FRAMESIZE_QVGA; // 320x240 - good balance 
config.jpeg_quality = 12; config.fb_count = 2; // Use 2 frame buffers for smoother operation return 
esp_camera_init(&config) == ESP_OK; } // ── Web Server Handlers 
─────────────────────────────────────────────────── void 
handleStream() { WiFiClient client = server.client(); client.println("HTTP/1.1 200 OK"); 
client.println("Content-Type: multipart/x-mixed-replace; boundary=frame"); client.println(); while 
(client.connected()) { camera_fb_t *fb = esp_camera_fb_get(); if (!fb) { Serial.println("Stream: 
Frame capture failed"); break; } client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent
Length: %zu\r\n\r\n", fb->len); client.write(fb->buf, fb->len); client.println(); 
esp_camera_fb_return(fb); } } void handleRoot() { server.send(200, "text/html", "<!DOCTYPE 
html><html><body style='margin:0; background:#000;'><img src='/stream' style='width:100vw; 
height:100vh; object-fit:contain;'/></body></html>"); } // ── SETUP: Runs once at boot 
────────────────────────────────────────────── void setup() 
{ Serial.begin(115200); // 1. Motor Setup pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); 
pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENA, OUTPUT); pinMode(ENB, 
OUTPUT); stopMotors(); // 2. Camera Setup Serial.print("Initialising camera... "); if (!
initCamera()) { Serial.println("FAILED — halting"); while (true) delay(1000); } 
Serial.println("OK"); // 3. WiFi and Web Server Setup Serial.printf("Connecting to %s ", SSID); 
WiFi.begin(SSID, PASSWORD); while (WiFi.status() != WL_CONNECTED) { delay(500); 
Serial.print("."); } Serial.printf("\nConnected! Open http://%s in your browser\n", 
WiFi.localIP().toString().c_str()); server.on("/", handleRoot); server.on("/stream", handleStream); 
server.begin(); Serial.println("Web server started. Robot is now in autonomous mode."); } // ── 
LOOP: Main robot logic 
──────────────────────────────────────────────── void loop() 
{ // MUST RUN: This keeps the web server responsive. server.handleClient(); // 1. Capture a frame 
for analysis // We need to temporarily switch to RGB565 to read pixel data sensor_t * s = 
esp_camera_sensor_get(); s->set_pixformat(s, PIXFORMAT_RGB565); // Switch to RGB 
camera_fb_t * fb = esp_camera_fb_get(); if (!fb) { Serial.println("Analysis: Frame capture failed"); 
s->set_pixformat(s, PIXFORMAT_JPEG); // Switch back before exiting return; } // 2. Scan the 
picture for RED pixels (same logic as before) long total_x = 0; int red_pixel_count = 0; for (int i = 
0; i < fb->len; i += 2) { uint8_t high = fb->buf[i]; uint8_t low = fb->buf[i+1]; uint8_t r = (high & 
0xF8); uint8_t g = ((high & 0x07) << 5) | ((low & 0xE0) >> 3); uint8_t b = (low & 0x1F) << 3; if (r 
> RED_MIN && r > (g + COLOR_MARGIN) && r > (b + COLOR_MARGIN)) { int pixel_index 
= i / 2; int x = pixel_index % fb->width; // Use fb->width for accuracy total_x += x; 
red_pixel_count++; } } esp_camera_fb_return(fb); // IMPORTANT: Return RGB frame memory // 
3. Switch camera back to JPEG mode for the web stream s->set_pixformat(s, 
PIXFORMAT_JPEG); // 4. Make driving decisions if (red_pixel_count > MIN_BLOB_SIZE) { int 
center_x = total_x / red_pixel_count; int center_of_view = fb->width / 2; int error_margin = fb
>width / 5; // e.g., 320 / 5 = 64px margin if (center_x < (center_of_view - error_margin)) 
{ turnLeft(); } else if (center_x > (center_of_view + error_margin)) { turnRight(); } else 
{ moveForward(); } } else { stopMotors(); } } // ── Motor Control Helper Functions 
──────────────────────────────────────── void moveForward() 
{ digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, HIGH); digitalWrite(IN4, 
LOW); analogWrite(ENA, forwardLeftSpeed); analogWrite(ENB, forwardRightSpeed); } void 
turnLeft() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); digitalWrite(IN3, HIGH); 
digitalWrite(IN4, LOW); analogWrite(ENA, turningSpeed); analogWrite(ENB, turningSpeed); } 
void turnRight() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); 
digitalWrite(IN4, HIGH); analogWrite(ENA, turningSpeed); analogWrite(ENB, turningSpeed); } 
void stopMotors() { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); 
digitalWrite(IN4, LOW); analogWrite(ENA, 0); analogWrite(ENB, 0); } How to Upload and Run 
Arduino IDE Settings: Board: "AI Thinker ESP32-CAM" Partition Scheme: "Huge APP (3MB No 
OTA/1MB SPIFFS)" Update your WiFi credentials at the top of the code. Upload the code. Open 
the Serial Monitor at 115200 baud. Wait for it to connect to your WiFi and print the IP address. 
Unplug the USB programmer. Power the robot with its batteries (it needs full power to run the 
motors and camera). Open a web browser on your phone or computer (must be on the same WiFi 
network) and go to the IP address shown in the Serial Monitor. You should now see a live video 
stream, and the robot should start searching for and following red objects autonomously. Citations 
https://visualgdb.com/tutorials/esp32/remotevideo/ https://github.com/tensorflow/tflite-micro 
https://forum.arduino.cc/t/matchboxscope-with-esp32-cam/1127958 Model 9:26 AM 
ThinkingThoughts Auto Expand to view model thoughts chevron_right You are absolutely correct. 
My apologies. The standard ESP32-CAM development board does not have a header pin for 
GPIO33, even though the chip itself has that pin (it's hardwired to the small red LED on the back). 
Thank you for catching that! We need to choose a different "safe" pin that is actually available on 
your board's headers. Based on the pins you are already using (13, 14, 15, 2), you still need two 
PWM-capable pins for ENA and ENB. The Best Available Pin: GPIO 4 The best and easiest 
solution is to use Pin 4. This pin is normally used to control the super-bright white LED (the flash). 
Since the robot will be driving autonomously and doesn't need the flash, we can use that pin to 
control the motor speed instead. Your Corrected Wiring and Code Here is the complete plan. Let's 
assume you have the Left Motor's ENA on Pin 12. Step 1: The Final Wiring Configuration This is 
the complete and correct wiring for your board: IN1 (Left Motor) -> Pin 13 IN2 (Left Motor) -> Pin 
14 IN3 (Right Motor) -> Pin 15 IN4 (Right Motor) -> Pin 2 ENA (Left Speed) -> Pin 12 ENB 
(Right Speed) -> Pin 4 (This is the new, correct pin) Important Note: Pins 2, 12, 13, 14, and 15 are 
also used by the SD card slot. For this to work, you must make sure you do not have an SD card 
inserted into the ESP32-CAM. Step 2: The Updated Code I have taken the previous combined code 
and changed only one line to reflect the new wiring. code C++ #include "esp_camera.h" #include 
"Arduino.h" #include <WiFi.h> #include <WebServer.h> // 
============================================================= // ESP32
CAM AUTONOMOUS RED TRACKING ROBOT WITH WIFI STREAM // 
============================================================= // ── 
Credentials (CHANGE THESE!) 
────────────────────────────────────────── const char* SSID = 
"YOUR_WIFI_SSID"; const char* PASSWORD = "YOUR_WIFI_PASSWORD"; // ── Motor Pin 
definitions (CORRECTED FOR YOUR BOARD) ───────────────────── #define 
IN1 13 #define IN2 14 #define IN3 15 #define IN4 2 #define ENA 12 #define ENB 4 // MOVED 
TO PIN 4 (The Flash LED Pin) - THIS IS A SAFE PIN // ── Camera Pin config (AI-Thinker 
ESP32-CAM) ────────────────────────────── #define PWDN_GPIO_NUM 
32 // ... (rest of the camera pins are the same as before) #define RESET_GPIO_NUM -1 #define 
XCLK_GPIO_NUM 0 #define SIOD_GPIO_NUM 26 #define SIOC_GPIO_NUM 27 #define 
Y9_GPIO_NUM 35 #define Y8_GPIO_NUM 34 #define Y7_GPIO_NUM 39 #define 
Y6_GPIO_NUM 36 #define Y5_GPIO_NUM 21 #define Y4_GPIO_NUM 19 #define 
Y3_GPIO_NUM 18 #define Y2_GPIO_NUM 5 #define VSYNC_GPIO_NUM 25 #define 
HREF_GPIO_NUM 23 #define PCLK_GPIO_NUM 22 // ── Tuning Parameters 
───────────────────────────────────────────────────── // 
Speeds int forwardLeftSpeed = 105; int forwardRightSpeed = 85; int turningSpeed = 95; // Vision 
const int RED_MIN = 130; const int COLOR_MARGIN = 40; const int MIN_BLOB_SIZE = 30; 
WebServer server(80); // The rest of the code (initCamera, web server, loop, motor functions) // 
remains exactly the same as the previous version. // You can copy and paste the rest of the code 
from the previous response. // ... PASTE THE REST OF THE CODE HERE ... // (The full code is 
long, so I am omitting it for brevity, // but it is identical to the last complete example I gave you). // 
Let's include it all for clarity: // ── Camera init 
────────────────────────────────────────────────────────
─── bool initCamera() { camera_config_t config; config.ledc_channel = LEDC_CHANNEL_0; 
config.ledc_timer = LEDC_TIMER_0; config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = 
Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM; 
config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM; config.pin_d6 = 
Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM; config.pin_xclk = XCLK_GPIO_NUM; 
config.pin_pclk = PCLK_GPIO_NUM; config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href 
= HREF_GPIO_NUM; config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = 
SIOC_GPIO_NUM; config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = 
RESET_GPIO_NUM; config.xclk_freq_hz = 20000000; config.pixel_format = 
PIXFORMAT_JPEG; config.frame_size = FRAMESIZE_QVGA; // 320x240 - good balance 
config.jpeg_quality = 12; config.fb_count = 2; return esp_camera_init(&config) == ESP_OK; } // 
── Web Server Handlers 
─────────────────────────────────────────────────── void 
handleStream() { WiFiClient client = server.client(); client.println("HTTP/1.1 200 OK"); 
client.println("Content-Type: multipart/x-mixed-replace; boundary=frame"); client.println(); while 
(client.connected()) { camera_fb_t *fb = esp_camera_fb_get(); if (!fb) { Serial.println("Stream: 
Frame capture failed"); break; } client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent
Length: %zu\r\n\r\n", fb->len); client.write(fb->buf, fb->len); client.println(); 
esp_camera_fb_return(fb); } } void handleRoot() { server.send(200, "text/html", "<!DOCTYPE 
html><html><body style='margin:0; background:#000;'><img src='/stream' style='width:100vw; 
height:100vh; object-fit:contain;'/></body></html>"); } // ── SETUP: Runs once at boot 
────────────────────────────────────────────── void setup() 
{ Serial.begin(115200); // 1. Motor Setup pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); 
pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENA, OUTPUT); pinMode(ENB, 
OUTPUT); stopMotors(); // 2. Camera Setup Serial.print("Initialising camera... "); if (!
initCamera()) { Serial.println("FAILED — halting"); while (true) delay(1000); } 
Serial.println("OK"); // 3. WiFi and Web Server Setup Serial.printf("Connecting to %s ", SSID); 
WiFi.begin(SSID, PASSWORD); while (WiFi.status() != WL_CONNECTED) { delay(500); 
Serial.print("."); } Serial.printf("\nConnected! Open http://%s in your browser\n", 
WiFi.localIP().toString().c_str()); server.on("/", handleRoot); server.on("/stream", handleStream); 
server.begin(); Serial.println("Web server started. Robot is now in autonomous mode."); } // ── 
LOOP: Main robot logic 
──────────────────────────────────────────────── void loop() 
{ server.handleClient(); sensor_t * s = esp_camera_sensor_get(); s->set_pixformat(s, 
PIXFORMAT_RGB565); camera_fb_t * fb = esp_camera_fb_get(); if (!fb) 
{ Serial.println("Analysis: Frame capture failed"); s->set_pixformat(s, PIXFORMAT_JPEG); 
return; } long total_x = 0; int red_pixel_count = 0; for (int i = 0; i < fb->len; i += 2) { uint8_t high = 
fb->buf[i]; uint8_t low = fb->buf[i+1]; uint8_t r = (high & 0xF8); uint8_t g = ((high & 0x07) << 5) 
| ((low & 0xE0) >> 3); uint8_t b = (low & 0x1F) << 3; if (r > RED_MIN && r > (g + 
COLOR_MARGIN) && r > (b + COLOR_MARGIN)) { int pixel_index = i / 2; int x = pixel_index 
% fb->width; total_x += x; red_pixel_count++; } } esp_camera_fb_return(fb); s->set_pixformat(s, 
PIXFORMAT_JPEG); if (red_pixel_count > MIN_BLOB_SIZE) { int center_x = total_x / 
red_pixel_count; int center_of_view = fb->width / 2; int error_margin = fb->width / 5; if (center_x 
< (center_of_view - error_margin)) { turnLeft(); } else if (center_x > (center_of_view + 
error_margin)) { turnRight(); } else { moveForward(); } } else { stopMotors(); } } // ── Motor 
Control Helper Functions ──────────────────────────────────────── 
void moveForward() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); digitalWrite(IN3, 
HIGH); digitalWrite(IN4, LOW); analogWrite(ENA, forwardLeftSpeed); analogWrite(ENB, 
forwardRightSpeed); } void turnLeft() { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); 
digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENA, turningSpeed); 
analogWrite(ENB, turningSpeed); } void turnRight() { digitalWrite(IN1, HIGH); digitalWrite(IN2, 
LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENA, turningSpeed); 
analogWrite(ENB, turningSpeed); } void stopMotors() { digitalWrite(IN1, LOW); digitalWrite(IN2, 
LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENA, 0); 
analogWrite(ENB, 0); } Citations https://visualgdb.com/tutorials/esp32/remotevideo/ 
https://github.com/tensorflow/tflite-micro https://forum.arduino.cc/t/matchboxscope-with-esp32
cam/1127958 Model 9:35 AM ThinkingThoughts Auto