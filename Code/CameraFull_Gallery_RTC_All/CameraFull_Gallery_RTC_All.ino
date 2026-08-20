/*
   ButtonCam v3.1 — ESP32-S3-CAM (FORIOT board, OV3660 sensor)
                    DS3231 RTC + 1.54" ST7789 240x240 viewfinder + GALLERY MODE

   ------------------------------------------------------------------
   CONTROLS
   ------------------------------------------------------------------
   CAMERA MODE
     single tap        -> capture full-res JPEG + .json sidecar
     double tap        -> enter gallery mode
     hold ~1.5 s       -> USB card-reader mode (RST to exit)

   GALLERY MODE
     single tap        -> next photo (newest -> oldest, wraps around)
     double tap        -> back to camera mode
     hold ~1.5 s       -> USB card-reader mode

   Screen blanks after IDLE_BLANK_MS in either mode. The wake press does not
   fire the shutter, and waking always returns you to camera mode.

   ------------------------------------------------------------------
   NEW IN v3.1
     - readGesture(): one blocking classifier returning SINGLE / DOUBLE / LONG,
       replacing the inline button handling. All modes share it.
     - Gallery mode: reads photo_NNNN.jpg back off the card, decodes at 1/8
       scale, and shows it with its capture time pulled from the sidecar.
     - scaleNN(): the old fixed 320x240->240x180 scaler is now generic with a
       source crop rect, so preview and gallery share one code path.
     - grabFullRes(): retry + per-attempt logging on the QVGA->QXGA switch.
       This is the fix for the "FAILED" banner discussed separately — it is
       included here so you are not merging two changes at once.

   ------------------------------------------------------------------
   WIRING (unchanged from v3)
     ST7789  SCLK 21 | MOSI 47 | DC 1 | CS 3 | BL 2 | RST->3V3 | VCC 3V3
     DS3231  SDA 41  | SCL 42  | VCC 3V3 (NOT 5V)
     Shutter GPIO 14 to GND

   Tools: USB Mode = USB-OTG (TinyUSB), USB CDC On Boot = Disabled,
          ESP32S3 Dev Module, 16MB flash, OPI PSRAM
   Libraries: Adafruit GFX, Adafruit ST7735/ST7789, RTClib
   ------------------------------------------------------------------
*/

#if ARDUINO_USB_MODE
#error "Tools > USB Mode must be 'USB-OTG (TinyUSB)' for card-reader support"
#endif
#if ARDUINO_USB_CDC_ON_BOOT
#error "Tools > USB CDC On Boot must be 'Disabled' - it auto-starts the USB stack before setup(), which blocks MSC registration"
#endif

#include "esp_camera.h"
#include "img_converters.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "USB.h"
#include "USBMSC.h"

#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>

// ---------- camera pins: ESP32S3_EYE mapping ----------
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM   4
#define SIOC_GPIO_NUM   5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM     8
#define Y3_GPIO_NUM     9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM   7
#define PCLK_GPIO_NUM  13

// ---------- microSD ----------
#define SD_CLK_PIN 39
#define SD_CMD_PIN 38
#define SD_D0_PIN  40
#define MOUNT_POINT "/sdcard"

// ---------- shutter + status LED ----------
#define BUTTON_PIN   14
#define WS2812_PIN   48
#define DEBOUNCE_MS  30
#define LONGPRESS_MS 1500

// How long to wait after a release before deciding a tap was single.
// This is pure added latency on every photo. 250 ms is comfortable for most
// people; drop toward 180 if capture feels sluggish, raise toward 350 if your
// double taps keep registering as two separate presses.
#define DOUBLETAP_MS 250

// ---------- ST7789 ----------
#define TFT_SCLK 21
#define TFT_MOSI 47
#define TFT_DC    1
#define TFT_CS    3
#define TFT_BL    2
#define TFT_RST  -1

#define TFT_W    240
#define TFT_H    240
#define TFT_ROTATION 1 
#define TFT_SPI_HZ 40000000      // lower this first if the panel misbehaves

#define PREVIEW_BIG_ENDIAN false // flip if gallery/preview colours are wrong

// ---------- DS3231 ----------
#define RTC_SDA 41
#define RTC_SCL 42
#define I2C_HZ  100000

// ---------- geometry ----------
#define CAM_W 320                // QVGA preview source
#define CAM_H 240
#define IMG_W 240                // letterboxed image area
#define IMG_H 180
#define BAR_Y IMG_H              // status bar y = 180..239

#define PREVIEW_FRAMESIZE FRAMESIZE_QVGA
#define CAPTURE_FRAMESIZE FRAMESIZE_QXGA

// Gallery: false letterboxes the 4:3 photo at 240x180 with its timestamp
// below. true fills all 240x240 by cropping the left and right edges — looks
// bolder, but you lose part of the frame you actually shot.
#define GALLERY_FULLSCREEN false

#define IDLE_BLANK_MS 30000      // 0 disables blanking

// Decode scratch sizes
#define RGB_BUF_PX   (CAM_W * CAM_H)   // 76800 px = 153600 B, also holds 256x192
#define SCALE_BUF_PX (TFT_W * TFT_H)   // 57600 px = 115200 B, covers fullscreen

USBCDC USBSerial;
USBMSC msc;
RTC_DS3231 rtc;

SPIClass tftSPI(HSPI);
Adafruit_ST7789 tft = Adafruit_ST7789(&tftSPI, TFT_CS, TFT_DC, TFT_RST);

static sdmmc_card_t *card = nullptr;
static int  photoIndex = 1;
static bool readerMode = false;
static bool rtcOk      = false;
static bool displayOn  = true;

static uint16_t *rgbBuf   = nullptr;
static uint16_t *scaleBuf = nullptr;

static uint32_t lastActivity = 0;
static int lastBarMinute = -1;
static int lastBarIndex  = -1;

enum AppMode { MODE_CAMERA, MODE_GALLERY };
static AppMode mode = MODE_CAMERA;
static int galleryIndex = -1;

enum Gesture { GESTURE_NONE, GESTURE_SINGLE, GESTURE_DOUBLE, GESTURE_LONG };

#define LOG(...) do { Serial.printf(__VA_ARGS__); USBSerial.printf(__VA_ARGS__); } while (0)

static void led(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(WS2812_PIN, r, g, b);
}

static void fatal(const char *why) {
  LOG("\nFATAL: %s\n", why);
  for (;;) { led(64, 0, 0); delay(250); led(0, 0, 0); delay(250); }
}

// ---------------- button gestures ----------------

// Blocks until the gesture resolves. Returns immediately if the button is up.
// A long press is reported the moment the threshold is crossed, while still
// held, so the mode change is felt under the finger.
static Gesture readGesture() {
  if (digitalRead(BUTTON_PIN) != LOW) return GESTURE_NONE;
  delay(DEBOUNCE_MS);
  if (digitalRead(BUTTON_PIN) != LOW) return GESTURE_NONE;

  uint32_t t0 = millis();
  while (digitalRead(BUTTON_PIN) == LOW) {
    if (millis() - t0 >= LONGPRESS_MS) return GESTURE_LONG;
    delay(5);
  }

  // Released. Watch for a second press inside the window.
  uint32_t t1 = millis();
  while (millis() - t1 < DOUBLETAP_MS) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      delay(DEBOUNCE_MS);
      if (digitalRead(BUTTON_PIN) == LOW) {
        while (digitalRead(BUTTON_PIN) == LOW) delay(5);   // consume the release
        return GESTURE_DOUBLE;
      }
    }
    delay(5);
  }
  return GESTURE_SINGLE;
}

// ---------------- display ----------------

static void backlight(bool on) { digitalWrite(TFT_BL, on ? HIGH : LOW); }

static bool initDisplay() {
  pinMode(TFT_BL, OUTPUT);
  backlight(false);

  tftSPI.begin(TFT_SCLK, -1, TFT_MOSI, -1);
  tft.init(TFT_W, TFT_H, SPI_MODE0);
  tft.setSPISpeed(TFT_SPI_HZ);
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(ST77XX_BLACK);

  rgbBuf   = (uint16_t *)ps_malloc(RGB_BUF_PX * 2);
  scaleBuf = (uint16_t *)ps_malloc(SCALE_BUF_PX * 2);
  if (!rgbBuf || !scaleBuf) {
    LOG("preview buffers failed to allocate (PSRAM enabled in Tools?)\n");
    return false;
  }

  backlight(true);
  return true;
}

static void splash(const char *line1, const char *line2) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 100);
  tft.println(line1);
  if (line2) { tft.setCursor(10, 128); tft.println(line2); }
}

static void banner(const char *text, uint16_t colour) {
  tft.fillRect(0, IMG_H / 2 - 16, TFT_W, 32, ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(colour);
  int16_t w = strlen(text) * 18;
  tft.setCursor((TFT_W - w) / 2, IMG_H / 2 - 12);
  tft.print(text);
}

// Generic nearest-neighbour rescale of a crop rect out of an RGB565 buffer.
// Integer-only; the column map is cached and rebuilt only when geometry moves.
static void scaleNN(const uint16_t *src, int srcW,
                    int sx, int sy, int cropW, int cropH,
                    uint16_t *dst, int dstW, int dstH) {
  static uint16_t colLUT[TFT_W];
  static int lutCropW = -1, lutDstW = -1, lutSx = -1;

  if (cropW != lutCropW || dstW != lutDstW || sx != lutSx) {
    for (int x = 0; x < dstW; x++) colLUT[x] = (uint16_t)(sx + (x * cropW) / dstW);
    lutCropW = cropW; lutDstW = dstW; lutSx = sx;
  }
  for (int y = 0; y < dstH; y++) {
    const uint16_t *srow = src + ((sy + (y * cropH) / dstH) * (size_t)srcW);
    uint16_t *drow = dst + ((size_t)y * dstW);
    for (int x = 0; x < dstW; x++) drow[x] = srow[colLUT[x]];
  }
}

static void pushBlock(int x, int y, int w, int h, uint16_t *buf) {
  tft.startWrite();
  tft.setAddrWindow(x, y, w, h);
  tft.writePixels(buf, (uint32_t)w * h, true, PREVIEW_BIG_ENDIAN);
  tft.endWrite();
}

static void drawStatusBar(bool force) {
  int minute = -1, hour = 0;
  if (rtcOk) {
    DateTime now = rtc.now();
    hour = now.hour();
    minute = now.minute();
  }
  if (!force && minute == lastBarMinute && photoIndex == lastBarIndex) return;
  lastBarMinute = minute;
  lastBarIndex  = photoIndex;

  tft.fillRect(0, BAR_Y, TFT_W, TFT_H - BAR_Y, ST77XX_BLACK);

  tft.setTextSize(3);
  tft.setTextColor(rtcOk ? ST77XX_WHITE : ST77XX_YELLOW);
  tft.setCursor(6, BAR_Y + 18);
  if (rtcOk) tft.printf("%02d:%02d", hour, minute);
  else       tft.print("--:--");

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(TFT_W - 6 - (5 * 12), BAR_Y + 24);
  tft.printf("#%04d", photoIndex);
}

static void previewFrame() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  bool ok = jpg2rgb565(fb->buf, fb->len, (uint8_t *)rgbBuf, JPG_SCALE_NONE);
  esp_camera_fb_return(fb);
  if (!ok) return;

  scaleNN(rgbBuf, CAM_W, 0, 0, CAM_W, CAM_H, scaleBuf, IMG_W, IMG_H);
  pushBlock(0, 0, IMG_W, IMG_H, scaleBuf);
}

// ---------------- RTC ----------------

static bool initRTC() {
  Wire.begin(RTC_SDA, RTC_SCL, I2C_HZ);
  if (!rtc.begin(&Wire)) {
    LOG("DS3231 not found on SDA=%d SCL=%d — check wiring and that VCC is 3V3\n",
        RTC_SDA, RTC_SCL);
    return false;
  }
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    LOG("RTC had lost power; seeded from compile time. Set it with T<epoch>.\n");
  }
  DateTime now = rtc.now();
  struct timeval tv = { .tv_sec = (time_t)now.unixtime(), .tv_usec = 0 };
  settimeofday(&tv, nullptr);

  LOG("RTC OK: %04d-%02d-%02d %02d:%02d:%02d  (die temp %.2f C)\n",
      now.year(), now.month(), now.day(),
      now.hour(), now.minute(), now.second(), rtc.getTemperature());
  return true;
}

static void handleSerialTimeSet() {
  static char buf[24];
  static uint8_t n = 0;

  while (Serial.available() || USBSerial.available()) {
    int c = Serial.available() ? Serial.read() : USBSerial.read();
    if (c == '\n' || c == '\r') {
      buf[n] = 0;
      if (n > 1 && (buf[0] == 'T' || buf[0] == 't')) {
        uint32_t epoch = strtoul(buf + 1, nullptr, 10);
        if (epoch > 1600000000UL && rtcOk) {
          rtc.adjust(DateTime(epoch));
          struct timeval tv = { .tv_sec = (time_t)epoch, .tv_usec = 0 };
          settimeofday(&tv, nullptr);
          lastBarMinute = -1;
          LOG("clock set to epoch %lu\n", (unsigned long)epoch);
        } else {
          LOG("ignored '%s' — need T<epoch> and a working RTC\n", buf);
        }
      }
      n = 0;
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = (char)c;
    }
  }
}

// ---------------- camera ----------------

static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = CAPTURE_FRAMESIZE;   // allocate for the largest size
  config.jpeg_quality = 10;
  config.fb_count     = 1;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { LOG("esp_camera_init -> 0x%x\n", err); return false; }

  sensor_t *s = esp_camera_sensor_get();
  LOG("camera OK, sensor PID 0x%x\n", s->id.PID);

  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  s->set_framesize(s, PREVIEW_FRAMESIZE);
  delay(200);
  return true;
}

// ---------------- microSD ----------------

static bool initSD() {
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.flags = SDMMC_HOST_FLAG_1BIT;

  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.clk   = (gpio_num_t)SD_CLK_PIN;
  slot.cmd   = (gpio_num_t)SD_CMD_PIN;
  slot.d0    = (gpio_num_t)SD_D0_PIN;
  slot.width = 1;
  slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

  esp_vfs_fat_sdmmc_mount_config_t mcfg = {};
  mcfg.format_if_mount_failed = false;
  mcfg.max_files = 4;
  mcfg.allocation_unit_size = 16 * 1024;

  esp_err_t err = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot, &mcfg, &card);
  if (err != ESP_OK) {
    LOG("SD mount failed (0x%x) — card seated? FAT32-formatted?\n", err);
    return false;
  }
  LOG("SD OK: %u MB\n",
      (uint32_t)(((uint64_t)card->csd.capacity * card->csd.sector_size) >> 20));
  return true;
}

static bool fileExists(const char *path) {
  struct stat st;
  return stat(path, &st) == 0;
}

static bool photoExists(int i) {
  char path[40];
  snprintf(path, sizeof(path), MOUNT_POINT "/photo_%04d.jpg", i);
  return fileExists(path);
}

static int nextFreeIndex() {
  for (int i = 1; i <= 9999; i++) if (!photoExists(i)) return i;
  return -1;
}

// ---------------- capture ----------------

// Retries around the QVGA->QXGA switch. The OV3660 needs several frames to
// re-window, and the driver hands back NULL or an undersized frame until it
// settles. Per-attempt logging so a failure tells you where it stalled.
static camera_fb_t *grabFullRes() {
  const size_t MIN_QXGA_BYTES = 30000;
  for (int i = 0; i < 8; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      LOG("  grab %d: %ux%u, %u bytes\n", i,
          (unsigned)fb->width, (unsigned)fb->height, (unsigned)fb->len);
      if (i >= 2 && fb->len >= MIN_QXGA_BYTES) return fb;
      esp_camera_fb_return(fb);
    } else {
      LOG("  grab %d: NULL\n", i);
    }
    delay(60);
  }
  return nullptr;
}

static void writeSidecar(int index, const DateTime &ts, size_t bytes, float dieTemp) {
  char path[40];
  snprintf(path, sizeof(path), MOUNT_POINT "/photo_%04d.json", index);

  FILE *f = fopen(path, "w");
  if (!f) { LOG("sidecar open failed: %s\n", path); return; }

  fprintf(f,
    "{\n"
    "  \"file\": \"photo_%04d.jpg\",\n"
    "  \"seq\": %d,\n"
    "  \"local_iso\": \"%04d-%02d-%02dT%02d:%02d:%02d\",\n"
    "  \"epoch_local\": %lu,\n"
    "  \"rtc_valid\": %s,\n"
    "  \"bytes\": %u,\n"
    "  \"sensor\": \"OV3660\",\n"
    "  \"framesize\": \"QXGA\",\n"
    "  \"width\": 2048,\n"
    "  \"height\": 1536,\n"
    "  \"jpeg_quality\": 10,\n"
    "  \"rtc_temp_c\": %.2f,\n"
    "  \"uptime_ms\": %lu,\n"
    "  \"fw\": \"buttoncam-v3.1\"\n"
    "}\n",
    index, index,
    ts.year(), ts.month(), ts.day(), ts.hour(), ts.minute(), ts.second(),
    (unsigned long)ts.unixtime(),
    rtcOk ? "true" : "false",
    (unsigned)bytes, dieTemp, (unsigned long)millis());

  fclose(f);
}

static bool takePhoto() {
  DateTime ts = rtcOk ? rtc.now() : DateTime((uint32_t)0);
  float dieTemp = rtcOk ? rtc.getTemperature() : 0.0f;

  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, CAPTURE_FRAMESIZE);
  delay(300);

  camera_fb_t *fb = grabFullRes();
  if (!fb) {
    LOG("capture failed (no usable frame after retries)\n");
    s->set_framesize(s, PREVIEW_FRAMESIZE);
    delay(100);
    return false;
  }

  char path[40];
  snprintf(path, sizeof(path), MOUNT_POINT "/photo_%04d.jpg", photoIndex);

  FILE *f = fopen(path, "wb");
  if (!f) {
    LOG("could not open %s for writing\n", path);
    esp_camera_fb_return(fb);
    s->set_framesize(s, PREVIEW_FRAMESIZE);
    delay(100);
    return false;
  }

  size_t len     = fb->len;
  size_t written = fwrite(fb->buf, 1, len, f);
  fclose(f);
  esp_camera_fb_return(fb);

  s->set_framesize(s, PREVIEW_FRAMESIZE);
  delay(100);

  if (written != len) {
    LOG("short write on %s (%u of %u bytes) — card full?\n",
        path, (unsigned)written, (unsigned)len);
    return false;
  }

  writeSidecar(photoIndex, ts, len, dieTemp);
  LOG("saved %s (%u bytes) @ %04d-%02d-%02d %02d:%02d:%02d\n",
      path, (unsigned)len,
      ts.year(), ts.month(), ts.day(), ts.hour(), ts.minute(), ts.second());
  photoIndex++;
  return true;
}

// ---------------- gallery ----------------

// Minimal SOF scan so we know the decoded size instead of assuming QXGA.
// Handles baseline and progressive markers; skips the rest.
static bool jpegDimensions(const uint8_t *d, size_t len, int *w, int *h) {
  if (len < 4 || d[0] != 0xFF || d[1] != 0xD8) return false;
  size_t i = 2;
  while (i + 9 < len) {
    if (d[i] != 0xFF) { i++; continue; }
    uint8_t m = d[i + 1];
    if (m == 0xD8 || m == 0x01 || (m >= 0xD0 && m <= 0xD7)) { i += 2; continue; }
    size_t seg = ((size_t)d[i + 2] << 8) | d[i + 3];
    if ((m >= 0xC0 && m <= 0xC3) || (m >= 0xC5 && m <= 0xC7) ||
        (m >= 0xC9 && m <= 0xCB) || (m >= 0xCD && m <= 0xCF)) {
      *h = ((int)d[i + 5] << 8) | d[i + 6];
      *w = ((int)d[i + 7] << 8) | d[i + 8];
      return (*w > 0 && *h > 0);
    }
    if (m == 0xDA) break;              // start of scan, no SOF found
    i += 2 + seg;
  }
  return false;
}

// Pull "local_iso" out of the sidecar. We wrote the file, so a substring
// search is safe here — no need for a JSON parser on device.
static bool readSidecarTime(int index, char *out, size_t outSz) {
  char path[40];
  snprintf(path, sizeof(path), MOUNT_POINT "/photo_%04d.json", index);
  FILE *f = fopen(path, "r");
  if (!f) return false;

  char buf[400];
  size_t n = fread(buf, 1, sizeof(buf) - 1, f);
  fclose(f);
  buf[n] = 0;

  const char *key = "\"local_iso\": \"";
  const char *k = strstr(buf, key);
  if (!k) return false;
  k += strlen(key);
  const char *e = strchr(k, '"');
  if (!e) return false;

  size_t len = (size_t)(e - k);
  if (len >= outSz) len = outSz - 1;
  memcpy(out, k, len);
  out[len] = 0;
  return true;
}

static void drawGalleryBar(int index) {
  tft.fillRect(0, BAR_Y, TFT_W, TFT_H - BAR_Y, ST77XX_BLACK);

  char iso[24];
  tft.setTextSize(2);
  if (readSidecarTime(index, iso, sizeof(iso)) && strlen(iso) >= 16) {
    // "2026-08-07T19:30:12" -> "08-07  19:30"
    char pretty[16];
    snprintf(pretty, sizeof(pretty), "%.5s  %.5s", iso + 5, iso + 11);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(6, BAR_Y + 12);
    tft.print(pretty);
  } else {
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(6, BAR_Y + 12);
    tft.print("no sidecar");
  }

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(6, BAR_Y + 36);
  tft.printf("#%04d", index);

  tft.setTextColor(ST77XX_MAGENTA);
  tft.setCursor(TFT_W - 6 - (7 * 12), BAR_Y + 36);
  tft.print("GALLERY");
}

static bool showPhoto(int index) {
  char path[40];
  snprintf(path, sizeof(path), MOUNT_POINT "/photo_%04d.jpg", index);

  struct stat st;
  if (stat(path, &st) != 0) return false;
  size_t sz = (size_t)st.st_size;
  if (sz < 128 || sz > 4u * 1024 * 1024) { LOG("gallery: odd size %u\n", (unsigned)sz); return false; }

  uint8_t *jpg = (uint8_t *)ps_malloc(sz);
  if (!jpg) { LOG("gallery: no PSRAM for %u bytes\n", (unsigned)sz); return false; }

  FILE *f = fopen(path, "rb");
  if (!f) { free(jpg); return false; }
  size_t rd = fread(jpg, 1, sz, f);
  fclose(f);
  if (rd != sz) { free(jpg); LOG("gallery: short read\n"); return false; }

  int jw = 0, jh = 0;
  if (!jpegDimensions(jpg, sz, &jw, &jh)) { free(jpg); LOG("gallery: no SOF\n"); return false; }

  int dw = jw / 8, dh = jh / 8;                  // JPG_SCALE_8X
  if ((size_t)dw * dh > RGB_BUF_PX) {
    free(jpg);
    LOG("gallery: %dx%d too large for scratch buffer\n", dw, dh);
    return false;
  }

  bool ok = jpg2rgb565(jpg, sz, (uint8_t *)rgbBuf, JPG_SCALE_8X);
  free(jpg);
  if (!ok) { LOG("gallery: decode failed\n"); return false; }

  if (GALLERY_FULLSCREEN) {
    int side = (dw < dh) ? dw : dh;              // centre square crop
    scaleNN(rgbBuf, dw, (dw - side) / 2, (dh - side) / 2, side, side,
            scaleBuf, TFT_W, TFT_H);
    pushBlock(0, 0, TFT_W, TFT_H, scaleBuf);
  } else {
    scaleNN(rgbBuf, dw, 0, 0, dw, dh, scaleBuf, IMG_W, IMG_H);
    pushBlock(0, 0, IMG_W, IMG_H, scaleBuf);
    drawGalleryBar(index);
  }
  return true;
}

// Newest existing photo at or below `from`, wrapping to the top if needed.
static int findPhotoAtOrBelow(int from) {
  for (int i = from; i >= 1; i--) if (photoExists(i)) return i;
  return -1;
}

static void enterGallery() {
  int newest = findPhotoAtOrBelow(photoIndex - 1);
  if (newest < 0) {
    banner("NO PHOTOS", ST77XX_YELLOW);
    delay(900);
    drawStatusBar(true);
    return;
  }
  mode = MODE_GALLERY;
  galleryIndex = newest;
  led(32, 0, 40);                       // purple: gallery
  tft.fillScreen(ST77XX_BLACK);
  banner("...", ST77XX_WHITE);
  if (!showPhoto(galleryIndex)) banner("READ ERR", ST77XX_RED);
  LOG("gallery: showing #%04d\n", galleryIndex);
}

static void galleryNext() {
  int next = findPhotoAtOrBelow(galleryIndex - 1);
  if (next < 0) next = findPhotoAtOrBelow(photoIndex - 1);   // wrap to newest
  if (next < 0) { mode = MODE_CAMERA; return; }

  galleryIndex = next;
  banner("...", ST77XX_WHITE);
  if (!showPhoto(galleryIndex)) banner("READ ERR", ST77XX_RED);
  LOG("gallery: showing #%04d\n", galleryIndex);
}

static void exitGallery() {
  mode = MODE_CAMERA;
  galleryIndex = -1;
  tft.fillScreen(ST77XX_BLACK);
  drawStatusBar(true);
  led(0, 8, 0);
  LOG("gallery: back to camera\n");
}

// ---------------- USB mass storage ----------------

static int32_t onMscRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
  (void)offset;
  if (sdmmc_read_sectors(card, buffer, lba, bufsize / card->csd.sector_size) != ESP_OK)
    return -1;
  return bufsize;
}

static int32_t onMscWrite(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
  (void)offset;
  if (sdmmc_write_sectors(card, buffer, lba, bufsize / card->csd.sector_size) != ESP_OK)
    return -1;
  return bufsize;
}

static bool onMscStartStop(uint8_t power_condition, bool start, bool load_eject) {
  (void)power_condition;
  if (load_eject && !start) {
    msc.mediaPresent(false);
    led(4, 4, 4);
    splash("EJECTED", "press RST");
    LOG("ejected by host — press RST to return to camera mode\n");
  }
  return true;
}

static void enterReaderMode() {
  readerMode = true;
  mode = MODE_CAMERA;
  backlight(true);
  splash("USB MODE", "press RST");
  LOG("card-reader mode: SD card exposed over USB. Press RST for camera mode.\n");
  msc.mediaPresent(true);
  led(0, 0, 48);
  while (digitalRead(BUTTON_PIN) == LOW) delay(10);
}

// ---------------- arduino ----------------

void setup() {
  Serial.begin(115200);
  USBSerial.begin();
  led(8, 8, 8);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  if (!initDisplay()) fatal("display init failed");
  splash("ButtonCam", "booting...");

  if (!initCamera()) fatal("camera init failed");
  if (!initSD())     fatal("SD card init failed");

  rtcOk = initRTC();
  if (!rtcOk) led(48, 32, 0);

  photoIndex = nextFreeIndex();
  if (photoIndex < 0) fatal("no free filenames — card holds 9999 photos");

  msc.vendorID("ESP32S3");
  msc.productID("ButtonCam SD");
  msc.productRevision("3.1");
  msc.onRead(onMscRead);
  msc.onWrite(onMscWrite);
  msc.onStartStop(onMscStartStop);
  msc.mediaPresent(false);
  msc.begin(card->csd.capacity, card->csd.sector_size);
  USB.begin();

  delay(2000);
  LOG("\n=== ButtonCam v3.1 ===\n");
  LOG("PSRAM: %u bytes\n", ESP.getPsramSize());
  LOG("next file: " MOUNT_POINT "/photo_%04d.jpg\n", photoIndex);
  if (!rtcOk) LOG("WARNING: no RTC — sidecar timestamps will be marked invalid\n");

  for (int i = 0; i < 4; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
  }

  tft.fillScreen(ST77XX_BLACK);
  drawStatusBar(true);
  lastActivity = millis();
  displayOn = true;
  mode = MODE_CAMERA;

  led(0, 8, 0);
  LOG("ready — tap: photo, double tap: gallery, hold %d ms: USB reader\n", LONGPRESS_MS);
}

void loop() {
  if (readerMode) { delay(100); return; }

  handleSerialTimeSet();

  // Idle blanking. Always drops back to camera mode so waking is predictable.
  if (IDLE_BLANK_MS > 0 && displayOn && (millis() - lastActivity > IDLE_BLANK_MS)) {
    displayOn = false;
    mode = MODE_CAMERA;
    galleryIndex = -1;
    backlight(false);
    tft.fillScreen(ST77XX_BLACK);
    led(0, 4, 0);
    LOG("idle — press to wake\n");
  }

  // Gallery holds a still image, so there is nothing to redraw between taps.
  if (displayOn && mode == MODE_CAMERA) {
    previewFrame();
    drawStatusBar(false);
  } else {
    delay(20);
  }

  Gesture g = readGesture();
  if (g == GESTURE_NONE) return;
  lastActivity = millis();

  if (!displayOn) {                      // wake press: never fires the shutter
    displayOn = true;
    backlight(true);
    tft.fillScreen(ST77XX_BLACK);
    drawStatusBar(true);
    led(0, 8, 0);
    return;
  }

  if (g == GESTURE_LONG) { enterReaderMode(); return; }

  if (mode == MODE_GALLERY) {
    if (g == GESTURE_DOUBLE) exitGallery();
    else                     galleryNext();
    lastActivity = millis();
    return;
  }

  // ---- camera mode ----
  if (g == GESTURE_DOUBLE) {
    enterGallery();
    lastActivity = millis();
    return;
  }

  led(0, 16, 32);
  banner("...", ST77XX_WHITE);
  bool ok = takePhoto();

  if (ok) {
    led(0, 48, 0);
    banner("SAVED", ST77XX_GREEN);
    delay(400);
  } else {
    led(64, 0, 0);
    banner("FAILED", ST77XX_RED);
    delay(1000);
  }

  drawStatusBar(true);
  led(0, 8, 0);
  lastActivity = millis();
}
