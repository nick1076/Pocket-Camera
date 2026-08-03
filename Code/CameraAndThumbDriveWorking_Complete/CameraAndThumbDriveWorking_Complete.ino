/*
   ButtonCam v2 — ESP32-S3-CAM (FORIOT board, OV3660 sensor)

   Short press        -> capture a JPEG to the microSD card
   Hold ~1.5 seconds  -> USB card-reader mode: the SD card appears as a
                         removable drive on your PC (read/write).
                         Press RST to return to camera mode.

   REQUIRED Tools settings — TWO CHANGES from the v1 build:
     USB Mode:         USB-OTG (TinyUSB)     <-- was "Hardware CDC and JTAG"
     USB CDC On Boot:  Disabled              <-- was "Enabled"
   Everything else unchanged (ESP32S3 Dev Module, 16MB flash, OPI PSRAM).

   Serial mapping in this mode:
     Serial    = UART0   -> TTL port  (your PuTTY window)
     USBSerial = USB CDC -> OTG port  (IDE Serial Monitor)
   The LOG() macro prints to both, so you keep both terminals.

   Status LED (WS2812):
     white        booting
     dim green    ready — short press = photo, hold = card reader
     cyan         capturing / writing to SD
     green blink  photo saved
     red (1 s)    capture or save failed — see serial
     red blinking fatal init error (camera or SD) — see serial, press RST
     BLUE         card-reader mode (PC owns the card; RST to exit)
     dim white    ejected from the PC — safe to press RST
*/

#if ARDUINO_USB_MODE
#error "Tools > USB Mode must be 'USB-OTG (TinyUSB)' for card-reader support"
#endif
#if ARDUINO_USB_CDC_ON_BOOT
#error "Tools > USB CDC On Boot must be 'Disabled' - it auto-starts the USB stack before setup(), which blocks MSC registration"
#endif

#include "esp_camera.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "USB.h"
#include "USBMSC.h"
#include <stdio.h>
#include <sys/stat.h>

// ---------- camera pins: ESP32S3_EYE mapping (matches this board) ----------
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

// ---------- microSD: SD_MMC peripheral, 1-bit bus (board only routes DATA0) ----------
#define SD_CLK_PIN 39
#define SD_CMD_PIN 38
#define SD_D0_PIN  40
#define MOUNT_POINT "/sdcard"

// ---------- shutter button ----------
// GPIO0 = onboard BOOT button (zero wiring). External: 14 or 21, wired to GND.
#define BUTTON_PIN   0
#define WS2812_PIN   48
#define LONGPRESS_MS 1500

USBCDC USBSerial;
USBMSC msc;

static sdmmc_card_t *card = nullptr;
static int  photoIndex = 1;
static bool readerMode = false;

// print to both UART0 (TTL port) and USB CDC (OTG port)
#define LOG(...) do { Serial.printf(__VA_ARGS__); USBSerial.printf(__VA_ARGS__); } while (0)

static void led(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(WS2812_PIN, r, g, b);
}

static void fatal(const char *why) {
  LOG("\nFATAL: %s\n", why);
  for (;;) { led(64, 0, 0); delay(250); led(0, 0, 0); delay(250); }
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
  config.frame_size   = FRAMESIZE_QXGA;   // 2048x1536 — OV3660 native max
  config.jpeg_quality = 10;               // 0..63, lower = better
  config.fb_count     = 1;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    LOG("esp_camera_init -> 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  LOG("camera OK, sensor PID 0x%x\n", s->id.PID);

  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);       // sensor is mounted upside down
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  return true;
}

// ---------------- microSD (IDF-level mount so we keep the raw card handle) ----------------

static bool initSD() {
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.flags = SDMMC_HOST_FLAG_1BIT;              // only D0 is routed on this board

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

// first unused photo_NNNN.jpg — numbering survives reboots and card swaps
static int nextFreeIndex() {
  char path[32];
  for (int i = 1; i <= 9999; i++) {
    snprintf(path, sizeof(path), MOUNT_POINT "/photo_%04d.jpg", i);
    if (!fileExists(path)) return i;
  }
  return -1;
}

static bool takePhoto() {
  // With fb_count = 1 the driver keeps the buffer filled continuously, so the
  // first frame we grab may be stale. Toss two so the shot reflects "now".
  for (int i = 0; i < 2; i++) {
    camera_fb_t *stale = esp_camera_fb_get();
    if (stale) esp_camera_fb_return(stale);
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    LOG("capture failed (no frame)\n");
    return false;
  }

  char path[32];
  snprintf(path, sizeof(path), MOUNT_POINT "/photo_%04d.jpg", photoIndex);

  FILE *f = fopen(path, "wb");
  if (!f) {
    LOG("could not open %s for writing\n", path);
    esp_camera_fb_return(fb);
    return false;
  }

  size_t len     = fb->len;
  size_t written = fwrite(fb->buf, 1, len, f);
  fclose(f);
  esp_camera_fb_return(fb);

  if (written != len) {
    LOG("short write on %s (%u of %u bytes) — card full?\n",
        path, (unsigned)written, (unsigned)len);
    return false;
  }

  LOG("saved %s (%u bytes)\n", path, (unsigned)len);
  photoIndex++;
  return true;
}

// ---------------- USB mass storage ----------------

static int32_t onMscRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
  (void)offset;  // TinyUSB delivers sector-aligned transfers
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
  if (load_eject && !start) {          // the PC clicked "Eject"
    msc.mediaPresent(false);
    led(4, 4, 4);                      // dim white: safe to press RST
    LOG("ejected by host — press RST to return to camera mode\n");
  }
  return true;
}

static void enterReaderMode() {
  readerMode = true;
  // Every photo file was closed after writing, so our FATFS mount holds no
  // dirty state. We simply stop touching the filesystem and hand the raw
  // card to the PC. A reset remounts fresh and sees whatever the PC changed.
  LOG("card-reader mode: SD card exposed over USB. Press RST for camera mode.\n");
  msc.mediaPresent(true);
  led(0, 0, 48);                       // blue: PC owns the card
  while (digitalRead(BUTTON_PIN) == LOW) delay(10);
}

// ---------------- arduino ----------------

void setup() {
  Serial.begin(115200);    // UART0 -> TTL port
  USBSerial.begin();       // USB CDC -> OTG port (registers before USB.begin)
  led(8, 8, 8);            // white: booting

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  if (!initCamera()) fatal("camera init failed");
  if (!initSD())     fatal("SD card init failed");

  photoIndex = nextFreeIndex();
  if (photoIndex < 0) fatal("no free filenames — card holds 9999 photos");

  // USB identity + mass-storage hookup. All interfaces must be registered
  // before USB.begin(), which is why CDC-on-boot has to be off.
  msc.vendorID("ESP32S3");             // max 8 chars
  msc.productID("ButtonCam SD");       // max 16 chars
  msc.productRevision("2.0");          // max 4 chars
  msc.onRead(onMscRead);
  msc.onWrite(onMscWrite);
  msc.onStartStop(onMscStartStop);
  msc.mediaPresent(false);             // "empty card reader" until long-press
  msc.begin(card->csd.capacity, card->csd.sector_size);
  USB.begin();

  delay(2000);                         // let the host enumerate the CDC port
  LOG("\n=== ButtonCam v2 ===\n");
  LOG("PSRAM: %u bytes\n", ESP.getPsramSize());
  LOG("next file: " MOUNT_POINT "/photo_%04d.jpg\n", photoIndex);

  // let auto-exposure / white balance settle before the first shot
  for (int i = 0; i < 4; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
  }

  led(0, 8, 0);                        // dim green: ready
  LOG("ready — short press: photo, hold %d ms: USB card reader\n", LONGPRESS_MS);
}

void loop() {
  if (readerMode) { delay(100); return; }   // PC owns the card; RST to exit

  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(30);                              // debounce
    if (digitalRead(BUTTON_PIN) != LOW) return;

    uint32_t t0 = millis();
    while (digitalRead(BUTTON_PIN) == LOW) {
      if (millis() - t0 >= LONGPRESS_MS) {  // held long enough: switch modes
        enterReaderMode();
        return;
      }
      delay(10);
    }

    // released before the threshold: take a photo
    led(0, 16, 32);                         // cyan: working
    bool ok = takePhoto();
    if (ok) { led(0, 48, 0); delay(150); }  // green blink: saved
    else    { led(64, 0, 0); delay(1000); } // red: failed, see serial
    led(0, 8, 0);                           // back to ready
  }
}
