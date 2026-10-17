// Renders every UI page from mock CSI to PNGs (docs/*.png in the README), using
// the real drawing code in src/main.cpp against a stub canvas that draws with
// LovyanGFX's own 6x8 font. Regenerate after UI changes:
//   c++ -std=c++17 -O2 -Isrc -I.pio/libdeps/cardrone/M5GFX/src/lgfx/Fonts test/screens.cpp -lz -o /tmp/screens && /tmp/screens docs
#include <zlib.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#define PROGMEM
#include "glcdfont.h"  // const unsigned char font[]: 5 columns per glyph, LSB on top

// ── Arduino / ESP-IDF stand-ins: just enough for main.cpp to compile ─────────
using std::max;
using std::min;
static uint32_t nowMs = 0;
static uint32_t millis() { return nowMs; }
static uint32_t micros() { return nowMs * 1000; }
static void delay(uint32_t ms) { nowMs += ms; }
template <class T> static T constrain(T v, T lo, T hi) { return v < lo ? lo : v > hi ? hi : v; }
static uint32_t esp_random() { return (uint32_t)rand() * 2654435761u; }
struct String : std::string { using std::string::string; };
static struct { void printf(const char*, ...) {} void println(const char*) {} } Serial;
static struct { uint32_t getFreeHeap() { return 61000; } uint32_t getMaxAllocHeap() { return 40000; } uint32_t getMinFreeHeap() { return 52000; } } ESP;
enum { ESP_RST_POWERON, ESP_RST_PANIC, ESP_RST_INT_WDT, ESP_RST_TASK_WDT, ESP_RST_WDT, ESP_RST_BROWNOUT };
static int esp_reset_reason() { return ESP_RST_POWERON; }
constexpr int APP_CPU_NUM = 1, ESP_OK = 0;
static void xTaskCreatePinnedToCore(void (*)(void*), const char*, int, void*, int, void*, int) {}

struct wifi_csi_info_t { struct { int rssi, channel; } rx_ctrl; uint8_t mac[6]; int8_t* buf; int len; };
struct wifi_promiscuous_filter_t { uint32_t filter_mask; };
struct wifi_csi_config_t { bool lltf_en, htltf_en, stbc_htltf2_en, ltf_merge_en, channel_filter_en; };
enum { WIFI_PROMIS_FILTER_MASK_MGMT = 1, WIFI_PROMIS_FILTER_MASK_DATA = 2, WIFI_SECOND_CHAN_NONE = 0 };
static int esp_wifi_set_channel(int, int) { return 0; }
static int esp_wifi_set_csi(bool) { return 0; }
static int esp_wifi_set_promiscuous(bool) { return 0; }
static int esp_wifi_set_promiscuous_filter(wifi_promiscuous_filter_t*) { return 0; }
static int esp_wifi_set_csi_config(wifi_csi_config_t*) { return 0; }
static int esp_wifi_set_csi_rx_cb(void (*)(void*, wifi_csi_info_t*), void*) { return 0; }
typedef void* esp_ping_handle_t;
struct esp_ping_config_t { struct { uint32_t addr; } target_addr; uint32_t count, interval_ms, data_size; };
struct esp_ping_callbacks_t {};
#define ESP_PING_DEFAULT_CONFIG() esp_ping_config_t{}
#define ESP_PING_COUNT_INFINITE 0
#define IP_ADDR4(a, b, c, d, e) ((a)->addr = 0)
static int esp_ping_new_session(esp_ping_config_t*, esp_ping_callbacks_t*, esp_ping_handle_t*) { return 1; }
static int esp_ping_start(esp_ping_handle_t) { return 0; }
static int esp_ping_stop(esp_ping_handle_t) { return 0; }
static int esp_ping_delete_session(esp_ping_handle_t) { return 0; }

struct IPAddress { uint8_t operator[](int) const { return 0; } };
enum { WIFI_STA, WL_CONNECTED = 3, WIFI_AUTH_OPEN = 0 };
static uint8_t mockBssid[6] = {2, 0, 0, 0xa1, 0xb2, 0xc3};
static struct {
  void mode(int) {}
  int scanNetworks() { return 0; }
  String SSID(int) { return "net"; }
  uint8_t* BSSID(int = 0) { return mockBssid; }
  int channel(int = 0) { return 6; }
  int encryptionType(int) { return 3; }
  int RSSI(int) { return -60; }
  void begin(const char*, const char* = nullptr) {}
  int status() { return 0; }
  void disconnect() {}
  void setAutoReconnect(bool) {}
  void setSleep(bool) {}
  IPAddress gatewayIP() { return {}; }
} WiFi;
struct Preferences {
  bool begin(const char*, bool) { return false; }
  void end() {}
  uint8_t getUChar(const char*, uint8_t d) { return d; }
  void putUChar(const char*, uint8_t) {}
  void getString(const char*, char* b, int) { *b = 0; }
  void putString(const char*, const char*) {}
  void getBytes(const char*, void*, int) {}
  void putBytes(const char*, const void*, int) {}
};
struct File {
  explicit operator bool() const { return false; }
  size_t write(const uint8_t*, size_t) { return 0; }
  size_t readBytes(char*, size_t) { return 0; }
  void close() {}
};
#define FILE_WRITE "w"
static struct { void begin(int, int, int, int) {} } SPI;
static struct { bool begin(int, decltype(SPI)&, int) { return false; } void mkdir(const char*) {} File open(const char*, const char* = "r") { return {}; } } SD;

// The canvas: RGB565 framebuffer with the subset of LovyanGFX main.cpp draws with.
struct M5Canvas {
  explicit M5Canvas(void*) {}
  uint16_t px[135][240] = {}, fg = 0xFFFF;
  int size = 1;
  void setColorDepth(int) {}
  void createSprite(int, int) {}
  void pushSprite(int, int) {}
  void setTextSize(int s) { size = s; }
  void setTextColor(uint16_t c) { fg = c; }
  void drawPixel(int x, int y, uint16_t c) { if (x >= 0 && x < 240 && y >= 0 && y < 135) px[y][x] = c; }
  void fillRect(int x, int y, int w, int h, uint16_t c) { for (int j = 0; j < h; j++) for (int i = 0; i < w; i++) drawPixel(x + i, y + j, c); }
  void fillSprite(uint16_t c) { fillRect(0, 0, 240, 135, c); }
  void drawFastHLine(int x, int y, int w, uint16_t c) { fillRect(x, y, w, 1, c); }
  void drawFastVLine(int x, int y, int h, uint16_t c) { fillRect(x, y, 1, h, c); }
  void drawRect(int x, int y, int w, int h, uint16_t c) { fillRect(x, y, w, 1, c); fillRect(x, y + h - 1, w, 1, c); fillRect(x, y, 1, h, c); fillRect(x + w - 1, y, 1, h, c); }
  void fillRoundRect(int x, int y, int w, int h, int, uint16_t c) { fillRect(x + 1, y, w - 2, h, c); fillRect(x, y + 1, w, h - 2, c); }
  int textWidth(const char* s) { return (int)strlen(s) * 6 * size; }
  void drawString(const char* s, int x, int y) {
    for (; *s; s++, x += 6 * size)
      for (int col = 0; col < 5; col++)
        for (int row = 0; row < 8; row++)
          if (font[(uint8_t)*s * 5 + col] >> row & 1) fillRect(x + col * size, y + row * size, size, size, fg);
  }
  void drawCenterString(const char* s, int x, int y) { drawString(s, x - textWidth(s) / 2, y); }
  void drawRightString(const char* s, int x, int y) { drawString(s, x - textWidth(s), y); }
};
struct M5Cfg { int serial_baudrate; };
struct KeysState { bool tab, enter, del, fn; std::vector<char> word; };
static struct {
  struct { void setRotation(int) {} } Display;
  struct Spk {
    struct Cfg { int sample_rate, dma_buf_count, dma_buf_len, task_pinned_core; };
    Cfg config() { return {}; }
    void config(Cfg) {}
    void begin() {}
    void setVolume(int) {}
    void playRaw(const int16_t*, int, int, bool, int, int, bool) {}
  } Speaker;
  struct { bool isChange() { return false; } bool isPressed() { return false; } KeysState keysState() { return {}; } } Keyboard;
  void update() {}
  void begin(M5Cfg, bool) {}
} M5Cardputer;
static struct { M5Cfg config() { return {}; } struct { int getBatteryLevel() { return 78; } } Power; } M5;

#define CARDRONE_HOST
#include "../src/main.cpp"

// ── mock room: two static reflections plus a person walking through ─────────
static void feedFrame(float t, bool moving) {
  int8_t b[128] = {};
  for (int i = 0; i < 64; i++) {
    int k = i < 32 ? i : i - 64;  // bin -> subcarrier index
    if (k == 0 || k < -26 || k > 26) continue;
    std::complex<float> h = 1.f + 0.55f * std::polar(1.f, -2 * PI_F * k * 0.031f) +
                            0.35f * std::polar(1.f, -2 * PI_F * k * 0.083f + 1.f);
    if (moving)  // the walker's path: its delay and strength drift, rippling the shape
      h += (0.35f + 0.25f * sinf(t * 1.3f)) * std::polar(1.f, -2 * PI_F * k * (0.055f + 0.02f * sinf(t * 0.7f)) + t * 4);
    h *= 26.f;
    b[2 * i] = (int8_t)(h.imag() + (rand() % 3 - 1));
    b[2 * i + 1] = (int8_t)(h.real() + (rand() % 3 - 1));
  }
  csi.frame(b, 128, moving ? -52 + (int)(4 * sinf(t)) : -55);
}

static void savePng(const char* path) {  // 3x nearest-neighbour, RGB565 -> RGB888
  const int S = 3, w = W * S, h = H * S;
  std::vector<uint8_t> raw;
  for (int y = 0; y < h; y++) {
    raw.push_back(0);
    for (int x = 0; x < w; x++) {
      uint16_t c = cv.px[y / S][x / S];
      raw.push_back((c >> 11) * 255 / 31);
      raw.push_back((c >> 5 & 63) * 255 / 63);
      raw.push_back((c & 31) * 255 / 31);
    }
  }
  uLongf n = compressBound(raw.size());
  std::vector<uint8_t> z(n);
  compress2(z.data(), &n, raw.data(), raw.size(), 9);
  FILE* f = fopen(path, "wb");
  auto u32 = [&](uint32_t v) { uint8_t b[4] = {(uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v}; fwrite(b, 1, 4, f); };
  auto chunk = [&](const char* type, const uint8_t* d, uint32_t len) {
    u32(len);
    fwrite(type, 1, 4, f);
    if (len) fwrite(d, 1, len, f);
    uLong c = crc32(crc32(0, (const Bytef*)type, 4), d, len);
    u32(c);
  };
  fwrite("\x89PNG\r\n\x1a\n", 1, 8, f);
  uint8_t ihdr[13] = {0, 0, (uint8_t)(w >> 8), (uint8_t)w, 0, 0, (uint8_t)(h >> 8), (uint8_t)h, 8, 2, 0, 0, 0};
  chunk("IHDR", ihdr, 13);
  chunk("IDAT", z.data(), n);
  chunk("IEND", nullptr, 0);
  fclose(f);
  printf("wrote %s\n", path);
}

int main(int argc, char** argv) {
  const char* dir = argc > 1 ? argv[1] : ".";
  srand(7);
  applyTheme();
  netMode = NET_AUTO, netCh = 6;
  playing = true;
  dspLoad = 0.27f;
  static int16_t pcm[BUF];
  uint32_t nextFrame = 0, nextBlock = 0;
  bool calStarted = false;
  while (millis() < 11200) {  // 1-4 s: calibrate the empty room; from 9.5 s someone walks in
    if (millis() >= 1000 && !calStarted) calStarted = true, csi.calibrate();
    if (millis() >= nextFrame) feedFrame(millis() / 1000.f, millis() > 9500), nextFrame += 20;
    if (millis() >= nextBlock) syn.render(pcm, BUF, csi.src, csi.prof), nextBlock += 12;
    loop();  // history, battery, CSI rate; advances the clock 2 ms
  }
  char path[256];
  auto shot = [&](const char* name) {
    toastUntil = 0;
    draw();
    snprintf(path, sizeof path, "%s/%s.png", dir, name);
    savePng(path);
  };
  page = PG_WATERFALL, shot("waterfall");
  page = PG_BARS, shot("bars");
  page = PG_PARAMS, pCur = P_TIMBRE, shot("params");
  page = PG_MATRIX, mRow = 0, mCol = 2, shot("matrix");
  page = PG_WATERFALL, overlay = OV_OPTIONS, optCur = 2, shot("options");
  overlay = OV_HELP, shot("help");
  overlay = OV_NONE, calAt = millis() + 3500, shot("cal_countdown");
  calAt = 0, csi.calLeft = kCalFrames * 2 / 5, shot("cal_capture");

  // LoRa mode: a still link, then someone walks through the flat (fades at 2-3 Hz) and,
  // for the last 2 s, a hand on the Heltec's touch pin.
  csi = Csi();
  netMode = NET_LORA, netCh = 0;
  for (uint32_t t0 = millis(), next = t0; millis() - t0 < 10000;) {
    float t = (millis() - t0) / 1000.f, walk = t > 6 ? 1 : 0;
    if (millis() >= next) {
      float v = -64 + walk * (3 * sinf(6.283f * 2.4f * t) + 1.5f * sinf(6.283f * 0.7f * t)) + 0.3f * (rand() / (float)RAND_MAX - 0.5f);
      csi.level(roundf(v * 2) / 2, t > 8 ? 0.4f : 0);
      loraPkts += (next / 20) % 2;  // the Heltec sends 25 a second
      next += 20;
    }
    loop();
  }
  page = PG_WATERFALL, shot("lora");
}
