// CARDRONE: a drone synth for the M5Stack Cardputer ADV, played by the room.
// WiFi Channel State Information (bodies moving bend the multipath) drives
// seven drone engines and an FX chain through a mod matrix.
#ifndef CARDRONE_HOST  // test/screens.cpp supplies stand-ins to render the UI on a laptop
#include <M5Cardputer.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "ping/ping_sock.h"
#include <RadioLib.h>
#include "utility/PI4IOE5V6408_Class.hpp"
#endif
#include <functional>
#ifndef CARDRONE_VERSION  // merge_bin.py stamps `git describe` here
#define CARDRONE_VERSION "dev"
#endif
#include "csi.h"
#include "dsp.h"

constexpr int W = 240, H = 135, BUF = 512, WF_ROWS = 48;
constexpr int HIST = 56;  // sparkline ends at x=136, before the DSP/VOL readout

static Synth syn;
static Csi csi;
static M5Canvas cv(&M5Cardputer.Display);

// ── phosphor palette ─────────────────────────────────────────────────────────
static constexpr uint16_t rgb(int r, int g, int b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); }
constexpr uint16_t C_BG = 0;
static uint16_t C_SCAN, C_BAR, C_DIM, C_MID, C_HI, C_HOT, C_WARN;  // set by applyTheme()
static uint16_t ramp[32];  // black -> phosphor -> white-hot
struct Theme { const char* name; uint8_t r, g, b; uint16_t warn; };
static const Theme THEMES[] = {{"GREEN", 0, 255, 90, rgb(255, 176, 0)}, {"YELLOW", 255, 200, 0, rgb(255, 70, 40)},
                               {"RED", 255, 40, 30, rgb(255, 200, 0)}, {"BLUE", 40, 140, 255, rgb(255, 176, 0)},
                               {"B&W", 255, 255, 255, rgb(255, 176, 0)}};  // grey UI; warnings, ticks, engine tags keep colour
constexpr int NTHEME = sizeof THEMES / sizeof *THEMES;
static uint8_t theme = 0;  // saved in NVS
// Header tag per engine, the same in every theme (no amber: that means "warning").
static const uint16_t ENGINE_COLOR[] = {rgb(0, 220, 255),  rgb(255, 120, 30), rgb(180, 110, 255), rgb(120, 255, 60),
                                        rgb(255, 70, 170), rgb(255, 225, 40), rgb(60, 140, 255)};
static_assert(sizeof ENGINE_COLOR / sizeof *ENGINE_COLOR == NENGINE, "one colour per engine");

static void applyTheme() {
  const Theme& t = THEMES[theme];
  auto c = [&](float k, float w) {  // k: brightness of the phosphor hue, w: blend toward white
    auto ch = [&](int v) { float x = v * k; return (int)(x + (255 - x) * w); };
    return rgb(ch(t.r), ch(t.g), ch(t.b));
  };
  C_SCAN = c(0.063f, 0), C_BAR = c(0.14f, 0), C_DIM = c(0.31f, 0), C_MID = c(0.65f, 0);
  C_HI = c(1, 0.27f), C_HOT = c(1, 0.82f), C_WARN = t.warn;
  for (int i = 0; i < 32; i++) {
    float v = i / 31.f;
    ramp[i] = c(0.78f * fminf(v / 0.6f, 1), 0.86f * fmaxf((v - 0.6f) / 0.4f, 0));
  }
}

enum Page { PG_WATERFALL, PG_BARS, PG_PARAMS, PG_MATRIX, NPAGE };
enum Overlay { OV_NONE, OV_HELP, OV_OPTIONS };
constexpr int NOPT = 8;  // options: audio out, volume, mod profile, lfo shape, lfo rate, theme, network, calibrate
static const char* const HINTS[NPAGE] = {"TAB view  z-m notes  r random  i help", "e calibrate  w/l preset  i help",
                                         ";. select  ,/ adjust  ENTER fx on/off", ";. row  ENTER column  ,/ adj  DEL off"};
// M5Unified doubles a mono stream, then scales by 32*vol^2*255^2/2^36: 180 is
// unity gain, so a full-scale drone hard-clips in its mixer above that.
constexpr int VOL_MAX = 180;
static int page = PG_WATERFALL, overlay = OV_NONE, pCur = 0, mRow = 0, mCol = 0, optCur = 0, vol = 180;
static bool audioOn = true;  // ADV hardware: a plugged-in jack switches the speaker amp off by itself
static volatile bool playing = false;  // silent until ENTER
static char pending = 0;     // 's' or 'l' waiting for a slot digit
static char toast[40];
static uint32_t toastUntil = 0, csiHz = 0;
static int batt = -1;  // %, -1 = unknown
constexpr uint32_t CAL_COUNTDOWN_MS = 5000;  // time to leave the room before the empty-room capture
static uint32_t calAt = 0;                   // when the countdown ends (millis), 0 = no countdown
static bool calibrating() { return calAt || csi.calLeft > 0; }
constexpr uint32_t HOLD_MS = 3000;  // r and a: a tap does one thing, holding this long another
static uint32_t holdAt = 0;         // when r or a went down, while it's held and nothing has fired yet
static char holdKey = 0;

// u: undo, back through the patches from before big changes (randomize, load, notes, engine...).
constexpr int UNDO_N = 8;
static Patch undoBuf[UNDO_N];
static int undoN = 0;
static void remember() {
  if (undoN && !memcmp(&undoBuf[undoN - 1], &syn.pt, sizeof(Patch))) return;  // nothing new to keep
  if (undoN == UNDO_N) memmove(undoBuf, undoBuf + 1, sizeof(Patch) * --undoN);
  undoBuf[undoN++] = syn.pt;
}
static int curNote() { return 12 * (syn.pt.octave + 1) + syn.pt.root; }
// a: drift. The root wanders among the scale's notes within a fifth of home.
static bool drift = false;
static uint32_t driftAt = 0;
static int driftHome = 0;   // a note number, as curNote()
// Hold a: autopilot (see autopilot()). autoSwapAt: a section change is fading out.
static bool autoOn = false;
static uint32_t autoSwapAt = 0;
static bool upOct = false;  // the last note was Shift+note: one octave above the octave setting
static void goNote(int note) {  // a note number (as curNote()), folded into the octaves 0-5
  while (note < 12) note += 12;
  while (note > 83) note -= 12;
  syn.pt.octave = note / 12 - 1, syn.pt.root = note % 12, upOct = false;
}
static uint8_t wf[WF_ROWS][NSC];
static int wfHead = 0, histHead = 0;
static float motHist[HIST], srcPeak[NSRC], barPeak[NSC];
static volatile float dspLoad = 0;  // render time / block time

static void say(const char* s) {
  snprintf(toast, sizeof toast, "%s", s);
  toastUntil = millis() + 1800;
}

// ── audio ────────────────────────────────────────────────────────────────────
static void audioTask(void*) {
  static int16_t bufs[4][BUF];  // playRaw keeps the pointer: rotate so queued blocks stay intact
  bool was = false;
  for (int bi = 0;; bi = (bi + 1) & 3) {
    bool on = playing;
    int16_t* b = bufs[bi];
    if (on || was) {  // stopped: synth state pauses, so play resumes where it left off
      uint32_t t0 = micros();
      syn.render(b, BUF, csi.src, csi.prof);
      dspLoad += ((micros() - t0) * (SR / 1e6f) / BUF - dspLoad) * 0.05f;
      if (on != was)  // one-block fade in/out, no click
        for (int i = 0; i < BUF; i++) b[i] = b[i] * (on ? i : BUF - 1 - i) / BUF;
    } else memset(b, 0, sizeof bufs[0]);
    was = on;
    M5Cardputer.Speaker.playRaw(b, BUF, SR, false, 1, 0, false);  // blocks on a semaphore while the queue is full
  }
}

static void applyVolume() { M5Cardputer.Speaker.setVolume(audioOn ? vol : 0); }

// ── network modes + CSI capture ──────────────────────────────────────────────
// JOIN: associate and ping the router at 50 Hz (steadiest rate, needs the password).
// LISTEN: promiscuous on one AP's channel, no association.
// AUTO: hop channels 1-13, lock onto whichever transmitter yields the most CSI.
// LORA: no WiFi; the level of a Heltec's packets on 869.85 MHz instead of CSI.
enum NetMode { NET_OFF, NET_JOIN, NET_LISTEN, NET_AUTO, NET_LORA };
enum CsiState { CS_HOLD, CS_SURVEY, CS_RUN };
static volatile uint8_t csiState = CS_HOLD;
static uint8_t netMode = NET_OFF, netCh = 0, apMac[6];
static char netName[33] = "";
static esp_ping_handle_t ping = nullptr;
struct Talker { uint8_t mac[6], ch; uint16_t n; };
static Talker talk[24];
static volatile int nTalk = 0;

static void onCsi(void*, wifi_csi_info_t* info) {  // WiFi task, core 0
  if (!info || !info->buf || info->len < 128) return;
  if (csiState == CS_RUN) {
    if (!memcmp(info->mac, apMac, 6))  // one transmitter: one coherent channel
      csi.frame(info->buf, info->len, info->rx_ctrl.rssi);
  } else if (csiState == CS_SURVEY) {  // count CSI-bearing frames per transmitter
    int i = 0;
    while (i < nTalk && memcmp(talk[i].mac, info->mac, 6)) i++;
    if (i < nTalk) {
      talk[i].n++;
    } else if (i < (int)(sizeof talk / sizeof *talk)) {
      memcpy(talk[i].mac, info->mac, 6);
      talk[i].ch = info->rx_ctrl.channel;
      talk[i].n = 1;
      nTalk = i + 1;
    }
  }
}

// LoRa mode: the Heltec (src/heltec/beacon.cpp) sends 100 small packets a second;
// the cap on the Cardputer measures how strongly each arrives (Csi::level()).
constexpr float LORA_FREQ = 869.85f;              // MHz: EU 869.7-870 allows 5 mW with no duty-cycle limit
static uint8_t LORA_SYNC[] = {0xCA, 0xD0, 0x4E};  // as the Heltec's: only CARDRONE packets count
static uint32_t loraPkts = 0;
#ifndef CARDRONE_HOST
static SX1262 lora = new Module(5, 4, 3, 6, SPI);  // cap: NSS G5, IRQ G4, RST G3, BUSY G6; SPI shared with the SD card
static bool loraUp = false;
static volatile bool loraRx = false;
static void IRAM_ATTR onLoraRx() { loraRx = true; }

static bool loraInit() {
  if (loraUp) return true;
  m5::PI4IOE5V6408_Class ioe(0x43, 400000, &m5::In_I2C);  // the cap's IO expander: P0 powers the antenna switch
  if (!ioe.begin()) return false;  // no cap
  ioe.setDirection(0, true);
  ioe.setHighImpedance(0, false);
  ioe.digitalWrite(0, true);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);  // SD card deselected while the radio talks
  SPI.begin(40, 39, 14, 12);
  int st = -1;
  for (float tcxo : {3.0f, 1.8f, 0.f})  // 3.0 V as in M5's examples; fall back if this cap differs
    if ((st = lora.beginFSK(LORA_FREQ, 50, 25, 156.2f, 0, 16, tcxo, true)) == RADIOLIB_ERR_NONE) break;
  if (st == RADIOLIB_ERR_NONE) st = lora.setDataShaping(RADIOLIB_SHAPING_0_5);
  if (st == RADIOLIB_ERR_NONE) st = lora.setSyncWord(LORA_SYNC, sizeof LORA_SYNC);
  if (st != RADIOLIB_ERR_NONE) return false;
  lora.setPacketReceivedAction(onLoraRx);
  return loraUp = true;
}

static void loraStop() {
  if (loraUp) lora.standby();
}

// From loop(), like the SD card (they share SPI): packets in, 50 Hz frames out.
static void loraPoll(uint32_t now) {
  static float sum = 0, lvl = 0, tch = 0;
  static int n = 0;
  static uint32_t tFrame = 0, lastPkt = 0;
  if (loraRx) {  // packets that arrived while loop() was busy are lost; the latest is kept
    loraRx = false;
    uint8_t b[8];
    size_t len = lora.getPacketLength();
    float dbm = lora.getRSSI(true);
    if (lora.readData(b, sizeof b) == RADIOLIB_ERR_NONE && len == 5 && b[0] == 'C' && b[1] == 'D') {
      sum += dbm, n++, loraPkts++, lastPkt = now;
      tch = (b[3] | b[4] << 8) / 1000.f;
    }
  }
  if (now - tFrame > 200) tFrame = now - 20;  // after a stall, don't replay it frame by frame
  for (; now - tFrame >= 20; tFrame += 20) {  // 50 Hz: the rate the smoothing is tuned to
    if (n) lvl = sum / n, sum = 0, n = 0;
    if (now - lastPkt < 250) csi.level(lvl, tch);
  }
  if (now - lastPkt > 2000) lastPkt = now - 250, lora.startReceive();  // quiet for 2 s: re-arm the receiver
}
#else
static void loraStop() {}
static void loraPoll(uint32_t) {}
#endif

static void radioOff() {  // CSI off, disassociated, no background reconnect hopping channels
  loraStop();
  csiState = CS_HOLD;
  if (ping) {
    esp_ping_stop(ping);
    esp_ping_delete_session(ping);
    ping = nullptr;
  }
  esp_wifi_set_csi(false);
  esp_wifi_set_promiscuous(false);
  WiFi.setAutoReconnect(false);
  WiFi.disconnect();
  netMode = NET_OFF;
}

static void csiEnable() {
  WiFi.setSleep(false);  // modem sleep starves the CSI rate
  wifi_promiscuous_filter_t pf = {WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA};
  esp_wifi_set_promiscuous_filter(&pf);
  esp_wifi_set_promiscuous(true);
  wifi_csi_config_t cfg = {};
  cfg.lltf_en = cfg.htltf_en = cfg.stbc_htltf2_en = cfg.ltf_merge_en = cfg.channel_filter_en = true;
  esp_wifi_set_csi_config(&cfg);
  esp_wifi_set_csi_rx_cb(onCsi, nullptr);
  esp_wifi_set_csi(true);
}

// Lock onto one transmitter, with a fresh Csi: the old one learned a different channel.
static void csiRun(uint8_t mode, const uint8_t* mac, int ch, const char* name) {
  csiState = CS_HOLD;
  delay(5);  // let an in-flight frame() finish
  csi = Csi();
  memcpy(apMac, mac, 6);
  csiEnable();
  if (mode != NET_JOIN) esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  netMode = mode;
  netCh = ch;
  snprintf(netName, sizeof netName, "%s", name);
  csiState = CS_RUN;
}

static bool loraStart() {  // false: no cap, or its radio didn't answer
  radioOff();
#ifndef CARDRONE_HOST
  if (!loraInit()) return false;
  csi = Csi();
  lora.startReceive();
#endif
  netMode = NET_LORA;
  netCh = 0;
  snprintf(netName, sizeof netName, "HELTEC");
  return true;
}

// ── drawing ──────────────────────────────────────────────────────────────────
static void backdrop() {
  cv.fillSprite(C_BG);
  for (int y = 0; y < H; y += 2) cv.drawFastHLine(0, y, W, C_SCAN);  // CRT scanlines
}

// Boot: a CRT warming up, then the title over a rippling subcarrier skyline.
static void bootAnimation(const char* crash) {
  static const char* const TAGLINES[] = {"the room plays the drone",        "every body bends the signal",
                                         "listening to the air",            "your movement is the instrument",
                                         "52 subcarriers, one drone",       "slow music for still rooms"};
  const char* sub = TAGLINES[esp_random() % (sizeof TAGLINES / sizeof *TAGLINES)];
  const int len = strlen(sub), cy = H / 2;
  const uint32_t end = 900 + len * 30 + 1500;  // typing, then 1.5 s to read
  for (uint32_t t0 = millis(), t = 0; t < end; t = millis() - t0) {
    cv.fillSprite(C_BG);
    if (t < 180) {  // power-on: a line stretches out from the center...
      int w = W * t / 180;
      cv.fillRect((W - w) / 2, cy - 1, w, 2, C_HOT);
    } else if (t < 360) {  // ...and opens into the raster
      int h = 2 + (H - 2) * (t - 180) / 180;
      for (int y = cy - h / 2; y < cy + h / 2; y += 2) cv.drawFastHLine(0, y, W, C_SCAN);
      cv.drawFastHLine(0, cy - h / 2, W, C_HI);
      cv.drawFastHLine(0, cy + h / 2, W, C_HI);
    } else {
      backdrop();
      float grow = fminf((t - 360) / 500.f, 1);
      for (int i = 0; i < NSC; i++) {
        float v = 0.55f + 0.25f * sinf(i * 0.31f + t * 0.005f) + 0.2f * sinf(i * 0.83f - t * 0.0035f);
        int h = 1 + (int)(v * grow * 24);
        cv.fillRect(16 + i * 4, 126 - h, 3, h, ramp[4 + (int)(v * 27)]);
      }
      cv.setTextSize(3);
      cv.setTextColor(ramp[constrain((int)t - 450, 0, 416) / 16]);  // fades up the phosphor ramp
      cv.drawCenterString("CARDRONE", W / 2, 38);
      cv.setTextSize(1);
      int n = constrain((int)t - 900, 0, len * 30) / 30;
      char typed[48];
      memcpy(typed, sub, n);
      typed[n] = n < len ? '_' : 0;  // typewriter cursor
      typed[n + 1] = 0;
      cv.setTextColor(C_MID);
      cv.drawString(typed, (W - len * 6) / 2, 74);
      if (n == len && t / 400 % 2) {  // the drone starts silent: say how to start it
        cv.setTextColor(C_HI);
        cv.drawCenterString("press SPACE to play", W / 2, 88);
      }
      cv.setTextColor(C_DIM);
      cv.drawRightString(CARDRONE_VERSION, W - 3, 3);
      if (crash && t > 1000) {
        cv.setTextColor(C_WARN);
        cv.drawCenterString(crash, W / 2, 20);
      }
    }
    cv.pushSprite(0, 0);
    delay(16);
  }
}

static void panel(const char* title, const char* msg, const char* hint) {
  backdrop();
  cv.fillRect(0, 0, W, 12, C_BAR);
  cv.drawFastHLine(0, 12, W, C_MID);
  cv.setTextColor(C_HI);
  cv.drawCenterString(title, W / 2, 2);
  cv.setTextColor(C_MID);
  if (msg) cv.drawCenterString(msg, W / 2, 60);
  cv.drawFastHLine(0, 122, W, C_MID);
  cv.setTextColor(C_DIM);
  cv.drawCenterString(hint, W / 2, 126);
}

static float srcLevel(int s) { return s == S_LFO ? 0.5f + 0.5f * csi.src[s] : csi.src[s]; }
static const char* srcName(int s) { return (netMode == NET_LORA ? SRC_NAME_LORA : SRC_NAME)[s]; }

static void netDesc(char* s, int cap) {
  static const char* const M[] = {"OFF", "JOIN", "LISTEN", "AUTO"};
  if (netMode == NET_OFF) snprintf(s, cap, "OFF  >");
  else if (netMode == NET_LORA) snprintf(s, cap, "LORA %.2f MHz  >", LORA_FREQ);
  else snprintf(s, cap, "%s %.10s ch%d  >", M[netMode], netName, netCh);
}

static void drawHeader() {
  const Patch& P = syn.pt;
  cv.fillRect(0, 0, W, 12, C_BAR);
  cv.drawFastHLine(0, 12, W, C_MID);
  if (autoOn) {  // autopilot: the title becomes a pill
    cv.fillRoundRect(1, 1, 30, 10, 2, C_HI);
    cv.setTextColor(C_BG);
    cv.drawString("AUTO", 4, 2);
  } else {
    cv.setTextColor(C_HI);
    cv.drawString("CARDRONE", 3, 2);
  }
  const char* en = ENGINE_NAME[P.engine];
  int ew = cv.textWidth(en) + 6;
  cv.fillRoundRect(55, 1, ew, 10, 2, ENGINE_COLOR[P.engine]);
  cv.setTextColor(C_BG);
  cv.drawString(en, 58, 2);
  char s[24];
  snprintf(s, sizeof s, "%s%d %s%s", NOTE_NAME[P.root], P.octave, SCALES[P.scale].name, drift ? "~" : "");
  cv.setTextColor(C_HI);
  cv.drawString(s, 60 + ew, 2);
  const char tag = "-JLA"[netMode];  // network mode, then channel
  if (csi.calLeft > 0) snprintf(s, sizeof s, "CAL %d%%", 100 - csi.calLeft * 100 / kCalFrames);
  else if (netMode == NET_OFF) snprintf(s, sizeof s, "NO NET");
  else if (netMode == NET_LORA) snprintf(s, sizeof s, "LR %luHz", (unsigned long)csiHz);
  else if (csiHz) snprintf(s, sizeof s, "%c%d %luHz", tag, netCh, (unsigned long)csiHz);
  else snprintf(s, sizeof s, "%c%d 0Hz", tag, netCh);
  cv.setTextColor(csi.calLeft > 0 || !csiHz ? C_WARN : C_MID);
  cv.drawRightString(s, W - 20, 2);
  uint16_t bc = batt >= 0 && batt < 20 ? C_WARN : C_MID;  // battery: outline, nub, fill
  cv.drawRect(W - 17, 3, 12, 7, bc);
  cv.fillRect(W - 5, 5, 2, 3, bc);
  if (batt > 0) cv.fillRect(W - 15, 5, max(1, batt * 8 / 100), 3, bc);
}

static void drawWaterfall() {  // subcarrier ->, time v
  for (int r = 0; r < WF_ROWS; r++) {
    const uint8_t* row = wf[(wfHead + r) % WF_ROWS];
    for (int k = 0; k < NSC; k++) cv.fillRect(4 + k * 3, 15 + r * 2, 3, 2, ramp[row[k]]);
  }
  cv.drawRect(3, 14, NSC * 3 + 2, WF_ROWS * 2 + 2, C_DIM);
  if (!csi.frames) {
    cv.setTextColor(C_WARN);
    cv.drawCenterString(netMode == NET_LORA ? "NO LINK - is the Heltec on?" : "NO CSI - ` for network", 82, 58);
  }
  for (int s = 0; s < NSRC; s++) {  // source meters with a decaying phosphor peak
    int x = 166, y = 16 + s * 12;
    float v = srcLevel(s);
    srcPeak[s] = fmaxf(v, srcPeak[s] - 0.012f);
    cv.setTextColor(C_MID);
    cv.drawString(srcName(s), x, y);
    cv.fillRect(x + 20, y + 1, 50, 6, C_BAR);
    cv.fillRect(x + 20, y + 1, (int)(srcPeak[s] * 50), 6, C_DIM);
    cv.fillRect(x + 20, y + 1, (int)(v * 50), 6, C_HI);
  }
}

static void drawBars() {  // live subcarrier shape, peak ghosts, amber baseline ticks
  const int x0 = 16, yb = 110, hmax = 84;
  cv.setTextColor(C_DIM);
  cv.drawString("LO", x0 + 30, 15);
  cv.drawString("MID", x0 + 100, 15);
  cv.drawString("HI", x0 + 172, 15);
  for (int k : {18, 35})
    for (int y = 16; y < yb; y += 3) cv.drawPixel(x0 + k * 4 - 1, y, C_DIM);
  float span = csi.profHi - csi.profLo + 1e-6f;
  for (int k = 0; k < NSC; k++) {
    float v = csi.prof[k];
    barPeak[k] = fmaxf(v, barPeak[k] - 0.01f);
    int x = x0 + k * 4, h = 2 + (int)(v * hmax), hp = 2 + (int)(barPeak[k] * hmax);
    cv.fillRect(x, yb - hp, 3, hp - h, C_DIM);
    cv.fillRect(x, yb - h, 3, h, ramp[8 + (int)(v * 23)]);
    if (csi.calibrated && netMode != NET_LORA)  // LoRa's baseline is one level, not a shape
      cv.drawFastHLine(x, yb - 2 - (int)(clamp01((csi.base[k] - csi.profLo) / span) * hmax), 3, C_WARN);
  }
  cv.drawFastHLine(x0 - 2, yb + 1, NSC * 4 + 2, C_MID);
}

static void drawParams() {
  const Patch& P = syn.pt;
  for (int i = 0; i < NPARAM; i++) {
    int x = 4 + (i / 8) * 118, y = 16 + (i % 8) * 12, bx = x + 40, bw = 52;
    bool sel = i == pCur, off = (PARAM_FX[i] && !(P.fx & PARAM_FX[i])) || (i == P_SHIMMER && !(P.fx & FX_REVERB));
    if (sel) cv.fillRect(x - 2, y - 2, 116, 12, C_BAR);
    cv.setTextColor(off ? C_DIM : sel ? C_HOT : C_MID);
    cv.drawString(PARAM_NAME[i], x, y);
    cv.drawRect(bx, y, bw, 8, C_DIM);
    cv.fillRect(bx + 1, y + 1, (int)(P.p[i] * (bw - 2)), 6, off ? C_DIM : C_MID);
    if (!off) cv.drawFastVLine(bx + 1 + (int)(syn.eff[i] * (bw - 3)), y - 1, 10, C_HOT);  // after modulation
    char s[4] = "--";
    if (!off) snprintf(s, sizeof s, "%2d", (int)(P.p[i] * 99));
    cv.drawString(s, x + 96, y);
  }
}

static void drawMatrix() {
  for (int r = 0; r < NSLOT; r++) {
    const Slot& s = syn.pt.mod[r];
    int y = 16 + r * 12;
    bool sel = r == mRow, live = s.depth != 0;
    auto col = [&](int c) { return sel && mCol == c ? C_HOT : live ? C_MID : C_DIM; };
    if (sel) cv.fillRect(0, y - 2, W, 12, C_BAR);
    char t[8];
    snprintf(t, sizeof t, "%d", r + 1);
    cv.setTextColor(C_DIM);
    cv.drawString(t, 4, y);
    cv.drawString(">", 60, y);
    cv.setTextColor(col(0));
    cv.drawString(srcName(s.src), 14, y);
    cv.fillRect(34, y + 1, 20, 6, C_BAR);
    cv.fillRect(34, y + 1, (int)(srcLevel(s.src) * 20), 6, live ? C_HI : C_DIM);
    cv.setTextColor(col(1));
    cv.drawString(PARAM_NAME[s.dst], 70, y);
    snprintf(t, sizeof t, "%+4d", s.depth);
    cv.setTextColor(col(2));
    cv.drawString(t, 112, y);
    const int cx = 188;  // bipolar depth bar, live contribution on top
    int d = s.depth * 46 / 100, lv = (int)(csi.src[s.src] * s.depth * 0.46f);
    cv.drawFastVLine(cx, y - 1, 10, C_DIM);
    cv.fillRect(min(cx, cx + d), y + 1, abs(d), 6, C_DIM);
    cv.fillRect(min(cx, cx + lv), y + 2, abs(lv), 4, C_HI);
  }
}

static void drawHelp() {
  static const char* const K[][2] = {  // three columns of nine: keys <= 3 chars, notes <= 9
      {"SPC", "play/stop"}, {"z-m", "notes"},     {"s-j", "sharps"},    {"SH", "note +8va"}, {"1-7", "engine"},
      {"p", "voices"},      {"t", "interval"},    {"9 0", "octave"},    {"[ ]", "root -/+"}, {"k", "scale"},
      {"a", "drift"},       {"a", "hold:auto"},   {"f", "freeze"},      {"q", "hold:fade"},  {"u", "undo"},
      {"r", "rnd/hold"},    {"e", "calibrate"},   {"TAB", "page"},      {";.", "up/down"},   {",/", "adjust"},
      {"ENT", "edit/play"}, {"DEL", "clr slot"},  {"Fn", "+1-7: fx"},   {"w l", "save/load"}, {"- =", "volume"},
      {"`", "options"},     {"i", "help"}};
  for (int i = 0; i < (int)(sizeof K / sizeof *K); i++) {
    int x = 1 + (i / 9) * 80, y = 15 + (i % 9) * 11;
    cv.setTextColor(C_HI);
    cv.drawString(K[i][0], x, y);
    cv.setTextColor(C_MID);
    cv.drawString(K[i][1], x + 21, y);
  }
}

static void drawOptions() {
  static const char* const names[NOPT] = {"AUDIO OUT", "VOLUME",  "MOD PROFILE", "LFO SHAPE",
                                          "LFO RATE",  "THEME",   "NETWORK",     "CALIBRATE"};
  const Patch& P = syn.pt;
  for (int i = 0; i < NOPT; i++) {
    int y = 16 + i * 11;
    bool sel = i == optCur;
    char v[40];
    if (i == 0) snprintf(v, sizeof v, "%s", audioOn ? "ON" : "OFF (muted)");
    if (i == 1) snprintf(v, sizeof v, "%d%%", vol * 100 / VOL_MAX);
    if (i == 2) snprintf(v, sizeof v, "%s  %.2gs glide", MOD_PROFILES[syn.profile].name, MOD_PROFILES[syn.profile].modTau);
    if (i == 3) snprintf(v, sizeof v, "%s", LFO_NAME[P.lfo]);
    if (i == 4) snprintf(v, sizeof v, "%.2f Hz", lfoHz(P.p[P_LFO]));
    if (i == 5) snprintf(v, sizeof v, "%s", THEMES[theme].name);
    if (i == 6) netDesc(v, sizeof v);
    if (i == 7) snprintf(v, sizeof v, "%s", csi.calibrated ? "baseline set  >" : "empty room  >");
    if (sel) cv.fillRect(0, y - 2, W, 11, C_BAR);
    cv.setTextColor(sel ? C_HOT : C_MID);
    cv.drawString(names[i], 8, y);
    cv.setTextColor(sel ? C_HI : C_DIM);
    cv.drawString(v, 84, y);
  }
  cv.setTextColor(C_DIM);
  char m[48];  // lowest free heap since boot: if crashes return, is it memory?
  snprintf(m, sizeof m, "min free RAM %uK   jack = speaker off", ESP.getMinFreeHeap() / 1024);
  cv.drawString(m, 8, 104);
}

// Countdown to leave the room, then the capture with a progress bar.
static void drawCalibration() {
  char s[24];
  cv.setTextSize(2);
  if (calAt) {
    cv.setTextColor(C_HI);
    cv.drawCenterString("LEAVE THE ROOM", W / 2, 20);
    snprintf(s, sizeof s, "%lu", (unsigned long)(calAt - millis() + 999) / 1000);
    cv.setTextSize(5);
    cv.setTextColor(C_HOT);
    cv.drawCenterString(s, W / 2, 44);
    cv.setTextSize(1);
    cv.setTextColor(C_MID);
    cv.drawCenterString("then the empty room gets measured", W / 2, 96);
    return;
  }
  int pct = 100 - csi.calLeft * 100 / kCalFrames;
  cv.setTextColor(millis() / 500 % 2 ? C_HI : C_MID);  // pulse
  cv.drawCenterString("CALIBRATING", W / 2, 26);
  cv.setTextSize(1);
  cv.drawRect(30, 56, 180, 10, C_DIM);
  cv.fillRect(32, 58, pct * 176 / 100, 6, C_HI);
  snprintf(s, sizeof s, "%d%%", pct);
  cv.setTextColor(C_MID);
  cv.drawCenterString(s, W / 2, 72);
  cv.setTextColor(csiHz ? C_MID : C_WARN);
  cv.drawCenterString(csiHz ? "keep the room empty" : netMode == NET_LORA ? "waiting for the Heltec..." : "waiting for CSI...", W / 2, 92);
}

static void drawFooter() {
  cv.drawFastHLine(0, 113, W, C_MID);
  cv.setTextColor(syn.freeze ? C_WARN : C_DIM);
  cv.drawString(syn.freeze ? "FRZ" : "MOT", 3, 116);
  for (int i = 0; i < HIST; i++) {  // oldest -> newest, older is dimmer
    int h = 1 + (int)(motHist[(histHead + i) % HIST] * 8);
    cv.drawFastVLine(24 + i * 2, 124 - h, h, ramp[6 + i * 25 / HIST]);
  }
  char s[24];
  if (!playing) snprintf(s, sizeof s, "STOPPED");
  else if (audioOn) snprintf(s, sizeof s, "DSP%3d%%  VOL%3d", (int)(dspLoad * 100), vol * 100 / VOL_MAX);
  else snprintf(s, sizeof s, "DSP%3d%%  MUTED", (int)(dspLoad * 100));
  cv.setTextColor(playing ? C_DIM : C_WARN);
  cv.drawRightString(s, W - 3, 116);
  char hold[40] = "";  // r or a held: count down to the hold action
  if (holdAt)
    snprintf(hold, sizeof hold, "let go: %s   hold: %s in %lu", holdKey == 'r' ? "params" : "drift",
             holdKey == 'r' ? "mods" : "AUTO", (unsigned long)(HOLD_MS - min(HOLD_MS, (uint32_t)(millis() - holdAt)) + 999) / 1000);
  bool note = pending || millis() < toastUntil || *hold || syn.duck;
  bool idle = !playing && !overlay && !calibrating();  // pulse "ENTER play" while silent
  cv.setTextColor(note ? C_WARN : idle && millis() / 500 % 2 ? C_HI : C_DIM);
  cv.drawString(pending == 'w'              ? "SAVE to slot 1-8 (other key cancels)"
                : pending                   ? "LOAD slot 1-8 (other key cancels)"
                : autoSwapAt                ? "AUTO: next section..."
                : syn.duck                  ? "FADING OUT  release q to swell back"
                : *hold                     ? hold
                : note                      ? toast
                : calibrating()             ? "any key cancels"
                : overlay == OV_HELP        ? "any key closes"
                : overlay == OV_OPTIONS     ? ";. move  ,/ ENTER change  ` close"
                : !playing                  ? "SPACE play  z-m notes  i help"
                                            : HINTS[page],
                3, 126);
}

static void draw() {
  backdrop();
  drawHeader();
  if (calibrating()) drawCalibration();
  else if (overlay == OV_HELP) drawHelp();
  else if (overlay == OV_OPTIONS) drawOptions();
  else if (page == PG_WATERFALL) drawWaterfall();
  else if (page == PG_BARS) drawBars();
  else if (page == PG_PARAMS) drawParams();
  else drawMatrix();
  drawFooter();
  cv.pushSprite(0, 0);
}

// ── blocking network screens (the drone keeps playing on core 1) ────────────
static char readKey() {  // special keys as control chars
  auto ks = M5Cardputer.Keyboard.keysState();
  char w = ks.word.empty() ? 0 : ks.word.back();
  if (ks.fn && w >= '1' && w <= '7') return 0x10 + (w - '0');  // Fn + 1-7: 0x11-0x17, FX switches
  return ks.tab ? '\t' : ks.enter ? '\n' : ks.del ? '\b' : w;
}

static char waitKey() {
  for (;;) {
    M5Cardputer.update();
    if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed())
      if (char c = readKey()) return c;
    delay(10);
  }
}

// Scrollable list; returns the picked row or -1 on ` (ESC).
static int choose(const char* title, int n, const std::function<void(int, char*, char*)>& row) {
  for (int cur = 0;;) {
    panel(title, nullptr, ";. move  ENTER pick  ` back");
    int top = max(0, min(cur - 3, n - 8));
    for (int i = top; i < min(n, top + 8); i++) {
      char l[40] = "", r[16] = "";
      row(i, l, r);
      int y = 16 + (i - top) * 12;
      if (i == cur) cv.fillRect(0, y - 2, W, 12, C_BAR);
      cv.setTextColor(i == cur ? C_HOT : C_MID);
      cv.drawString(l, 6, y);
      cv.setTextColor(C_DIM);
      cv.drawRightString(r, W - 4, y);
    }
    cv.pushSprite(0, 0);
    char c = waitKey();
    if (c == ';') cur = max(0, cur - 1);
    else if (c == '.') cur = min(n - 1, cur + 1);
    else if (c == '\n' || c == '/') return cur;
    else if (c == '`') return -1;
  }
}

static bool join(const char* ssid, const char* pass) {
  char m[48];
  snprintf(m, sizeof m, "joining %.30s", ssid);
  panel("NETWORK", m, "");
  cv.pushSprite(0, 0);
  radioOff();
  WiFi.setAutoReconnect(true);
  if (*pass) WiFi.begin(ssid, pass);
  else WiFi.begin(ssid);
  for (uint32_t t0 = millis(); WiFi.status() != WL_CONNECTED && millis() - t0 < 12000;) delay(100);
  const uint8_t* b = WiFi.BSSID();
  if (WiFi.status() != WL_CONNECTED || !b) {
    radioOff();  // or it keeps retrying, and hopping channels, in the background
    return false;
  }
  csiRun(NET_JOIN, b, WiFi.channel(), ssid);
  esp_ping_config_t pc = ESP_PING_DEFAULT_CONFIG();  // 50 Hz pings: the router answers with frames to measure
  IPAddress gw = WiFi.gatewayIP();
  IP_ADDR4(&pc.target_addr, gw[0], gw[1], gw[2], gw[3]);
  pc.count = ESP_PING_COUNT_INFINITE;
  pc.interval_ms = 20;
  pc.data_size = 8;
  esp_ping_callbacks_t cb = {};
  if (esp_ping_new_session(&pc, &cb, &ping) == ESP_OK) esp_ping_start(ping);
  return true;
}

static void listen(const uint8_t* bssid, int ch, const char* ssid) {
  radioOff();
  csiRun(NET_LISTEN, bssid, ch, ssid);
}

// Hop channels 1-13 counting CSI frames per transmitter, then lock onto the busiest.
static void autoSurvey() {
  radioOff();
  csiEnable();
  nTalk = 0;
  csiState = CS_SURVEY;
  for (int ch = 1; ch <= 13; ch++) {
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    char m[40];
    snprintf(m, sizeof m, "channel %d/13   %d talkers", ch, nTalk);
    panel("AUTO SURVEY", m, "counting CSI frames on every channel");
    cv.drawRect(40, 76, 160, 6, C_DIM);
    cv.fillRect(40, 76, ch * 160 / 13, 6, C_HI);
    cv.pushSprite(0, 0);
    delay(220);
  }
  csiState = CS_HOLD;
  int best = -1;
  for (int i = 0; i < nTalk; i++)
    if (best < 0 || talk[i].n > talk[best].n) best = i;
  if (best < 0) {
    netMode = NET_AUTO;  // nothing heard: the loop re-surveys after a few quiet seconds
    netCh = 0;
    return say("AUTO: no CSI heard, retrying");
  }
  const uint8_t* m = talk[best].mac;
  char name[8];
  snprintf(name, sizeof name, "%02x%02x%02x", m[3], m[4], m[5]);
  csiRun(NET_AUTO, m, talk[best].ch, name);
}

static void saveNet(uint8_t mode, const char* ssid, const char* pass, const uint8_t* bssid, int ch) {
  Preferences p;
  p.begin("cardrone", false);
  p.putUChar("mode", mode);
  if (mode == NET_JOIN) {
    p.putString("ssid", ssid);
    p.putString("pass", pass);
  }
  if (mode == NET_LISTEN) {
    p.putString("lssid", ssid);
    p.putBytes("bssid", bssid, 6);
    p.putUChar("ch", ch);
  }
  p.end();
}

static void loadCreds(char* ssid, char* pass) {  // 64-byte buffers
  Preferences p;
  ssid[0] = pass[0] = 0;
  if (!p.begin("cardrone", true)) return;
  p.getString("ssid", ssid, 64);
  p.getString("pass", pass, 64);
  p.end();
}

static bool resume() {  // restart the last network mode saved in NVS
  char ssid[64], pass[64], lssid[33] = "";
  uint8_t bssid[6] = {}, mode = NET_OFF, ch = 0;
  loadCreds(ssid, pass);
  Preferences p;
  if (p.begin("cardrone", true)) {
    mode = p.getUChar("mode", *ssid ? NET_JOIN : NET_OFF);  // older builds saved creds only
    ch = p.getUChar("ch", 0);
    p.getString("lssid", lssid, sizeof lssid);
    p.getBytes("bssid", bssid, 6);
    p.end();
  }
  if (mode == NET_LORA) return loraStart();
  if (mode == NET_JOIN && *ssid) return join(ssid, pass);
  if (mode == NET_LISTEN && ch) listen(bssid, ch, lssid);
  else if (mode == NET_AUTO) autoSurvey();
  else return false;
  return true;
}

static bool connectTo(int i) {  // scan index -> join or listen; false = back to the list
  char ssid[33], pass[64], s2[64], p2[64], c = 0;
  uint8_t bssid[6];
  snprintf(ssid, sizeof ssid, "%s", WiFi.SSID(i).c_str());
  memcpy(bssid, WiFi.BSSID(i), 6);
  int ch = WiFi.channel(i);
  bool open = WiFi.encryptionType(i) == WIFI_AUTH_OPEN;
  int how = choose(ssid, 2, [](int i, char* l, char*) {
    strcpy(l, i ? "LISTEN  passive, no password" : "JOIN    connect + ping, steadiest");
  });
  if (how < 0) return false;
  if (how == 1) {
    listen(bssid, ch, ssid);
    saveNet(NET_LISTEN, ssid, "", bssid, ch);
    return true;
  }
  pass[0] = 0;
  loadCreds(s2, p2);
  if (!open && !strcmp(s2, ssid)) strcpy(pass, p2);  // prefill a known network
  while (!open) {
    int l = strlen(pass);
    char m[70] = "> ";
    memset(m + 2, '*', l);
    strcpy(m + 2 + l, "_");
    panel(ssid, m, "type password  ENTER join  ` back");
    cv.pushSprite(0, 0);
    c = waitKey();
    if (c == '\n') break;
    if (c == '`') return false;
    if (c == '\b') {
      if (l) pass[l - 1] = 0;
    } else if (c >= 0x20 && c < 0x7f && l < 63) {
      pass[l] = c;
      pass[l + 1] = 0;
    }
  }
  if (join(ssid, pass)) {
    saveNet(NET_JOIN, ssid, pass, bssid, ch);
    return true;
  }
  panel("NETWORK", "join failed", "any key");
  cv.pushSprite(0, 0);
  waitKey();
  return false;
}

static void pickNetwork() {
  radioOff();
  panel("NETWORK", "scanning...", "");
  cv.pushSprite(0, 0);
  int n = max(0, min((int)WiFi.scanNetworks(), 16));
  for (;;) {
    int i = choose("NETWORK", n + 2, [](int i, char* l, char* r) {
      if (i < 2) {
        strcpy(l, i ? "LORA    a Heltec's packets, no WiFi" : "AUTO    survey every channel");
        return;
      }
      snprintf(l, 40, "%.24s", WiFi.SSID(i - 2).c_str());
      snprintf(r, 16, "%sch%d %d", WiFi.encryptionType(i - 2) == WIFI_AUTH_OPEN ? "" : "*", (int)WiFi.channel(i - 2),
               (int)WiFi.RSSI(i - 2));
    });
    if (i < 0) break;
    if (i == 0) {
      autoSurvey();
      saveNet(NET_AUTO, "", "", nullptr, 0);
      return;
    }
    if (i == 1) {
      if (loraStart()) return saveNet(NET_LORA, "", "", nullptr, 0);
      panel("LORA", "no LoRa cap found", "any key");
      cv.pushSprite(0, 0);
      waitKey();
      continue;
    }
    if (connectTo(i - 2)) return;
  }
  resume();  // cancelled: back to the saved mode
}

// ── presets on SD ────────────────────────────────────────────────────────────
static bool sdMount() {  // lazy: the mount costs ~27 KB of DRAM
  static bool ok = false;
  if (!ok) {
    SPI.begin(40, 39, 14, 12);
    ok = SD.begin(12, SPI, 25000000);
    if (ok) SD.mkdir("/cardrone");
  }
  return ok;
}

static bool savePreset(int slot) {
  char path[32], js[1024];
  snprintf(path, sizeof path, "/cardrone/preset%d.json", slot);
  int n = patchToJson(syn.pt, js, sizeof js);
  if (!sdMount()) return false;
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  bool ok = f.write((const uint8_t*)js, n) == (size_t)n;
  f.close();
  return ok;
}

static bool loadPreset(int slot) {
  char path[32], js[1024];
  snprintf(path, sizeof path, "/cardrone/preset%d.json", slot);
  if (!sdMount()) return false;
  File f = SD.open(path);
  if (!f) return false;
  js[f.readBytes(js, sizeof js - 1)] = 0;
  f.close();
  Patch p;
  if (!patchFromJson(js, p)) return false;
  remember();
  syn.pt = p;
  upOct = false;
  driftHome = curNote();
  return true;
}

// ── keys ─────────────────────────────────────────────────────────────────────
static void adjust(int up, int adj) {
  Patch& P = syn.pt;
  if (page == PG_PARAMS) {
    pCur = (pCur + up + NPARAM) % NPARAM;
    P.p[pCur] = clamp01(P.p[pCur] + adj * 0.02f);
  } else if (page == PG_MATRIX) {
    mRow = (mRow + up + NSLOT) % NSLOT;
    Slot& s = P.mod[mRow];
    if (mCol == 0) s.src = (s.src + adj + NSRC) % NSRC;
    else if (mCol == 1) s.dst = (s.dst + adj + NPARAM) % NPARAM;
    else s.depth = constrain(s.depth + adj * 5, -100, 100);
  }
}

static void saveByte(const char* key, uint8_t v) {  // device settings in NVS
  Preferences p;
  p.begin("cardrone", false);
  p.putUChar(key, v);
  p.end();
}

static int rndN(int n) { return (int)(esp_random() % n); }
static float rnd01() { return (esp_random() >> 8) * (1 / 16777216.f); }

// New params in drone-friendly ranges, into p (the patch, or the autopilot's targets).
// PITCH, NOISE and LEVEL stay put (lo < 0).
static void rollParams(float* p) {
  static const float lo[NPARAM] = {-1, .25f, 0, 0, 0, -1, 0, -1, 0, 0, 0, .2f, .1f, .1f, .3f, -1};
  static const float hi[NPARAM] = {0, .7f, .6f, 1, .7f, 0, .6f, 0, .5f, .25f, .5f, .9f, .9f, .7f, .9f, 0};
  for (int i = 0; i < NPARAM; i++)
    if (lo[i] >= 0) p[i] = lo[i] + (hi[i] - lo[i]) * rnd01();
  // Shimmer is a strong colour: usually left out, otherwise just a touch.
  p[P_SHIMMER] = rndN(4) ? 0 : 0.05f + 0.25f * rnd01();
}

static void randomize() {
  remember();
  rollParams(syn.pt.p);
  say("RANDOMIZED");
}

// A drone-safe routing: no PITCH, LEVEL, NOISE, CRUSH or SHIMMER targets, depths of
// 10-40 (mostly positive), about a quarter of slots off.
static void rollSlot(Slot& s) {
  static const uint8_t DST[] = {P_CUTOFF, P_RESO, P_TIMBRE, P_SPREAD, P_LFO,  P_DRIVE,
                                P_RING,   P_CHORUS, P_DTIME, P_DFB,   P_REVERB};
  s.src = rndN(NSRC);
  s.dst = DST[rndN(sizeof DST)];
  s.depth = rndN(4) == 0 ? 0 : (rndN(4) == 0 ? -5 : 5) * (2 + rndN(7));
}

// Held r: new routings, depths and LFO shape only, params untouched.
static void randomizeMods() {
  remember();
  for (Slot& s : syn.pt.mod) rollSlot(s);
  syn.pt.lfo = rndN(NLFO);
  page = PG_MATRIX;  // show the new routings
  say("RANDOMIZED MODS");
}

static void toggleDrift() {
  drift = !drift;
  driftHome = curNote();
  driftAt = millis() + 10000;
  say(drift ? "DRIFT: the root wanders the scale" : "DRIFT off");
}

// ── autopilot (hold a) ───────────────────────────────────────────────────────
// It plays itself: the root drifts, params ease toward new random targets, a matrix
// slot is re-rolled every half minute or so, and every 3-6 minutes a new section
// fades in: another scale, often another engine, voicing or key.
static uint32_t autoParAt, autoModAt, autoSecAt, autoTick;  // when the next of each is due
static float autoTo[NPARAM];  // where the params are easing to; < 0: left alone
static uint32_t inMs(uint32_t lo, uint32_t hi) { return millis() + lo + esp_random() % (hi - lo); }

static void setAuto(bool on) {
  if (on) remember();  // u gets back to the patch from before
  autoOn = drift = on;
  driftHome = curNote();
  driftAt = millis() + 10000;
  for (float& t : autoTo) t = -1;
  autoParAt = millis();  // first targets right away
  autoModAt = inMs(15000, 30000);
  autoSecAt = inMs(180000, 360000);
  if (autoSwapAt) syn.duck = false, autoSwapAt = 0;
  say(on ? "AUTO: it plays itself (a stops it)" : "AUTO off");
}

static void autopilot(uint32_t now) {
  Patch& P = syn.pt;
  if (now - autoTick >= 100) {  // params ease toward their targets (~25 s)
    autoTick = now;
    for (int i = 0; i < NPARAM; i++)
      if (autoTo[i] >= 0) P.p[i] += (autoTo[i] - P.p[i]) * 0.004f;
  }
  if (now >= autoParAt) rollParams(autoTo), autoParAt = inMs(60000, 120000);
  if (now >= autoModAt) rollSlot(P.mod[rndN(NSLOT)]), autoModAt = inMs(20000, 45000);
  if (now >= autoSecAt && !autoSwapAt) syn.duck = true, autoSwapAt = now + 5000;  // new section: fade out...
  if (autoSwapAt && now >= autoSwapAt) {  // ...change under the silence, swell back in
    int sc;
    do sc = rndN(NSCALE - 1);  // never chromatic: it drifts too freely
    while (sc == P.scale);
    P.scale = sc;
    if (rndN(2)) P.engine = rndN(NENGINE);
    if (!rndN(3)) P.voices = 1 + rndN(3);
    if (!rndN(3)) P.interval = rndN(NINTERVAL);
    if (!rndN(3)) P.lfo = rndN(NLFO);
    if (!rndN(3)) {  // key change: up a fourth or down a fifth
      int n = driftHome + (rndN(2) ? 5 : -7);
      driftHome = n < 24 ? n + 12 : n > 71 ? n - 12 : n;
      goNote(driftHome);
    }
    syn.duck = false;
    autoSwapAt = 0;
    autoSecAt = inMs(180000, 360000);
  }
}

static void startCalibration() {
  overlay = OV_NONE;
  calAt = millis() + CAL_COUNTDOWN_MS;
}

static void optionsKey(char c) {
  int dir = c == ',' ? -1 : c == '/' ? 1 : 0, step = dir ? dir : c == '\n';  // ENTER steps forward
  Patch& P = syn.pt;
  if (c == '`') overlay = OV_NONE;
  if (c == ';') optCur = (optCur + NOPT - 1) % NOPT;
  if (c == '.') optCur = (optCur + 1) % NOPT;
  if (optCur == 0 && step) audioOn = !audioOn;
  if (optCur == 1 && dir) vol = constrain(vol + dir * 12, 0, VOL_MAX);
  if (optCur == 2 && step) {
    syn.profile = (syn.profile + step + NPROFILE) % NPROFILE;
    saveByte("modp", syn.profile);
  }
  if (optCur == 3 && step) P.lfo = (P.lfo + step + NLFO) % NLFO;
  if (optCur == 4 && dir) P.p[P_LFO] = clamp01(P.p[P_LFO] + dir * 0.02f);
  if (optCur == 5 && step) {
    theme = (theme + step + NTHEME) % NTHEME;
    applyTheme();
    saveByte("theme", theme);
  }
  if (optCur == 6 && c == '\n') {
    overlay = OV_NONE;
    pickNetwork();
  }
  if (optCur == 7 && c == '\n') startCalibration();
  applyVolume();
}

static void onKey(char c) {
  Patch& P = syn.pt;
  if (calibrating()) {  // any key cancels the countdown or the capture
    calAt = 0;
    csi.calLeft = 0;
    return say("calibration cancelled");
  }
  if (overlay == OV_HELP) {
    overlay = OV_NONE;
    return;
  }
  if (overlay == OV_OPTIONS) return optionsKey(c);
  if (pending) {
    bool save = pending == 'w';
    pending = 0;
    if (c < '1' || c > '8') return say("cancelled");
    bool ok = save ? savePreset(c - '0') : loadPreset(c - '0');
    char s[32];
    snprintf(s, sizeof s, "%s %c %s", save ? "SAVE" : "LOAD", c, ok ? "ok" : "FAILED (SD card?)");
    return say(s);
  }
  // Piano on the bottom rows (index = semitone above C); with Shift, an octave up.
  static const char NOTE_KEYS[] = "zsxdcvgbhnjm", NOTE_KEYS_UP[] = "ZSXDCVGBHNJM";
  const char *k = c ? strchr(NOTE_KEYS, c) : nullptr, *ku = c ? strchr(NOTE_KEYS_UP, c) : nullptr;
  if (k || ku) {
    remember();
    bool up = ku;
    if (up != upOct && P.octave + (up ? 1 : -1) >= 0 && P.octave + (up ? 1 : -1) <= 5) {
      P.octave += up ? 1 : -1;  // a Shift+note lifts the octave; the next plain note drops it back
      upOct = up;
    }
    P.root = up ? ku - NOTE_KEYS_UP : k - NOTE_KEYS;
    driftHome = curNote();
    playing = true;  // a note while stopped starts the drone on it
    return;
  }
  switch (c) {
    case ' ': playing = !playing; break;
    case '\t': page = (page + 1) % NPAGE; break;
    case '\n':
      if (!playing || page < PG_PARAMS) playing = !playing;  // Params/Matrix keep ENTER for editing while playing
      else if (page == PG_PARAMS) P.fx ^= PARAM_FX[pCur];
      else mCol = (mCol + 1) % 3;
      break;
    case '\b': if (page == PG_MATRIX) P.mod[mRow].depth = 0; break;
    case 'i': overlay = OV_HELP; break;
    case '`': overlay = OV_OPTIONS; break;
    case '1': case '2': case '3': case '4': case '5': case '6': case '7':
      remember();
      P.engine = c - '1';
      break;
    case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17: {  // Fn + 1-7
      static const char* const FXN[7] = {"DRIVE", "CRUSH", "RING", "CHORUS", "DELAY", "REVERB", "SHIMMER"};
      int b = c - 0x11;
      remember();
      P.fx ^= 1 << b;
      char s[24];
      snprintf(s, sizeof s, "%s %s", FXN[b], P.fx >> b & 1 ? "on" : "off");
      say(s);
      break;
    }
    case 'u':
      if (!undoN) {
        say("nothing to undo");
        break;
      }
      P = undoBuf[--undoN];
      upOct = false;
      driftHome = curNote();
      say("UNDO");
      break;
    case 'f':
      syn.freeze = !syn.freeze;
      say(syn.freeze ? "FROZEN: the room stops steering" : "UNFROZEN");
      break;
    case 't': {
      remember();
      P.interval = (P.interval + 1) % NINTERVAL;
      char s[32];
      snprintf(s, sizeof s, "INTERVAL: %s (voice 2)", INTERVAL_NAME[P.interval]);
      say(s);
      break;
    }
    case 'a':
      if (autoOn) setAuto(false);  // a press stops the autopilot at once
      else holdKey = c, holdAt = millis();  // tap: drift; hold: autopilot
      break;
    case 'r': holdKey = c, holdAt = millis(); break;  // tap: params; hold: mods
    case 'p': {
      remember();
      P.voices = P.voices % 3 + 1;
      char s[40];
      snprintf(s, sizeof s, "VOICES %d: root%s%s%s", P.voices, P.voices > 1 ? " + " : "",
               P.voices > 1 ? INTERVAL_NAME[P.interval] : "", P.voices > 2 ? " + sub" : "");
      say(s);
      break;
    }
    case '[': case ']': case '9': case '0':
      remember();
      if (c == '[') P.root = (P.root + 11) % 12;
      if (c == ']') P.root = (P.root + 1) % 12;
      if (c == '9' && P.octave > 0) P.octave--;
      if (c == '0' && P.octave < 5) P.octave++;
      upOct = false;
      driftHome = curNote();
      break;
    case 'k':
      remember();
      P.scale = (P.scale + 1) % NSCALE;
      break;
    case 'e': startCalibration(); break;
    case 'w': case 'l': pending = c; break;
    case '-': case '=':
      vol = constrain(vol + (c == '=' ? 12 : -12), 0, VOL_MAX);
      applyVolume();
      break;
    case ';': adjust(-1, 0); break;
    case '.': adjust(1, 0); break;
    case ',': adjust(0, -1); break;
    case '/': adjust(0, 1); break;
  }
}

static void serviceKeys() {
  static char held = 0;
  static uint32_t repeatAt = 0;
  M5Cardputer.update();
  auto& kb = M5Cardputer.Keyboard;
  uint32_t now = millis();
  if (kb.isChange()) {
    held = kb.isPressed() ? readKey() : 0;
    if (holdAt && !held) holdKey == 'r' ? randomize() : toggleDrift();  // let go before the hold fired
    syn.duck = held == 'q' || autoSwapAt;  // q: fade out while held, swell back on release
    repeatAt = now + 350;
    holdAt = 0;  // any key change ends a hold (another key cancels it); a press of r or a restarts it
    if (held) onKey(held);
  } else if (holdAt && now - holdAt >= HOLD_MS) {
    holdAt = 0;
    holdKey == 'r' ? randomizeMods() : setAuto(true);
  } else if (held && strchr(";.,/-=", held) && now >= repeatAt) {  // auto-repeat nav + volume
    repeatAt = now + 45;
    onKey(held);
  }
}

// ── main ─────────────────────────────────────────────────────────────────────
void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  M5Cardputer.Display.setRotation(1);
  cv.setColorDepth(16);
  cv.createSprite(W, H);
  Preferences p;
  if (p.begin("cardrone", true)) {
    theme = p.getUChar("theme", 0) % NTHEME;
    syn.profile = p.getUChar("modp", 2) % NPROFILE;
    p.end();
  }
  applyTheme();
  const char* crash = nullptr;  // why the last run ended, if it wasn't a clean power-up
  switch (esp_reset_reason()) {
    case ESP_RST_PANIC: crash = "last run CRASHED (panic)"; break;
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: crash = "last run CRASHED (watchdog)"; break;
    case ESP_RST_BROWNOUT: crash = "last run lost power (brownout)"; break;
    default: break;
  }
  if (crash) Serial.println(crash);
  bootAnimation(crash);

  auto sc = M5Cardputer.Speaker.config();
  sc.sample_rate = SR;
  sc.dma_buf_count = 8;
  sc.dma_buf_len = 512;
  // Audio owns core 1: WiFi preempting the synth on core 0 made M5Unified pad
  // late blocks with silence (pops). UI loop runs on core 0 (ARDUINO_RUNNING_CORE).
  sc.task_pinned_core = APP_CPU_NUM;
  M5Cardputer.Speaker.config(sc);
  M5Cardputer.Speaker.begin();
  applyVolume();
  xTaskCreatePinnedToCore(audioTask, "audio", 4096, nullptr, 5, nullptr, APP_CPU_NUM);  // above spk_task (2)
  if (crash) delay(1500);  // time to read it

  WiFi.mode(WIFI_STA);
  if (!resume()) pickNetwork();
  Serial.printf("free heap %u, largest block %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());
}

void loop() {
  static uint32_t tSec = 0, tRow = 0, tDraw = 0, lastFrames = 0, lastPkts = 0;
  static int quietSecs = 0;
  static bool wasCal = false;
  serviceKeys();
  uint32_t now = millis();
  if (netMode == NET_LORA) loraPoll(now);
  if (now - tSec >= 1000) {
    tSec = now;
    batt = M5.Power.getBatteryLevel();
    csiHz = netMode == NET_LORA ? loraPkts - lastPkts : csi.frames - lastFrames;  // LoRa: packets heard
    lastFrames = csi.frames;
    lastPkts = loraPkts;
    if (netMode == NET_JOIN && WiFi.status() == WL_CONNECTED)
      if (auto* b = WiFi.BSSID()) memcpy(apMac, b, 6);  // follow roaming
    quietSecs = netMode == NET_AUTO && csiHz < 3 ? quietSecs + 1 : 0;
    if (quietSecs >= 8) {  // the transmitter AUTO locked onto went quiet
      quietSecs = 0;
      autoSurvey();
      lastFrames = csi.frames;
      tSec = millis();
    }
  }
  if (now - tRow >= 66) {  // 15 Hz history for waterfall + sparkline
    tRow = now;
    if (!csiHz)
      for (int s = 0; s < S_LFO; s++) csi.src[s] *= 0.9f;  // no data: mods relax to base values
    wfHead = (wfHead + WF_ROWS - 1) % WF_ROWS;
    for (int k = 0; k < NSC; k++) wf[wfHead][k] = (uint8_t)(clamp01(csi.prof[k]) * 31);
    motHist[histHead] = csi.src[S_MOT];
    histHead = (histHead + 1) % HIST;
  }
  if (drift && playing && now >= driftAt) {  // auto drift: step to another scale note near home
    uint16_t mask = SCALES[syn.pt.scale].mask;
    int pick[15], np = 0;
    for (int d = -7; d <= 7; d++) {
      int note = driftHome + d;
      if ((mask >> ((d + 12) % 12) & 1) && note != curNote() && note >= 12 && note <= 83) pick[np++] = note;
    }
    if (np) {
      int note = pick[esp_random() % np];
      goNote(note);
    }
    driftAt = now + (uint32_t)((20000 + esp_random() % 40000) * (1 - 0.6f * clamp01(csi.src[S_MOT])));  // movement hurries it
  }
  if (autoOn && playing) autopilot(now);
  if (calAt && millis() >= calAt) {  // countdown over: capture the empty room
    calAt = 0;
    csi.calibrate();
  }
  bool cal = csi.calLeft > 0;
  if (wasCal && !cal && csi.calibrated) say("BASELINE SET");
  wasCal = cal;
  if (now - tDraw >= 33) {
    tDraw = now;
    draw();
  }
  delay(2);
}
