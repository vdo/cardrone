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
static const char* const HINTS[NPAGE] = {"TAB view  1-7 drone  r random  h help", "c calibrate  s/l preset  h help",
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
constexpr uint32_t R_HOLD_MS = 3000;  // hold r this long to roll the mod matrix too
static uint32_t rHeldAt = 0;          // when a press of r randomized the params, while it stays held
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
enum NetMode { NET_OFF, NET_JOIN, NET_LISTEN, NET_AUTO };
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

static void radioOff() {  // CSI off, disassociated, no background reconnect hopping channels
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
        cv.drawCenterString("press ENTER to play", W / 2, 88);
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

static void netDesc(char* s, int cap) {
  static const char* const M[] = {"OFF", "JOIN", "LISTEN", "AUTO"};
  if (netMode == NET_OFF) snprintf(s, cap, "OFF  >");
  else snprintf(s, cap, "%s %.10s ch%d  >", M[netMode], netName, netCh);
}

static void drawHeader() {
  const Patch& P = syn.pt;
  cv.fillRect(0, 0, W, 12, C_BAR);
  cv.drawFastHLine(0, 12, W, C_MID);
  cv.setTextColor(C_HI);
  cv.drawString("CARDRONE", 3, 2);
  const char* en = ENGINE_NAME[P.engine];
  int ew = cv.textWidth(en) + 6;
  cv.fillRoundRect(55, 1, ew, 10, 2, ENGINE_COLOR[P.engine]);
  cv.setTextColor(C_BG);
  cv.drawString(en, 58, 2);
  char s[24];
  snprintf(s, sizeof s, "%s%d %s", NOTE_NAME[P.root], P.octave, SCALES[P.scale].name);
  cv.setTextColor(C_HI);
  cv.drawString(s, 60 + ew, 2);
  const char tag = "-JLA"[netMode];  // network mode, then channel
  if (csi.calLeft > 0) snprintf(s, sizeof s, "CAL %d%%", 100 - csi.calLeft * 100 / kCalFrames);
  else if (netMode == NET_OFF) snprintf(s, sizeof s, "NO NET");
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
    cv.drawCenterString("NO CSI - ` for network", 82, 58);
  }
  for (int s = 0; s < NSRC; s++) {  // source meters with a decaying phosphor peak
    int x = 166, y = 16 + s * 12;
    float v = srcLevel(s);
    srcPeak[s] = fmaxf(v, srcPeak[s] - 0.012f);
    cv.setTextColor(C_MID);
    cv.drawString(SRC_NAME[s], x, y);
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
    if (csi.calibrated)
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
    cv.drawString(SRC_NAME[s.src], 14, y);
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
  static const char* const K[][2] = {
      {"TAB", "next page"}, {"1-7", "drone engine"}, {"[ ]", "root note"}, {"9 0", "octave"},
      {"k", "scale"},       {"r", "rnd; hold: mods"},      {"c", "calibrate room"}, {"s l", "save/load 1-8"},
      {"- =", "volume"},    {"`", "options+network"}, {";.", "up / down"}, {",/", "adjust"},
      {"ENT", "play/stop, edit"}, {"DEL", "clear mod slot"}, {"n", "voices 1-3"}, {"h", "this help"}};
  for (int i = 0; i < (int)(sizeof K / sizeof *K); i++) {
    int x = 6 + (i / 8) * 120, y = 16 + (i % 8) * 11;
    cv.setTextColor(C_HI);
    cv.drawString(K[i][0], x, y);
    cv.setTextColor(C_MID);
    cv.drawString(K[i][1], x + 24, y);
  }
  cv.setTextColor(C_DIM);
  cv.drawCenterString("headphones plugged in = speaker off", W / 2, 103);
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
  cv.drawCenterString(csiHz ? "keep the room empty" : "waiting for CSI...", W / 2, 92);
}

static void drawFooter() {
  cv.drawFastHLine(0, 113, W, C_MID);
  cv.setTextColor(C_DIM);
  cv.drawString("MOT", 3, 116);
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
  char hold[40] = "";  // r held: count down to the mod roll
  if (rHeldAt)
    snprintf(hold, sizeof hold, "RANDOMIZED  hold for mods: %lu",
             (unsigned long)(R_HOLD_MS - min(R_HOLD_MS, (uint32_t)(millis() - rHeldAt)) + 999) / 1000);
  bool note = pending || millis() < toastUntil || *hold;
  bool idle = !playing && !overlay && !calibrating();  // pulse "ENTER play" while silent
  cv.setTextColor(note ? C_WARN : idle && millis() / 500 % 2 ? C_HI : C_DIM);
  cv.drawString(pending == 's'              ? "SAVE to slot 1-8 (other key cancels)"
                : pending                   ? "LOAD slot 1-8 (other key cancels)"
                : *hold                     ? hold
                : note                      ? toast
                : calibrating()             ? "any key cancels"
                : overlay == OV_HELP        ? "any key closes"
                : overlay == OV_OPTIONS     ? ";. move  ,/ ENTER change  ` close"
                : !playing                  ? "ENTER play  TAB view  h help"
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
  return ks.tab ? '\t' : ks.enter ? '\n' : ks.del ? '\b' : ks.word.empty() ? 0 : ks.word.back();
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
    int i = choose("NETWORK", n + 1, [](int i, char* l, char* r) {
      if (!i) {
        strcpy(l, "AUTO    survey every channel");
        return;
      }
      snprintf(l, 40, "%.24s", WiFi.SSID(i - 1).c_str());
      snprintf(r, 16, "%sch%d %d", WiFi.encryptionType(i - 1) == WIFI_AUTH_OPEN ? "" : "*", (int)WiFi.channel(i - 1),
               (int)WiFi.RSSI(i - 1));
    });
    if (i < 0) break;
    if (i == 0) {
      autoSurvey();
      saveNet(NET_AUTO, "", "", nullptr, 0);
      return;
    }
    if (connectTo(i - 1)) return;
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
  syn.pt = p;
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

// New params in drone-friendly ranges; PITCH, NOISE and LEVEL stay put (lo < 0).
static void randomize() {
  static const float lo[NPARAM] = {-1, .25f, 0, 0, 0, -1, 0, -1, 0, 0, 0, .2f, .1f, .1f, .3f, -1};
  static const float hi[NPARAM] = {0, .7f, .6f, 1, .7f, 0, .6f, 0, .5f, .25f, .5f, .9f, .9f, .7f, .9f, 0};
  auto u = [] { return (esp_random() >> 8) * (1 / 16777216.f); };  // 0..1
  for (int i = 0; i < NPARAM; i++)
    if (lo[i] >= 0) syn.pt.p[i] = lo[i] + (hi[i] - lo[i]) * u();
  // Shimmer is a strong colour: usually left out, otherwise just a touch.
  syn.pt.p[P_SHIMMER] = esp_random() % 4 ? 0 : 0.05f + 0.25f * u();
  rHeldAt = millis();
  say("RANDOMIZED");
}

// Held r: new routings, depths and LFO shape. Drone-safe: no PITCH, LEVEL, NOISE,
// CRUSH or SHIMMER targets, depths of 10-40 (mostly positive), about a quarter of slots off.
static void randomizeMods() {
  static const uint8_t DST[] = {P_CUTOFF, P_RESO, P_TIMBRE, P_SPREAD, P_LFO,  P_DRIVE,
                                P_RING,   P_CHORUS, P_DTIME, P_DFB,   P_REVERB};  // never SHIMMER
  auto rnd = [](int n) { return (int)(esp_random() % n); };
  for (Slot& s : syn.pt.mod) {
    s.src = rnd(NSRC);
    s.dst = DST[rnd(sizeof DST)];
    s.depth = rnd(4) == 0 ? 0 : (rnd(4) == 0 ? -5 : 5) * (2 + rnd(7));
  }
  syn.pt.lfo = rnd(NLFO);
  page = PG_MATRIX;  // show the new routings
  say("RANDOMIZED PARAMS + MODS");
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
    bool save = pending == 's';
    pending = 0;
    if (c < '1' || c > '8') return say("cancelled");
    bool ok = save ? savePreset(c - '0') : loadPreset(c - '0');
    char s[32];
    snprintf(s, sizeof s, "%s %c %s", save ? "SAVE" : "LOAD", c, ok ? "ok" : "FAILED (SD card?)");
    return say(s);
  }
  switch (c) {
    case '\t': page = (page + 1) % NPAGE; break;
    case '\n':
      if (!playing || page < PG_PARAMS) playing = !playing;  // Params/Matrix keep ENTER for editing while playing
      else if (page == PG_PARAMS) P.fx ^= PARAM_FX[pCur];
      else mCol = (mCol + 1) % 3;
      break;
    case '\b': if (page == PG_MATRIX) P.mod[mRow].depth = 0; break;
    case 'h': overlay = OV_HELP; break;
    case '`': overlay = OV_OPTIONS; break;
    case '1': case '2': case '3': case '4': case '5': case '6': case '7': P.engine = c - '1'; break;
    case 'r': randomize(); break;
    case 'n': {
      static const char* const DESC[3] = {"VOICES 1: root", "VOICES 2: root + fifth", "VOICES 3: root + fifth + sub"};
      P.voices = P.voices % 3 + 1;
      say(DESC[P.voices - 1]);
      break;
    }
    case '[': P.root = (P.root + 11) % 12; break;
    case ']': P.root = (P.root + 1) % 12; break;
    case '9': if (P.octave > 0) P.octave--; break;
    case '0': if (P.octave < 5) P.octave++; break;
    case 'k': P.scale = (P.scale + 1) % NSCALE; break;
    case 'c': startCalibration(); break;
    case 's': case 'l': pending = c; break;
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
    repeatAt = now + 350;
    rHeldAt = 0;  // any key change ends an r hold; a press of r that randomizes restarts it
    if (held) onKey(held);
  } else if (held == 'r' && rHeldAt && now - rHeldAt >= R_HOLD_MS) {
    rHeldAt = 0;
    randomizeMods();
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
  static uint32_t tSec = 0, tRow = 0, tDraw = 0, lastFrames = 0;
  static int quietSecs = 0;
  static bool wasCal = false;
  serviceKeys();
  uint32_t now = millis();
  if (now - tSec >= 1000) {
    tSec = now;
    batt = M5.Power.getBatteryLevel();
    csiHz = csi.frames - lastFrames;
    lastFrames = csi.frames;
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
