// CSI feature extraction: one WiFi frame in, eight 0..1 mod sources out.
// LoRa mode feeds the same struct one link level per frame instead (level()).
// Pure C++ (no Arduino) so test/host_test.cpp can drive it on a laptop.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>

constexpr int NSC = 52;          // used LLTF subcarriers, ordered -26..-1, 1..26
constexpr int DC_L = NSC / 2 - 1, DC_R = NSC / 2;  // subcarriers -1 and +1, either side of DC
constexpr int NBAND = 3;         // LO / MID / HI subcarrier groups
constexpr int kCalFrames = 150;  // ~3 s of baseline at the 50 Hz ping rate
// Smallest normaliser range, as a multiple of the noise floor. Higher = a still
// room sits nearer 0, but small motion reads weaker. Tune on real hardware.
constexpr float kMinRange = 1.0f;

enum Src { S_MOT, S_PHS, S_FLX, S_RSSI, S_LO, S_MID, S_HI, S_LFO, NSRC };
static const char* const SRC_NAME[NSRC] = {"MOT", "PHS", "FLX", "RSI", "LO", "MID", "HI", "LFO"};
// LoRa mode: same slots, so the matrix and presets carry over (see level()).
static const char* const SRC_NAME_LORA[NSRC] = {"MOT", "TCH", "FLX", "LVL", "LO", "MID", "HI", "LFO"};
constexpr int LN = 256;             // LoRa wobble spectrum: the last 5 s of 50 Hz frames
constexpr float kLoraFloor = 0.5f;  // dB: wobble smaller than this never reads as full scale (RSSI steps are 0.5 dB)
constexpr float kLvlRange = 6;       // dB away from the empty-room level that reads as full LVL
constexpr float kTouchMin = 0.05f;   // touch rise (fraction of the untouched reading) that can read as full TCH

static inline float clamp01(float v) { return fminf(fmaxf(v, 0.f), 1.f); }

// Adaptive normaliser: floor drops fast and creeps up,
// ceiling jumps up and decays slowly. Learns fast for the first ~2 s.
struct Norm {
  float lo = -1, hi = 0;
  int n = 0;
  // floor: smallest range, in v's units. creep: how fast steady activity becomes the new floor.
  float operator()(float v, float floor = 1e-5f, float creep = 0.002f) {
    if (lo < 0) lo = hi = v;
    lo += (v - lo) * (v < lo ? 0.1f : ++n < 100 ? 0.05f : creep);
    hi = v > hi ? v : hi + (v - hi) * 0.005f;
    return clamp01((v - lo) / fmaxf(hi - lo, lo * kMinRange + floor));
  }
};

struct Csi {
  float fast[NSC] = {}, slow[NSC] = {}, base[NSC] = {}, prev[NSC] = {};
  float prof[NSC] = {};  // 0..1 spectrum shape: waterfall, bars, organ partials
  float profLo = 0, profHi = 1;
  float src[NSRC] = {};  // mod sources; S_LFO is written by the synth (bipolar)
  volatile uint32_t frames = 0;
  int rssi = -100;
  volatile int calLeft = 0;  // frames still to capture; <= 0 = not calibrating (the UI may zero it to cancel)
  bool calibrated = false;
  float phWin[32] = {};
  int phIdx = 0, phFill = 0;
  Norm nMot, nPhs, nFlx, nBand[NBAND];
  float lv[4] = {}, env[NBAND] = {}, lvHist[LN] = {}, spec[NSC] = {}, specPeak = 0, tchHi = 0;  // LoRa
  int lvIdx = 0;

  // Empty-room baseline: LO/MID/HI then measure deviation from it instead of
  // from the slow running average. Cancel by zeroing calLeft.
  void calibrate() {
    memset(base, 0, sizeof base);
    calibrated = false;
    nMot = nPhs = nFlx = Norm();
    for (auto& n : nBand) n = Norm();
    calLeft = kCalFrames;
  }

  // b: raw CSI buffer, LLTF first (64 bins of imag,real int8 pairs).
  void frame(const int8_t* b, int len, int rssiDbm) {
    if (len < 128) return;
    float a[NSC], sum = 0, sinSum = 0;
    int nv = 0;
    for (int k = 0; k < NSC; k++) {
      int i = k < 26 ? 38 + k : k - 25;  // bins -26..-1 live at 38..63, 1..26 at 1..26
      float im = b[2 * i], re = b[2 * i + 1];
      a[k] = sqrtf(re * re + im * im);
      if (k != DC_L && k != DC_R && a[k] > 0.5f) { sinSum += im / a[k]; nv++; }
    }
    // -1/+1 sit beside the radio's own carrier leakage (and +1 in the S3's
    // sometimes-garbage first word): rebuild them from their neighbours, or they
    // draw a seam down the middle of the waterfall.
    a[DC_L] = (2 * a[DC_L - 1] + a[DC_R + 1]) / 3;
    a[DC_R] = (a[DC_L - 1] + 2 * a[DC_R + 1]) / 3;
    for (float v : a) sum += v;
    if (sum < 1) return;
    float inv = NSC / sum;  // divide out AGC gain, keep only the multipath shape
    bool first = frames == 0;
    float flux = 0, dev = 0, lo = 1e9f, hi = 0, band[NBAND] = {}, cnt[NBAND] = {};
    for (int k = 0; k < NSC; k++) {
      float v = a[k] * inv;
      if (first) fast[k] = slow[k] = prev[k] = v;
      flux += fabsf(v - prev[k]);
      prev[k] = v;
      fast[k] += (v - fast[k]) * 0.3f;
      slow[k] += (v - slow[k]) * 0.01f;
      dev += fabsf(fast[k] - slow[k]);
      int bi = k * NBAND / NSC;
      band[bi] += fabsf(fast[k] - (calibrated ? base[k] : slow[k]));
      cnt[bi]++;
      lo = fminf(lo, fast[k]);
      hi = fmaxf(hi, fast[k]);
      if (calLeft > 0) base[k] += v / kCalFrames;
    }
    if (calLeft > 0 && --calLeft == 0) calibrated = true;  // a cancel racing the decrement leaves -1: idle
    for (int k = 0; k < NSC; k++) prof[k] = (fast[k] - lo) / (hi - lo + 1e-6f);
    profLo = lo;
    profHi = hi;

    // Phase: variance of mean sin(phase) over a short window.
    phWin[phIdx] = nv ? sinSum / nv : 0;
    phIdx = (phIdx + 1) % 32;
    if (phFill < 32) phFill++;
    float pm = 0, pv = 0;
    for (int i = 0; i < phFill; i++) pm += phWin[i];
    pm /= phFill;
    for (int i = 0; i < phFill; i++) pv += (phWin[i] - pm) * (phWin[i] - pm);
    pv /= phFill;

    src[S_MOT] += (nMot(dev / NSC) - src[S_MOT]) * 0.2f;
    src[S_PHS] += (nPhs(pv) - src[S_PHS]) * 0.2f;
    src[S_FLX] += (nFlx(flux / NSC) - src[S_FLX]) * 0.2f;  // as the others: ~100 ms at 50 Hz
    src[S_RSSI] = clamp01((rssiDbm + 90) / 60.f);
    for (int j = 0; j < NBAND; j++) src[S_LO + j] += (nBand[j](band[j] / cnt[j]) - src[S_LO + j]) * 0.2f;
    rssi = rssiDbm;
    frames = frames + 1;
  }

  // LoRa mode: one frame = the Heltec's packets in the last 20 ms, as their mean level
  // (dBm), plus its touch reading. No subcarriers and no phase, so the slots refill:
  // MOT, FLX as for CSI, from the level; PHS -> TCH, the touch; RSI -> LVL, the level,
  // or once calibrated its distance from the empty room's; LO/MID/HI -> how fast the
  // level wobbles: under 0.5 Hz (settling, doors), 0.5-2 Hz (gestures), 2-8 Hz
  // (walking). prof becomes the wobble spectrum, 0.2-10 Hz.
  void level(float dbm, float touch) {
    static float cs[LN], sn[LN], win[LN];
    if (!win[1])
      for (int n = 0; n < LN; n++) {
        cs[n] = cosf(6.2831853f * n / LN), sn[n] = sinf(6.2831853f * n / LN);
        win[n] = 0.5f - 0.5f * cs[n];  // Hann
      }
    if (frames == 0) {
      fast[0] = slow[0] = prev[0] = dbm;
      for (float& v : lv) v = dbm;
      for (float& h : lvHist) h = dbm;
    }
    fast[0] += (dbm - fast[0]) * 0.3f;
    slow[0] += (dbm - slow[0]) * 0.01f;
    float flux = fabsf(dbm - prev[0]);
    prev[0] = dbm;
    // Speed bands: differences of one-pole lowpasses at 8, 2, 0.5 and 0.05 Hz. The slower
    // the band, the slower its envelope, and the longer steady wobble takes to fade (C).
    static const float A[4] = {0.634f, 0.222f, 0.0609f, 0.00626f}, E[NBAND] = {0.005f, 0.05f, 0.2f},
                       C[NBAND] = {0.0005f, 0.001f, 0.002f};
    for (int i = 0; i < 4; i++) lv[i] += (dbm - lv[i]) * A[i];
    for (int j = 0; j < NBAND; j++) env[j] += (fabsf(lv[2 - j] - lv[3 - j]) - env[j]) * E[j];
    if (calLeft > 0) base[0] += dbm / kCalFrames;
    if (calLeft > 0 && --calLeft == 0) calibrated = true;

    lvHist[lvIdx] = dbm;
    lvIdx = (lvIdx + 1) % LN;
    float x[LN], mean = 0, peak = 0;
    for (float h : lvHist) mean += h;
    mean /= LN;
    for (int n = 0; n < LN; n++) x[n] = (lvHist[(lvIdx + n) % LN] - mean) * win[n];
    for (int k = 0; k < NSC; k++) {  // bin k+1: (k+1) * 50/LN Hz
      float re = 0, im = 0;
      for (int n = 0, ph = 0; n < LN; n++, ph = (ph + k + 1) & (LN - 1)) re += x[n] * cs[ph], im += x[n] * sn[ph];
      spec[k] += (sqrtf(re * re + im * im) * (4.f / LN) - spec[k]) * 0.3f;  // dB: a 1 dB sine reads ~1
      peak = fmaxf(peak, spec[k]);
    }
    specPeak = fmaxf(peak, specPeak * 0.995f);
    float span = fmaxf(specPeak, 2 * kLoraFloor);  // a still link stays dim
    for (int k = 0; k < NSC; k++) prof[k] = spec[k] / span;
    profLo = 0;
    profHi = span;

    tchHi = fmaxf(touch, tchHi * 0.999f);  // the strongest recent touch reads as full, decaying over ~20 s
    src[S_MOT] += (nMot(fabsf(fast[0] - slow[0]), kLoraFloor) - src[S_MOT]) * 0.2f;
    src[S_PHS] += (clamp01(touch / fmaxf(tchHi, kTouchMin)) - src[S_PHS]) * 0.2f;
    src[S_FLX] += (nFlx(flux, kLoraFloor) - src[S_FLX]) * 0.2f;
    src[S_RSSI] = calibrated ? clamp01(fabsf(lv[2] - base[0]) / kLvlRange) : clamp01((fast[0] + 100) / 60.f);
    for (int j = 0; j < NBAND; j++) src[S_LO + j] += (nBand[j](env[j], kLoraFloor, C[j]) - src[S_LO + j]) * 0.2f;
    rssi = (int)dbm;
    frames = frames + 1;
  }
};
