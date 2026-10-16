// CSI feature extraction: one WiFi frame in, eight 0..1 mod sources out.
// Pure C++ (no Arduino) so test/host_test.cpp can drive it on a laptop.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>

constexpr int NSC = 52;          // used LLTF subcarriers, ordered -26..-1, 1..26
constexpr int NBAND = 3;         // LO / MID / HI subcarrier groups
constexpr int kCalFrames = 150;  // ~3 s of baseline at the 50 Hz ping rate
// Smallest normaliser range, as a multiple of the noise floor. Higher = a still
// room sits nearer 0, but small motion reads weaker. Tune on real hardware.
constexpr float kMinRange = 1.0f;

enum Src { S_MOT, S_PHS, S_FLX, S_RSSI, S_LO, S_MID, S_HI, S_LFO, NSRC };
static const char* const SRC_NAME[NSRC] = {"MOT", "PHS", "FLX", "RSI", "LO", "MID", "HI", "LFO"};

static inline float clamp01(float v) { return fminf(fmaxf(v, 0.f), 1.f); }

// Adaptive normaliser: floor drops fast and creeps up,
// ceiling jumps up and decays slowly. Learns fast for the first ~2 s.
struct Norm {
  float lo = -1, hi = 0;
  int n = 0;
  float operator()(float v) {
    if (lo < 0) lo = hi = v;
    lo += (v - lo) * (v < lo ? 0.1f : ++n < 100 ? 0.05f : 0.002f);
    hi = v > hi ? v : hi + (v - hi) * 0.005f;
    return clamp01((v - lo) / fmaxf(hi - lo, lo * kMinRange + 1e-5f));
  }
};

struct Csi {
  float fast[NSC] = {}, slow[NSC] = {}, base[NSC] = {}, prev[NSC] = {};
  float prof[NSC] = {};  // 0..1 spectrum shape: waterfall, bars, RF organ partials
  float profLo = 0, profHi = 1;
  float src[NSRC] = {};  // mod sources; S_LFO is written by the synth (bipolar)
  volatile uint32_t frames = 0;
  int rssi = -100;
  volatile int calLeft = 0;  // frames still to capture; <= 0 = not calibrating (the UI may zero it to cancel)
  bool calibrated = false;
  float phWin[32] = {};
  int phIdx = 0, phFill = 0;
  Norm nMot, nPhs, nFlx, nBand[NBAND];

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
  void frame(const int8_t* b, int len, int rssiDbm, bool firstWordBad) {
    if (len < 128) return;
    float a[NSC], sum = 0, sinSum = 0;
    int nv = 0;
    for (int k = 0; k < NSC; k++) {
      int i = k < 26 ? 38 + k : k - 25;    // bins -26..-1 live at 38..63, 1..26 at 1..26
      if (i == 1 && firstWordBad) i = 2;   // S3 quirk: first 4 bytes may be garbage
      float im = b[2 * i], re = b[2 * i + 1];
      a[k] = sqrtf(re * re + im * im);
      sum += a[k];
      if (a[k] > 0.5f) { sinSum += im / a[k]; nv++; }
    }
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
    src[S_FLX] += (nFlx(flux / NSC) - src[S_FLX]) * 0.5f;
    src[S_RSSI] = clamp01((rssiDbm + 90) / 60.f);
    for (int j = 0; j < NBAND; j++) src[S_LO + j] += (nBand[j](band[j] / cnt[j]) - src[S_LO + j]) * 0.2f;
    rssi = rssiDbm;
    frames = frames + 1;
  }
};
