// Drone engines, FX chain, mod matrix and patch JSON.
// Pure C++ (no Arduino) so test/host_test.cpp can render it on a laptop.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "csi.h"

// The per-sample loop runs from IRAM: the flash cache is shared with WiFi and
// the UI on the other core, and a miss stalls ~2 us. Helpers inline into it.
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define DSP_IRAM IRAM_ATTR
#else
#define DSP_IRAM
#endif
#define DSP_AI inline __attribute__((always_inline))

constexpr int SR = 44100;
constexpr float PI_F = 3.14159265f;
// Drone pacing, in seconds: how slowly CSI mods and scale steps glide (picked in
// the options), and knob/preset changes.
struct ModProfile { const char* name; float modTau, pitchTau; };
static const ModProfile MOD_PROFILES[] = {{"SHARP", 0.15f, 0.15f}, {"NORMAL", 0.5f, 0.3f}, {"SMOOTH", 1.5f, 0.6f}, {"SLOW", 4.f, 1.5f}};
constexpr int NPROFILE = sizeof MOD_PROFILES / sizeof *MOD_PROFILES;
constexpr float PARAM_TAU = 0.15f, FADE_TAU = 1.2f;  // FADE: held q's fade out / swell back

enum Engine { E_PWM, E_SAW, E_FM, E_ORGAN, E_WAVE, E_VOICE, E_NOISE, NENGINE };
static const char* const ENGINE_NAME[NENGINE] = {"PWM", "SAW", "FM", "ORG", "WAVE", "VOICE", "NOISE"};
enum NoiseKind { N_DUST, N_WIND, N_SURF, N_STATIC, N_METAL, NNOISE };  // TIMBRE morphs through these
enum Lfo { L_SINE, L_TRI, L_SAW, L_SQR, L_SH, L_DRIFT, NLFO };
static const char* const LFO_NAME[NLFO] = {"SINE", "TRIANGLE", "SAW DOWN", "SQUARE", "S&H", "DRIFT"};

enum Param { P_PITCH, P_CUTOFF, P_RESO, P_TIMBRE, P_SPREAD, P_NOISE, P_LFO, P_LEVEL,
             P_DRIVE, P_CRUSH, P_RING, P_CHORUS, P_DTIME, P_DFB, P_REVERB, P_SHIMMER, NPARAM };
static const char* const PARAM_NAME[NPARAM] = {"PITCH", "CUTOFF", "RESO", "TIMBRE", "SPREAD", "NOISE", "LFO", "LEVEL",
                                              "DRIVE", "CRUSH", "RING", "CHORUS", "D.TIME", "D.FB", "REVERB", "SHIMMR"};
enum Fx { FX_DRIVE = 1, FX_CRUSH = 2, FX_RING = 4, FX_CHORUS = 8, FX_DELAY = 16, FX_REVERB = 32, FX_SHIMMER = 64 };
// Which FX switch each param belongs to (0 = voice param, always live).
static const uint8_t PARAM_FX[NPARAM] = {0, 0, 0, 0, 0, 0, 0, 0,
                                         FX_DRIVE, FX_CRUSH, FX_RING, FX_CHORUS, FX_DELAY, FX_DELAY, FX_REVERB, FX_SHIMMER};

struct Scale { const char* name; uint16_t mask; };  // bit n = semitone n above root allowed
static const Scale SCALES[] = {{"MINOR", 0x5AD}, {"MAJOR", 0xAB5}, {"DORIAN", 0x6AD}, {"PHRYG", 0x5AB},
                               {"PENTA", 0x4A9}, {"FIFTHS", 0x081}, {"CHROM", 0xFFF}};
constexpr int NSCALE = sizeof(SCALES) / sizeof(SCALES[0]);
static const char* const NOTE_NAME[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

constexpr int NSLOT = 8;
struct Slot { uint8_t src, dst; int8_t depth; };  // depth -100..100, 0 = off

struct Patch {
  uint8_t engine, root, octave, scale, fx, lfo, voices, interval;  // voices: 1 root, 2 +interval, 3 +sub
  float p[NPARAM];
  Slot mod[NSLOT];
};

static const Patch DEFAULT_PATCH = {
    E_PWM, 9, 2, 0, FX_CHORUS | FX_DELAY | FX_REVERB | FX_SHIMMER, L_SINE, 3, 0,
    {0.5f, 0.45f, 0.3f, 0.4f, 0.35f, 0.f, 0.3f, 0.7f, 0.2f, 0.f, 0.3f, 0.5f, 0.45f, 0.45f, 0.6f, 0.2f},
    {{S_MOT, P_CUTOFF, 35}, {S_PHS, P_TIMBRE, 25}, {S_MOT, P_SHIMMER, 30}, {S_MID, P_REVERB, 20},
     {S_RSSI, P_SPREAD, 15}, {S_LFO, P_CUTOFF, 10}, {S_HI, P_DFB, 20}, {S_LO, P_CHORUS, 20}},
};

static inline float lfoHz(float p) { return 0.02f * exp2f(p * 8); }  // LFO param -> 0.02..5 Hz

// Voice 2's interval above the root. THIRD follows the scale: minor if it has one.
enum Interval { I_FIFTH, I_FOURTH, I_OCTAVE, I_THIRD, NINTERVAL };
static const char* const INTERVAL_NAME[NINTERVAL] = {"FIFTH", "FOURTH", "OCTAVE", "THIRD"};
static inline int intervalSemis(int iv, uint16_t mask) {
  return iv == I_FOURTH ? 5 : iv == I_OCTAVE ? 12 : iv == I_THIRD ? (mask >> 3 & 1 || !(mask >> 4 & 1) ? 3 : 4) : 7;
}

// Nearest semitone to s whose pitch class is in the scale.
static inline int quantize(float s, uint16_t mask) {
  int c = (int)lroundf(s);
  for (int d = 0; d < 12; d++) {
    int a = s >= c ? c + d : c - d, b = s >= c ? c - d : c + d;  // closer side first
    if (mask >> (((a % 12) + 12) % 12) & 1) return a;
    if (mask >> (((b % 12) + 12) % 12) & 1) return b;
  }
  return c;
}

// ── patch JSON (hand-rolled: the format is ours and flat) ────────────────────
static int patchToJson(const Patch& p, char* out, int cap) {  // cap >= 512
  int n = snprintf(out, cap, "{\"v\":2,\"engine\":%d,\"root\":%d,\"octave\":%d,\"scale\":%d,\"fx\":%d,\"lfo\":%d,\"voices\":%d,\"interval\":%d,\n\"params\":[",
                   p.engine, p.root, p.octave, p.scale, p.fx, p.lfo, p.voices, p.interval);
  for (int i = 0; i < NPARAM; i++) n += snprintf(out + n, cap - n, "%s%.3f", i ? "," : "", p.p[i]);
  n += snprintf(out + n, cap - n, "],\n\"matrix\":[");
  for (int i = 0; i < NSLOT; i++)
    n += snprintf(out + n, cap - n, "%s[%d,%d,%d]", i ? "," : "", p.mod[i].src, p.mod[i].dst, p.mod[i].depth);
  return n + snprintf(out + n, cap - n, "]}\n");
}

// Up to n numbers after "key", stopping at the next key or end of object.
static int jsonNums(const char* js, const char* key, float* out, int n) {
  const char* p = strstr(js, key);
  if (!p) return 0;
  p += strlen(key);
  int i = 0;
  while (i < n && *p && *p != '"' && *p != '}') {
    char* e;
    if (*p == '-' || (*p >= '0' && *p <= '9')) {
      out[i] = strtof(p, &e);
      if (e > p) { i++; p = e; continue; }
    }
    p++;
  }
  return i;
}

static int clampInt(float v, int lo, int hi) { return v >= lo ? (v <= hi ? (int)v : hi) : lo; }  // NaN -> lo

// SD files are user-editable: every field is range-checked before use.
static bool patchFromJson(const char* js, Patch& out) {
  static const char* const keys[5] = {"\"engine\"", "\"root\"", "\"octave\"", "\"scale\"", "\"fx\""};
  float h[5], v[NPARAM], m[NSLOT * 3], lfo = 0, voices = 3, interval = 0;
  for (int i = 0; i < 5; i++)
    if (jsonNums(js, keys[i], &h[i], 1) != 1) return false;
  if (jsonNums(js, "\"params\"", v, NPARAM) != NPARAM) return false;
  if (jsonNums(js, "\"matrix\"", m, NSLOT * 3) != NSLOT * 3) return false;
  jsonNums(js, "\"lfo\"", &lfo, 1);  // optional: older presets lack these two
  jsonNums(js, "\"voices\"", &voices, 1);
  jsonNums(js, "\"interval\"", &interval, 1);
  float ver = 1;  // format 2 gave shimmer its own FX bit; before, it came with the reverb
  jsonNums(js, "\"v\"", &ver, 1);
  Patch p;
  p.engine = clampInt(h[0], 0, NENGINE - 1);
  p.root = clampInt(h[1], 0, 11);
  p.octave = clampInt(h[2], 0, 5);
  p.scale = clampInt(h[3], 0, NSCALE - 1);
  p.fx = clampInt(h[4], 0, 127);
  if (ver < 2 && (p.fx & FX_REVERB)) p.fx |= FX_SHIMMER;
  p.lfo = clampInt(lfo, 0, NLFO - 1);
  p.voices = clampInt(voices, 1, 3);
  p.interval = clampInt(interval, 0, NINTERVAL - 1);
  for (int i = 0; i < NPARAM; i++) p.p[i] = clamp01(v[i]);
  for (int i = 0; i < NSLOT; i++)
    p.mod[i] = {(uint8_t)clampInt(m[3 * i], 0, NSRC - 1), (uint8_t)clampInt(m[3 * i + 1], 0, NPARAM - 1),
                (int8_t)clampInt(m[3 * i + 2], -100, 100)};
  out = p;
  return true;
}

// ── DSP helpers ──────────────────────────────────────────────────────────────
// No libm calls or divisions per sample: on the ESP32 those are function calls.
static float SINT[1025];
static DSP_AI float sinT(float ph) {  // ph in cycles, any range
  int n = (int)ph;
  ph -= n - (ph < n);  // floor, negatives too
  float f = ph * 1024;
  int i = (int)f;
  float fr = f - i;
  i &= 1023;  // ph rounded up to 1.0 wraps to 0
  return SINT[i] + (SINT[i + 1] - SINT[i]) * fr;
}
static DSP_AI float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }
static DSP_AI float sat(float x) {  // cubic soft clip, |out| <= 1
  x = clampf(x, -1.5f, 1.5f);
  return x - x * x * x * (4.f / 27);
}
static DSP_AI float blep(float t, float dt, float idt) {
  if (t < dt) { t *= idt; return t + t - t * t - 1; }
  if (t > 1 - dt) { t = (t - 1) * idt; return t * t + t + t + 1; }
  return 0;
}
static DSP_AI float sawB(float p, float dt, float idt) { return 2 * p - 1 - blep(p, dt, idt); }
static DSP_AI float pulseB(float p, float dt, float idt, float pw) {
  float q = p - pw;
  if (q < 0) q += 1;
  return (p < pw ? 1.f : -1.f) + blep(p, dt, idt) - blep(q, dt, idt) - (2 * pw - 1);  // DC removed
}
static DSP_AI float wrap1(float p) { return p >= 1 ? p - 1 : p; }

// Sized for a no-PSRAM board: every KB here is a KB WiFi and SD can't have.
// Reverb lines are float: int16 lines put rounding hiss 39 dB under the drone.
// The tape is stored at half rate to pay for it (its repeats are darkened anyway).
constexpr int DLY_N = SR * 45 / 100 / 2;  // 0.45 s tape, one slot per 2 samples
constexpr int CHO_N = 1024, SHM_N = 1024;
constexpr float SHIM_CEIL = 1.1f;  // reverb's share of the mix (REVERB x wet/dry) the shimmer loop is held under; healthy peaks ~1.06
constexpr int WT_F = 8, WT_N = 256;  // wavetable: frames x samples
static int16_t WT[WT_F][WT_N];
// Formants (Hz) of a bass voice singing a, e, i, o, u.
static const float VOWEL[5][3] = {{600, 1040, 2250}, {400, 1620, 2400}, {250, 1750, 2600}, {400, 750, 2400}, {350, 600, 2400}};
static const int COMB_LEN[6] = {1116, 1188, 1277, 1356, 1422, 1491};  // Freeverb tunings @44.1k
static const int AP_LEN[3] = {556, 441, 341};
constexpr int RVB_N = 1116 + 1188 + 1277 + 1356 + 1422 + 1491 + 556 + 441 + 341;

struct Synth {
  Patch pt = DEFAULT_PATCH;  // edited live by the UI
  float eff[NPARAM] = {};    // modulated values, shown by the UI
  float freq = 110;
  uint8_t profile = 2;  // MOD_PROFILES index, SMOOTH
  bool freeze = false;  // hold the room's mods (and ORG's partials) where they are; the LFO keeps going
  bool duck = false;    // fade the output down (slowly); false swells it back

  Synth() {
    for (int i = 0; i <= 1024; i++) SINT[i] = sinf(2 * PI_F * i / 1024);
    memcpy(baseS, pt.p, sizeof baseS);
    for (int k = 0; k < NSLOT; k++) route[k] = pt.mod[k], routeD[k] = pt.mod[k].depth;
    // Wavetable frames, dark to bright: sine, warm, triangle, saw, square, "ah", nasal, glass.
    for (int f = 0; f < WT_F; f++) {
      float t[WT_N] = {}, peak = 0;
      for (int h = 1; h <= 32; h++) {
        float a = f == 0 ? h == 1
                : f == 1 ? (h <= 4 ? 1.f / (h * h) : 0)
                : f == 2 ? (h & 1 ? 1.f / (h * h) : 0)
                : f == 3 ? 1.f / h
                : f == 4 ? (h & 1 ? 1.f / h : 0)
                : f == 5 ? expf(-(h - 6) * (h - 6) / 6.f) + 0.4f / h
                : f == 6 ? expf(-(h - 14) * (h - 14) / 16.f) + 0.3f / h
                : (h == 1 || h == 4 || h == 9 || h == 16 || h == 25) ? 1 / sqrtf(h) : 0;
        if (a)
          for (int i = 0; i < WT_N; i++) t[i] += a * SINT[(h * i * (1024 / WT_N)) & 1023];
      }
      for (float v : t) peak = fmaxf(peak, fabsf(v));
      for (int i = 0; i < WT_N; i++) WT[f][i] = (int16_t)(t[i] / peak * 32000);
    }
  }

  // src: NSRC mod sources (S_LFO is written here). prof: NSC subcarrier shape.
  void render(int16_t* out, int n, float* src, const float* prof);

 private:
  float ph[16] = {}, pwmPh[6] = {}, mph[3] = {}, partAmp[16] = {}, fz1[3] = {}, fz2[3] = {};
  // NOISE: dust click, wind bands (log2 Hz) + gusts, surf wave + low-pass, static hold + swell, metal LFSRs
  float dustV = 0, windC[2] = {9, 10.5f}, wz1[2] = {}, wz2[2] = {}, gust = 0.5f, gustT = 0.5f;
  float surfPh = 0, surfRate = 0.1f, sz1 = 0, sz2 = 0, shV = 0, gate = 0.5f, gateT = 0.5f, metPh[3] = {};
  int shCnt = 0, shHold = 1;
  uint16_t lfsr[3] = {1, 1, 1};
  float drvLp = 0, drvDc = 0;  // analog drive: circuit bandwidth, DC blocker
  Slot route[NSLOT];    // the routings as currently applied: glide toward pt.mod
  float routeD[NSLOT];  // their depths, gliding
  float baseN = -1;  // root + octave as a gliding MIDI note (-1: not started)
  float baseS[NPARAM], modS[NSRC] = {}, lfoPh = 0, lfoA = 0, lfoB = 0, semis = 0, fade = 0, cut = 1000, ic1 = 0, ic2 = 0;
  int engine = -1, curVoices = 3, curInterval = 0;
  float outGain = 1;
  uint8_t fxOn = 0;
  uint32_t rng = 22222;
  float crushWob = 0, ringPh = 0;  // crusher: wandering bit depth
  float cho[CHO_N] = {}, choPh = 0;
  int choW = 0;
  int16_t dly[DLY_N] = {};
  int dlyW = 0;
  bool dlyOdd = false;
  float dlyS = 0, dlyLp = 0, dlyAcc = 0, wowPh = 0, flutPh = 0;
  float rvb[RVB_N] = {};
  float combLp[6] = {}, shm[SHM_N] = {}, shmPh = 0, shimOut = 0, shimLp = 0, wetEnv = 0, dryEnv = 0;
  int ci[6] = {}, ai[3] = {}, shmW = 0;

  float dlyTarget() const { return (0.04f + eff[P_DTIME] * 0.38f) * SR; }
  DSP_AI float rnd() {  // xorshift, -1..1
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return (int32_t)rng * 4.6566e-10f;
  }

  static DSP_AI float tap(const float* buf, int len, int w, float d) {  // len: power of two
    float pos = w - d;
    if (pos < 0) pos += len;
    int i0 = (int)pos;
    float a = buf[i0], b = buf[(i0 + 1) & (len - 1)];
    return a + (b - a) * (pos - i0);
  }

  // Two crossfaded read heads sweeping at double speed = one octave up.
  DSP_AI float octaveUp(float x) {
    shm[shmW] = x;
    shmPh = wrap1(shmPh + 1.f / SHM_N);
    float y = 0;
    for (int h = 0; h < 2; h++) {
      float p = wrap1(shmPh + h * 0.5f);
      y += (1 - fabsf(2 * p - 1)) * tap(shm, SHM_N, shmW, 1 + (1 - p) * (SHM_N - 3));  // triangles sum to 1
    }
    shmW = (shmW + 1) & (SHM_N - 1);
    return y;
  }
};

// Out of line, not inline: an inline function's literals land in flash, out of
// reach of IRAM code. So only one translation unit per program includes dsp.h.
DSP_IRAM void Synth::render(int16_t* out, int n, float* src, const float* prof) {
  const Patch& P = pt;
  lfoPh += n * lfoHz(eff[P_LFO]) / SR;
  if (lfoPh >= 1) {  // new cycle: S&H and DRIFT pick their next random value
    lfoPh -= floorf(lfoPh);
    lfoA = lfoB;
    lfoB = rnd();
  }
  float t = lfoPh;
  src[S_LFO] = P.lfo == L_TRI ? 1 - 4 * fabsf(t - 0.5f)
             : P.lfo == L_SAW ? 1 - 2 * t
             : P.lfo == L_SQR ? (t < 0.5f ? 1 : -1)
             : P.lfo == L_SH ? lfoB
             : P.lfo == L_DRIFT ? lfoA + (lfoB - lfoA) * t * t * (3 - 2 * t)
             : sinT(t);
  // CSI sources jump frame to frame: glide them so the drone swells instead of
  // twitching. The LFO gets a short glide that only rounds SQUARE and S&H edges.
  const ModProfile& mp = MOD_PROFILES[profile < NPROFILE ? profile : 2];
  float glide = n / (mp.modTau * SR), lfoGlide = fminf(n / (0.05f * SR), 1.f), pg = fminf(n / (PARAM_TAU * SR), 1.f);
  for (int s = 0; s < NSRC; s++)
    if (s == S_LFO || !freeze) modS[s] += (src[s] - modS[s]) * (s == S_LFO ? lfoGlide : glide);
  for (int i = 0; i < NPARAM; i++) eff[i] = baseS[i] += (P.p[i] - baseS[i]) * pg;
  // The matrix glides too: depths ramp, and a slot re-routed to another source or
  // param fades out on the old route before fading in on the new one.
  for (int k = 0; k < NSLOT; k++) {
    const Slot& s = P.mod[k];
    Slot& r = route[k];
    bool moved = s.src != r.src || s.dst != r.dst;
    routeD[k] += ((moved ? 0 : s.depth) - routeD[k]) * pg;
    if (moved && fabsf(routeD[k]) < 0.5f) r.src = s.src, r.dst = s.dst;
    if (r.src < NSRC && r.dst < NPARAM) eff[r.dst] += modS[r.src] * routeD[k] * 0.01f;
  }
  for (float& e : eff) e = clamp01(e);

  // Engine, voice-count or interval swap: fade out, switch, fade back in
  // (oscillators appearing, vanishing or jumping mid-wave would click).
  bool same = P.engine == engine && P.voices == curVoices && P.interval == curInterval;
  if (!same && fade < 0.002f) {
    engine = P.engine < NENGINE ? P.engine : 0;
    curVoices = P.voices;
    curInterval = P.interval;
  }
  float fadeTo = same ? 1.f : 0.f;
  // Slow fade (held q): ramp the output gain sample by sample.
  float g0 = outGain;
  outGain += ((duck ? 0.f : 1.f) - outGain) * fminf(n / (FADE_TAU * SR), 1.f);
  float gStep = (outGain - g0) / n;

  // An FX switched on starts clean instead of replaying a stale buffer.
  uint8_t fx = P.fx, on = fx & ~fxOn;
  fxOn = fx;
  if (on & FX_DELAY) { memset(dly, 0, sizeof dly); dlyS = dlyTarget(); }
  if (on & FX_CHORUS) memset(cho, 0, sizeof cho);
  if (on & FX_REVERB) { memset(rvb, 0, sizeof rvb); wetEnv = dryEnv = 0; }
  if (on & (FX_REVERB | FX_SHIMMER)) { memset(shm, 0, sizeof shm); shimOut = shimLp = 0; }

  // Pitch: quantised to the scale, then glided.
  float target = quantize((eff[P_PITCH] - 0.5f) * 24, SCALES[P.scale < NSCALE ? P.scale : 0].mask);
  float pitchGlide = fminf(n / (mp.pitchTau * SR), 1.f);
  semis += (target - semis) * pitchGlide;
  // Root and octave changes (note keys, [ ], 9 0) glide at the same pace: steering, not stepping.
  float base = 12 * (P.octave + 1) + P.root;
  baseN = baseN < 0 ? base : baseN + (base - baseN) * pitchGlide;
  float f0 = 440 * exp2f((baseN + semis - 69) / 12);
  freq = f0;
  float fv[3] = {f0, f0 * exp2f(intervalSemis(curInterval, SCALES[P.scale < NSCALE ? P.scale : 0].mask) / 12.f),
                 f0 * 0.5f};  // root, interval (a fifth by default), sub
  float tb = eff[P_TIMBRE], sp = eff[P_SPREAD];

  float inc[16], idt[16], amp[16], pw[6], idx[3], minc[3], fa1[3], fa2[3], fa3[3], fk[3];
  float norm = 1, wfr = 0, nzF = 0, dustP = 0, surfA = 0, surfG = 0;
  int nzA = 0, shMax = 2;
  const int16_t *wA = WT[0], *wB = WT[1];
  // Voices stack root, fifth (fv[1]) and sub (fv[2]); fewer voices get a lift so
  // the level stays put (ORGAN normalises itself).
  int tones = curVoices < 1 ? 1 : curVoices > 3 ? 3 : curVoices;
  float vg = tones == 1 ? 1.25f : tones == 2 ? 1.1f : 1.f;
  switch (engine) {
    case E_PWM: {
      static const float rate[6] = {0.11f, 0.17f, 0.07f, 0.13f, 0.05f, 0.09f};
      float du = exp2f(sp * 25 / 1200);
      for (int j = 0; j < 6; j++) {
        inc[j] = fv[j / 2] * (j & 1 ? du : 1 / du) / SR;
        idt[j] = 1 / inc[j];
        pwmPh[j] = wrap1(pwmPh[j] + n * rate[j] / SR);
        pw[j] = 0.5f + 0.42f * tb * sinT(pwmPh[j]);
      }
      break;
    }
    case E_SAW:
      for (int j = 0; j < 7; j++) {
        inc[j] = fv[tones > 1 ? j % 2 : 0] * exp2f((j - 3) / 3.f * sp * 40 / 1200) / SR;
        idt[j] = 1 / inc[j];
      }
      inc[7] = f0 * 0.5f / SR;
      break;
    case E_FM: {
      static const float ratio[3] = {1.f, 3.5f, 2.f}, depth[3] = {1.f, 0.6f, 0.8f};
      for (int v = 0; v < 3; v++) {
        inc[v] = fv[v] / SR;
        minc[v] = fv[v] * ratio[v] * (1 + sp * 0.006f * (v + 1)) / SR;
        idx[v] = tb * 5 * depth[v] / (2 * PI_F);
      }
      break;
    }
    case E_ORGAN: {  // organ: 16 partials shaped by the subcarrier profile, shared among the voices
      float sum = 0;
      for (int k = 0, h = 0, last = -1; k < 16; k++) {
        int s0 = k * NSC / 16, s1 = (k + 1) * NSC / 16, v = k * tones / 16;
        h = v == last ? h + 1 : 1;  // harmonic number within this voice
        last = v;
        float b = 0;
        for (int s = s0; s < s1; s++) b += prof[s];
        b /= s1 - s0;
        inc[k] = fv[v] * powf(h, 1 + sp * 0.12f) / SR;
        float t = inc[k] < 0.45f ? powf(h, -0.8f) * (1 - tb + tb * 1.6f * b) : 0;
        if (!freeze) partAmp[k] += (t - partAmp[k]) * glide;
        amp[k] = partAmp[k];
        sum += amp[k];
      }
      norm = 0.9f / fmaxf(sum, 0.5f);
      break;
    }
    case E_WAVE: {  // wavetable: TIMBRE scans the frames, SPREAD detunes the root pair
      float du = exp2f(sp * 30 / 1200), pos = tb * (WT_F - 1.001f);
      inc[0] = f0 * du / SR, inc[1] = f0 / du / SR, inc[2] = fv[1] / SR, inc[3] = fv[2] / SR;
      wA = WT[(int)pos], wB = WT[(int)pos + 1], wfr = pos - (int)pos;
      break;
    }
    case E_VOICE: {  // choir: four saws through three formant bandpasses, TIMBRE sings a-e-i-o-u
      float du = exp2f(sp * 30 / 1200), vp = tb * 3.999f, vf = vp - (int)vp;
      int vi = (int)vp;
      inc[0] = f0 * du / SR, inc[1] = f0 / du / SR, inc[2] = fv[1] / SR, inc[3] = fv[2] / SR;
      for (int j = 0; j < 4; j++) idt[j] = 1 / inc[j];
      for (int f = 0; f < 3; f++) {
        float hz = VOWEL[vi][f] + (VOWEL[vi + 1][f] - VOWEL[vi][f]) * vf, gk = tanf(PI_F * hz / SR);
        fk[f] = (80 + 30 * f) / hz;  // bandwidths 80/110/140 Hz
        fa1[f] = 1 / (1 + gk * (gk + fk[f])), fa2[f] = gk * fa1[f], fa3[f] = gk * fa2[f];
      }
      break;
    }
    case E_NOISE: {  // five kinds of noise, unpitched but for METAL: TIMBRE morphs between neighbours
      float pos = tb * (NNOISE - 1) * 0.9999f;
      nzA = (int)pos, nzF = pos - nzA;
      dustP = 0.002f + sp * sp * 0.03f;  // DUST: clicks per sample
      // WIND: two broad bands wandering 256 Hz - 4 kHz on their own (not on the drone's
      // harmonics, or it whistles), blowing in gusts. SPREAD widens them.
      float bw = 0.6f + sp * 1.2f;
      if (rnd() > 0.92f) gustT = 0.25f + 0.75f * fabsf(rnd());
      gust += (gustT - gust) * 0.04f;
      for (int v = 0; v < 2; v++) {
        windC[v] = clampf(windC[v] + rnd() * 0.06f, 8, 12);  // log2 Hz
        float hz = exp2f(windC[v]), g = tanf(PI_F * hz / SR);
        fa1[v] = 1 / (1 + g * (g + bw)), fa2[v] = g * fa1[v], fa3[v] = g * fa2[v];
        fk[v] = gust * bw * 0.37f / sqrtf(bw * hz / SR);  // unity peak, level-matched across widths
      }
      // SURF: a wave builds over ~30% of its 6-12 s cycle, then breaks; it opens a low-pass
      // on the hiss and swells it. SPREAD brightens the foam.
      surfPh += n * surfRate / SR;
      if (surfPh >= 1) surfPh -= 1, surfRate = 0.08f + 0.08f * fabsf(rnd());
      float e = surfPh < 0.3f ? surfPh / 0.3f : (1 - surfPh) / 0.7f;
      e *= e;
      surfA = 1 - expf(-2 * PI_F * (200 + (1500 + 4000 * sp) * e) / SR);
      surfG = (0.25f + 0.75f * e) * 1.1f / sqrtf(surfA);
      shMax = 2 + (int)(sp * 14);  // STATIC: longer holds, crunchier
      for (int v = 0; v < tones; v++) idt[v] = fv[v] * 93 / SR;  // METAL: a 93-step LFSR loop rings at the voice's pitch
      break;
    }
  }

  float fc = fminf(30 * exp2f(eff[P_CUTOFF] * 9.2f), SR * 0.42f);
  cut += (fc - cut) * 0.3f;
  float g = tanf(PI_F * cut / SR), k = 2 - 1.9f * eff[P_RESO];
  float a1 = 1 / (1 + g * (g + k)), a2 = g * a1, a3 = g * a2;
  float ng = eff[P_NOISE] * eff[P_NOISE] * 0.6f, level = eff[P_LEVEL];
  float dg = 1 + eff[P_DRIVE] * 8, dm = 1 - eff[P_DRIVE] * 0.45f;
  float dBias = 0.3f * eff[P_DRIVE], dOff = sat(dBias), dLp = 1 - expf(-2 * PI_F * (7000 - 4500 * eff[P_DRIVE]) / SR);
  // Unstable bit crusher (no sample-rate reduction): 14 bits down to 4, the depth
  // wandering about +-2 bits and now and then sputtering 3 bits lower, more so as CRUSH rises.
  float cr = eff[P_CRUSH];
  crushWob = crushWob * 0.97f + rnd() * 0.35f;
  float bits = 14 - cr * 10 + crushWob * cr * 1.5f - (rnd() > 1 - 0.06f * cr ? 3 : 0);
  float q = exp2f(fmaxf(bits, 2.f)), iq = 1 / q;
  float ringInc = 8 * exp2f(eff[P_RING] * 8) / SR;
  float cw = eff[P_CHORUS] * 0.8f, choNorm = 1 / (1 + 0.5f * cw);
  float dT = dlyTarget(), dfb = eff[P_DFB] * 0.92f;
  float rev = eff[P_REVERB], rfb = 0.78f + 0.195f * rev, shim = eff[P_SHIMMER] * 0.6f;
  // A longer decay piles up more energy: scale the input to keep the tail's level
  // steady (unchanged at the default REVERB 0.6), or high REVERB swamps the mix.
  float rin = 0.02f * sqrtf((1 - rfb * rfb) / (1 - 0.897f * 0.897f));
  // Shimmer feeds the reverb its own octave-up; on a held drone the resonances
  // can push that loop past unity. Once the reverb's share of the mix outgrows a
  // healthy level, the feedback backs off: it blooms instead of running into clipped fizz.
  float share = rev * wetEnv / (dryEnv + 1e-4f), over = share > SHIM_CEIL ? SHIM_CEIL / share : 1;
  float shimG = shim * over * over * over * over;  // steep: a hot loop settles just above the ceiling

  // Chorus sweep and tape wow/flutter are slow: evaluate at the block edges, ramp between.
  auto choD = [&](float ph, float base) { return SR * (base + 0.005f * cw * (1 + sinT(ph))); };
  auto wobble = [&] { return 60 * sinT(wowPh) + 4 * sinT(flutPh); };
  float c0 = choPh, wob = wobble();
  choPh = wrap1(choPh + n * 0.3f / SR);
  wowPh = wrap1(wowPh + n * 0.4f / SR);
  flutPh = wrap1(flutPh + n * 6.5f / SR);
  float d1 = choD(c0, 0.007f), d2 = choD(c0 + 0.25f, 0.009f);
  float d1s = (choD(choPh, 0.007f) - d1) / n, d2s = (choD(choPh + 0.25f, 0.009f) - d2) / n;
  float wobS = (wobble() - wob) / n;

  for (int i = 0; i < n; i++) {
    float x = 0;
    switch (engine) {
      case E_PWM:
        for (int j = 0; j < 2 * tones; j++) {  // a detuned pair per voice
          ph[j] = wrap1(ph[j] + inc[j]);
          x += (j < 2 ? 0.5f : j < 4 ? 0.3f : 0.4f) * pulseB(ph[j], inc[j], idt[j], pw[j]);
        }
        x *= 0.42f * vg;
        break;
      case E_SAW: {
        float side = 0.2f + 0.8f * tb;
        for (int j = 0; j < 7; j++) {
          ph[j] = wrap1(ph[j] + inc[j]);
          x += (j == 3 ? 1.f : side) * sawB(ph[j], inc[j], idt[j]);
        }
        ph[7] = wrap1(ph[7] + inc[7]);
        x = (x * 0.12f + (tones == 3 ? 0.3f * sinT(ph[7]) : 0)) * vg;
        break;
      }
      case E_FM:
        for (int v = 0; v < tones; v++) {
          mph[v] = wrap1(mph[v] + minc[v]);
          ph[v] = wrap1(ph[v] + inc[v]);
          x += (v == 1 ? 0.25f : 0.4f) * sinT(ph[v] + idx[v] * sinT(mph[v]));
        }
        x *= 0.55f * vg;  // the bells peak ~2x their average as the detuned modulators drift
        break;
      case E_ORGAN:
        for (int j = 0; j < 16; j++)
          if (amp[j] > 1e-4f) {
            ph[j] = wrap1(ph[j] + inc[j]);
            x += amp[j] * sinT(ph[j]);
          }
        x *= norm;
        break;
      case E_WAVE:
        for (int v = 0; v < tones + 1; v++) {  // root pair, fifth, sub
          ph[v] = wrap1(ph[v] + inc[v]);
          float pf = ph[v] * WT_N, fr = pf - (int)pf;
          int i0 = (int)pf, i1 = (i0 + 1) & (WT_N - 1);
          float a = wA[i0] + (wA[i1] - wA[i0]) * fr, b = wB[i0] + (wB[i1] - wB[i0]) * fr;
          x += (v < 2 ? 0.35f : v == 2 ? 0.2f : 0.3f) * (a + (b - a) * wfr);
        }
        x *= vg * (1 / 32000.f);  // constant reciprocal: a float divide is a function call here
        break;
      case E_VOICE: {
        float s = 0;
        for (int j = 0; j < tones + 1; j++) {  // root pair, fifth, sub
          ph[j] = wrap1(ph[j] + inc[j]);
          s += (j == 2 ? 0.6f : 1.f) * sawB(ph[j], inc[j], idt[j]);
        }
        s *= vg;
        for (int f = 0; f < 3; f++) {  // TPT bandpass; k * band = unity gain at the formant
          float v3 = s - fz2[f], v1 = fa1[f] * fz1[f] + fa2[f] * v3, v2 = fz2[f] + fa2[f] * fz1[f] + fa3[f] * v3;
          fz1[f] = 2 * v1 - fz1[f];
          fz2[f] = 2 * v2 - fz2[f];
          x += (f == 0 ? 1.f : f == 1 ? 0.5f : 0.35f) * fk[f] * v1;
        }
        break;
      }
      case E_NOISE: {
        float y[2] = {0, 0};
        for (int s = 0; s < (nzF > 0.001f ? 2 : 1); s++) {
          float o = 0;
          switch (nzA + s) {
            case N_DUST:  // sparse clicks a few samples long: vinyl, embers
              if (rnd() > 1 - 2 * dustP) dustV = rnd() * 2.5f;
              o = dustV * 3;
              dustV *= 0.55f;
              break;
            case N_WIND:  // broad bands of hiss, wandering and gusting
              for (int v = 0; v < 2; v++) {
                float v3 = rnd() - wz2[v], v1 = fa1[v] * wz1[v] + fa2[v] * v3, v2 = wz2[v] + fa2[v] * wz1[v] + fa3[v] * v3;
                wz1[v] = 2 * v1 - wz1[v];
                wz2[v] = 2 * v2 - wz2[v];
                o += fk[v] * v1;
              }
              break;
            case N_SURF:  // hiss under a 2-pole low-pass that opens as each wave breaks
              sz1 += (rnd() - sz1) * surfA;
              sz2 += (sz1 - sz2) * surfA;
              o = sz2 * surfG;
              break;
            case N_STATIC:  // shortwave: crushed noise in random holds, swelling and dropping out
              if (++shCnt >= shHold) shCnt = 0, shV = rnd(), shHold = 1 + (rng >> 24) % shMax;
              if ((rng & 2047) == 0) gateT = 0.1f + 0.9f * fabsf(rnd());
              gate += (gateT - gate) * 0.002f;
              o = shV * gate * 1.2f;
              break;
            default:  // N_METAL: NES-style short-mode noise, metallic and pitched
              for (int v = 0; v < tones; v++) {
                for (metPh[v] += idt[v]; metPh[v] >= 1; metPh[v] -= 1) {
                  unsigned fb = (lfsr[v] ^ (lfsr[v] >> 6)) & 1;
                  lfsr[v] = (uint16_t)((lfsr[v] >> 1) | (fb << 14));
                }
                o += lfsr[v] & 1 ? 0.17f * vg : -0.17f * vg;
              }
              break;
          }
          y[s] = o;
        }
        x = y[0] + (y[1] - y[0]) * nzF;
        break;
      }
    }
    fade += (fadeTo - fade) * 0.002f;
    x *= fade;
    if (ng > 0) x += rnd() * ng;

    // TPT state-variable lowpass
    float v3 = x - ic2, v1 = a1 * ic1 + a2 * v3, v2 = ic2 + a2 * ic1 + a3 * v3;
    ic1 = 2 * v1 - ic1;
    ic2 = 2 * v2 - ic2;
    x = v2;

    if (fx & FX_DRIVE) {  // analog-style: biased like a tube (even harmonics), darker the harder it's pushed
      float d = sat(x * dg + dBias) - dOff;
      drvLp += (d - drvLp) * dLp;           // circuit bandwidth: ~7 kHz clean, ~2.5 kHz flat out
      drvDc += (drvLp - drvDc) * 0.0015f;   // the bias leaves DC behind: block it (~10 Hz)
      x = (drvLp - drvDc) * dm;
    }
    if (fx & FX_CRUSH) x = (int)(x * q + (x < 0 ? -0.5f : 0.5f)) * iq;
    if (fx & FX_RING) {
      ringPh = wrap1(ringPh + ringInc);
      x *= 0.35f + 0.65f * sinT(ringPh);
    }
    if (fx & FX_CHORUS) {
      cho[choW] = x;
      x = (x + cw * 0.5f * (tap(cho, CHO_N, choW, d1) + tap(cho, CHO_N, choW, d2))) * choNorm;
      choW = (choW + 1) & (CHO_N - 1);
    }
    d1 += d1s;
    d2 += d2s;
    if (fx & FX_DELAY) {  // tape: slewed time (pitch warp), wow + flutter, darkening repeats
      dlyS += (dT - dlyS) * 0.0003f;
      float d = dlyS + wob;
      if (d < 4) d = 4;  // stay clear of the slot being filled
      float pos = dlyW + (dlyOdd ? 0.5f : 0.f) - d * 0.5f;  // in half-rate slots
      if (pos < 0) pos += DLY_N;
      int i0 = (int)pos, i1 = i0 + 1 == DLY_N ? 0 : i0 + 1;
      float r = (dly[i0] + (dly[i1] - dly[i0]) * (pos - i0)) * (1 / 16000.f);
      dlyLp += (r - dlyLp) * 0.35f;
      float w = sat(x + dlyLp * dfb);
      if (dlyOdd) {  // store the pair's average
        dly[dlyW] = (int16_t)((dlyAcc + w) * 8000);
        if (++dlyW == DLY_N) dlyW = 0;
      }
      dlyAcc = w;
      dlyOdd = !dlyOdd;
      x += 0.45f * r;
    }
    wob += wobS;
    if (fx & FX_REVERB) {  // mono Freeverb + octave-up shimmer fed back into it
      float in = (x + shimOut * shimG) * rin, acc = 0;
      dryEnv += (fabsf(x) - dryEnv) * 0.0005f;
      float* b = rvb;
      for (int c = 0; c < 6; c++) {
        float y = b[ci[c]];
        combLp[c] = y * 0.75f + combLp[c] * 0.25f;
        b[ci[c]] = in + combLp[c] * rfb;
        if (++ci[c] == COMB_LEN[c]) ci[c] = 0;
        acc += y;
        b += COMB_LEN[c];
      }
      for (int a = 0; a < 3; a++) {
        float y = b[ai[a]];
        b[ai[a]] = acc + y * 0.5f;
        acc = y - acc;
        if (++ai[a] == AP_LEN[a]) ai[a] = 0;
        b += AP_LEN[a];
      }
      acc *= 3;
      wetEnv += (fabsf(acc) - wetEnv) * 0.0005f;
      if ((fx & FX_SHIMMER) && shim > 0.001f) {
        shimLp += (acc - shimLp) * 0.5f;  // ~4 kHz: an octave up stays under Nyquist, no aliased fizz
        shimOut = sat(octaveUp(shimLp));
      } else {
        shimOut = 0;
      }
      x = x * (1 - 0.35f * rev) + acc * rev;
    }
    out[i] = (int16_t)(sat(x * level) * 30000 * g0);
    g0 += gStep;
  }
}
