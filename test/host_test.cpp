// Host-side check for the pure DSP/CSI code. Optional arg: dir to write engine WAVs.
//   c++ -std=c++17 -O2 -Isrc test/host_test.cpp -o /tmp/cardrone_test && /tmp/cardrone_test /tmp
#include <cassert>
#include <cstdio>
#include <initializer_list>
#include "dsp.h"

static Synth syn;  // ~110 KB of buffers: keep off the stack
static Csi csi;
static int16_t pcm[SR * 4 + 512];  // last 512-block overhangs

static void writeWav(const char* path, const int16_t* s, int n) {
  FILE* f = fopen(path, "wb");
  if (!f) return;
  auto u32 = [&](uint32_t v) { fwrite(&v, 4, 1, f); };
  auto u16 = [&](uint16_t v) { fwrite(&v, 2, 1, f); };
  fwrite("RIFF", 1, 4, f); u32(36 + n * 2); fwrite("WAVEfmt ", 1, 8, f);
  u32(16); u16(1); u16(1); u32(SR); u32(SR * 2); u16(2); u16(16);
  fwrite("data", 1, 4, f); u32(n * 2); fwrite(s, 2, n, f);
  fclose(f);
}

static void feed(float wobble, float noise, int n, float& t) {
  int8_t b[128] = {};
  for (int f = 0; f < n; f++, t += 0.05f) {
    for (int i = 0; i < 64; i++) {
      float amp = 30 + 12 * sinf(i * 0.4f) + wobble * sinf(i * 0.9f + t * 3) + noise * (rand() / (float)RAND_MAX - 0.5f);
      b[2 * i] = (int8_t)(amp * 0.6f);
      b[2 * i + 1] = (int8_t)(amp * 0.8f);
    }
    csi.frame(b, 128, -55, true);
  }
}

int main(int argc, char** argv) {
  // Scale quantiser: minor = 0 2 3 5 7 8 10
  assert(quantize(0.4f, 0x5AD) == 0);
  assert(quantize(1.4f, 0x5AD) == 2);
  assert(quantize(-1.2f, 0x5AD) == -2);
  assert(quantize(5.5f, 0x081) == 7);

  // Table sine with the hand-rolled floor: negatives, wraps and the 1.0 edge (SINT filled by syn).
  for (float ph : {-2.3f, -1.f, -1e-9f, 0.f, 0.25f, 0.9999999f, 1.f, 1.75f, 3.1f})
    assert(fabsf(sinT(ph) - sinf(2 * PI_F * ph)) < 1e-3f);
  assert(sat(9) == 1 && sat(-9) == -1 && fabsf(sat(0.1f) - 0.1f) < 1e-3f);

  // Patch JSON round trip, then hostile input gets clamped, truncated input rejected.
  char js[1024];
  Patch p = DEFAULT_PATCH, q;
  p.p[P_TIMBRE] = 0.123f;
  p.mod[2].depth = -40;
  patchToJson(p, js, sizeof js);
  assert(patchFromJson(js, q));
  assert(q.engine == p.engine && q.fx == p.fx && q.mod[2].depth == -40 && q.mod[3].dst == P_REVERB);
  assert(fabsf(q.p[P_TIMBRE] - 0.123f) < 1e-3f);
  assert(q.lfo == p.lfo);
  assert(!patchFromJson("{\"engine\":1}", q));
  const char* evil =
      "{\"engine\":99,\"root\":-5,\"octave\":7,\"scale\":-nan,\"fx\":999,\"params\":[9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,-x9],"
      "\"matrix\":[[99,99,500],[0,0,-500],[1,1,1],[1,1,1],[1,1,1],[1,1,1],[1,1,1],[1,1,1]]}";
  assert(patchFromJson(evil, q));
  assert(q.engine < NENGINE && q.root == 0 && q.octave == 5 && q.scale == 0 && q.fx == 63);
  assert(q.p[0] == 1 && q.p[15] == 1);  // "-x9" skips the '-' and reads 9
  assert(q.mod[0].src < NSRC && q.mod[0].dst < NPARAM && q.mod[0].depth == 100 && q.mod[1].depth == -100);

  // CSI: a still room settles low, changing multipath reads as motion.
  float t = 0;
  srand(1);
  for (int i = 0; i < 40; i++) feed(0, 1.5f, 10, t);
  float still = csi.src[S_MOT];
  for (int i = 0; i < 10; i++) feed(10, 1.5f, 10, t);
  float moving = csi.src[S_MOT];
  printf("csi: still MOT %.2f  moving MOT %.2f  FLX %.2f  PHS %.2f\n", still, moving, csi.src[S_FLX], csi.src[S_PHS]);
  assert(still < 0.4f && moving > still + 0.3f);
  for (float v : csi.src) assert(v >= -1 && v <= 1);
  csi.calibrate();
  for (int i = 0; i < kCalFrames; i++) feed(0, 1.5f, 1, t);
  assert(csi.calibrated && !csi.calLeft);

  // Every engine with every FX and swinging sources: finite, audible, not pinned.
  float src[NSRC] = {}, prof[NSC];
  for (int e = 0; e < NENGINE; e++) {
    syn.pt = DEFAULT_PATCH;
    syn.pt.engine = e;
    syn.pt.fx = 63;
    syn.pt.p[P_CRUSH] = 0.15f;
    double sq = 0;
    int peak = 0, n = SR * 4;
    for (int i = 0; i < n; i += 512) {
      float ts = i / (float)SR;
      for (int s = 0; s < S_LFO; s++) src[s] = 0.5f + 0.5f * sinf(ts * (0.7f + s * 0.3f));
      for (int k = 0; k < NSC; k++) prof[k] = 0.5f + 0.5f * sinf(k * 0.3f + ts);
      syn.render(pcm + i, 512, src, prof);
    }
    for (int i = SR; i < n; i++) {  // skip the engine fade-in
      sq += (double)pcm[i] * pcm[i];
      peak = abs(pcm[i]) > peak ? abs(pcm[i]) : peak;
    }
    float rms = sqrtf(sq / (n - SR)) / 32767;
    printf("%-7s rms %.3f  peak %.3f\n", ENGINE_NAME[e], rms, peak / 32767.f);
    assert(rms > 0.05f && rms < 0.7f);
    if (argc > 1) {
      char path[256];
      snprintf(path, sizeof path, "%s/cardrone_%s.wav", argv[1], ENGINE_NAME[e]);
      writeWav(path, pcm, n);
    }
  }
  puts("ok");
}
