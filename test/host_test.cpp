// Host-side check for the pure DSP/CSI code. Optional arg: dir to write engine WAVs.
//   c++ -std=c++17 -O2 -Isrc test/host_test.cpp -o /tmp/fresneldrone_test && /tmp/fresneldrone_test /tmp
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
    csi.frame(b, 128, -55);
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
  assert(q.lfo == p.lfo && q.voices == p.voices && q.interval == p.interval);
  p.interval = I_THIRD;
  patchToJson(p, js, sizeof js);
  assert(patchFromJson(js, q) && q.interval == I_THIRD);
  assert(intervalSemis(I_THIRD, SCALES[0].mask) == 3 && intervalSemis(I_THIRD, SCALES[1].mask) == 4);  // minor, major
  p.fx = FX_REVERB;  // shimmer switched off survives a round trip...
  patchToJson(p, js, sizeof js);
  assert(patchFromJson(js, q) && q.fx == FX_REVERB);
  assert(patchFromJson("{\"engine\":1,\"root\":0,\"octave\":2,\"scale\":0,\"fx\":32,\"params\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"
                       "\"matrix\":[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]}", q) &&
         q.fx == (FX_REVERB | FX_SHIMMER));  // ...and a pre-switch preset with reverb keeps its shimmer
  assert(patchFromJson("{\"engine\":1,\"root\":0,\"octave\":2,\"scale\":0,\"fx\":0,\"params\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"
                       "\"matrix\":[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]}", q) && q.voices == 3);  // pre-voices preset
  assert(!patchFromJson("{\"engine\":1}", q));
  assert(patchFromJson("{\"v\":2,\"engine\":6,\"root\":0,\"octave\":2,\"scale\":0,\"fx\":0,\"params\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],"
                       "\"matrix\":[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]}", q) &&
         q.engine == E_NOISE);  // NOISE was 6 before FOLD took that place
  p.engine = E_FOLD;
  patchToJson(p, js, sizeof js);
  assert(patchFromJson(js, q) && q.engine == E_FOLD);
  const char* evil =
      "{\"engine\":99,\"root\":-5,\"octave\":7,\"scale\":-nan,\"fx\":999,\"params\":[9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,-x9],"
      "\"matrix\":[[99,99,500],[0,0,-500],[1,1,1],[1,1,1],[1,1,1],[1,1,1],[1,1,1],[1,1,1]]}";
  assert(patchFromJson(evil, q));
  assert(q.engine < NENGINE && q.root == 0 && q.octave == 5 && q.scale == 0 && q.fx == 127);
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
  csi.calibrate();  // a cancel racing frame()'s decrement must not leave it capturing forever
  csi.calLeft = -1;
  feed(0, 1.5f, 3, t);
  assert(csi.calLeft == -1 && !csi.calibrated);
  csi.calibrate();
  feed(0, 1.5f, kCalFrames, t);

  // LoRa: a still link stays low; quicker movement lifts MOT, FLX and HI within 2 s and
  // peaks the wobble spectrum at its rate; slow tides lift LO over HI; touch; calibrated LVL.
  {
    static Csi c;
    float tt = 0;
    auto link = [&](float dbm, float amp, float hz, float touch, int n) {
      for (int f = 0; f < n; f++, tt += 0.02f) {
        float v = dbm + amp * sinf(6.2831853f * hz * tt) + 0.3f * (rand() / (float)RAND_MAX - 0.5f);
        c.level(roundf(v * 2) / 2, touch);  // the radio reports 0.5 dB steps
      }
    };
    link(-62, 0, 0, 0, 1500);  // 30 s still
    float sMot = c.src[S_MOT], sFlx = c.src[S_FLX], sHi = c.src[S_HI];
    int react = -1;  // 8 s of gestures: 4 dB fades at 0.7 Hz; sources averaged over the last 4 s
    float mot = 0, flx = 0, hi = 0;
    for (int f = 0; f < 400; f++) {
      if (link(-62, 4, 0.7f, 0, 1), react < 0 && c.src[S_HI] > 0.5f) react = f;
      if (f >= 200) mot += c.src[S_MOT] / 200, flx += c.src[S_FLX] / 200, hi += c.src[S_HI] / 200;
    }
    int pk = 0;
    for (int k = 1; k < NSC; k++)
      if (c.benv[k] > c.benv[pk]) pk = k;
    printf("lora: still MOT %.2f FLX %.2f HI %.2f  moving MOT %.2f FLX %.2f HI %.2f  peak %.1f Hz\n", sMot, sFlx, sHi,
           mot, flx, hi, wobHz(pk));
    assert(sMot < 0.35f && sHi < 0.35f && mot > sMot + 0.3f && flx > sFlx + 0.3f && hi > 0.6f);
    assert(fabsf(wobHz(pk) - 0.7f) < 0.15f && c.prof[pk] > 0.8f);  // its rate lights up
    assert(react >= 0 && react * 20 < 2000);
    link(-62, 0, 0, 0, 1500);  // 30 s still, then 30 s of slow tides: 3 dB at 0.06 Hz
    link(-62, 3, 0.06f, 0, 1500);
    printf("lora: tides LO %.2f MID %.2f HI %.2f\n", c.src[S_LO], c.src[S_MID], c.src[S_HI]);
    assert(c.src[S_LO] > 0.5f && c.src[S_LO] > c.src[S_HI] + 0.3f);
    link(-62, 0, 0, 0.3f, 50);
    assert(c.src[S_PHS] > 0.9f);  // TCH
    link(-62, 0, 0, 0, 50);
    assert(c.src[S_PHS] < 0.1f);
    c.calibrate();
    link(-62, 0, 0, 0, kCalFrames);
    assert(c.calibrated && fabsf(c.base[0] + 62) < 0.5f && c.src[S_RSSI] < 0.1f);
    link(-68, 0, 0, 0, 250);  // someone stands in the path: 6 dB down
    assert(c.src[S_RSSI] > 0.8f);
    for (float v : c.src) assert(v >= -1 && v <= 1);
    for (float v : c.prof) assert(v >= 0 && v < 1.01f);
  }

  // A carrier-leak spike beside DC must not show up as a seam: -1/+1 land between their neighbours.
  {
    Csi c;
    int8_t b[128] = {};
    for (int i = 0; i < 64; i++) b[2 * i + 1] = 30;
    b[2 * 63 + 1] = 120, b[2 * 1 + 1] = 2;  // -1 hot, +1 dead
    c.frame(b, 128, -55);
    assert(fabsf(c.fast[DC_L] - c.fast[DC_L - 1]) < 1e-3f && fabsf(c.fast[DC_R] - c.fast[DC_R + 1]) < 1e-3f);
  }

  // Re-routing a slot glides: cutoff lets go gradually, then the reverb takes the slot over.
  {
    syn.pt = DEFAULT_PATCH;
    for (auto& s : syn.pt.mod) s.depth = 0;
    syn.pt.mod[0] = {S_MOT, P_CUTOFF, 40};
    float src1[NSRC] = {}, flat[NSC];
    src1[S_MOT] = 1;
    for (float& f : flat) f = 0.5f;
    for (int i = 0; i < 600; i++) syn.render(pcm, 512, src1, flat);
    float before = syn.eff[P_CUTOFF];
    syn.pt.mod[0].dst = P_REVERB;
    syn.render(pcm, 512, src1, flat);
    assert(before > 0.8f && syn.eff[P_CUTOFF] > before - 0.05f);  // no jump
    for (int i = 0; i < 300; i++) syn.render(pcm, 512, src1, flat);
    assert(fabsf(syn.eff[P_CUTOFF] - 0.45f) < 0.02f && syn.eff[P_REVERB] > 0.95f);  // fully moved
  }

  // Note keys set the root: the drone slides there instead of jumping.
  {
    syn.pt = DEFAULT_PATCH;  // A2 = 110 Hz
    float src0[NSRC] = {}, flat[NSC];
    for (float& f : flat) f = 0.5f;
    for (int i = 0; i < 300; i++) syn.render(pcm, 512, src0, flat);
    syn.pt.root = 4;  // E2 = 82.4 Hz
    syn.render(pcm, 512, src0, flat);
    assert(syn.freq > 100);  // one block in: still near A
    for (int i = 0; i < 400; i++) syn.render(pcm, 512, src0, flat);
    assert(fabsf(syn.freq - 82.41f) < 0.5f);
  }

  // Freeze holds the room's mods; the slow fade goes quiet and swells back.
  {
    syn.pt = DEFAULT_PATCH;
    float src1[NSRC] = {}, src0[NSRC] = {}, flat[NSC];
    src1[S_MOT] = 1;
    for (float& f : flat) f = 0.5f;
    for (int i = 0; i < 600; i++) syn.render(pcm, 512, src1, flat);
    float held = syn.eff[P_SHIMMER];  // MOT-driven only (the LFO, which freeze leaves running, steers CUTOFF)
    syn.freeze = true;
    for (int i = 0; i < 600; i++) syn.render(pcm, 512, src0, flat);  // the room goes still...
    assert(held > 0.4f && fabsf(syn.eff[P_SHIMMER] - held) < 0.01f);  // ...but the mods stay put
    syn.freeze = false;
    auto rms = [&](int blocks) {
      double sq = 0;
      for (int i = 0; i < blocks; i++) {
        syn.render(pcm, 512, src0, flat);
        for (int j = 0; j < 512; j++) sq += (double)pcm[j] * pcm[j];
      }
      return sqrt(sq / (blocks * 512)) / 32767;
    };
    syn.duck = true;
    rms(600);
    assert(rms(50) < 0.005);
    syn.duck = false;
    rms(600);
    assert(rms(50) > 0.05);
  }

  // Analog drive: a pushed pure tone grows a 2nd harmonic (tube-like bias), not just odd ones.
  {
    syn.pt = DEFAULT_PATCH;
    syn.pt.engine = E_FM, syn.pt.voices = 1, syn.pt.fx = FX_DRIVE;  // FM at TIMBRE 0: a sine at A2 = 110 Hz
    syn.pt.p[P_TIMBRE] = 0, syn.pt.p[P_CUTOFF] = 1, syn.pt.p[P_DRIVE] = 0.8f, syn.pt.p[P_LEVEL] = 0.5f;
    for (auto& s : syn.pt.mod) s.depth = 0;
    float src0[NSRC] = {}, flat[NSC];
    for (float& f : flat) f = 0.5f;
    for (int i = 0; i + 512 <= SR * 4; i += 512) syn.render(pcm + i, 512, src0, flat);
    auto mag = [&](double hz) {  // DFT magnitude over the last second (whole cycles of 110 Hz)
      double re = 0, im = 0;
      for (int i = 0; i < SR; i++) re += pcm[SR * 3 + i] * cos(2 * M_PI * hz * i / SR), im += pcm[SR * 3 + i] * sin(2 * M_PI * hz * i / SR);
      return sqrt(re * re + im * im);
    };
    double h2 = 20 * log10(mag(220) / mag(110));
    printf("drive 0.8: 2nd harmonic %.1f dB\n", h2);
    assert(h2 > -30);
  }

  // FOLD: a sine at SPREAD 0; SPREAD folds in odd harmonics, TIMBRE skews the folds into even ones.
  {
    for (float u = -9; u < 9; u += 0.37f)  // ADAA needs foldF to be foldT's antiderivative
      assert(fabsf((foldF(u + 0.01f) - foldF(u - 0.01f)) / 0.02f - foldT(u)) < 0.02f);
    float src0[NSRC] = {}, flat[NSC];
    for (float& f : flat) f = 0.5f;
    auto tone = [&](float spread, float timbre, double& odd, double& even) {  // harmonics 2-16 vs the fundamental, dB
      syn.pt = DEFAULT_PATCH;
      syn.pt.engine = E_FOLD, syn.pt.voices = 1, syn.pt.fx = 0;
      syn.pt.p[P_SPREAD] = spread, syn.pt.p[P_TIMBRE] = timbre, syn.pt.p[P_CUTOFF] = 1, syn.pt.p[P_NOISE] = 0;
      for (auto& s : syn.pt.mod) s.depth = 0;
      for (int i = 0; i + 512 <= SR * 4; i += 512) syn.render(pcm + i, 512, src0, flat);
      auto mag = [&](double hz) {
        double re = 0, im = 0;
        for (int i = 0; i < SR; i++) re += pcm[SR * 3 + i] * cos(2 * M_PI * hz * i / SR), im += pcm[SR * 3 + i] * sin(2 * M_PI * hz * i / SR);
        return re * re + im * im;
      };
      double e[2] = {1e-12, 1e-12}, f = mag(syn.freq);
      for (int h = 2; h <= 16; h++) e[h % 2] += mag(syn.freq * h);
      even = 10 * log10(e[0] / f), odd = 10 * log10(e[1] / f);
    };
    double o0, e0, o1, e1, o2, e2;
    tone(0, 0, o0, e0), tone(1, 0, o1, e1), tone(1, 1, o2, e2);
    printf("fold: clean odd %.0f even %.0f dB  folded odd %.0f even %.0f  skewed odd %.0f even %.0f\n", o0, e0, o1, e1, o2, e2);
    assert(o0 < -40 && e0 < -40);  // a sine
    assert(o1 > -10 && e1 < -30);  // folded: odd harmonics (even ones measure ~-38 dB)
    assert(e2 > -20);              // skewed: even ones too
  }

  // REVERB and SHIMMER maxed on a held drone: the shimmer loop must bloom, not run away and pin.
  {
    syn.pt = DEFAULT_PATCH;
    syn.pt.p[P_REVERB] = syn.pt.p[P_SHIMMER] = 1;
    for (auto& s : syn.pt.mod) s.depth = 0;
    float src0[NSRC] = {}, flat[NSC];
    for (float& v : flat) v = 0.5f;
    long pinned = 0;
    for (int i = 0; i < SR * 20; i += 512) {
      syn.render(pcm, 512, src0, flat);
      if (i > SR * 10)
        for (int j = 0; j < 512; j++) pinned += abs(pcm[j]) >= 29900;
    }
    printf("reverb+shimmer maxed: %.2f%% of samples at the clipper\n", 100.0 * pinned / (SR * 10));
    assert(pinned < SR * 10 / 100);
  }

  // Every engine at 1, 2 and 3 voices: audible, not crushed into the clipper (4 s average:
  // FM's level swings slowly as its detuned modulators drift).
  for (int e = 0; e < NENGINE; e++) {
    printf("%-6s voices", ENGINE_NAME[e]);
    for (int v = 1; v <= 3; v++) {
      syn.pt = DEFAULT_PATCH;
      syn.pt.engine = e, syn.pt.voices = v;
      float src0[NSRC] = {}, flat[NSC];
      for (float& f : flat) f = 0.5f;
      double sq = 0;
      long cnt = 0;
      for (int i = 0; i < SR * 6; i += 512) {
        syn.render(pcm, 512, src0, flat);
        if (i >= SR * 2) for (int j = 0; j < 512; j++, cnt++) sq += (double)pcm[j] * pcm[j];
      }
      float rms = sqrtf(sq / cnt) / 32767;
      printf("  %d: rms %.3f", v, rms);
      assert(rms > 0.03f && rms < 0.5f);
    }
    puts("");
  }

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
      snprintf(path, sizeof path, "%s/fresneldrone_%s.wav", argv[1], ENGINE_NAME[e]);
      writeWav(path, pcm, n);
    }
  }
  puts("ok");
}
