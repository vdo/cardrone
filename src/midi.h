// BLE MIDI: packet parser and CC map. Pure C++ so test/host_test.cpp can check it.
#pragma once
#include <cstdint>
#include "dsp.h"

// A BLE MIDI packet is a header byte, then messages each led by a timestamp byte:
// [hdr] [ts] [status] [data..] [ts] [status] [data..]. Running status may drop the
// status byte (a timestamp may still lead the data); SysEx may span packets and is
// skipped. Calls cb(status, d1, d2) per channel message.
struct BleMidi {
  uint8_t run = 0, d1 = 0;
  bool half = false, sysex = false;  // half: first of two data bytes in hand
  template <class F> void packet(const uint8_t* p, int len, F&& cb) {
    if (len < 2 || !(p[0] & 0x80)) return;
    bool ts = false;  // the last byte was a timestamp: a high byte now is a status
    for (int i = 1; i < len; i++) {
      uint8_t b = p[i];
      if (b & 0x80) {
        if (!ts) { ts = true; continue; }
        ts = false;
        if (b >= 0xF8) continue;  // real-time: leaves running status (and SysEx) alone
        sysex = b == 0xF0;
        run = b < 0xF0 ? b : 0;
        half = false;
        continue;
      }
      ts = false;
      if (sysex || !run) continue;
      bool one = (run & 0xE0) == 0xC0;  // program change, channel pressure: one data byte
      if (one || half) cb(run, one ? b : d1, one ? 0 : b), half = false;
      else d1 = b, half = true;
    }
  }
};

// CC -> param: 20-35 follow the param list; the General MIDI numbers work too.
static inline int ccParam(int cc) {
  if (cc >= 20 && cc < 20 + NPARAM) return cc - 20;
  switch (cc) {
    case 1: return P_TIMBRE;   // mod wheel
    case 7: return P_LEVEL;    // volume
    case 71: return P_RESO;    // resonance
    case 74: return P_CUTOFF;  // brightness
    case 91: return P_REVERB;
    case 93: return P_CHORUS;
    default: return -1;
  }
}
constexpr int CC_RELEASE = 121;  // reset all controllers: every param back to the room
