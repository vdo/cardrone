// LoRa mode's radio link: the Heltec (src/heltec/beacon.cpp) and the Cardputer
// (src/main.cpp) both build from this, so they always agree.
#pragma once
#include <cstdint>

// LoRa SF6, 250 kHz, CR 4/5: 869.725-869.975 MHz, inside EU 869.7-870 MHz, which allows
// 5 mW ERP with no duty-cycle limit (+5 dBm into a 2-3 dBi antenna stays under). One
// channel, nothing hops. A beacon is ~11 ms on air: 25 a second is ~27% of the time.
// LoRa hears ~13 dB further than the 50 kbps GFSK this replaced (~-118 vs ~-105 dBm).
constexpr float LORA_FREQ = 869.85f, LORA_BW = 250;
constexpr uint8_t LORA_SF = 6, LORA_CR = 5, LORA_SYNC = 0x12;  // private-network sync word
constexpr int8_t LORA_POWER = 5;
constexpr uint16_t LORA_PREAMBLE = 12;  // symbols: SF5/SF6 want at least 12
constexpr uint32_t LORA_PERIOD_MS = 40;  // 25 beacons a second: plenty for drone modulation
// After begin, both radios set standbyXOSC and send SetRxTxFallbackMode(STDBY_XOSC): from
// RadioLib's default RC standby (also where the chip lands after each packet) the TCXO is
// off, and every switch to TX or RX waits ~5 ms for it, longer than the gap after a beacon.

// Heltec -> Cardputer, every LORA_PERIOD_MS: the Cardputer measures how strongly each arrives.
struct __attribute__((packed)) Beacon {
  char tag[2] = {'C', 'D'};
  uint8_t seq = 0;
  uint16_t touch = 0;  // the most-touched pin, in thousandths above its untouched reading
};

// Cardputer -> Heltec, a few a second, sent right after a beacon arrives: the Heltec
// listens in the gaps between its own and shows how well it's heard.
struct __attribute__((packed)) Status {
  char tag[2] = {'C', 'S'};
  uint8_t hz = 0;  // beacons read in the last second
  int8_t dbm = 0;  // their level
};

static_assert(sizeof(Beacon) == 5 && sizeof(Status) == 4, "packet layout changed: reflash both boards");
template <class T> static bool isPacket(const uint8_t* b, int len) { return len == sizeof(T) && b[0] == T().tag[0] && b[1] == T().tag[1]; }
