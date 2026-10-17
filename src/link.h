// LoRa mode's radio link: the Heltec (src/heltec/beacon.cpp) and the Cardputer
// (src/main.cpp) both build from this, so they always agree.
#pragma once
#include <cstdint>

// 50 kbps GFSK, 25 kHz deviation, BT 0.5 shaping: ~100 kHz wide, inside EU 869.7-870 MHz,
// which allows 5 mW ERP with no duty-cycle limit. +5 dBm into a 2-3 dBi antenna stays under.
constexpr float LORA_FREQ = 869.85f, LORA_BR = 50, LORA_DEV = 25, LORA_RXBW = 156.2f;
constexpr int8_t LORA_POWER = 5;
static uint8_t LORA_SYNC[] = {0xCA, 0xD0, 0x4E};  // only CARDRONE packets count

// Heltec -> Cardputer, 100 a second: the Cardputer measures how strongly each arrives.
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
