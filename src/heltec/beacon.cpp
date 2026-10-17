// CARDRONE beacon for the Heltec WiFi LoRa 32 V3: the far end of LoRa mode.
// Sends a 5-byte GFSK packet 100 times a second on 869.85 MHz; the Cardputer measures
// how strongly each one arrives, and bodies moving through the flat change that.
// Each packet also carries a touch reading: touch GPIO2-7 (bare header pins work; a
// wire or foil on one senses a hand from further away) and TCH rises.
// EU868: 869.7-870 MHz allows 5 mW ERP with no duty-cycle limit; +5 dBm into the
// stock antenna stays under it. Flash: pio run -e heltec -t upload
#include <Arduino.h>
#include <RadioLib.h>

constexpr float FREQ = 869.85f;       // MHz: as LORA_FREQ in src/main.cpp
constexpr int8_t POWER = 5;           // dBm
constexpr float kTouchNoise = 0.01f;  // touch rise ignored as noise (fraction of the untouched reading)
static uint8_t SYNC[] = {0xCA, 0xD0, 0x4E};          // as LORA_SYNC in src/main.cpp
static const uint8_t TOUCH[] = {2, 3, 4, 5, 6, 7};  // free touch pins: 1 reads the battery, 8-14 drive the radio
constexpr int NT = sizeof TOUCH;

static SX1262 radio = new Module(8, 14, 12, 13);  // NSS, DIO1, RST, BUSY; SPI on the board's default pins
static float base[NT];                            // untouched reading per pin

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  int st = radio.beginFSK(FREQ, 50, 25, 156.2f, POWER, 16, 1.8f);  // 50 kbps GFSK; the V3's TCXO runs at 1.8 V
  if (st == RADIOLIB_ERR_NONE) st = radio.setDataShaping(RADIOLIB_SHAPING_0_5);  // keeps it ~100 kHz wide
  if (st == RADIOLIB_ERR_NONE) st = radio.setSyncWord(SYNC, sizeof SYNC);
  while (st != RADIOLIB_ERR_NONE) {  // fast blink: no radio
    Serial.printf("radio init failed: %d\n", st);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
  for (uint8_t p : TOUCH) touchRead(p);  // starts the touch sensor; the first readings come a little later
  delay(100);
  for (int i = 0; i < NT; i++) base[i] = fmaxf(touchRead(TOUCH[i]), 1);
}

void loop() {
  static TickType_t wake = xTaskGetTickCount();
  static uint8_t seq = 0;
  static uint32_t sent = 0, tLog = 0;
  vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));  // 100 packets a second
  float rise = 0;  // the most-touched pin, as a fraction above its untouched reading
  for (int i = 0; i < NT; i++) {
    float r = touchRead(TOUCH[i]);
    base[i] = fmaxf(base[i] + (r - base[i]) * (r < base[i] ? 0.1f : 0.0005f), 1);  // tracks drift; a held touch fades over ~20 s
    rise = fmaxf(rise, r / base[i] - 1 - kTouchNoise);
  }
  uint16_t t = (uint16_t)constrain(rise * 1000, 0.f, 65535.f);
  uint8_t pkt[5] = {'C', 'D', seq++, (uint8_t)t, (uint8_t)(t >> 8)};
  if (radio.transmit(pkt, sizeof pkt) == RADIOLIB_ERR_NONE) sent++;
  digitalWrite(LED_BUILTIN, seq & 64);  // slow blink: sending
  if (millis() - tLog >= 1000) {
    tLog = millis();
    Serial.printf("sent %lu  touch %.3f\n", (unsigned long)sent, rise);
  }
}
