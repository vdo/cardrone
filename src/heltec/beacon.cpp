// CARDRONE beacon for the Heltec WiFi LoRa 32 V3: the far end of LoRa mode.
// Sends a Beacon 100 times a second on 869.85 MHz (src/link.h); the Cardputer measures
// how strongly each one arrives, and bodies moving through the flat change that.
// Each beacon also carries a touch reading: touch GPIO2-7 (bare header pins work; a
// wire or foil on one senses a hand from further away) and TCH rises. Between beacons
// it listens for the Cardputer's Status, and the OLED shows both ends of the link.
// Flash: pio run -e heltec -t upload
#include <Arduino.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include "dsp.h"  // engine, note and scale names
#include "link.h"

constexpr float kTouchNoise = 0.01f;  // touch rise ignored as noise (fraction of the untouched reading)
static const uint8_t TOUCH[] = {2, 3, 4, 5, 6, 7};  // free touch pins: 1 reads the battery, 8-14 drive the radio
constexpr int NT = sizeof TOUCH;

static SX1262 radio = new Module(8, 14, 12, 13);  // NSS, DIO1, RST, BUSY; SPI on the board's default pins
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, RST_OLED, SCL_OLED, SDA_OLED);
static volatile bool rxDone = false;
static void IRAM_ATTR onRx() { rxDone = true; }
static float base[NT];  // untouched reading per pin

// loop() writes, the screen task reads: a torn value only shows for 200 ms.
static struct {
  float rise[NT];               // per touch pin, as a fraction above its untouched reading
  uint32_t sent = 0, txUs = 0;  // beacons sent; how long the last took
  Status st;                    // the Cardputer's last word
  float backDbm = 0;            // its level here
  uint32_t heardAt = 0;         // millis(); 0 = never
} v;

static float vbat() {  // GPIO37 low connects the battery's divider to GPIO1 (as Meshtastic's heltec_v3)
  pinMode(37, OUTPUT);
  digitalWrite(37, LOW);
  delay(5);
  float mv = analogReadMilliVolts(1) * 4.9f * 1.045f;
  pinMode(37, INPUT);  // pulled up: divider off
  return mv / 1000;
}

static void screenTask(void*) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);  // powers the OLED
  delay(50);
  oled.begin();
  oled.setFont(u8g2_font_4x6_tr);
  oled.setFontPosTop();
  analogSetPinAttenuation(1, ADC_2_5db);
  uint32_t lastSent = 0, rate = 0, tRate = 0, tBat = 0;
  float bat = vbat(), peak = 0;
  char s[48];
  for (;; vTaskDelay(pdMS_TO_TICKS(200))) {
    uint32_t now = millis(), up = now / 1000;
    if (now - tRate >= 1000) rate = v.sent - lastSent, lastSent = v.sent, tRate = now;
    if (now - tBat >= 5000) bat = vbat(), tBat = now;
    Status st = v.st;
    bool heard = v.heardAt && now - v.heardAt < 3000;
    oled.clearBuffer();

    oled.drawBox(0, 0, 128, 7);  // title bar
    oled.setDrawColor(0);
    snprintf(s, sizeof s, "CARDRONE LORA %.2fMHz +%ddBm", LORA_FREQ, LORA_POWER);
    oled.drawStr(1, 1, s);
    oled.setDrawColor(1);
    snprintf(s, sizeof s, "TX %lu/s  air %lu%%  #%lu", rate, rate * v.txUs / 10000, v.sent);
    oled.drawStr(0, 9, s);

    if (heard) snprintf(s, sizeof s, "HEARD %u/s %ddBm  BACK %.0f", st.hz, st.dbm, v.backDbm);
    else snprintf(s, sizeof s, v.heardAt ? "CARDPUTER LOST %lus AGO" : "WAITING: Cardputer on LORA?",
                  (unsigned long)(now - v.heardAt) / 1000);
    oled.drawStr(0, 16, s);
    if (heard) {  // what the drone is doing
      snprintf(s, sizeof s, "%s %s%d %s%s %s%s%s%s", ENGINE_NAME[st.engine % NENGINE], NOTE_NAME[st.root % 12], st.octave,
               SCALES[st.scale % NSCALE].name, st.flags & ST_DRIFT ? "~" : "", st.flags & ST_PLAYING ? "PLAY" : "STOP",
               st.flags & ST_AUTO ? " AUTO" : "", st.flags & ST_FREEZE ? " FRZ" : "", st.flags & ST_CALIBRATED ? " CAL" : "");
      oled.drawStr(0, 23, s);
    }

    for (int i = 0; i < NSRC; i++) {  // the Cardputer's mod sources
      int x = i * 16 + 2, h = heard ? st.src[i] * 10 / 255 : 0;
      oled.drawFrame(x, 30, 12, 12);
      oled.drawBox(x + 1, 41 - h, 10, h);
      oled.drawStr(x + (12 - oled.getStrWidth(SRC_NAME_LORA[i])) / 2, 43, SRC_NAME_LORA[i]);
    }

    float mx = 0;  // touch pins, scaled to the strongest recent touch
    for (float r : v.rise) mx = fmaxf(mx, r);
    peak = fmaxf(mx, peak * 0.98f);
    oled.drawStr(0, 50, "T");
    for (int i = 0; i < NT; i++) {
      int x = 6 + i * 20;
      snprintf(s, sizeof s, "%d", TOUCH[i]);
      oled.drawStr(x, 50, s);
      oled.drawFrame(x + 5, 50, 13, 6);
      oled.drawBox(x + 6, 51, (int)(clamp01(v.rise[i] / fmaxf(peak, kTouchMin)) * 11), 4);
    }

    char b[8] = "--";
    if (bat > 2.5f) snprintf(b, sizeof b, "%.2fV", bat);  // lower: no battery, or this board's divider differs
    snprintf(s, sizeof s, "BAT %s  %.0fC  UP %lu:%02lu:%02lu", b, temperatureRead(), up / 3600, up / 60 % 60, up % 60);
    oled.drawStr(0, 57, s);
    oled.sendBuffer();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  int st = radio.beginFSK(LORA_FREQ, LORA_BR, LORA_DEV, LORA_RXBW, LORA_POWER, 16, 1.8f);  // the V3's TCXO runs at 1.8 V
  if (st == RADIOLIB_ERR_NONE) st = radio.setDataShaping(RADIOLIB_SHAPING_0_5);
  if (st == RADIOLIB_ERR_NONE) st = radio.setSyncWord(LORA_SYNC, sizeof LORA_SYNC);
  while (st != RADIOLIB_ERR_NONE) {  // fast blink: no radio
    Serial.printf("radio init failed: %d\n", st);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
  radio.setPacketReceivedAction(onRx);
  for (uint8_t p : TOUCH) touchRead(p);  // starts the touch sensor; the first readings come a little later
  delay(100);
  for (int i = 0; i < NT; i++) base[i] = fmaxf(touchRead(TOUCH[i]), 1);
  xTaskCreatePinnedToCore(screenTask, "screen", 4096, nullptr, 1, nullptr, 0);  // loop() has core 1
}

void loop() {
  static TickType_t wake = xTaskGetTickCount();
  static Beacon bc;
  static uint32_t tLog = 0;
  vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));  // 100 beacons a second
  if (rxDone) {  // the Cardputer answered in the gap after the last beacon
    rxDone = false;
    uint8_t b[sizeof(Status)];
    size_t len = radio.getPacketLength();
    float dbm = radio.getRSSI(true);
    if (radio.readData(b, sizeof b) == RADIOLIB_ERR_NONE && isPacket<Status>(b, len))
      memcpy(&v.st, b, sizeof v.st), v.backDbm = dbm, v.heardAt = millis();
  }
  float rise = 0;  // the most-touched pin
  for (int i = 0; i < NT; i++) {
    float r = touchRead(TOUCH[i]);
    base[i] = fmaxf(base[i] + (r - base[i]) * (r < base[i] ? 0.1f : 0.0005f), 1);  // tracks drift; a held touch fades over ~20 s
    v.rise[i] = fmaxf(r / base[i] - 1 - kTouchNoise, 0);
    rise = fmaxf(rise, v.rise[i]);
  }
  bc.touch = (uint16_t)constrain(rise * 1000, 0.f, 65535.f);
  uint32_t t0 = micros();
  if (radio.transmit((uint8_t*)&bc, sizeof bc) == RADIOLIB_ERR_NONE) v.sent++;
  v.txUs = micros() - t0;
  bc.seq++;
  rxDone = false;  // the transmit's own interrupt
  radio.startReceive();
  digitalWrite(LED_BUILTIN, bc.seq & 64);  // slow blink: sending
  if (millis() - tLog >= 1000) {
    tLog = millis();
    Serial.printf("sent %lu  touch %.3f  cardputer %s\n", (unsigned long)v.sent, rise,
                  v.heardAt && millis() - v.heardAt < 3000 ? "heard" : "not heard");
  }
}
