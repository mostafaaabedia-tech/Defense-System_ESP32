/*
  ============================================================
   LASER TRIPWIRE BASE ALARM + RADAR  —  ESP32
  ============================================================
   4 laser beams shine on 4 LDRs. If any beam is broken:
     - buzzer siren starts and STAYS ON (latched, like a real panel)
     - status screen shows which zone was breached
     - a notification is sent to your website / phone (ntfy.sh)
   Press the RESET switch to silence and re-arm.
   HOLD the RESET switch 3 seconds to re-calibrate lasers + radar.

   RADAR: a servo sweeps an HC-SR04 from 0 to 180 degrees.
   It first learns what is normally there (walls, buildings),
   then reports anything NEW that appears = "radar contact":
     - 2 short chirps, contact shown on the map screen + website
     - the siren is NOT latched for radar (it is an early warning)

   SCREENS (1.5" 128x128 OLED, SH1107 by default)
     The code FINDS your screens by itself at power on:
       - one screen  (on pins 21/22 OR 16/17): it switches between
         the status page and the map + radar page
       - two screens (one on 21/22, one on 16/17):
         21/22 = status page, 16/17 = map + radar page
     Open Serial Monitor (115200) to see which screens were found.

   Libraries (Arduino Library Manager):  U8g2  (by olikraus)
   Board: "ESP32 Dev Module"   (esp32 core 2.x or 3.x both work)
  ============================================================ */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>

// ================== YOUR SETTINGS ==================
const char* WIFI_SSID  = "YOUR_WIFI_NAME";
const char* WIFI_PASS  = "YOUR_WIFI_PASSWORD";

// Pick a hard-to-guess name. Put the SAME name in the website settings.
// Only letters, numbers, - and _   (max 57 characters)
const char* NTFY_TOPIC = "base-alarm-CHANGE-ME-4821";
const char* BASE_NAME  = "BASE ALPHA";

// ---- SCREEN ----
// Which chip is your screen? (written on the shop page or the back of the board)
//   1 = SH1107  I2C, 4 pins (GND VCC SCL SDA)  <- 1.5" mono 128x128
//   2 = SSD1327 I2C, 4 pins (1.5" grayscale)
//   3 = SH1107  SPI, 7 pins (GND VCC DIN CLK CS DC RST)
//   4 = SSD1327 SPI, 7 pins
// A WRONG number here gives a screen full of random dots (looks like a QR code).
#define SCREEN_TYPE 1
// SH1107 only: if the picture is shifted sideways or wraps around the edge, try 2, then 3
//   1 = GME128128-01-IIC and most 1.5" SH1107 boards (no offset)
//   2 = boards that need a 96-column offset
//   3 = Seeed Grove 1.12" style
#define SH1107_VARIANT 1
// Screen I2C speed. If you see a few random dots or a flickering picture
// (long wires, breadboard), change to 100000
#define SCREEN_I2C_HZ 400000

// Buzzer: 0 = ACTIVE buzzer (beeps by itself on 3.3V), 1 = PASSIVE buzzer (needs a tone)
#define BUZZER_PASSIVE 0

// Radar: 1 = servo + HC-SR04 connected, 0 = no radar
#define RADAR_ENABLED 1
const int RADAR_RANGE_CM  = 50;   // radar ignores anything farther than this
const int RADAR_MARGIN_CM = 6;    // object must be this much closer than the learned background
const int SERVO_MIN_US    = 500;  // servo pulse at 0 degrees   (SG90: 500)
const int SERVO_MAX_US    = 2400; // servo pulse at 180 degrees (SG90: 2400)
// ===================================================

// ---------- Pins ----------
// LDRs MUST be on ADC1 pins (ADC2 stops working when WiFi is on)
const int LDR_PINS[4] = {32, 33, 34, 35};
const int BUZZER_PIN  = 25;
const int RESET_PIN   = 27;   // switch to GND (internal pull-up)
const int LED_PIN     = 2;    // on-board LED, blinks in alarm
const int SERVO_PIN   = 13;   // servo signal (orange/yellow wire)
const int TRIG_PIN    = 26;   // HC-SR04 Trig
const int ECHO_PIN    = 14;   // HC-SR04 Echo  (through a 1k / 2k voltage divider! Echo is 5 V)
const int SCR1_SDA    = 21;   // screen on the first I2C pins
const int SCR1_SCL    = 22;
const int SCR2_SDA    = 16;   // second screen (map + radar), always SH1107
const int SCR2_SCL    = 17;

// SPI screen (type 3 or 4): DIN/MOSI 23, CLK 18, CS 5, DC 4, RST 15

// ---------- Tuning ----------
const float TRIP_RATIO     = 0.55;  // beam is "broken" if light drops below 55% of calibrated level
const int   MIN_LIGHT      = 700;   // below this at calibration = laser not hitting LDR (zone bypassed)
const int   TRIP_MS        = 40;    // beam must stay broken this long (ignores flicker / insects)
const int   ARM_DELAY_S    = 5;     // countdown before arming
const unsigned long HEARTBEAT_MS  = 15UL * 60UL * 1000UL; // status report every 15 min (website uses it to know the base is online)
const unsigned long LONG_PRESS_MS = 3000;

// radar timing
const int R_STEP = 3;                          // degrees per step
const int R_BINS = 180 / R_STEP + 1;           // 61 directions
const unsigned long RADAR_STEP_MS     = 45;    // time per step (servo move + echo)
const unsigned long RADAR_HOLD_MS     = 6500;  // contact counts as "gone" after this long unseen (> one full sweep there and back)
const unsigned long RADAR_NOTIFY_GAP  = 60UL * 1000UL;   // at most one radar alert per minute
const unsigned long RADAR_UPDATE_MS   = 3UL * 60UL * 1000UL; // "still there" update every 3 min
const int           RADAR_DAILY_MAX   = 60;    // radar messages per 24 h (free ntfy.sh has a daily limit)

// ---------- Screens ----------
#define SCREEN1_IS_I2C (SCREEN_TYPE == 1 || SCREEN_TYPE == 2)
#if SCREEN_TYPE == 1
  #if SH1107_VARIANT == 2
  U8G2_SH1107_128X128_F_HW_I2C scr1(U8G2_R0, U8X8_PIN_NONE);
  #elif SH1107_VARIANT == 3
  U8G2_SH1107_SEEED_128X128_F_HW_I2C scr1(U8G2_R0, U8X8_PIN_NONE);
  #else
  U8G2_SH1107_PIMORONI_128X128_F_HW_I2C scr1(U8G2_R0, U8X8_PIN_NONE);
  #endif
#elif SCREEN_TYPE == 2
U8G2_SSD1327_WS_128X128_F_HW_I2C scr1(U8G2_R0, U8X8_PIN_NONE);
#elif SCREEN_TYPE == 3
  #if SH1107_VARIANT == 2
  U8G2_SH1107_128X128_F_4W_HW_SPI scr1(U8G2_R0, /*cs=*/5, /*dc=*/4, /*reset=*/15);
  #elif SH1107_VARIANT == 3
  U8G2_SH1107_SEEED_128X128_F_4W_HW_SPI scr1(U8G2_R0, /*cs=*/5, /*dc=*/4, /*reset=*/15);
  #else
  U8G2_SH1107_PIMORONI_128X128_F_4W_HW_SPI scr1(U8G2_R0, /*cs=*/5, /*dc=*/4, /*reset=*/15);
  #endif
#else
U8G2_SSD1327_WS_128X128_F_4W_HW_SPI scr1(U8G2_R0, /*cs=*/5, /*dc=*/4, /*reset=*/15);
#endif

// second screen on pins 16/17 (always SH1107)
#if SH1107_VARIANT == 2
U8G2_SH1107_128X128_F_2ND_HW_I2C scr2(U8G2_R0, U8X8_PIN_NONE);
#elif SH1107_VARIANT == 3
U8G2_SH1107_SEEED_128X128_F_2ND_HW_I2C scr2(U8G2_R0, U8X8_PIN_NONE);
#else
U8G2_SH1107_PIMORONI_128X128_F_2ND_HW_I2C scr2(U8G2_R0, U8X8_PIN_NONE);
#endif

U8G2* statusScr = nullptr;   // set at power on, depending on which screens answer
U8G2* mapScr    = nullptr;
U8G2* onlyScr   = nullptr;   // used when there is just one screen

// ---------- State ----------
enum State { CALIBRATING, ARMING, ARMED, ALARM };
volatile State state = CALIBRATING;

int   level[4];          // live light level 0..4095
int   baseLevel[4];      // calibrated level with laser on
int   threshold[4];
bool  zoneActive[4];     // false = laser not aligned, zone bypassed
bool  zoneTripped[4];    // latched until reset
unsigned long lowSince[4];

unsigned long stateStart = 0;
unsigned long alarmStart = 0;
unsigned long lastHeartbeat = 0;
int   tripCount = 0;
int   firstZone = -1;
bool  recalibrateNext = true;

// radar
int   rDist[R_BINS];            // last reading in each direction (cm, 0 = nothing)
int   rBase[R_BINS];            // learned background (cm, 0 = nothing in range)
int   rHitDist[R_BINS];         // distance of the last NEW object seen in each direction
unsigned long rHitAt[R_BINS];   // when it was seen
int   rAngle = 90, rDir = 1;
unsigned long rMovedAt = 0;
bool  rLearning = true;
int   rLearnSweeps = 0;
bool  rPrevHit = false;
int   rPrevHitDist = 0;
bool  contactActive = false;
int   cAngle = 0, cDist = 0;
unsigned long cSeenAt = 0;
int   contactCount = 0;         // contacts since power on
unsigned long lastRadarNote = 0;
bool  radarNoteOpen = false;    // a "contact" message was sent and not yet "cleared"
int   radarNotes = 0;
unsigned long radarNotesDay = 0;
unsigned long chirpStart = 0;
bool  chirping = false;

// ---------- Notification queue (sent from a background task so the siren never freezes) ----------
struct Note { char tag[16]; int zone; char msg[120]; int prio; };
QueueHandle_t noteQueue;

void queueNote(const char* tag, int zone, const char* msg, int prio) {
  Note n;
  strncpy(n.tag, tag, sizeof(n.tag) - 1); n.tag[sizeof(n.tag) - 1] = 0;
  n.zone = zone;
  strncpy(n.msg, msg, sizeof(n.msg) - 1); n.msg[sizeof(n.msg) - 1] = 0;
  n.prio = prio;
  xQueueSend(noteQueue, &n, 0);
}

// Builds the "z1-z4" string: O=ok, X=tripped, -=bypassed
void zoneString(char* out) {
  for (int i = 0; i < 4; i++) out[i] = !zoneActive[i] ? '-' : (zoneTripped[i] ? 'X' : 'O');
  out[4] = 0;
}

bool publish(const char* topic, const Note& n) {
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiClientSecure client;
  client.setInsecure();            // school project: skip certificate check
  HTTPClient http;
  if (!http.begin(client, "https://ntfy.sh/")) return false;
  http.addHeader("Content-Type", "application/json");

  // Every report carries a full snapshot so the website is always accurate:
  // zones-OXO-   O=ok X=breached -=bypassed
  // lvl-a-b-c-d  beam strength of each zone in % of its calibrated brightness
  // st-armed / st-alarm / st-arming,  up-<minutes>,  rssi-<dBm>,  trips-<count>
  // rdr-on / rdr-learn / rdr-off,  rng-<radar range cm>,  rc-<1 if contact now>
  // rad-<angle>-<cm>   the radar contact,  rb-<angle>x<cm>-...  other new objects this sweep
  char zones[5]; zoneString(zones);
  char lvl[24];
  int pct[4];
  for (int i = 0; i < 4; i++) {
    pct[i] = (!zoneActive[i] || baseLevel[i] <= 0) ? 0 : constrain((long)level[i] * 100 / baseLevel[i], 0, 99);
  }
  snprintf(lvl, sizeof(lvl), "%d-%d-%d-%d", pct[0], pct[1], pct[2], pct[3]);
  const char* st = state == ALARM ? "alarm" : state == ARMED ? "armed" : "arming";

  char radar[150] = "";
#if RADAR_ENABLED
  {
    int n0 = snprintf(radar, sizeof(radar), ",\"rdr-%s\",\"rng-%d\",\"rc-%d\"",
                      rLearning ? "learn" : "on", RADAR_RANGE_CM, contactActive ? 1 : 0);
    if (contactActive)
      n0 += snprintf(radar + n0, sizeof(radar) - n0, ",\"rad-%d-%d\"", cAngle, cDist);
    // up to 5 other directions where something new was seen recently
    char rb[64] = "rb"; int len = 2, cnt = 0;
    unsigned long now = millis();
    for (int b = 0; b < R_BINS && cnt < 5; b += 2) {
      if (rHitAt[b] && now - rHitAt[b] < RADAR_HOLD_MS) {
        len += snprintf(rb + len, sizeof(rb) - len, "-%dx%d", b * R_STEP, rHitDist[b]);
        cnt++;
      }
    }
    if (cnt) snprintf(radar + n0, sizeof(radar) - n0, ",\"%s\"", rb);
  }
#else
  snprintf(radar, sizeof(radar), ",\"rdr-off\"");
#endif

  const char* emoji = strcmp(n.tag, "intruder") == 0 || strcmp(n.tag, "breach") == 0 ? "rotating_light"
                    : strcmp(n.tag, "radar") == 0    ? "satellite_antenna"
                    : strcmp(n.tag, "armed") == 0    ? "shield"
                    : strcmp(n.tag, "reset") == 0 || strcmp(n.tag, "radarclear") == 0 ? "white_check_mark"
                    : strcmp(n.tag, "boot") == 0     ? "electric_plug"
                    : "satellite";
  char body[700];
  snprintf(body, sizeof(body),
    "{\"topic\":\"%s\",\"title\":\"%s\",\"message\":\"%s\",\"priority\":%d,"
    "\"tags\":[\"%s\",\"%s\",\"zone-%d\",\"zones-%s\",\"lvl-%s\",\"st-%s\",\"rssi%d\",\"trips-%d\",\"up-%lu\"%s]}",
    topic, BASE_NAME, n.msg, n.prio, emoji, n.tag, n.zone, zones, lvl, st,
    (int)WiFi.RSSI(), tripCount, millis() / 60000UL, radar);

  int code = http.POST((uint8_t*)body, strlen(body));
  http.end();
  Serial.printf("[ntfy] %s -> %d\n", n.tag, code);
  return code == 200;
}

void notifyTask(void*) {
  Note n;
  for (;;) {
    if (xQueueReceive(noteQueue, &n, portMAX_DELAY)) {
      // heartbeats + boot go to a quiet topic so your phone doesn't buzz for them
      char topic[80];
      if (strcmp(n.tag, "heartbeat") == 0 || strcmp(n.tag, "boot") == 0) snprintf(topic, sizeof(topic), "%s-status", NTFY_TOPIC);
      else snprintf(topic, sizeof(topic), "%s", NTFY_TOPIC);
      for (int attempt = 0; attempt < 3; attempt++) {
        if (publish(topic, n)) break;
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
    }
  }
}

// ---------- Laser sensors ----------
int readLDR(int i) {
  long sum = 0;
  for (int k = 0; k < 4; k++) sum += analogRead(LDR_PINS[i]);
  return sum / 4;
}

void calibrate() {
  long sum[4] = {0, 0, 0, 0};
  const int N = 40;
  for (int s = 0; s < N; s++) {
    for (int i = 0; i < 4; i++) sum[i] += readLDR(i);
    delay(10);
  }
  for (int i = 0; i < 4; i++) {
    baseLevel[i]  = sum[i] / N;
    threshold[i]  = baseLevel[i] * TRIP_RATIO;
    zoneActive[i] = baseLevel[i] >= MIN_LIGHT;
    Serial.printf("Zone %d: light=%d threshold=%d %s\n", i + 1, baseLevel[i], threshold[i],
                  zoneActive[i] ? "OK" : "NOT ALIGNED -> bypassed");
  }
}

// ---------- Radar ----------
void servoWrite(int angle) {
#if RADAR_ENABLED
  uint32_t us = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  uint32_t duty = (us * 16383UL) / 20000UL;     // 14-bit duty at 50 Hz (20 ms)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SERVO_PIN, duty);
#else
  ledcWrite(4, duty);
#endif
#endif
}

int pingCm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // only wait long enough for objects inside the radar range (+30 cm), max ~6 ms
  unsigned long us = pulseIn(ECHO_PIN, HIGH, (RADAR_RANGE_CM + 30) * 58UL + 1000UL);
  if (us == 0) return 0;
  int cm = us / 58;
  return (cm < 2 || cm > 400) ? 0 : cm;
}

void radarStartLearning() {
  for (int b = 0; b < R_BINS; b++) { rBase[b] = 0; rHitAt[b] = 0; rHitDist[b] = 0; }
  rLearning = true;
  rLearnSweeps = 0;
  rPrevHit = false;
  contactActive = false;
  Serial.println("Radar: learning background (keep the area clear)");
}

void radarContact(int angle, int cm) {
  unsigned long now = millis();
  bool isNew = !contactActive;
  contactActive = true;
  cSeenAt = now;
  cAngle = constrain(angle, 0, 180);
  cDist = cm;
  if (isNew) {
    contactCount++;
    if (state != ALARM) { chirpStart = now; chirping = true; }
    Serial.printf("Radar contact: %d cm at %d deg\n", cm, angle);
  }
}

void radarSweepDone() {
  rPrevHit = false;
  if (rLearning && ++rLearnSweeps >= 2) {       // 2 sweeps = every direction seen twice
    rLearning = false;
    Serial.println("Radar: background learned, watching");
    // quiet report so the website switches from "learning" to "on watch" right away
    if (state == ARMED || state == ALARM) queueNote("heartbeat", 0, "Radar learned the area. Radar on watch.", 1);
  }
}

void radarUpdate() {
#if RADAR_ENABLED
  unsigned long now = millis();
  if (now - rMovedAt < RADAR_STEP_MS) return;
  int b = rAngle / R_STEP;
  int d = pingCm();
  rDist[b] = d;
  if (rLearning) {
    if (d > 0 && (rBase[b] == 0 || d < rBase[b])) rBase[b] = d;   // nearest thing normally there
  } else {
    bool hit = d > 0 && d <= RADAR_RANGE_CM && (rBase[b] == 0 || d < rBase[b] - RADAR_MARGIN_CM);
    if (hit) {
      rHitAt[b] = now; rHitDist[b] = d;
      // a real object shows up in two neighbouring directions at about the same distance
      if (rPrevHit && abs(d - rPrevHitDist) <= 10) radarContact(rAngle - rDir * R_STEP / 2, min(d, rPrevHitDist));
    }
    rPrevHit = hit; rPrevHitDist = d;
  }
  int next = rAngle + rDir * R_STEP;
  if (next > 180) { rDir = -1; next = rAngle - R_STEP; radarSweepDone(); }
  else if (next < 0) { rDir = 1; next = rAngle + R_STEP; radarSweepDone(); }
  rAngle = next;
  servoWrite(rAngle);
  rMovedAt = now;
#endif
}

void radarNotifications() {
#if RADAR_ENABLED
  unsigned long now = millis();
  if (contactActive && now - cSeenAt > RADAR_HOLD_MS) {
    contactActive = false;
    Serial.println("Radar: contact gone");
    if (radarNoteOpen && radarNotes < RADAR_DAILY_MAX) {
      radarNotes++;
      queueNote("radarclear", 0, "Radar clear. Contact has left the area.", 2);
    }
    radarNoteOpen = false;
  }
  if (now - radarNotesDay > 24UL * 3600UL * 1000UL) { radarNotesDay = now; radarNotes = 0; }
  if (!contactActive || (state != ARMED && state != ALARM) || radarNotes >= RADAR_DAILY_MAX) return;

  char msg[120];
  if (!radarNoteOpen && (lastRadarNote == 0 || now - lastRadarNote > RADAR_NOTIFY_GAP)) {
    snprintf(msg, sizeof(msg), "Radar contact! Object %d cm away at %d degrees.", cDist, cAngle);
    queueNote("radar", 0, msg, 4);
    radarNotes++; radarNoteOpen = true; lastRadarNote = now;
  } else if (radarNoteOpen && now - lastRadarNote > RADAR_UPDATE_MS) {
    snprintf(msg, sizeof(msg), "Radar contact still there: %d cm at %d degrees.", cDist, cAngle);
    queueNote("radar", 0, msg, 3);
    radarNotes++; lastRadarNote = now;
  }
#endif
}

// ---------- Buzzer ----------
void buzzerUpdate() {
  static bool wasOn = false;
  unsigned long now = millis();
  bool on = false;
  int freq = 0;
  if (state == ALARM) {
    unsigned long t = now - alarmStart;
#if BUZZER_PASSIVE
    int phase = t % 1000;                     // rising/falling wail 700..1700 Hz
    freq = phase < 500 ? 700 + phase * 2 : 1700 - (phase - 500) * 2;
    on = true;
#else
    on = (t % 250) < 150;                     // fast military-style pulse
#endif
    digitalWrite(LED_PIN, (t % 500) < 250 ? HIGH : LOW);
  } else if (chirping) {
    unsigned long t = now - chirpStart;       // radar: two short chirps
    if (t > 260) chirping = false;
    else { on = t < 70 || (t > 130 && t < 200); freq = 2600; }
  }

#if BUZZER_PASSIVE
  static int lastFreq = 0;
  static unsigned long lastTone = 0;
  if (on && (freq != lastFreq || now - lastTone > 15)) { tone(BUZZER_PIN, freq); lastFreq = freq; lastTone = now; }
  if (!on && wasOn) { noTone(BUZZER_PIN); lastFreq = 0; }
#else
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
#endif
  if (state != ALARM && wasOn) digitalWrite(LED_PIN, LOW);
  wasOn = on || state == ALARM;
}

void beep(int ms) {
#if BUZZER_PASSIVE
  tone(BUZZER_PIN, 2000, ms); delay(ms);
#else
  digitalWrite(BUZZER_PIN, HIGH); delay(ms); digitalWrite(BUZZER_PIN, LOW);
#endif
}

// ---------- Reset button ----------
// returns 0 = nothing, 1 = short press, 2 = long press (3 s)
int readButton() {
  static bool down = false;
  static unsigned long downAt = 0;
  static bool longFired = false;
  bool pressed = digitalRead(RESET_PIN) == LOW;
  if (pressed && !down) { down = true; downAt = millis(); longFired = false; }
  if (pressed && down && !longFired && millis() - downAt > LONG_PRESS_MS) { longFired = true; return 2; }
  if (!pressed && down) {
    down = false;
    if (!longFired && millis() - downAt > 30) return 1;   // 30 ms debounce
  }
  return 0;
}

// ==== SCREENS BEGIN ====
// Both screens are drawn by these two functions. They only READ the variables above.

void drawCentered(U8G2 &d, const char* s, int y) {
  d.drawStr((128 - d.getStrWidth(s)) / 2, y, s);
}

// ---- page 1: status ----
void drawStatus(U8G2 &d) {
  unsigned long now = millis();
  bool flash = (now / 300) % 2;
  char buf[32];
  d.clearBuffer();
  d.setDrawColor(1);

  // header
  d.setFont(u8g2_font_6x10_tf);
  d.drawStr(0, 9, BASE_NAME);
  const char* net = WiFi.status() == WL_CONNECTED ? "NET" : "NO NET";
  d.drawStr(128 - d.getStrWidth(net), 9, net);
  d.drawHLine(0, 12, 128);

  // big status
  if (state == ALARM) {
    if (flash) { d.drawBox(0, 15, 128, 30); d.setDrawColor(0); }
    d.setFont(u8g2_font_helvB18_tf);
    drawCentered(d, "INTRUDER", 38);
    d.setDrawColor(1);
  } else {
    d.setFont(u8g2_font_helvB14_tf);
    if (state == CALIBRATING) drawCentered(d, "CALIBRATING", 36);
    else if (state == ARMING) {
      int left = ARM_DELAY_S - (int)((now - stateStart) / 1000);
      snprintf(buf, sizeof(buf), "ARMING %d", left < 0 ? 0 : left);
      drawCentered(d, buf, 36);
    } else drawCentered(d, "ARMED", 36);
  }

  // 4 zone panels with light bars
  d.setFont(u8g2_font_5x8_tf);
  for (int i = 0; i < 4; i++) {
    int x = i * 32 + 1, y = 50, w = 30, h = 50;
    d.drawFrame(x, y, w, h);
    snprintf(buf, sizeof(buf), "Z%d", i + 1);
    d.drawStr(x + 9, y + 9, buf);
    int barH = map(constrain(level[i], 0, 4095), 0, 4095, 0, 30);
    d.drawFrame(x + 10, y + 12, 10, 32);
    d.drawBox(x + 11, y + 43 - barH, 8, barH);
    if (zoneActive[i] && state != CALIBRATING) {
      int ty = y + 43 - map(threshold[i], 0, 4095, 0, 30);
      d.drawHLine(x + 6, ty, 18);
    }
    const char* st = !zoneActive[i] ? "OFF" : (zoneTripped[i] ? "TRIP" : "OK");
    if (zoneTripped[i] && flash) { d.drawBox(x, y + h, w, 10); d.setDrawColor(0); }
    d.drawStr(x + (w - d.getStrWidth(st)) / 2, y + h + 8, st);
    d.setDrawColor(1);
  }

  // footer: radar line or trip counter
  d.setFont(u8g2_font_5x8_tf);
#if RADAR_ENABLED
  if (contactActive && !(state == ALARM && (now / 1500) % 2)) {   // in an alarm, alternate with the reset hint
    snprintf(buf, sizeof(buf), "RADAR: %dcm AT %d DEG", cDist, cAngle);
    if (flash) { d.drawBox(0, 119, 128, 9); d.setDrawColor(0); }
    drawCentered(d, buf, 127);
    d.setDrawColor(1);
  } else if (state == ALARM) drawCentered(d, "PRESS RESET TO CLEAR", 127);
  else {
    snprintf(buf, sizeof(buf), "TRIPS:%d  RADAR:%s", tripCount, rLearning ? "LEARN" : "CLEAR");
    drawCentered(d, buf, 127);
  }
#else
  if (state == ALARM) drawCentered(d, "PRESS RESET TO CLEAR", 127);
  else { snprintf(buf, sizeof(buf), "TRIPS:%d  UP:%lum", tripCount, now / 60000); drawCentered(d, buf, 127); }
#endif
  d.sendBuffer();
}

// one wall beam on the mini map
void drawBeam(U8G2 &d, int x1, int y1, int x2, int y2, int i, unsigned long now) {
  bool horiz = (y1 == y2);
  int len = horiz ? x2 - x1 : y2 - y1;
  if (!zoneActive[i] && state != CALIBRATING) {              // bypassed: sparse dots
    for (int k = 0; k <= len; k += 4) horiz ? d.drawPixel(x1 + k, y1) : d.drawPixel(x1, y1 + k);
  } else if (zoneTripped[i]) {                               // breached: thick, blinking
    if ((now / 250) % 2) {
      if (horiz) { d.drawBox(x1, y1 - 1, len + 1, 3); }
      else       { d.drawBox(x1 - 1, y1, 3, len + 1); }
    }
  } else if (state == ARMING || state == CALIBRATING) {     // arming: dashes
    for (int k = 0; k <= len; k++) if ((k % 6) < 3) horiz ? d.drawPixel(x1 + k, y1) : d.drawPixel(x1, y1 + k);
  } else {                                                   // on watch: solid with a moving gap
    int gap = (now / 80) % 8;
    for (int k = 0; k <= len; k++) if ((k + gap) % 8 != 0) horiz ? d.drawPixel(x1 + k, y1) : d.drawPixel(x1, y1 + k);
  }
}

// ---- page 2: tactical map + radar scope ----
void drawMap(U8G2 &d) {
  unsigned long now = millis();
  bool flash = (now / 300) % 2;
  char buf[32];
  d.clearBuffer();
  d.setDrawColor(1);

  // header
  d.setFont(u8g2_font_5x8_tf);
  d.drawStr(0, 8, "TAC MAP");
  const char* st = state == ALARM ? "BREACH" : state == ARMED ? (contactActive ? "CONTACT" : "SECURE")
                 : state == ARMING ? "ARMING" : "CALIB";
  int sw = d.getStrWidth(st);
  if ((state == ALARM || contactActive) && flash) { d.drawBox(126 - sw, 0, sw + 2, 10); d.setDrawColor(0); }
  d.drawStr(127 - sw, 8, st);
  d.setDrawColor(1);
  d.drawHLine(0, 10, 128);

  // --- mini map (left, 60 x 60) ---
  const int L = 7, T = 18, R = 55, B = 66;          // wall / beam square
  for (int k = 2; k <= 60; k += 2) {                  // barbed-wire fence (dotted)
    d.drawPixel(1 + k, 13); d.drawPixel(1 + k, 71);
    d.drawPixel(2, 12 + k); d.drawPixel(62, 12 + k);
  }
  drawBeam(d, L + 3, T, R - 3, T, 0, now);            // Z1 north
  drawBeam(d, R, T + 3, R, B - 3, 1, now);            // Z2 east
  drawBeam(d, L + 3, B, R - 3, B, 2, now);            // Z3 south (gate)
  drawBeam(d, L, T + 3, L, B - 3, 3, now);            // Z4 west
  // watchtowers
  d.drawFrame(L - 2, T - 2, 5, 5); d.drawFrame(R - 2, T - 2, 5, 5);
  d.drawFrame(R - 2, B - 2, 5, 5); d.drawFrame(L - 2, B - 2, 5, 5);
  // gate marks
  d.drawVLine(28, B + 2, 3); d.drawVLine(34, B + 2, 3);
  // buildings
  d.drawFrame(24, 36, 16, 10);                        // HQ
  d.drawVLine(31, 31, 5); d.drawBox(32, 31, 3, 2);    // flag
  d.drawFrame(13, 52, 7, 5); d.drawFrame(21, 52, 7, 5);    // barracks
  d.drawCircle(16, 28, 4);  d.drawVLine(15, 26, 5); d.drawVLine(17, 26, 5); d.drawPixel(16, 28); // helipad
  d.drawFrame(39, 53, 10, 6);                         // motor pool
  // radar dish with its live heading
  const int RX = 46, RY = 28;
  if (contactActive && flash) d.drawDisc(RX, RY, 3); else d.drawCircle(RX, RY, 3);
#if RADAR_ENABLED
  float ra = rAngle * PI / 180.0;
  d.drawLine(RX, RY, RX + (int)round(7 * cos(ra)), RY - (int)round(7 * sin(ra)));
#endif
  // zone numbers
  d.setFont(u8g2_font_4x6_tf);
  d.drawStr(30, 25, "1"); d.drawStr(50, 44, "2"); d.drawStr(30, 63, "3"); d.drawStr(10, 44, "4");

  // --- zone list (right) ---
  d.setFont(u8g2_font_5x8_tf);
  for (int i = 0; i < 4; i++) {
    int y = 22 + i * 10;
    const char* s = (state == CALIBRATING) ? "CAL" : !zoneActive[i] ? "OFF"
                  : zoneTripped[i] ? "BREACH" : state == ARMING ? "ARMING" : "OK";
    snprintf(buf, sizeof(buf), "Z%d", i + 1);
    d.drawStr(67, y, buf);
    if (zoneTripped[i] && flash) { d.drawBox(80, y - 8, 48, 10); d.setDrawColor(0); }
    d.drawStr(82, y, s);
    d.setDrawColor(1);
  }
#if RADAR_ENABLED
  if (rLearning) snprintf(buf, sizeof(buf), "RDR LEARN");
  else if (contactActive) snprintf(buf, sizeof(buf), "RDR %dcm", cDist);
  else snprintf(buf, sizeof(buf), "RDR CLEAR");
#else
  snprintf(buf, sizeof(buf), "RDR OFF");
#endif
  d.drawStr(67, 66, buf);

  // --- radar scope (bottom half) ---
  const int CX = 64, CY = 126, RR = 50;
  d.drawHLine(CX - RR, CY, 2 * RR + 1);
  d.drawCircle(CX, CY, RR, U8G2_DRAW_UPPER_RIGHT | U8G2_DRAW_UPPER_LEFT);
  for (int a = 0; a <= 180; a += 6) {                 // dotted range rings at 1/3 and 2/3
    float r = a * PI / 180.0;
    d.drawPixel(CX + (int)round(RR * 2 / 3.0 * cos(r)), CY - (int)round(RR * 2 / 3.0 * sin(r)));
    if (a % 12 == 0) d.drawPixel(CX + (int)round(RR / 3.0 * cos(r)), CY - (int)round(RR / 3.0 * sin(r)));
  }
  for (int a = 45; a <= 135; a += 45) {               // dotted bearing lines
    float r = a * PI / 180.0;
    for (int k = 6; k < RR; k += 4) d.drawPixel(CX + (int)round(k * cos(r)), CY - (int)round(k * sin(r)));
  }
#if RADAR_ENABLED
  float sr = rAngle * PI / 180.0;                     // live sweep line
  d.drawLine(CX, CY, CX + (int)round(RR * cos(sr)), CY - (int)round(RR * sin(sr)));
  for (int b = 0; b < R_BINS; b++) {                  // new objects seen in the last sweep
    if (rHitAt[b] && now - rHitAt[b] < RADAR_HOLD_MS) {
      float r = b * R_STEP * PI / 180.0;
      int dd = min(rHitDist[b], RADAR_RANGE_CM) * RR / RADAR_RANGE_CM;
      d.drawBox(CX + (int)round(dd * cos(r)) - 1, CY - (int)round(dd * sin(r)) - 1, 2, 2);
    }
  }
  if (contactActive) {                                // the contact itself
    float r = cAngle * PI / 180.0;
    int dd = min(cDist, RADAR_RANGE_CM) * RR / RADAR_RANGE_CM;
    int px = CX + (int)round(dd * cos(r)), py = CY - (int)round(dd * sin(r));
    d.drawCircle(px, py, flash ? 4 : 3);
    d.drawDisc(px, py, 1);
  }
#endif
  d.setFont(u8g2_font_4x6_tf);
  d.drawStr(0, 80, "RADAR");
  snprintf(buf, sizeof(buf), "%d", RADAR_RANGE_CM);
  d.drawStr(128 - d.getStrWidth(buf), 126, buf);
  d.drawStr(0, 126, "cm");
#if RADAR_ENABLED
  if (rLearning) snprintf(buf, sizeof(buf), "LEARNING");
  else if (contactActive) snprintf(buf, sizeof(buf), "%dcm %d%c", cDist, cAngle, (char)176);
  else snprintf(buf, sizeof(buf), "CLEAR");
#else
  snprintf(buf, sizeof(buf), "NO RADAR");
#endif
  d.drawStr(128 - d.getStrWidth(buf), 80, buf);
  d.sendBuffer();
}
// ==== SCREENS END ====

// Screens are drawn on the other CPU core, so slow screen updates never delay the lasers or radar.
void displayTask(void*) {
  for (;;) {
    if (statusScr && mapScr) {
      drawStatus(*statusScr);
      drawMap(*mapScr);
    } else if (onlyScr) {
      unsigned long now = millis();
      bool showMap = state == ALARM ? (now / 2500) % 2 : (now / 5000) % 2;
      if (contactActive && state != ALARM) showMap = true;   // radar contact: show the scope
      if (showMap) drawMap(*onlyScr); else drawStatus(*onlyScr);
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000));                       // no screen found
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ---------- Finding the screens ----------
uint8_t findScreen(TwoWire& bus) {          // OLEDs answer on I2C address 0x3C or 0x3D
  const uint8_t addrs[2] = {0x3C, 0x3D};
  for (uint8_t a : addrs) {
    bus.beginTransmission(a);
    if (bus.endTransmission() == 0) return a;
  }
  return 0;
}

void splash(U8G2& d, const char* role) {
  d.clearBuffer();
  d.drawFrame(0, 0, 128, 128);
  d.setFont(u8g2_font_helvB12_tf);
  drawCentered(d, BASE_NAME, 40);
  d.setFont(u8g2_font_6x10_tf);
  drawCentered(d, "SCREEN OK", 70);
  drawCentered(d, role, 88);
  d.sendBuffer();
}

void setupScreens() {
  Wire.begin(SCR1_SDA, SCR1_SCL);
  Wire1.begin(SCR2_SDA, SCR2_SCL);
#if SCREEN1_IS_I2C
  uint8_t a1 = findScreen(Wire);
  bool have1 = a1 != 0;
#else
  uint8_t a1 = 0;
  bool have1 = true;                          // SPI screens can't be detected, assume it's there
#endif
  uint8_t a2 = findScreen(Wire1);

  if (have1) {
#if SCREEN1_IS_I2C
    scr1.setI2CAddress(a1 * 2);
    scr1.setBusClock(SCREEN_I2C_HZ);
#endif
    scr1.begin();
    scr1.setContrast(255);
  }
  if (a2) {
    scr2.setI2CAddress(a2 * 2);
    scr2.setBusClock(SCREEN_I2C_HZ);
    scr2.begin();
    scr2.setContrast(255);
  }

  if (have1 && a2)  { statusScr = &scr1; mapScr = &scr2; splash(scr1, "STATUS PAGE"); splash(scr2, "MAP + RADAR PAGE"); }
  else if (have1)   { onlyScr = &scr1; splash(scr1, "STATUS + MAP PAGES"); }
  else if (a2)      { onlyScr = &scr2; splash(scr2, "STATUS + MAP PAGES"); }

  Serial.println("---- Screens ----");
#if SCREEN1_IS_I2C
  if (a1) Serial.printf("Pins 21/22: screen found at 0x%02X\n", a1);
  else    Serial.println("Pins 21/22: no screen");
#else
  Serial.println("SPI screen: CS 5, DC 4, RST 15, DIN 23, CLK 18");
#endif
  if (a2) Serial.printf("Pins 16/17: screen found at 0x%02X\n", a2);
  else    Serial.println("Pins 16/17: no screen");
  if (!have1 && !a2) Serial.println("NO SCREEN FOUND: check VCC->3.3V, GND, SDA, SCL (SDA and SCL swapped is very common)");
  Serial.printf("Screen type setting: %d (1 = SH1107 I2C)\n", SCREEN_TYPE);
  delay(1200);                                // show the "SCREEN OK" page for a moment
}

// ---------- State changes ----------
void startArming(bool recalibrate) {
  for (int i = 0; i < 4; i++) { zoneTripped[i] = false; lowSince[i] = 0; }
  firstZone = -1;
  recalibrateNext = recalibrate;
  state = ARMING;
  stateStart = millis();
}

void triggerAlarm(int zone) {
  if (zoneTripped[zone]) return;        // this zone already reported
  zoneTripped[zone] = true;
  if (state == ALARM) {
    // a second (third...) zone broken while the alarm is already on
    char msg[120];
    snprintf(msg, sizeof(msg), "Second breach! Laser zone %d also broken.", zone + 1);
    queueNote("breach", zone + 1, msg, 5);
    Serial.printf("!!! ALSO zone %d\n", zone + 1);
  } else {
    state = ALARM;
    alarmStart = millis();
    firstZone = zone;
    tripCount++;
    chirping = false;
    char msg[120];
    snprintf(msg, sizeof(msg), "INTRUDER DETECTED! Laser zone %d broken.", zone + 1);
    queueNote("intruder", zone + 1, msg, 5);
    Serial.printf("!!! ALARM zone %d\n", zone + 1);
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  analogReadResolution(12);
  for (int i = 0; i < 4; i++) {
    analogSetPinAttenuation(LDR_PINS[i], ADC_11db);   // full 0..3.3 V range
    zoneActive[i] = true; zoneTripped[i] = false; level[i] = 0;
  }

#if RADAR_ENABLED
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(SERVO_PIN, 50, 14);
#else
  ledcSetup(4, 50, 14);
  ledcAttachPin(SERVO_PIN, 4);
#endif
  servoWrite(rAngle);
#endif
  radarStartLearning();

  setupScreens();
  state = CALIBRATING;

  noteQueue = xQueueCreate(8, sizeof(Note));
  xTaskCreatePinnedToCore(notifyTask, "notify", 12288, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(displayTask, "screens", 6144, nullptr, 1, nullptr, 0);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(200);
  Serial.println(WiFi.status() == WL_CONNECTED ? "WiFi connected" : "WiFi failed (alarm still works offline)");
  WiFi.setAutoReconnect(true);
  queueNote("boot", 0, "Base powered on. Calibrating lasers and radar.", 2);

  beep(80);
  startArming(true);
}

// ---------- Loop ----------
void loop() {
  unsigned long now = millis();

  for (int i = 0; i < 4; i++) level[i] = readLDR(i);

  int btn = readButton();
  if (btn == 2) {                       // long press: re-calibrate lasers + radar
    beep(60); delay(80); beep(60);
    startArming(true);
  } else if (btn == 1) {                // short press: reset / re-arm
    if (state == ALARM) queueNote("reset", firstZone + 1, "Alarm reset by guard. Re-arming.", 3);
    beep(60);
    startArming(false);
  }

  switch (state) {
    case CALIBRATING:
      break;

    case ARMING:
      if (now - stateStart >= (unsigned long)ARM_DELAY_S * 1000) {
        if (recalibrateNext) {
          state = CALIBRATING;
          calibrate();
          radarStartLearning();
        }
        state = ARMED;
        beep(150);
        char msg[120]; int active = 0;
        for (int i = 0; i < 4; i++) active += zoneActive[i];
        snprintf(msg, sizeof(msg), "System ARMED. %d of 4 laser zones active.%s", active,
                 !RADAR_ENABLED ? "" : rLearning ? " Radar learning the area." : " Radar on watch.");
        queueNote("armed", 0, msg, 2);
        lastHeartbeat = now;
      }
      break;

    case ARMED:
    case ALARM:
      for (int i = 0; i < 4; i++) {
        if (!zoneActive[i]) continue;
        if (level[i] < threshold[i]) {
          if (lowSince[i] == 0) lowSince[i] = now;
          if (now - lowSince[i] >= TRIP_MS) triggerAlarm(i);
        } else lowSince[i] = 0;
      }
      break;
  }

  radarUpdate();
  radarNotifications();
  buzzerUpdate();

  // periodic report (also during an alarm, so the website knows the siren is still on)
  if ((state == ARMED || state == ALARM) && now - lastHeartbeat > HEARTBEAT_MS) {
    lastHeartbeat = now;
    queueNote("heartbeat", 0, state == ALARM ? "Alarm still active. Waiting for reset." : "System online and armed.", 1);
  }

  // WiFi came back after a drop: report right away so the website shows the base online again
  static bool wasConnected = WiFi.status() == WL_CONNECTED;
  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !wasConnected) {
    lastHeartbeat = now;
    queueNote("heartbeat", 0, "Base back online after WiFi drop.", 1);
  }
  wasConnected = connected;

  delay(2);
}
