/**
 * ============================================================
 *  MedSync Smart Medicine Dispenser — ESP32 firmware v4.0.0
 *  Target: ESP32 Arduino core 3.x (tested API set: core 3.3.x)
 *          Firebase Arduino Client Library (Mobizt) 4.4.x
 * ============================================================
 *
 *  WHAT v4 CHANGES (robustness)
 *   - LCD: finds the I2C address by itself (0x27, 0x3F, ...), retries if missing,
 *     re-initialises every 10 min to clear noise corruption.
 *   - I2C: frees a stuck bus at boot; RTC missing is no longer fatal (retries
 *     every 5 s and uses internet time meanwhile).
 *   - Boot: no long waits. Each step shows on the LCD; Wi-Fi, time and cloud
 *     connect in the background while the dispenser already works offline.
 *   - Voice: no handshake wait (your DFPlayer never answers it).
 *   - Wi-Fi: up to 3 networks (home / hotspot / college), auto-reconnect.
 *   - Alarm: per-date logic. Cannot be skipped by a busy minute or a missed
 *     midnight, cannot fire twice (also across a reboot), fires up to 10 min
 *     late if the device was busy/booting at the dose time.
 *   - Dispenses doseCount capsules (one servo cycle per capsule).
 *   - Sync: ONE Firebase connection, no background stream (that second
 *     connection caused the SSL errors). Schedule/commands polled every 8 s.
 *   - Outbox: every dispensed/confirmed/missed event is stored in flash and
 *     retried until the cloud accepts it, so offline events are never lost and
 *     history and totals always agree. History keys are fixed
 *     (<doseId>_<status>), so a retry can never create a duplicate row.
 *   - Correct UTC timestamps (v3 stored IST as UTC: dates were 5.5 h off).
 *   - Heartbeat: device/status/lastSeen uses the Firebase server clock, so the
 *     dashboard knows reliably whether the device is online.
 *   - Commands from the dashboard: test alert, reset counters.
 *   - Watchdog reboots the board if anything ever hangs for 60 s.
 *   - Reports why it last restarted (brownout = power supply too weak).
 *
 *  v4.1: HIGH predicted risk -> voice heads-up at 10 and 5 min before the dose, LCD
 *        countdown, and a "prealert" notice so the dashboard e-mails caregivers + patient.
 *        While waiting for Button 2 the reminder repeats every 45 s; confirm window is
 *        3 min (HIGH risk) or 5 min.
 *
 *  SERIAL MONITOR (115200) COMMANDS:  t = test alert   s = status   r = reset counters
 *
 *  LIBRARIES: RTClib (Adafruit), LiquidCrystal_I2C (Frank de Brabander),
 *  ESP32Servo, DFRobotDFPlayerMini, Firebase Arduino Client Library (Mobizt).
 *
 *  WIRING (unchanged)
 *   GPIO21 SDA / GPIO22 SCL -> LCD backpack + DS3231
 *   GPIO13 -> servo signal (servo powered from 5 V, common GND)
 *   GPIO16 (RX2) <- DFPlayer TX ; GPIO17 (TX2) -> 1 kOhm -> DFPlayer RX
 *   GPIO18 -> Button 1 (dispense) ; GPIO19 -> Button 2 (confirm) ; other legs -> GND
 *   SD card: /mp3/0001.mp3 ... (0001-0004 English, 0005,0007,0008,0009 Kannada)
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <EEPROM.h>
#include <Preferences.h>
#include <time.h>
#include <esp_task_wdt.h>

#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <DFRobotDFPlayerMini.h>
#include <Firebase_ESP_Client.h>

#define FW_VERSION "4.1.0"

// ============================================================
//  USER SETTINGS  — edit these
// ============================================================
// Wi-Fi networks, Firebase project and device login live in secrets.h, which is NOT
// uploaded to GitHub. Copy secrets.h.example to secrets.h and fill it in.
struct WifiCred { const char* ssid; const char* pass; };
#include "secrets.h"

#define TZ_OFFSET_SEC    19800          // India Standard Time (UTC+5:30)

// ============================================================
//  PINS
// ============================================================
#define PIN_BUTTON1   18
#define PIN_BUTTON2   19
#define PIN_DFP_RX    16
#define PIN_DFP_TX    17
#define PIN_SERVO     13
#define PIN_SDA       21
#define PIN_SCL       22

// ============================================================
//  BEHAVIOUR CONSTANTS
// ============================================================
#define SERVO_HOME_ANGLE        0
#define SERVO_DISPENSE_ANGLE   90
#define SERVO_MOVE_MS         700
#define SERVO_RETURN_MS       600

#define BTN_MIN_PRESS_MS       40
#define ALERT_BTN_GUARD_MS   1000UL     // Button 1 ignored for 1 s after the alert starts
#define MISSED_TIMEOUT_MS          300000UL   // alert unanswered -> missed (5 min)
#define HIGH_RISK_MISSED_TIMEOUT_MS 180000UL  // 3 min when predicted risk is HIGH
#define CONFIRM_TIMEOUT_MS         300000UL   // dispensed but not confirmed -> missed
#define HIGH_RISK_CONFIRM_TIMEOUT_MS 180000UL // 3 min when predicted risk is HIGH
#define CONFIRM_REMIND_MS     45000UL   // repeat "press Button 2" while waiting
#define LATE_FIRE_WINDOW_MIN   10       // still start the alarm up to 10 min late
#define PRE_ALERT_LEAD_MIN     10       // HIGH risk: first heads-up 10 min before the dose
#define PRE_ALERT_SECOND_MIN    5       // ...and a second one 5 min before
// Early heads-up voice "your medicine is due in 10 minutes": 1 = clip is on the SD card.
// English = /mp3/0010.mp3, Kannada = /mp3/0011.mp3. With 0 the normal alert clip is used instead.
#define EARLY_EN_RECORDED       1
#define EARLY_KN_RECORDED       1
#define CONFIRMED_DISPLAY_MS 3000UL
#define MISSED_DISPLAY_MS    5000UL

#define VOICE_VOLUME           28       // 0..30
#define VOICE_DEFAULT_MODE      2       // 0 English, 1 Kannada, 2 alternate
#define VOICE_REPEAT_MS     18000UL     // Kannada alert clip is ~14 s
#define VOICE_SETTLE_MS      2500UL     // DFPlayer reads its card after power-up

#define STATUS_PUSH_MS      10000UL
#define SCHEDULE_POLL_MS     8000UL
#define COMMAND_POLL_MS      8000UL
#define SLOW_POLL_MS        20000UL     // alternates prediction / voice language
#define WIFI_RETRY_MS       20000UL
#define FB_RETRY_MS         30000UL
#define NTP_RESYNC_MS    21600000UL     // 6 h
#define HW_RETRY_MS          5000UL
#define LCD_REFRESH_MS       1000UL
#define LCD_REINIT_MS      600000UL     // 10 min
#define FB_READ_TIMEOUT_MS   8000
#define COMMAND_MAX_AGE_S     180       // ignore dashboard commands older than 3 min

#define USE_WATCHDOG            1
#define WATCHDOG_TIMEOUT_S     60

// EEPROM layout (0..20 kept from v3 so your schedule/counters survive the update)
#define EEPROM_SIZE            128
#define EEPROM_ADDR_SCHED_VALID  0
#define EEPROM_ADDR_HOUR         1
#define EEPROM_ADDR_MIN          2
#define EEPROM_ADDR_DOSE         3
#define EEPROM_ADDR_LOG_VALID   10
#define EEPROM_ADDR_DISPENSED   11
#define EEPROM_ADDR_CONFIRMED   13
#define EEPROM_ADDR_MISSED      15
#define EEPROM_ADDR_VOICE_VALID 19
#define EEPROM_ADDR_VOICE_MODE  20
#define EEPROM_ADDR_FIRED_VALID 24
#define EEPROM_ADDR_FIRED_DAY   25     // uint32 yyyymmdd of the last day the alarm fired

// Voice clip table: { alert, dispensed, confirmed, missed, early }  (0 = no clip)
#define CLIP_ALERT     0
#define CLIP_DISPENSED 1
#define CLIP_CONFIRMED 2
#define CLIP_MISSED    3
#define CLIP_EARLY     4
static const uint8_t CLIP_EN[5] = { 1, 2, 3, 4, EARLY_EN_RECORDED ? 10 : 1 };
static const uint8_t CLIP_KN[5] = { 5, 7, 8, 9, EARLY_KN_RECORDED ? 11 : 5 };

// ============================================================
//  TYPES & GLOBALS
// ============================================================
typedef enum { STATE_BOOT, STATE_IDLE, STATE_ALERT, STATE_DISPENSING,
               STATE_CONFIRMED, STATE_MISSED, STATE_RTC_ERROR } SystemState_t;

struct Schedule_t { uint8_t hour, minute, doseCount; bool valid; };
struct DoseLog_t  { uint16_t totalDispensed, totalConfirmed, totalMissed; };

struct DoseCtx {                 // the dose currently being handled
  char    doseId[16];            // yyyymmdd-HHMM (+ 't' for a test alert)
  char    sched[6];              // "HH:MM"
  char    med[24];
  uint8_t dose;
  bool    test;
};

struct OutEvent {                // one history row waiting to be uploaded
  char     key[28];
  char     status[10];
  char     doseId[16];
  char     sched[6];
  char     time[9];
  char     med[24];
  uint32_t epoch;                // UTC seconds
  uint8_t  dose;
  uint8_t  test;
};
#define OUTBOX_MAX 16

RTC_DS3231          rtc;
LiquidCrystal_I2C*  lcd = nullptr;
Servo               dispenserServo;
DFRobotDFPlayerMini dfPlayer;
WiFiMulti           wifiMulti;
Preferences         prefs;

FirebaseData   fbdo;
FirebaseAuth   auth;
FirebaseConfig config;

volatile SystemState_t systemState = STATE_BOOT;
Schedule_t schedule = { 8, 0, 1, false };
DoseLog_t  doseLog  = { 0, 0, 0 };
DoseCtx    cur;
char       medName[24]  = "Medicine";
char       riskLevel[8] = "LOW";
uint8_t    voiceMode    = VOICE_DEFAULT_MODE;

// hardware health
bool    rtcOk = false, rtcTimeSuspect = false, lcdOk = false;
uint8_t lcdAddr = 0;
unsigned long lastHwRetry = 0, lastLcdReinit = 0;

// time
bool    ntpStarted = false, ntpSynced = false;
unsigned long lastNtpSync = 0;

// scheduler
uint32_t lastFiredDay = 0, preAlertDay = 0;
uint8_t  preAlertStage = 0;              // 0 none, 1 = 10-min heads-up given, 2 = 5-min given
bool     preAlertActive = false;         // LCD shows the countdown
unsigned long alertStartTime = 0, alertArmedAt = 0, dispensedAt = 0, confirmedAt = 0, missedAt = 0,
              lastConfirmRemind = 0;

// voice
bool    dfConfigured = false, voiceLastKn = false, alertVoiceOn = false;
uint8_t voiceToggle = 0;
unsigned long lastAlertVoice = 0, lastVoiceCmd = 0;

// buttons (latched by interrupt so a press is never lost during a slow network call)
volatile bool btn1Down = false, btn2Down = false, btn1Isr = false, btn2Isr = false;
volatile unsigned long btn1DownAt = 0, btn2DownAt = 0;
bool btn1Pressed = false, btn2Pressed = false;

// network
bool wifiUp = false, fbStarted = false, netTurnUsed = false, logDirty = true, statusPushPending = true;
unsigned long lastWifiTry = 0, lastFbTry = 0, lastStatusPush = 0, lastSchedPoll = 0,
              lastCmdPoll = 0, lastSlowPoll = 0, nextFlushAt = 0, flushBackoff = 2000;
uint8_t slowPollTurn = 0, statusFails = 0;
char    lastCmdId[24] = "", lastCmdResult[16] = "";
bool    cmdBaselineOnly = false;
char    resetReason[12] = "unknown";

// outbox
OutEvent outbox[OUTBOX_MAX];
uint8_t  obCount = 0;

// LCD cache
char lcdShown1[17] = "", lcdShown2[17] = "";
unsigned long lastLcdRefresh = 0, lastLcdForce = 0;
bool lcdDirty = true;

// ============================================================
//  PROTOTYPES (only those used before their definition)
// ============================================================
void lcdShow(const char* l1, const char* l2, bool force = false);
void voicePlay(uint8_t ev);
void requestStatus() { statusPushPending = true; }
const char* stateName(SystemState_t s);

// ============================================================
//  SMALL HELPERS
// ============================================================
inline void wdtFeed() {
#if USE_WATCHDOG
  esp_task_wdt_reset();
#endif
}

bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

/** If a device holds SDA low (bus stuck after a glitch), clock SCL until it lets go. */
void i2cRecover() {
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  delayMicroseconds(20);
  if (digitalRead(PIN_SDA) == HIGH) return;
  Serial.println(F("[I2C] SDA held low - recovering bus"));
  pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 18 && digitalRead(PIN_SDA) == LOW; i++) {
    digitalWrite(PIN_SCL, LOW);  delayMicroseconds(6);
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(6);
  }
  pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);          // STOP condition
  digitalWrite(PIN_SDA, LOW);  delayMicroseconds(6);
  digitalWrite(PIN_SCL, HIGH); delayMicroseconds(6);
  digitalWrite(PIN_SDA, HIGH); delayMicroseconds(6);
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
}

void i2cStart() {
  i2cRecover();
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
}

// ---------- time ----------
bool clockAvailable() { return rtcOk || ntpSynced; }
bool clockTrusted()   { return ntpSynced || (rtcOk && !rtcTimeSuspect); }

DateTime nowLocal() {
  if (rtcOk) return rtc.now();
  if (ntpSynced) return DateTime((uint32_t)(time(nullptr) + TZ_OFFSET_SEC));
  return DateTime((uint32_t)946684800UL);       // 2000-01-01, "unknown"
}
uint32_t toUtcEpoch(const DateTime& d) { return d.unixtime() - TZ_OFFSET_SEC; }
uint32_t dayKey(const DateTime& d)     { return (uint32_t)d.year() * 10000UL + d.month() * 100UL + d.day(); }

void hhmmss(const DateTime& d, char* out) {     // out >= 9 chars, 24 h (data / logs)
  snprintf(out, 9, "%02d:%02d:%02d", d.hour(), d.minute(), d.second());
}

// 12-hour helpers for the LCD
uint8_t     hour12(uint8_t h) { h %= 12; return h ? h : 12; }
const char* ampm(uint8_t h)   { return h < 12 ? "AM" : "PM"; }

const char* voiceModeStr() { return voiceMode == 0 ? "en" : voiceMode == 1 ? "kn" : "both"; }

bool netTurn() {                 // at most ONE cloud request per loop pass (bounds blocking time)
  if (netTurnUsed) return false;
  netTurnUsed = true;
  return true;
}
bool fbOnline() { return fbStarted && WiFi.status() == WL_CONNECTED && Firebase.ready(); }

// ============================================================
//  EEPROM PERSISTENCE
// ============================================================
void saveSchedule() {
  EEPROM.write(EEPROM_ADDR_SCHED_VALID, 0xA5);
  EEPROM.write(EEPROM_ADDR_HOUR, schedule.hour);
  EEPROM.write(EEPROM_ADDR_MIN,  schedule.minute);
  EEPROM.write(EEPROM_ADDR_DOSE, schedule.doseCount);
  EEPROM.commit();
}
void loadSchedule() {
  if (EEPROM.read(EEPROM_ADDR_SCHED_VALID) != 0xA5) { Serial.println(F("[EEPROM] no schedule yet")); return; }
  schedule.hour      = EEPROM.read(EEPROM_ADDR_HOUR);
  schedule.minute    = EEPROM.read(EEPROM_ADDR_MIN);
  schedule.doseCount = EEPROM.read(EEPROM_ADDR_DOSE);
  if (schedule.hour > 23) schedule.hour = 8;
  if (schedule.minute > 59) schedule.minute = 0;
  if (schedule.doseCount < 1 || schedule.doseCount > 10) schedule.doseCount = 1;
  schedule.valid = true;
  Serial.printf("[EEPROM] schedule %02u:%02u x%u\n", schedule.hour, schedule.minute, schedule.doseCount);
}
void saveDoseLog() {
  EEPROM.write(EEPROM_ADDR_LOG_VALID, 0xA5);
  EEPROM.put(EEPROM_ADDR_DISPENSED, doseLog.totalDispensed);
  EEPROM.put(EEPROM_ADDR_CONFIRMED, doseLog.totalConfirmed);
  EEPROM.put(EEPROM_ADDR_MISSED,    doseLog.totalMissed);
  EEPROM.commit();
  logDirty = true;
}
void loadDoseLog() {
  if (EEPROM.read(EEPROM_ADDR_LOG_VALID) != 0xA5) { doseLog = { 0, 0, 0 }; return; }
  EEPROM.get(EEPROM_ADDR_DISPENSED, doseLog.totalDispensed);
  EEPROM.get(EEPROM_ADDR_CONFIRMED, doseLog.totalConfirmed);
  EEPROM.get(EEPROM_ADDR_MISSED,    doseLog.totalMissed);
}
void saveVoiceMode() {
  EEPROM.write(EEPROM_ADDR_VOICE_VALID, 0xA5);
  EEPROM.write(EEPROM_ADDR_VOICE_MODE, voiceMode);
  EEPROM.commit();
}
void loadVoiceMode() {
  if (EEPROM.read(EEPROM_ADDR_VOICE_VALID) != 0xA5) return;
  uint8_t v = EEPROM.read(EEPROM_ADDR_VOICE_MODE);
  if (v <= 2) voiceMode = v;
}
void saveFiredDay() {
  EEPROM.write(EEPROM_ADDR_FIRED_VALID, 0xA5);
  EEPROM.put(EEPROM_ADDR_FIRED_DAY, lastFiredDay);
  EEPROM.commit();
}
void loadFiredDay() {
  if (EEPROM.read(EEPROM_ADDR_FIRED_VALID) == 0xA5) EEPROM.get(EEPROM_ADDR_FIRED_DAY, lastFiredDay);
}

// ============================================================
//  OUTBOX (flash-backed queue of history rows)
// ============================================================
void outboxSave() {
  prefs.putUChar("obn", obCount);
  if (obCount) prefs.putBytes("ob", outbox, sizeof(OutEvent) * obCount);
}
void outboxLoad() {
  obCount = prefs.getUChar("obn", 0);
  if (obCount > OUTBOX_MAX) obCount = 0;
  if (obCount && prefs.getBytes("ob", outbox, sizeof(OutEvent) * obCount) != sizeof(OutEvent) * obCount) obCount = 0;
  if (obCount) Serial.printf("[OUTBOX] %u event(s) waiting from before reboot\n", obCount);
}

/** Fill a dose context for the dose at h:m on the day of n. */
void makeDoseCtx(DoseCtx& c, const DateTime& n, uint8_t h, uint8_t m, bool test) {
  snprintf(c.doseId, sizeof(c.doseId), "%08lu-%02u%02u%s", (unsigned long)dayKey(n), h, m, test ? "t" : "");
  snprintf(c.sched,  sizeof(c.sched),  "%02u:%02u", h, m);
  strlcpy(c.med, medName, sizeof(c.med));
  c.dose = schedule.valid ? schedule.doseCount : 1;
  c.test = test;
}

/** status: dispensed | confirmed | missed | prealert (prealert = HIGH-risk heads-up notice) */
void queueEvent(const char* status, const DoseCtx& c) {
  if (obCount == OUTBOX_MAX) {                   // full: drop the oldest
    memmove(&outbox[0], &outbox[1], sizeof(OutEvent) * (OUTBOX_MAX - 1));
    obCount--;
  }
  OutEvent& e = outbox[obCount];
  memset(&e, 0, sizeof(e));
  DateTime n = nowLocal();
  strlcpy(e.status, status, sizeof(e.status));
  strlcpy(e.doseId, c.doseId, sizeof(e.doseId));
  strlcpy(e.sched,  c.sched,  sizeof(e.sched));
  strlcpy(e.med,    c.med,    sizeof(e.med));
  hhmmss(n, e.time);
  e.epoch = clockAvailable() ? toUtcEpoch(n) : 0;
  e.dose  = c.dose;
  e.test  = c.test ? 1 : 0;
  snprintf(e.key, sizeof(e.key), "%s_%s", e.doseId, status);
  obCount++;
  outboxSave();
  Serial.printf("[OUTBOX] queued %s (%u waiting)\n", e.key, obCount);
}

bool uploadEvent(const OutEvent& e) {
  FirebaseJson j;
  j.set("time",      e.time);
  j.set("med",       e.med);
  j.set("dose",      (int)e.dose);
  j.set("status",    e.status);
  j.set("timestamp", (double)e.epoch);
  j.set("doseId",    e.doseId);
  j.set("sched",     e.sched);
  if (e.test) j.set("test", true);
  String path = String("/device/history/") + e.key;
  if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &j)) return true;
  Serial.printf("[OUTBOX] upload %s failed: %s\n", e.key, fbdo.errorReason().c_str());
  return false;
}

bool uploadLog() {
  FirebaseJson j;
  j.set("totalDispensed", (int)doseLog.totalDispensed);
  j.set("totalConfirmed", (int)doseLog.totalConfirmed);
  j.set("totalMissed",    (int)doseLog.totalMissed);
  return Firebase.RTDB.setJSON(&fbdo, "/device/log", &j);
}

void taskOutboxFlush() {
  if (!fbOnline()) return;
  if (obCount == 0) {
    if (logDirty && netTurn()) { if (uploadLog()) logDirty = false; }
    return;
  }
  if (millis() < nextFlushAt || !netTurn()) return;
  if (uploadEvent(outbox[0])) {
    memmove(&outbox[0], &outbox[1], sizeof(OutEvent) * (obCount - 1));
    obCount--;
    outboxSave();
    logDirty = true;
    flushBackoff = 2000;
    nextFlushAt = 0;
    requestStatus();
  } else {
    flushBackoff = min(flushBackoff * 2, 60000UL);
    nextFlushAt = millis() + flushBackoff;
  }
}

// ============================================================
//  LCD
// ============================================================
bool lcdDetect() {
  static const uint8_t cands[] = { 0x27, 0x3F, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21, 0x20,
                                   0x3E, 0x3D, 0x3C, 0x3B, 0x3A, 0x39, 0x38 };
  for (uint8_t a : cands) {
    if (!i2cProbe(a)) continue;
    if (lcd == nullptr || a != lcdAddr) {
      delete lcd;
      lcd = new LiquidCrystal_I2C(a, 16, 2);
    }
    lcdAddr = a;
    lcd->init();
    lcd->backlight();
    lcd->clear();
    lcdOk = true;
    lcdShown1[0] = lcdShown2[0] = '\0';
    lastLcdReinit = millis();
    Serial.printf("[LCD] found at 0x%02X\n", a);
    return true;
  }
  lcdOk = false;
  Serial.println(F("[LCD] not found on I2C (check SDA/SCL/5V/GND)"));
  return false;
}

void lcdShow(const char* l1, const char* l2, bool force) {
  if (!lcdOk) return;
  char a[17], b[17];
  snprintf(a, sizeof(a), "%-16.16s", l1);
  snprintf(b, sizeof(b), "%-16.16s", l2);
  if (force || strcmp(a, lcdShown1) != 0) { lcd->setCursor(0, 0); lcd->print(a); strcpy(lcdShown1, a); }
  if (force || strcmp(b, lcdShown2) != 0) { lcd->setCursor(0, 1); lcd->print(b); strcpy(lcdShown2, b); }
}

void lcdBoot(const char* step) { lcdShow("MedSync v4.0", step, true); }

void lcdRender() {
  char l1[17], l2[17];
  switch (systemState) {
    case STATE_IDLE: {
      DateTime n = nowLocal();
      if (preAlertActive) {                      // HIGH-risk countdown: "Dose in 09:41"
        long secs = (long)(schedule.hour * 60 + schedule.minute) * 60 - ((long)n.hour() * 3600 + n.minute() * 60 + n.second());
        if (secs < 0) secs = 0;
        snprintf(l1, sizeof(l1), "Dose in %02ld:%02ld", secs / 60, secs % 60);
      }
      else snprintf(l1, sizeof(l1), "Time%c%02u:%02u:%02u %s",           // "Time:08:30:15 PM"
                    clockTrusted() ? ':' : '?', hour12(n.hour()), n.minute(), n.second(), ampm(n.hour()));
      char net = fbStarted && wifiUp ? (obCount ? '^' : '*') : '!';   // * cloud ok, ^ uploading, ! offline
      if (!schedule.valid) snprintf(l2, sizeof(l2), "No schedule set");
      else snprintf(l2, sizeof(l2), schedule.doseCount >= 10 ? "Dose:%02u:%02u%sx%u"     // keep column 16 free
                                                             : "Dose:%02u:%02u%s x%u",
                    hour12(schedule.hour), schedule.minute, ampm(schedule.hour), schedule.doseCount);
      size_t len = strlen(l2);
      while (len < 15) l2[len++] = ' ';
      l2[15] = net; l2[16] = '\0';
      break;
    }
    case STATE_ALERT:      strcpy(l1, cur.test ? "TEST: Medicine!" : "Medicine Time!"); strcpy(l2, "Press Button 1"); break;
    case STATE_DISPENSING: strcpy(l1, "Take capsule(s)");  strcpy(l2, "Then Button 2");  break;
    case STATE_CONFIRMED:  strcpy(l1, "Dose Confirmed!");  strcpy(l2, "Stay healthy!");  break;
    case STATE_MISSED:     strcpy(l1, "DOSE MISSED!");     strcpy(l2, "Caregiver told"); break;
    case STATE_RTC_ERROR:  strcpy(l1, "Clock error");      strcpy(l2, wifiUp ? "Getting net time" : "Check RTC/WiFi"); break;
    default:               strcpy(l1, "MedSync v4.0");     strcpy(l2, "Starting..."); break;
  }
  bool force = millis() - lastLcdForce > 10000UL;   // rewrite everything every 10 s (heals garbled text)
  if (force) lastLcdForce = millis();
  lcdShow(l1, l2, force);
}

void taskLCD() {
  if (!lcdOk) return;
  if (!lcdDirty && millis() - lastLcdRefresh < LCD_REFRESH_MS) return;
  lastLcdRefresh = millis();
  lcdDirty = false;
  lcdRender();
}

// ============================================================
//  HARDWARE HEALTH (RTC / LCD retry, clock-error state)
// ============================================================
bool rtcStart() {
  if (!rtc.begin(&Wire)) return false;
  if (rtc.lostPower()) {
    Serial.println(F("[RTC] lost power - set to compile time until internet time arrives"));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    rtcTimeSuspect = true;
  }
  return true;
}

void taskHardwareHealth() {
  if (millis() - lastHwRetry >= HW_RETRY_MS) {
    lastHwRetry = millis();
    if (!rtcOk || !lcdOk) {
      bool busStuck = (gpio_get_level((gpio_num_t)PIN_SDA) == 0);
      if (busStuck) { Wire.end(); i2cStart(); }
      if (!rtcOk && (rtcOk = rtcStart())) { Serial.println(F("[RTC] recovered")); lcdDirty = true; }
      if (!lcdOk && lcdDetect()) lcdDirty = true;
    }
  }
  if (lcdOk && systemState == STATE_IDLE && millis() - lastLcdReinit > LCD_REINIT_MS) {
    lastLcdReinit = millis();
    lcd->init(); lcd->backlight();
    lcdShown1[0] = lcdShown2[0] = '\0';
    lcdDirty = true;
  }
  if (systemState == STATE_IDLE && !clockAvailable())      { systemState = STATE_RTC_ERROR; lcdDirty = true; requestStatus(); }
  else if (systemState == STATE_RTC_ERROR && clockAvailable()) { systemState = STATE_IDLE; lcdDirty = true; requestStatus(); }
}

// ============================================================
//  TIME SYNC (non-blocking SNTP)
// ============================================================
void taskClockSync() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!ntpStarted) {
    configTime(0, 0, "pool.ntp.org", "time.google.com", "time.nist.gov");   // system clock in UTC
    ntpStarted = true;
    return;
  }
  time_t t = time(nullptr);
  if (t < 1700000000) return;                    // not synced yet
  if (ntpSynced && millis() - lastNtpSync < NTP_RESYNC_MS) return;
  bool first = !ntpSynced;
  ntpSynced = true;
  lastNtpSync = millis();
  if (rtcOk) { rtc.adjust(DateTime((uint32_t)(t + TZ_OFFSET_SEC))); rtcTimeSuspect = false; }
  if (first) {
    char buf[9]; hhmmss(nowLocal(), buf);
    Serial.printf("[TIME] internet time OK -> %s IST\n", buf);
    lcdDirty = true;
    requestStatus();
  }
}

// ============================================================
//  VOICE (DFPlayer, no handshake)
// ============================================================
void initVoice() {
  Serial2.begin(9600, SERIAL_8N1, PIN_DFP_RX, PIN_DFP_TX);
  dfPlayer.begin(Serial2, false, false);         // no ACK, no reset: returns immediately
}

void taskVoiceSetup() {
  if (dfConfigured || millis() < VOICE_SETTLE_MS) return;
  dfPlayer.setTimeOut(500);
  dfPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  delay(150);
  dfPlayer.volume(VOICE_VOLUME);
  lastVoiceCmd = millis();
  dfConfigured = true;
  Serial.println(F("[VOICE] DFPlayer configured (no-handshake mode)"));
}

void voiceGap() {                                // DFPlayer drops commands sent back-to-back
  unsigned long since = millis() - lastVoiceCmd;
  if (since < 150) delay(150 - since);
  lastVoiceCmd = millis();
}

void voicePlay(uint8_t ev) {
  if (!dfConfigured || ev > 4) return;
  bool kn;
  if (voiceMode == 0)        kn = false;
  else if (voiceMode == 1)   kn = true;
  else if (ev == CLIP_ALERT) kn = (voiceToggle++ & 1);
  else                       kn = voiceLastKn;
  uint8_t clip = kn ? CLIP_KN[ev] : CLIP_EN[ev];
  if (clip == 0) return;
  voiceLastKn = kn;
  voiceGap();
  dfPlayer.playMp3Folder(clip);
}

void alertVoiceStart() { alertVoiceOn = true; lastAlertVoice = millis(); voicePlay(CLIP_ALERT); }
void alertVoiceStop()  { alertVoiceOn = false; if (dfConfigured) { voiceGap(); dfPlayer.stop(); } }
void taskAlertVoice() {
  if (alertVoiceOn && millis() - lastAlertVoice >= VOICE_REPEAT_MS) { lastAlertVoice = millis(); voicePlay(CLIP_ALERT); }
}

// ============================================================
//  SERVO
// ============================================================
void servoRelease() {
  if (dispenserServo.attached()) dispenserServo.detach();
  pinMode(PIN_SERVO, OUTPUT);
  digitalWrite(PIN_SERVO, LOW);
}

void servoDispense(uint8_t count) {
  if (count < 1) count = 1;
  if (!dispenserServo.attached()) dispenserServo.attach(PIN_SERVO, 500, 2400);
  for (uint8_t i = 0; i < count; i++) {
    Serial.printf("[SERVO] capsule %u/%u\n", i + 1, count);
    dispenserServo.write(SERVO_DISPENSE_ANGLE);
    delay(SERVO_MOVE_MS);
    dispenserServo.write(SERVO_HOME_ANGLE);
    delay(SERVO_RETURN_MS);
    wdtFeed();
  }
  servoRelease();
}

// ============================================================
//  BUTTONS (press counted on release after >= 40 ms; stuck-LOW never fires)
// ============================================================
void IRAM_ATTR isrButton1() {
  unsigned long t = millis();
  if (gpio_get_level((gpio_num_t)PIN_BUTTON1) == 0) { btn1DownAt = t; btn1Down = true; }
  else if (btn1Down) { btn1Down = false; if (t - btn1DownAt >= BTN_MIN_PRESS_MS) btn1Isr = true; }
}
void IRAM_ATTR isrButton2() {
  unsigned long t = millis();
  if (gpio_get_level((gpio_num_t)PIN_BUTTON2) == 0) { btn2DownAt = t; btn2Down = true; }
  else if (btn2Down) { btn2Down = false; if (t - btn2DownAt >= BTN_MIN_PRESS_MS) btn2Isr = true; }
}

void taskButtons() {
  if (btn1Isr) { btn1Isr = false; btn1Pressed = true; Serial.printf("[BTN] 1 (%s)\n", stateName(systemState)); }
  if (btn2Isr) { btn2Isr = false; btn2Pressed = true; Serial.printf("[BTN] 2 (%s)\n", stateName(systemState)); }
}

// ============================================================
//  SCHEDULER & STATE MACHINE
// ============================================================
void startDose(bool test) {
  DateTime n = nowLocal();
  makeDoseCtx(cur, n, test ? n.hour() : schedule.hour, test ? n.minute() : schedule.minute, test);
  preAlertActive = false;
  Serial.printf("[DOSE] %s alert %s\n", test ? "TEST" : "scheduled", cur.doseId);
  systemState   = STATE_ALERT;
  alertStartTime = alertArmedAt = millis();
  btn1Isr = btn2Isr = false;
  btn1Pressed = btn2Pressed = false;
  alertVoiceStart();
  lcdDirty = true;
  requestStatus();
}

void taskScheduler() {
  if (systemState != STATE_IDLE || !schedule.valid || !clockAvailable()) return;
  DateTime n = nowLocal();
  uint32_t today = dayKey(n);
  int nowMin   = n.hour() * 60 + n.minute();
  int schedMin = schedule.hour * 60 + schedule.minute;

  // HIGH predicted risk: heads-up at 10 and 5 min before the dose (voice + LCD countdown),
  // and one "prealert" notice to the cloud so the dashboard e-mails caregivers + patient.
  int untilMin = schedMin - nowMin;
  bool inWindow = strcmp(riskLevel, "HIGH") == 0 && lastFiredDay != today &&
                  untilMin > 0 && untilMin <= PRE_ALERT_LEAD_MIN;
  if (inWindow) {
    if (preAlertDay != today) { preAlertDay = today; preAlertStage = 0; }
    if (preAlertStage == 0) {
      preAlertStage = (untilMin <= PRE_ALERT_SECOND_MIN) ? 2 : 1;
      voicePlay(CLIP_EARLY);
      DoseCtx c;
      makeDoseCtx(c, n, schedule.hour, schedule.minute, false);
      queueEvent("prealert", c);
      Serial.printf("[DOSE] HIGH risk - heads-up, dose in %d min\n", untilMin);
    } else if (preAlertStage == 1 && untilMin <= PRE_ALERT_SECOND_MIN) {
      preAlertStage = 2;
      voicePlay(CLIP_EARLY);
    }
  }
  if (inWindow != preAlertActive) { preAlertActive = inWindow; lcdDirty = true; }
  if (lastFiredDay == today) return;
  if (nowMin >= schedMin && nowMin <= schedMin + LATE_FIRE_WINDOW_MIN) {
    lastFiredDay = today;
    saveFiredDay();                              // never fires twice today, even after a reboot
    startDose(false);
  }
}

void finishMissed() {
  doseLog.totalMissed++;
  saveDoseLog();
  voicePlay(CLIP_MISSED);
  queueEvent("missed", cur);
  missedAt    = millis();
  systemState = STATE_MISSED;
  lcdDirty    = true;
  requestStatus();
}

void runStateMachine() {
  switch (systemState) {
    case STATE_ALERT: {
      unsigned long timeout = strcmp(riskLevel, "HIGH") == 0 ? HIGH_RISK_MISSED_TIMEOUT_MS : MISSED_TIMEOUT_MS;
      if (millis() - alertStartTime >= timeout) {
        Serial.println(F("[STATE] no Button 1 -> MISSED"));
        alertVoiceStop();
        finishMissed();
      } else if (btn1Pressed && millis() - alertArmedAt > ALERT_BTN_GUARD_MS) {
        Serial.println(F("[STATE] Button 1 -> dispensing"));
        alertVoiceStop();
        lcdShow("Dispensing...", "Please wait", true);
        servoDispense(cur.dose);
        doseLog.totalDispensed++;
        saveDoseLog();
        voicePlay(CLIP_DISPENSED);
        queueEvent("dispensed", cur);
        dispensedAt = lastConfirmRemind = millis();
        systemState = STATE_DISPENSING;
        lcdDirty    = true;
        requestStatus();
      }
      break;
    }
    case STATE_DISPENSING:
      if (btn2Pressed) {
        Serial.println(F("[STATE] Button 2 -> CONFIRMED"));
        doseLog.totalConfirmed++;
        saveDoseLog();
        voicePlay(CLIP_CONFIRMED);
        queueEvent("confirmed", cur);
        confirmedAt = millis();
        systemState = STATE_CONFIRMED;
        lcdDirty    = true;
        requestStatus();
      } else if (millis() - dispensedAt >= (strcmp(riskLevel, "HIGH") == 0 ? HIGH_RISK_CONFIRM_TIMEOUT_MS : CONFIRM_TIMEOUT_MS)) {
        Serial.println(F("[STATE] dispensed, not confirmed -> MISSED"));
        finishMissed();
      } else if (millis() - lastConfirmRemind >= CONFIRM_REMIND_MS) {
        lastConfirmRemind = millis();            // remind the patient to press Button 2
        voicePlay(CLIP_DISPENSED);
      }
      break;
    case STATE_CONFIRMED:
      if (millis() - confirmedAt >= CONFIRMED_DISPLAY_MS) { systemState = STATE_IDLE; lcdDirty = true; requestStatus(); }
      break;
    case STATE_MISSED:
      if (millis() - missedAt >= MISSED_DISPLAY_MS) { systemState = STATE_IDLE; lcdDirty = true; requestStatus(); }
      break;
    default: break;
  }
  btn1Pressed = btn2Pressed = false;             // presses outside their state are ignored
}

// ============================================================
//  SCHEDULE / SETTINGS FROM THE CLOUD
// ============================================================
void applySchedule(int h, int m, int d, const char* med) {
  if (h < 0 || h > 23 || m < 0 || m > 59) return;          // ignore invalid data
  if (d < 1 || d > 10) d = 1;
  bool timeChanged = !schedule.valid || h != schedule.hour || m != schedule.minute;
  bool medChanged  = med && med[0] && strcmp(med, medName) != 0;
  if (!timeChanged && d == schedule.doseCount && !medChanged) return;   // nothing new

  if (timeChanged && clockAvailable()) {
    // Allow today's alarm again only if the new time is still ahead (never double-dose).
    DateTime n = nowLocal();
    if (h * 60 + m > n.hour() * 60 + n.minute() && lastFiredDay == dayKey(n)) { lastFiredDay = 0; saveFiredDay(); }
    preAlertDay = 0;
  }
  schedule.hour = h; schedule.minute = m; schedule.doseCount = d; schedule.valid = true;
  if (medChanged) strlcpy(medName, med, sizeof(medName));
  saveSchedule();
  Serial.printf("[SYNC] schedule -> %02d:%02d x%d (%s)\n", h, m, d, medName);
  lcdDirty = true;
  requestStatus();
}

void pollSchedule() {
  if (!Firebase.RTDB.getJSON(&fbdo, "/device/schedule")) return;
  FirebaseJson* j = fbdo.jsonObjectPtr();
  if (j == nullptr) return;
  FirebaseJsonData r;
  int h = -1, m = -1, d = schedule.doseCount;
  char med[24] = "";
  if (j->get(r, "hour"))      h = r.to<int>();
  if (j->get(r, "minute"))    m = r.to<int>();
  if (j->get(r, "doseCount")) d = r.to<int>();
  if (j->get(r, "med"))       strlcpy(med, r.to<const char*>(), sizeof(med));
  if (h < 0 || m < 0) return;                    // nothing stored yet
  applySchedule(h, m, d, med);
}

void pollPrediction() {
  if (!Firebase.RTDB.getJSON(&fbdo, "/device/prediction")) return;
  FirebaseJson* j = fbdo.jsonObjectPtr();
  if (j == nullptr) return;
  FirebaseJsonData r;
  if (j->get(r, "riskLevel")) {
    char lv[8]; strlcpy(lv, r.to<const char*>(), sizeof(lv));
    if (!strcmp(lv, "LOW") || !strcmp(lv, "MEDIUM") || !strcmp(lv, "HIGH")) {
      if (strcmp(lv, riskLevel)) Serial.printf("[SYNC] risk -> %s\n", lv);
      strlcpy(riskLevel, lv, sizeof(riskLevel));
    }
  }
}

void pollVoice() {
  if (!Firebase.RTDB.getString(&fbdo, "/device/voice/mode")) return;
  String s = fbdo.stringData();
  uint8_t v = voiceMode;
  if (s == "en") v = 0; else if (s == "kn") v = 1; else if (s == "both") v = 2;
  if (v != voiceMode) { voiceMode = v; saveVoiceMode(); Serial.printf("[SYNC] voice -> %s\n", voiceModeStr()); requestStatus(); }
}

void runCommand(const char* cmd, double atMs) {
  if (clockTrusted() && atMs > 0) {
    double age = (double)toUtcEpoch(nowLocal()) - atMs / 1000.0;
    if (age > COMMAND_MAX_AGE_S || age < -COMMAND_MAX_AGE_S) { strlcpy(lastCmdResult, "stale", sizeof(lastCmdResult)); return; }
  }
  if (!strcmp(cmd, "test_alert")) {
    if (systemState == STATE_IDLE || systemState == STATE_RTC_ERROR) { startDose(true); strlcpy(lastCmdResult, "started", sizeof(lastCmdResult)); }
    else strlcpy(lastCmdResult, "busy", sizeof(lastCmdResult));
  } else if (!strcmp(cmd, "reset_counters")) {
    doseLog = { 0, 0, 0 };
    saveDoseLog();
    strlcpy(lastCmdResult, "done", sizeof(lastCmdResult));
    Serial.println(F("[CMD] counters reset"));
  } else {
    strlcpy(lastCmdResult, "unknown", sizeof(lastCmdResult));
  }
}

void pollCommand() {
  if (!Firebase.RTDB.getJSON(&fbdo, "/device/command")) return;
  FirebaseJson* j = fbdo.jsonObjectPtr();
  if (j == nullptr) return;
  FirebaseJsonData r;
  char id[24] = "", cmd[20] = "";
  double atMs = 0;
  if (j->get(r, "id"))  strlcpy(id,  r.to<const char*>(), sizeof(id));
  if (j->get(r, "cmd")) strlcpy(cmd, r.to<const char*>(), sizeof(cmd));
  if (j->get(r, "at"))  atMs = r.to<double>();
  // First boot after flashing (no remembered command): skip whatever old command is stored,
  // unless the clock is trusted - then the 3-minute age check already filters old ones.
  if (cmdBaselineOnly && (clockTrusted() || !id[0])) cmdBaselineOnly = false;
  if (!id[0] || !strcmp(id, lastCmdId)) return;
  strlcpy(lastCmdId, id, sizeof(lastCmdId));
  prefs.putString("cmd", lastCmdId);
  if (cmdBaselineOnly) { cmdBaselineOnly = false; return; }
  Serial.printf("[CMD] %s (%s)\n", cmd, id);
  runCommand(cmd, atMs);
  requestStatus();
}

void pushStatus() {
  FirebaseJson j;
  DateTime n = nowLocal();
  char t[9]; hhmmss(n, t);
  j.set("state",          stateName(systemState));
  j.set("time",           clockAvailable() ? t : "--:--:--");
  j.set("scheduleHour",   (int)schedule.hour);
  j.set("scheduleMinute", (int)schedule.minute);
  j.set("doseCount",      (int)schedule.doseCount);
  j.set("scheduleValid",  schedule.valid);
  j.set("med",            medName);
  j.set("voiceMode",      voiceModeStr());
  j.set("riskLevel",      riskLevel);
  j.set("clockOk",        clockTrusted());
  j.set("rtcOk",          rtcOk);
  j.set("lcdOk",          lcdOk);
  j.set("outboxPending",  (int)obCount);
  j.set("wifiRSSI",       (int)WiFi.RSSI());
  j.set("fw",             FW_VERSION);
  j.set("resetReason",    resetReason);
  j.set("lastCmdId",      lastCmdId);
  j.set("lastCmdResult",  lastCmdResult);
  j.set("lastSeen/.sv",   "timestamp");          // Firebase server time -> reliable online check
  if (Firebase.RTDB.setJSON(&fbdo, "/device/status", &j)) statusFails = 0;
  else {
    if (statusFails < 250) statusFails++;
    Serial.printf("[SYNC] status push failed (%u, heap %u): %s\n", statusFails, (unsigned)ESP.getFreeHeap(), fbdo.errorReason().c_str());
  }
}

// ============================================================
//  NETWORK TASKS
// ============================================================
void taskWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiUp) {
      wifiUp = true;
      Serial.printf("[WIFI] connected to %s, IP %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
      lcdDirty = true;
      requestStatus();
    }
    return;
  }
  if (wifiUp) { wifiUp = false; lcdDirty = true; Serial.println(F("[WIFI] lost")); }
  if (systemState == STATE_ALERT || systemState == STATE_DISPENSING) return;   // never block a live dose
  if (millis() - lastWifiTry < WIFI_RETRY_MS) return;
  lastWifiTry = millis();
  wifiMulti.run(4000);
}

void taskFirebaseConnect() {
  if (fbStarted || WiFi.status() != WL_CONNECTED) return;
  if (lastFbTry && millis() - lastFbTry < FB_RETRY_MS) return;
  lastFbTry = millis();
  if (strlen(DEVICE_EMAIL) == 0) {
    if (!Firebase.signUp(&config, &auth, "", "")) {
      Serial.printf("[CLOUD] anonymous sign-up failed: %s (retry in 30 s)\n", config.signer.signupError.message.c_str());
      return;
    }
  }
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(false);                 // Wi-Fi is handled by taskWiFi()
  fbdo.setResponseSize(4096);
  Firebase.RTDB.setReadTimeout(&fbdo, FB_READ_TIMEOUT_MS);
  fbStarted = true;
  Serial.println(F("[CLOUD] Firebase started"));
}

void taskCloud() {
  if (!fbStarted || WiFi.status() != WL_CONNECTED) return;
  if (!Firebase.ready()) return;                 // also refreshes the sign-in token
  unsigned long now = millis();
  taskOutboxFlush();                             // 1st priority: dose events
  unsigned long statusEvery = STATUS_PUSH_MS;
  for (uint8_t i = 0; i < statusFails && i < 3; i++) statusEvery *= 2;   // back off while failing
  if ((statusPushPending || now - lastStatusPush >= statusEvery) && netTurn()) {
    lastStatusPush = now; statusPushPending = false; pushStatus();
  }
  if (now - lastSchedPoll >= SCHEDULE_POLL_MS && netTurn()) { lastSchedPoll = now; pollSchedule(); }
  if (now - lastCmdPoll  >= COMMAND_POLL_MS  && netTurn()) { lastCmdPoll  = now; pollCommand(); }
  if (now - lastSlowPoll >= SLOW_POLL_MS     && netTurn()) {
    lastSlowPoll = now;
    if (slowPollTurn++ & 1) pollVoice(); else pollPrediction();
  }
}

// ============================================================
//  SERIAL COMMANDS (for testing without the dashboard)
// ============================================================
void taskSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 't') { if (systemState == STATE_IDLE || systemState == STATE_RTC_ERROR) startDose(true); else Serial.println(F("busy")); }
  else if (c == 'r') { doseLog = { 0, 0, 0 }; saveDoseLog(); Serial.println(F("counters reset")); }
  else if (c == 's') {
    char t[9]; hhmmss(nowLocal(), t);
    Serial.printf("state=%s time=%s clockOk=%d rtc=%d lcd=%d(0x%02X) wifi=%d cloud=%d outbox=%u sched=%02u:%02u x%u valid=%d voice=%s risk=%s D/C/M=%u/%u/%u heap=%u\n",
      stateName(systemState), t, clockTrusted(), rtcOk, lcdOk, lcdAddr, wifiUp, fbStarted && Firebase.ready(), obCount,
      schedule.hour, schedule.minute, schedule.doseCount, schedule.valid, voiceModeStr(), riskLevel,
      doseLog.totalDispensed, doseLog.totalConfirmed, doseLog.totalMissed, (unsigned)ESP.getFreeHeap());
  }
}

// ============================================================
//  UTILITIES
// ============================================================
const char* stateName(SystemState_t s) {
  switch (s) {
    case STATE_BOOT: return "BOOT";       case STATE_IDLE: return "IDLE";
    case STATE_ALERT: return "ALERT";     case STATE_DISPENSING: return "DISPENSING";
    case STATE_CONFIRMED: return "CONFIRMED"; case STATE_MISSED: return "MISSED";
    case STATE_RTC_ERROR: return "RTC_ERROR";
    default: return "UNKNOWN";
  }
}

void readResetReason() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  strcpy(resetReason, "power-on");  break;
    case ESP_RST_BROWNOUT: strcpy(resetReason, "brownout");  break;
    case ESP_RST_TASK_WDT:
    case ESP_RST_INT_WDT:
    case ESP_RST_WDT:      strcpy(resetReason, "watchdog");  break;
    case ESP_RST_PANIC:    strcpy(resetReason, "crash");     break;
    case ESP_RST_SW:       strcpy(resetReason, "software");  break;
    case ESP_RST_EXT:      strcpy(resetReason, "reset-btn"); break;
    default:               strcpy(resetReason, "other");     break;
  }
}

void initWatchdog() {
#if USE_WATCHDOG
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t c;
  memset(&c, 0, sizeof(c));
  c.timeout_ms     = WATCHDOG_TIMEOUT_S * 1000;
  c.idle_core_mask = 0;
  c.trigger_panic  = true;
  if (esp_task_wdt_reconfigure(&c) != ESP_OK) esp_task_wdt_init(&c);
#else
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
#endif
}

// ============================================================
//  SETUP & LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  readResetReason();
  Serial.printf("\n=== MedSync firmware %s  (last restart: %s) ===\n", FW_VERSION, resetReason);

  i2cStart();
  lcdDetect();
  if (!strcmp(resetReason, "brownout")) { lcdShow("Power too weak!", "Use 5V 2A supply", true); delay(2500); }
  lcdBoot("Starting...");

  EEPROM.begin(EEPROM_SIZE);
  prefs.begin("medsync", false);
  loadSchedule(); loadDoseLog(); loadVoiceMode(); loadFiredDay(); outboxLoad();
  prefs.getString("cmd", lastCmdId, sizeof(lastCmdId));
  cmdBaselineOnly = (lastCmdId[0] == '\0');

  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON1), isrButton1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON2), isrButton2, CHANGE);

  initVoice();

  lcdBoot("Clock...");
  rtcOk = rtcStart();
  Serial.println(rtcOk ? F("[RTC] OK") : F("[RTC] not found - will retry and use internet time"));

  dispenserServo.attach(PIN_SERVO, 500, 2400);
  dispenserServo.write(SERVO_HOME_ANGLE);
  delay(400);
  servoRelease();

  lcdBoot("WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  uint8_t nets = 0;
  for (const WifiCred& w : WIFI_LIST) if (w.ssid && w.ssid[0]) { wifiMulti.addAP(w.ssid, w.pass); nets++; }
  if (nets) wifiMulti.run(8000);
  wifiUp = (WiFi.status() == WL_CONNECTED);
  lcdBoot(wifiUp ? "WiFi OK" : "WiFi later...");
  Serial.println(wifiUp ? F("[WIFI] connected") : F("[WIFI] not connected - working offline, retrying in background"));

  config.api_key      = API_KEY;
  config.database_url = DATABASE_URL;
  if (strlen(DEVICE_EMAIL)) { auth.user.email = DEVICE_EMAIL; auth.user.password = DEVICE_PASSWORD; }
  taskFirebaseConnect();
  if (wifiUp) taskClockSync();                   // starts SNTP; finishes in the background

  systemState = clockAvailable() ? STATE_IDLE : STATE_RTC_ERROR;
  lcdDirty = true;
  initWatchdog();
  Serial.println(F("[BOOT] ready  (serial: t=test alert, s=status, r=reset counters)"));
}

void loop() {
  wdtFeed();
  netTurnUsed = false;

  taskButtons();
  taskHardwareHealth();
  taskClockSync();
  taskVoiceSetup();
  taskScheduler();
  taskAlertVoice();
  runStateMachine();
  taskLCD();
  taskSerial();

  taskWiFi();
  taskFirebaseConnect();
  taskCloud();
}
