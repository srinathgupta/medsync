/* MedSync bench test - checks every part on its own. Flash this, open Serial Monitor at 115200.
 *  It runs once automatically, then you can type:
 *    i = I2C scan    l = LCD test    c = RTC clock    v = voice (plays 0001)
 *    m = servo move  a = run all again
 *  Buttons are watched all the time: pressing Button 1 / Button 2 prints a line.
 *  Same wiring and libraries as the main firmware. No Wi-Fi or Firebase needed.
 */
#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <DFRobotDFPlayerMini.h>

#define PIN_BUTTON1 18
#define PIN_BUTTON2 19
#define PIN_DFP_RX  16
#define PIN_DFP_TX  17
#define PIN_SERVO   13
#define PIN_SDA     21
#define PIN_SCL     22

RTC_DS3231 rtc;
LiquidCrystal_I2C* lcd = nullptr;
Servo servo;
DFRobotDFPlayerMini dfp;
uint8_t lcdAddr = 0;
bool lastB1 = HIGH, lastB2 = HIGH;

bool probe(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }

void testI2C() {
  Serial.println(F("\n[I2C] scanning..."));
  Serial.printf("[I2C] SDA line is %s, SCL line is %s (both should be HIGH when idle)\n",
                gpio_get_level((gpio_num_t)PIN_SDA) ? "HIGH" : "LOW (stuck!)",
                gpio_get_level((gpio_num_t)PIN_SCL) ? "HIGH" : "LOW (stuck!)");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) if (probe(a)) {
    n++;
    const char* what = (a >= 0x20 && a <= 0x27) || (a >= 0x38 && a <= 0x3F) ? "LCD backpack"
                     : a == 0x68 ? "DS3231 RTC" : a == 0x57 ? "RTC board EEPROM (normal)" : "?";
    Serial.printf("[I2C]   found 0x%02X  %s\n", a, what);
    if (!lcdAddr && strcmp(what, "LCD backpack") == 0) lcdAddr = a;
  }
  Serial.printf("[I2C] %d device(s). Expect the LCD (0x27 or 0x3F) and 0x68.\n", n);
  if (n == 0) Serial.println(F("[I2C] NOTHING found: check SDA->21, SCL->22, 5V and GND on LCD and RTC."));
}

void testLCD() {
  if (!lcdAddr) { Serial.println(F("[LCD] FAIL - no LCD address found in the scan")); return; }
  if (!lcd) lcd = new LiquidCrystal_I2C(lcdAddr, 16, 2);
  lcd->init(); lcd->backlight(); lcd->clear();
  lcd->setCursor(0, 0); lcd->print("MedSync LCD OK");
  lcd->setCursor(0, 1); lcd->printf("Address 0x%02X", lcdAddr);
  Serial.printf("[LCD] wrote text at 0x%02X. If you see only blocks or nothing, turn the blue contrast screw.\n", lcdAddr);
}

void testRTC() {
  if (!rtc.begin(&Wire)) { Serial.println(F("[RTC] FAIL - not found at 0x68")); return; }
  DateTime n = rtc.now();
  Serial.printf("[RTC] OK  %04d-%02d-%02d %02d:%02d:%02d%s\n", n.year(), n.month(), n.day(), n.hour(), n.minute(), n.second(),
                rtc.lostPower() ? "  (lost power: coin cell missing/flat - main firmware fixes the time from the internet)" : "");
}

void testVoice() {
  dfp.volume(25);
  delay(150);
  dfp.playMp3Folder(1);
  Serial.println(F("[VOICE] sent: play /mp3/0001.mp3 - you should HEAR the English alert now."));
  Serial.println(F("[VOICE] silent? check speaker on SPK_1+SPK_2, card FAT32 with /mp3 folder, DFPlayer TX->16, RX->1k->17."));
}

void testServo() {
  servo.attach(PIN_SERVO, 500, 2400);
  Serial.println(F("[SERVO] 0 -> 90 -> 0 degrees"));
  servo.write(0);  delay(500);
  servo.write(90); delay(700);
  servo.write(0);  delay(600);
  servo.detach();
  Serial.println(F("[SERVO] done. No movement? servo needs 5V (not 3.3V) and common GND; signal on GPIO13."));
}

void runAll() { testI2C(); testLCD(); testRTC(); testVoice(); delay(500); testServo();
  Serial.println(F("\nNow press Button 1 and Button 2. Keys: i l c v m a")); }

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== MedSync bench test ==="));
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);
  Serial2.begin(9600, SERIAL_8N1, PIN_DFP_RX, PIN_DFP_TX);
  delay(2500);                                   // DFPlayer reads its card
  dfp.begin(Serial2, false, false);
  runAll();
}

void loop() {
  bool b1 = digitalRead(PIN_BUTTON1), b2 = digitalRead(PIN_BUTTON2);
  if (b1 != lastB1) { lastB1 = b1; Serial.printf("[BTN] Button 1 %s\n", b1 == LOW ? "PRESSED" : "released"); delay(30); }
  if (b2 != lastB2) { lastB2 = b2; Serial.printf("[BTN] Button 2 %s\n", b2 == LOW ? "PRESSED" : "released"); delay(30); }
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'i') testI2C(); else if (c == 'l') testLCD(); else if (c == 'c') testRTC();
    else if (c == 'v') testVoice(); else if (c == 'm') testServo(); else if (c == 'a') runAll();
  }
}
