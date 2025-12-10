#include <U8g2lib.h>
#include <TimeLib.h>
#include <LittleFS.h>

#define DISPLAY_SCL_PIN 5
#define DISPLAY_SDA_PIN 4
#define REMOTE_PIN 0
#define ATN_PIN 16
#define BTN1_PIN 13
#define BTN2_PIN 12
#define BTN3_PIN 14

#define STATE_WAIT 0
#define STATE_ARM 1
#define STATE_LISTEN 2
#define STATE_IDLE 3

#define FONT_MAIN u8g2_font_profont15_mr
#define FONT_SMALL u8g2_font_profont10_mr

#define WAIT_DEBOUNCE_COUNT 2
#define ARM_DURATION 1300
#define LISTEN_DURATION 18550
#define IDLE_DURATION (6 * 60 * 60 * 1000)
#define IDLE_CLEAR_DELAY (10 * 1000)
#define TIME_OFFSET 1
// #define IDLE_DURATION 0

#define BIT_NONE 126
#define BIT_INVALID 127

#define EPOCH_TO_Y2k (2000 - 1970)
#define EPOCH_2025 1735689600
#define EPOCH_2035 2051222400
#define MAX_DIFF (30 * 60)

#define MS_PER_PIXEL (10000 / 128)

#define BTN_CYCLES 64

#define DAY_TMAX (24 * 60)
#define DAY_INCREMENT 15
#define DAY_START_LB (6 * 60)
#define DAY_START_UB (12 * 60 + DAY_INCREMENT)
#define DAY_END_LB (22 * 60)
#define DAY_END_UB (1 * 60 + DAY_INCREMENT)

#define BA_DH 64
#define BA_DW 128
#define BA_FH 64
#define BA_FW 88
#define BA_ROW_CNT (BA_FH / 8)
#define BA_OFFSET ((BA_DW - BA_FW) / 2)
#define BA_OFFSET_TILE (BA_OFFSET / 8)
#define BA_OFFSET_RES (BA_OFFSET % 8)
#define BA_TILE_CNT ((BA_OFFSET_RES + BA_FW + 7) / 8)

#define BA_CYCLES 4096
#define BA_COUNT 10

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, DISPLAY_SCL_PIN, DISPLAY_SDA_PIN);

uint8_t state = STATE_WAIT;
bool time_set = false;
uint32_t rtc_ms = 0;
uint32_t rtc_s = 0;
tmElements_t tm;
uint32_t last_sync = 0;

uint32_t t0 = 0;
uint32_t t_trigger = 0;
uint16_t tt_last = 0;
bool force_wait = false;

uint8_t cnt[40];
bool first_listen = true;
uint8_t first_bit_offset = 0;
uint8_t cnt0 = 0, cnt1 = 0;
char buf[32];

uint16_t count_nums[9] = { 10, 14, 32, 34, 52, 54, 72, 74, 92 };

uint16_t day_start = (6 * 60 + 45);
uint16_t day_end = (23 * 60 + 30);
bool light_state = false;

bool cleared_osc = false;
uint16_t loop_cnt = 0;
uint8_t last_ms = 0;

uint8_t ba_counter = 0;

void badApple() {
  LittleFS.begin();
  u8g2.clearDisplay();

  File f = LittleFS.open("ba.dat", "r");
  if (!f) {
    Serial.println("ba.dat not found!!");
    LittleFS.end();
    return;
  }
  uint32_t dt = f.read() | (f.read() << 8);
  uint16_t fcount = f.read() | (f.read() << 8);
  uint8_t* fbuf = u8g2.getBufferPtr();
  for (uint16_t t = 0; t < fcount; t++) {
    uint32_t st = micros();
    int i = 0, j = 0;
    uint8_t* fbuf2 = fbuf + BA_OFFSET;
    while (true) {
      uint8_t d = f.read();
      uint8_t l = f.read();
      while (l--) {
        *(fbuf2 + j) = d;
        j++;
        if (j == BA_FW) {
          j = 0;
          i++;
          fbuf2 += BA_DW;
        }
      }
      if (i == BA_ROW_CNT) {
        break;
      }
    }
    u8g2.updateDisplayArea(BA_OFFSET_TILE, 0, BA_TILE_CNT, 8);
    uint32_t delta = micros() - st;
    delayMicroseconds(dt - delta);
  }
  f.close();
  LittleFS.end();
  u8g2.clearDisplay();
}

void initCountTable() {
  Serial.print("cnt0=");
  Serial.print(cnt0);
  Serial.print(", cnt1=");
  Serial.print(cnt1);
  Serial.print("\ninitializing count table\n");
  first_bit_offset = cnt1 - cnt0;
  cnt[0] += first_bit_offset;

  for (int8_t i = 0; i < 9; i++) {
    count_nums[i] = (count_nums[i] * cnt1 + 50) / 100;
  }
  Serial.print("count_nums: ");
  for (int8_t i = 0; i < 9; i++) {
    Serial.print(count_nums[i]);
    Serial.print(',');
  }
  Serial.print('\n');
}

uint32_t pulseToBits(uint8_t count) {
  if (count <= count_nums[0]) {
    return BIT_NONE;
  }
  if (count_nums[1] <= count && count <= count_nums[2]) {
    return 0;
  }
  if (count_nums[3] <= count && count <= count_nums[4]) {
    return 1;
  }
  if (count_nums[5] <= count && count <= count_nums[6]) {
    return 2;
  }
  if (count_nums[7] <= count && count <= count_nums[8]) {
    return 3;
  }
  return BIT_INVALID;
}

uint32_t parity(uint32_t x) {
  x = (x >> 16) ^ (x & 0xffff);
  x = (x >> 8) ^ (x & 0xff);
  x = (x >> 4) ^ (x & 0xf);
  x = (x >> 2) ^ (x & 0b11);
  x = (x >> 1) ^ (x & 0b1);
  return x;
}

uint32_t betterNow() {
  uint32_t t = millis();
  while (t - rtc_ms >= 1000) {
    rtc_s++;
    rtc_ms += 1000;
  }
  return rtc_s;
}

bool isDay(uint8_t hour, uint8_t minute) {
  uint16_t t = (uint16_t)hour * 60 + minute;
  if (day_start <= day_end) {
    return day_start <= t && t < day_end;
  } else {
    return day_start <= t || t < day_end;
  }
}

void setLightState(bool st) {
  if (st != light_state) {
    Serial.print("set state to: ");
    Serial.println(st);
    light_state = st;
    digitalWrite(REMOTE_PIN, LOW);
    delay(500);
    digitalWrite(REMOTE_PIN, HIGH);
  }
}

bool decode() {
  u8g2.setCursor(0, 39);  // tile #4
  for (int8_t i = 1; i <= 37; i += 2) {
    if (pulseToBits(cnt[i]) != BIT_NONE) {
      Serial.println("low signal failed");
      u8g2.print("low signal failed");
      return false;
    }
  }
  for (int8_t i = 0; i <= 36; i += 2) {
    uint16_t tmp = pulseToBits(cnt[i]);
    if (tmp == BIT_NONE || tmp == BIT_INVALID) {
      Serial.println("high signal failed");
      u8g2.print("high signal failed");
      return false;
    }
  }

  /*
  d1:  ss?? hhhh mmmm mm?w wwHP
  d2:    ?D DDDD MMMM YYYY YYYP
  */
  uint32_t d1 = 0, d2 = 0;
  for (int8_t i = 0; i <= 18; i += 2) {
    d1 <<= 2;
    d1 |= pulseToBits(cnt[i]);
  }
  for (int8_t i = 0; i <= 16; i += 2) {
    d2 <<= 2;
    d2 |= pulseToBits(cnt[20 + i]);
  }

  bool p1_failed = false, p2_failed = false;
  if (parity(d1 >> 2) != (d1 & 0b1)) {
    Serial.println("p1 parity failed");
    p1_failed = true;
  }
  if (parity(d2 >> 2) != (d2 & 0b1)) {
    Serial.println("p2 parity failed");
    p2_failed = true;
  }

  if (p1_failed) {
    u8g2.setDrawColor(0);
  }
  for (int8_t i = 19; i >= 0; i--) {
    uint8_t b = (d1 >> i) & 0b1;
    Serial.print(b);
    u8g2.print(b);
  }
  if (p1_failed) {
    u8g2.setDrawColor(1);
  }

  Serial.print(' ');
  u8g2.setCursor(0, 47);  // tile #5

  if (p2_failed) {
    u8g2.setDrawColor(0);
  }
  for (int8_t i = 17; i >= 0; i--) {
    uint8_t b = (d2 >> i) & 0b1;
    Serial.print(b);
    u8g2.print(b);
  }
  if (p2_failed) {
    u8g2.setDrawColor(1);
  }

  Serial.print('\n');

  if (p1_failed || p2_failed) {
    return false;
  }

  tm.Second = (d1 >> 18) * 20;
  tm.Hour = (d1 >> 12) & 0b1111;
  tm.Minute = (d1 >> 6) & 0b111111;
  tm.Wday = (d1 >> 2) & 0b111;
  // tm.Wday = ((d1 >> 2) & 0b111) % 7 + 1;

  if (d1 & 0b10) {
    if (tm.Hour != 12) {
      tm.Hour += 12;
    }
  } else {
    if (tm.Hour == 12) {
      tm.Hour = 0;
    }
  }

  tm.Day = (d2 >> 12) & 0b11111;
  tm.Month = (d2 >> 8) & 0b1111;
  tm.Year = ((d2 >> 2) & 0b111111) + ((d2 >> 1) & 0b1) * 64 + EPOCH_TO_Y2k;

  sprintf(buf, "%02d-%02d-%02d %02d:%02d:%02d %d", tm.Year - EPOCH_TO_Y2k, tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second, tm.Wday);

  Serial.println(buf);
  u8g2.setCursor(0, 31);  // tile #3
  u8g2.print(buf);

  if (tm.Year < 25 + EPOCH_TO_Y2k || tm.Year >= 35 + EPOCH_TO_Y2k) {
    return false;
  }

  uint32_t time = makeTime(tm);
  if (time < EPOCH_2025 || time >= EPOCH_2035) {
    return false;
  }
  if (time_set && abs((int32_t)(time + 19 - betterNow())) > MAX_DIFF) {
    return false;
  }
  rtc_ms = t0;
  rtc_s = time + 1;
  betterNow();
  if (!time_set) {
    time_set = true;
    breakTime(rtc_s, tm);
    light_state = isDay(tm.Hour, tm.Minute);
  }
  last_sync = t0;
  Serial.println("synced!!");
  return true;
}

void clearOsc() {
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 48, 128, 16);
  u8g2.setDrawColor(1);
  u8g2.updateDisplayArea(0, 6, 16, 2);
}

void printTime(uint16_t t) {
  uint16_t hour = t / 60;
  uint16_t minute = t % 60;
  u8g2.print(hour / 10);
  u8g2.print(hour % 10);
  u8g2.print(':');
  u8g2.print(minute / 10);
  u8g2.print(minute % 10);
}

void displayDayTimes() {
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 32, 128, 16);
  u8g2.setDrawColor(1);

  u8g2.setCursor(0, 44);  // tile #4-5
  printTime(day_start);
  u8g2.setCursor(64, 44);
  printTime(day_end);

  u8g2.updateDisplayArea(0, 4, 16, 2);
}

void saveSettings() {
  LittleFS.begin();
  File f = LittleFS.open("settings.dat", "w");
  f.write(day_start & 0xff);
  f.write(day_start >> 8);
  f.write(day_end & 0xff);
  f.write(day_end >> 8);
  f.close();
  LittleFS.end();
}

void btn1Handler() {
  static uint8_t last_state = HIGH;
  uint8_t now_state = digitalRead(BTN1_PIN);
  if (now_state == LOW && last_state == HIGH) {
    day_end = (day_end + DAY_INCREMENT) % DAY_TMAX;
    if (day_end == DAY_END_UB) {
      day_end = DAY_END_LB;
    }
    displayDayTimes();
    saveSettings();
  }
  last_state = now_state;
}

void btn2Handler() {
  static uint8_t last_state = HIGH;
  uint8_t now_state = digitalRead(BTN2_PIN);
  if (now_state == LOW && last_state == HIGH) {
    day_start = (day_start + DAY_INCREMENT) % DAY_TMAX;
    if (day_start == DAY_START_UB) {
      day_start = DAY_START_LB;
    }
    displayDayTimes();
    saveSettings();
  }
  last_state = now_state;
}

void btn3Handler() {
  static uint8_t last_state = HIGH;
  uint8_t now_state = digitalRead(BTN3_PIN);
  if (now_state == LOW && last_state == HIGH) {
    force_wait = true;
    ba_counter++;
  }
  last_state = now_state;
}

void displayState(const char* str) {
  u8g2.setCursor(0, 15);
  u8g2.print(str);
  u8g2.updateDisplayArea(0, 0, 6, 2);
}


void setup() {
  Serial.begin(115200);
  Serial.print("\nauto_remote_v3 - startup..\n");

  LittleFS.begin();
  if (LittleFS.exists("settings.dat")) {
    Serial.println("reading from settings.dat...");
    File f = LittleFS.open("settings.dat", "r");
    if (f.size() < 4) {
      Serial.println("settings.dat invalid!!");
      f.close();
      LittleFS.end();
      Serial.println("creating settings.dat...");
      saveSettings();
    } else {
      day_start = f.read() | ((uint16_t)f.read() << 8);
      day_end = f.read() | ((uint16_t)f.read() << 8);
      f.close();
      Serial.print("start=");
      Serial.print(day_start);
      Serial.print(" end=");
      Serial.println(day_end);
      LittleFS.end();
    }
  } else {
    LittleFS.end();
    Serial.println("creating settings.dat...");
    saveSettings();
  }


  pinMode(REMOTE_PIN, OUTPUT);
  digitalWrite(REMOTE_PIN, HIGH);

  pinMode(ATN_PIN, INPUT);
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);

  u8g2.begin();
  u8g2.setFont(FONT_MAIN);
  u8g2.setFontMode(0);
  u8g2.setDrawColor(1);

  displayState("wait..");
}

void loop() {
  uint32_t t = millis();
  uint8_t v = 0;
  if (state != STATE_IDLE) {
    v = digitalRead(ATN_PIN);
  };
  if (state == STATE_ARM) {
    if (v == HIGH) {
      t0 = t;
      state = STATE_LISTEN;
      tt_last = 0;
      displayState("listen");

      clearOsc();

      memset(cnt, 0, sizeof(cnt));
      cnt[0] = first_bit_offset;
    }
    return;
  } else if (state == STATE_WAIT) {
    // Serial.print("wait..");
    // Serial.println(t - t0);
    if (v == HIGH) {
      t_trigger = t;
    } else {
      if (t - t_trigger >= ARM_DURATION) {
        // Serial.println("ARM!");
        state = STATE_ARM;
        displayState("arm   ");
        return;
      }
    }
  } else if (state == STATE_LISTEN) {
    // Serial.print("listen..");
    // Serial.println(t);
    uint32_t dt = t - t0;
    if (first_listen) {
      if (dt < 500) {
        cnt0++;
      } else if (dt < 1000) {
        cnt1++;
      }
    }
    if (v == HIGH) {
      cnt[dt / 500]++;
    }
    if (dt > LISTEN_DURATION) {
      Serial.print("data: ");
      for (int8_t i = 0; i <= 37; i++) {
        Serial.print(cnt[i]);
        Serial.print(',');
      }
      Serial.print('\n');

      if (first_listen) {
        first_listen = false;
        initCountTable();
      }

      u8g2.setDrawColor(0);
      u8g2.drawBox(0, 24, 128, 24);
      u8g2.setDrawColor(1);
      u8g2.setFont(FONT_SMALL);
      bool success = decode();
      u8g2.setFont(FONT_MAIN);
      u8g2.updateDisplayArea(0, 3, 16, 3);

      t0 = millis();
      tt_last = 20000;
      if (success) {
        state = STATE_IDLE;
        cleared_osc = false;
        u8g2.setCursor(0, 15);
        u8g2.print("  w   ");
        u8g2.drawBox(1, 5, 7, 2);
        u8g2.drawBox(26, 5, 7, 2);
        u8g2.drawPixel(33, 10);
        u8g2.updateDisplayArea(0, 0, 6, 2);
      } else {
        t_trigger = t0;
        state = STATE_WAIT;
        displayState("wait..");
      }
      return;
    }
  } else if (state == STATE_IDLE) {
    if (t - last_sync > IDLE_DURATION || force_wait) {
      force_wait = false;
      t0 = t;
      tt_last = 20000;
      t_trigger = t;
      state = STATE_WAIT;
      displayState("wait..");
      return;
    }
    if (!cleared_osc && t - t0 > IDLE_CLEAR_DELAY) {
      cleared_osc = true;
      u8g2.setDrawColor(0);
      u8g2.drawBox(0, 32, 128, 32);  // tile #4-7
      u8g2.setDrawColor(1);
      displayDayTimes();
      u8g2.sendBuffer();
    }
  }

  if (state == STATE_WAIT || (state == STATE_IDLE && loop_cnt % BTN_CYCLES == 0)) {
    btn1Handler();
    btn2Handler();
    btn3Handler();
  }

  if (state != STATE_IDLE) {
    uint16_t tt = (t - t0) % 20000;
    if (tt < tt_last) {
      clearOsc();
    }
    if (tt < 10000) {
      // 48-55 tile #6
      u8g2.drawPixel(tt / MS_PER_PIXEL, 54 - 5 * v);
      u8g2.updateDisplayArea(0, 6, 16, 1);
    } else {
      // 56-63 tile #7
      u8g2.drawPixel((tt - 10000) / MS_PER_PIXEL, 62 - 5 * v);
      u8g2.updateDisplayArea(0, 7, 16, 1);
    }
    tt_last = tt;
  }

  if (state == STATE_WAIT || state == STATE_IDLE) {
    if (time_set) {
      uint8_t ms = ((millis() - rtc_ms) % 1000) / 10;
      // Serial.println(ms);
      if (state == STATE_WAIT || ms < last_ms) {
        breakTime(betterNow(), tm);
        setLightState(isDay(tm.Hour, tm.Minute));
        sprintf(buf, "%02d:%02d:%02d.%02d", tm.Hour, tm.Minute, tm.Second, ms);
        u8g2.setCursor(48, 15);  // tile #0-1
        u8g2.print(buf);
        u8g2.updateDisplayArea(6, 0, 10, 2);
      }
      last_ms = ms;
    }
    if (ba_counter >= BA_COUNT) {
      ba_counter = 0;
      badApple();
      if (state == STATE_WAIT) {
        t_trigger = t;
      }
    }
    if (loop_cnt % BA_CYCLES == 0) {
      ba_counter = 0;
    }
  }

  loop_cnt++;
}