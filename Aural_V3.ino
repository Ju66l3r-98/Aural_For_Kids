/*
  AURAL ESP32-S3: Locked Lessons Browser + Auto-Lesson Playback + Next/Prev Lesson + Hold-Seek
  + Graphic OLED + TEACH MODE (looping word trainer with animated eyes)
  + RECORDING MODE (Kids AURAL doc): record per-lesson, save/discard, recordings browser, recordings playback
  + Recording playback TEACH button: loop last 5 seconds, L/R to move +/- 5 sec window

  RECORDINGS (UPDATED NAMING):
    Root: /AURAL/recordings/<Lang>/<LessonFolder>/
    Files:
      rec_<LessonFolder>.wav
      rec_<LessonFolder>_02.wav, rec_<LessonFolder>_03.wav, ...

  UI FIXES:
    1) Record Capture UI spacing fixed (no overlap)
    2) Mode letter shows R for Recordings browser + Record Capture + Recording playback
    3) Browser marquee: if selection is held 2s and text is too long -> slow scroll

  Pins (your PCB):
    SD SPI:   CS=GPIO10, MOSI=GPIO11, SCK=GPIO12, MISO=GPIO13
    OLED I2C: SDA=GPIO8,  SCL=GPIO9
    Keys:     Track-1=GPIO1 (Teacher/Exit/Mic), Track-2=GPIO2 (Top/Bottom/Left/Right/OK/Vol+/Vol-)
    I2S AMP:  BCLK=GPIO7, WS=GPIO15, DOUT=GPIO16

  NEW pin (set to your MIC DOUT pin):
    I2S MIC:  DIN=GPIO17 (default here; change if needed)

  Libraries:
    - U8g2
    - audio-tools (AudioTools.h)  (I2SStream)
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <U8g2lib.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include <string.h>
#include <AudioTools.h>
#include "driver/i2s.h"

using namespace audio_tools;

// ---------------------------- PINOUT ----------------------------
static const int SD_CS_PIN   = 10;
static const int SD_MOSI_PIN = 11;
static const int SD_SCK_PIN  = 12;
static const int SD_MISO_PIN = 13;

static const int OLED_SDA_PIN = 8;
static const int OLED_SCL_PIN = 9;

static const int KEYPAD_T1_PIN = 1;
static const int KEYPAD_T2_PIN = 2;

// I2S AMP (TX)
static const int I2S_AMP_BCLK_PIN = 7;
static const int I2S_AMP_WS_PIN   = 15;
static const int I2S_AMP_DOUT_PIN = 16;

// I2S MIC (RX)
static const int I2S_MIC_BCLK_PIN = 4;
static const int I2S_MIC_WS_PIN   = 5;
static const int I2S_MIC_DIN_PIN  = 17;

// ---------------------------- OLED -----------------------------
#define OLED_IS_SH1106  1
static const uint8_t OLED_I2C_ADDR = 0x3C;

#if OLED_IS_SH1106
  U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
#else
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
#endif

// ----------------------- SERIAL / DEBUG ------------------------
#define SERIAL_KEY_DEBUG  0

// ----------------------- ADC / KEYPAD --------------------------
static const int      ADC_BITS           = 12;
static const int      NUM_SAMPLES        = 10;
static const uint32_t DEBOUNCE_MS        = 35;

static const uint32_t REPEAT_DELAY_MS    = 350;
static const uint32_t REPEAT_RATE_MS     = 120;

static const uint32_t LR_LONGPRESS_MS    = 450;
static const uint32_t SEEK_TICK_MS       = 130;

static const uint32_t SEEK_STEP_MS_SLOW  = 1500;
static const uint32_t SEEK_STEP_MS_FAST  = 5000;
static const uint32_t SEEK_RAMP_MS       = 1800;

// Playback speed control (TOP/BOTTOM)
static const int SPEED_MIN_PCT = 50;
static const int SPEED_MAX_PCT = 150;
static const int SPEED_STEP_PCT = 10;

// ---------------------------- SD SPI ---------------------------
SPIClass sdSPI(FSPI);
static const uint32_t SD_SPI_HZ = 20000000UL;

// ---------------------------- PATHS ----------------------------
static const char* LESSONS_ROOT    = "/AURAL/lessons";
static const char* LEXICON_ROOT    = "/AURAL/lexicon";
static const char* RECORDINGS_ROOT = "/AURAL/recordings";

// ---------------------------- RECORDING ------------------------
static const uint32_t REC_SAMPLE_RATE_HZ = 16000;
static const uint16_t REC_BITS_PER_SAMPLE = 16;
static const uint16_t REC_CHANNELS_FILE   = 1;

// I2S mic capture config
static const uint16_t REC_I2S_BITS_PER_SAMPLE = 32;
static const uint16_t REC_I2S_CHANNELS        = 2;
static const int      REC_SHIFT_TO_16 = 16;

// ----------------------------- UI LAYOUT ------------------------
static const int DISP_W = 128;
static const int DISP_H = 64;

static const int NAV_H     = 10;
static const int FOOTER_H  = 10;

static const int LINE_H    = 10;
static const int LIST_TOP  = NAV_H;
static const int LIST_BOT  = DISP_H - FOOTER_H;
static const int LINES     = (LIST_BOT - LIST_TOP) / LINE_H;

static const int SEEK_H    = 6;
static const int SEEK_Y    = DISP_H - SEEK_H;

// ----------------------------- UI DATA --------------------------
struct Entry {
  String name;
  bool isDir;
  bool isScript;
  bool isWav;
};

static String currentPath = "/";
static std::vector<Entry> entries;
static int selIndex = 0;
static int scrollTop = 0;

// Which browser root we are currently browsing
static bool browserIsRecordings = false;

// ---------------------------- KEYS -----------------------------
enum ActionKey : uint8_t {
  AK_NONE = 0,
  AK_UP, AK_DOWN, AK_LEFT, AK_RIGHT, AK_OK,
  AK_TEACHER, AK_EXIT, AK_MIC,
  AK_VOL_UP, AK_VOL_DOWN
};

struct LadderKeyDef {
  const char* name;
  float expected;
  float tol;
  ActionKey action;
};

// Track-1
static LadderKeyDef TRACK1_KEYS[] = {
  { "TEACHER",  0.0f,    30.0f, AK_TEACHER },
  { "EXIT",     330.0f,  80.0f, AK_EXIT    },
  { "MIC",      102.0f,  50.0f, AK_MIC     },
};
static const size_t TRACK1_COUNT = sizeof(TRACK1_KEYS) / sizeof(TRACK1_KEYS[0]);

// Track-2
static LadderKeyDef TRACK2_KEYS[] = {
  { "TOP",      1940.0f, 60.0f, AK_UP       },
  { "BOTTOM",   1230.0f, 60.0f, AK_DOWN     },
  { "LEFT",     330.0f,  70.0f, AK_LEFT     },
  { "RIGHT",    1590.0f, 80.0f, AK_RIGHT    },
  { "OK",       2360.0f, 70.0f, AK_OK       },
  { "VOL+",     101.0f,  50.0f, AK_VOL_UP   },
  { "VOL-",     0.0f,    30.0f, AK_VOL_DOWN },
};
static const size_t TRACK2_COUNT = sizeof(TRACK2_KEYS) / sizeof(TRACK2_KEYS[0]);

struct KeyState {
  ActionKey stableKey = AK_NONE;
  ActionKey lastRawKey = AK_NONE;
  uint32_t  lastChangeMs = 0;

  bool      held = false;
  uint32_t  pressStartMs = 0;
  uint32_t  lastRepeatMs = 0;
};
static KeyState keyState;

// Device toggles
static int  volumePct   = 60;

// Teach mode state (Teach mode active)
static bool teacherMode = false;

// Battery placeholder
static int batteryPct = -1;

// Record capture flag
static bool recordingMode = false;

// ---------------------------- MODES ----------------------------
enum UiMode : uint8_t { UI_BROWSER = 0, UI_PLAYBACK, UI_RECORD_CAPTURE };
static UiMode uiMode = UI_BROWSER;

// Playback type
enum PlaybackType : uint8_t { PB_LESSON = 0, PB_RECORDING_FILE };
static PlaybackType playbackType = PB_LESSON;

// ---------------------------- STATUS ---------------------------
static String statusMsg = "";
static uint32_t statusUntilMs = 0;

static void setStatus(const String &msg, uint32_t ms) {
  statusMsg = msg;
  statusUntilMs = millis() + ms;
}
static void maybeClearStatus() {
  if (statusMsg.length() && millis() > statusUntilMs) {
    statusMsg = "";
    statusUntilMs = 0;
  }
}

static void resetKeyState() {
  keyState = KeyState{};
}

// ------------------------- UTIL: Strings -----------------------
static String trimLine(String s) {
  s.replace("\r", "");
  s.replace("\n", "");
  while (s.length() && isspace((unsigned char)s[0])) s.remove(0,1);
  while (s.length() && isspace((unsigned char)s[s.length()-1])) s.remove(s.length()-1,1);
  return s;
}

static bool endsWithIgnoreCase(const String &s, const char* suffix) {
  String a = s; a.toLowerCase();
  String b = String(suffix); b.toLowerCase();
  return a.endsWith(b);
}

static String withoutExtension(const String &s) {
  int dot = s.lastIndexOf('.');
  if (dot <= 0) return s;
  return s.substring(0, dot);
}

static bool isHiddenName(const String &s) {
  return (s.length() > 0 && s[0] == '.');
}

static String parentPathOf(const String &path) {
  if (path == "/" || path.length() == 0) return "/";
  String p = path;
  if (p.endsWith("/") && p.length() > 1) p.remove(p.length() - 1);
  int slash = p.lastIndexOf('/');
  if (slash <= 0) return "/";
  return p.substring(0, slash);
}

static String joinPath(const String &base, const String &child) {
  if (base == "/") return "/" + child;
  return base + "/" + child;
}

static String leafName(const String &path) {
  String p = path;
  if (p.endsWith("/") && p.length() > 1) p.remove(p.length()-1);
  int slash = p.lastIndexOf('/');
  if (slash >= 0 && slash < (int)p.length()-1) return p.substring(slash+1);
  return p;
}

static String activeBrowserRoot() {
  return browserIsRecordings ? String(RECORDINGS_ROOT) : String(LESSONS_ROOT);
}

static bool isInsideRoot(const String &path, const String &root) {
  if (path == root) return true;
  return path.startsWith(root + "/");
}

// ------------------------- UTIL: ADC READ -----------------------
static float readAveragedAnalog(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return (float)sum / (float)samples;
}

static ActionKey matchLadder(float reading, const LadderKeyDef* keys, size_t count, const char** matchedNameOut) {
  float bestDelta = 1e9f;
  ActionKey best = AK_NONE;
  const char* bestName = nullptr;
  for (size_t i = 0; i < count; i++) {
    float d = fabsf(reading - keys[i].expected);
    if (d <= keys[i].tol && d < bestDelta) {
      bestDelta = d;
      best = keys[i].action;
      bestName = keys[i].name;
    }
  }
  if (matchedNameOut) *matchedNameOut = bestName;
  return best;
}

static ActionKey readRawActionKey(float &t1Raw, float &t2Raw, const char** nameOut) {
  t1Raw = readAveragedAnalog(KEYPAD_T1_PIN, NUM_SAMPLES);
  t2Raw = readAveragedAnalog(KEYPAD_T2_PIN, NUM_SAMPLES);

  const char* n1 = nullptr;
  const char* n2 = nullptr;

  ActionKey k1 = matchLadder(t1Raw, TRACK1_KEYS, TRACK1_COUNT, &n1);
  ActionKey k2 = matchLadder(t2Raw, TRACK2_KEYS, TRACK2_COUNT, &n2);

  if (k1 != AK_NONE) { if (nameOut) *nameOut = n1; return k1; }
  if (k2 != AK_NONE) { if (nameOut) *nameOut = n2; return k2; }

  if (nameOut) *nameOut = nullptr;
  return AK_NONE;
}

static bool isRepeatable(ActionKey k) {
  if (uiMode == UI_PLAYBACK && playbackType == PB_LESSON &&
      (k == AK_LEFT || k == AK_RIGHT || k == AK_UP || k == AK_DOWN)) return false;
  if (uiMode == UI_PLAYBACK && playbackType == PB_RECORDING_FILE &&
      (k == AK_LEFT || k == AK_RIGHT)) return false;

  return (k == AK_UP || k == AK_DOWN || k == AK_LEFT || k == AK_RIGHT);
}

static bool pollKeyEvent(ActionKey &outKey) {
  uint32_t now = millis();

  float t1Raw = 0, t2Raw = 0;
  const char* matchedName = nullptr;
  ActionKey raw = readRawActionKey(t1Raw, t2Raw, &matchedName);

#if SERIAL_KEY_DEBUG
  static uint32_t lastDbg = 0;
  if (now - lastDbg > 120) {
    lastDbg = now;
    Serial.printf("ADC T1=%.1f  T2=%.1f  raw=%u  name=%s\n",
                  t1Raw, t2Raw, (unsigned)raw, matchedName ? matchedName : "-");
  }
#endif

  if (raw != keyState.lastRawKey) {
    keyState.lastRawKey = raw;
    keyState.lastChangeMs = now;
  }

  if ((now - keyState.lastChangeMs) >= DEBOUNCE_MS) {
    if (keyState.stableKey != keyState.lastRawKey) {
      keyState.stableKey = keyState.lastRawKey;

      if (keyState.stableKey != AK_NONE) {
        keyState.held = true;
        keyState.pressStartMs = now;
        keyState.lastRepeatMs = now;

        outKey = keyState.stableKey;
        return true;
      } else {
        keyState.held = false;
      }
    }
  }

  if (keyState.held && keyState.stableKey != AK_NONE && isRepeatable(keyState.stableKey)) {
    uint32_t heldFor = now - keyState.pressStartMs;
    if (heldFor >= REPEAT_DELAY_MS && (now - keyState.lastRepeatMs) >= REPEAT_RATE_MS) {
      keyState.lastRepeatMs = now;
      outKey = keyState.stableKey;
      return true;
    }
  }

  return false;
}

// ============================================================================
// SD LISTING (LOCKED) + ROOT TOGGLE (LESSONS / RECORDINGS)
// ============================================================================
static void loadDirectory(const String &path) {
  entries.clear();
  selIndex = 0;
  scrollTop = 0;

  File dir = SD.open(path.c_str());
  if (!dir || !dir.isDirectory()) return;

  bool insideLessons    = isInsideRoot(path, String(LESSONS_ROOT));
  bool insideRecordings = isInsideRoot(path, String(RECORDINGS_ROOT));

  File f = dir.openNextFile();
  while (f) {
    String name = String(f.name());
    bool isDir = f.isDirectory();

    int slash = name.lastIndexOf('/');
    if (slash >= 0 && slash < (int)name.length() - 1) name = name.substring(slash + 1);

    if (!isHiddenName(name)) {
      if (isDir) {
        entries.push_back({name, true, false, false});
      } else {
        if (insideLessons) {
          if (endsWithIgnoreCase(name, ".txt") && !endsWithIgnoreCase(name, "_original.txt")) {
            entries.push_back({name, false, true, false});
          }
        } else if (insideRecordings) {
          if (endsWithIgnoreCase(name, ".wav")) {
            entries.push_back({name, false, false, true});
          }
        }
      }
    }

    f.close();
    f = dir.openNextFile();
  }
  dir.close();

  std::sort(entries.begin(), entries.end(), [](const Entry &a, const Entry &b) {
    if (a.isDir != b.isDir) return a.isDir > b.isDir;
    String an = a.name; an.toLowerCase();
    String bn = b.name; bn.toLowerCase();
    return an < bn;
  });
}

// ============================================================================
// WAV PLAYER ENGINE
// ============================================================================
struct WavFmt {
  uint16_t audioFormat;
  uint16_t numChannels;
  uint32_t sampleRate;
  uint16_t bitsPerSample;
};

static bool wav_seek_data(File &f, WavFmt &fmt, uint32_t &dataBytes, uint32_t &dataStartPos);

// I2SStream used for BOTH TX and RX by reconfiguring
I2SStream i2s;

enum I2SRole : uint8_t { I2S_NONE=0, I2S_TX, I2S_RX };
static I2SRole   g_i2sRole = I2S_NONE;
static uint32_t  g_i2sRate = 0;
static bool      g_i2sLegacyRxActive = false;

static const i2s_port_t MIC_I2S_PORT = I2S_NUM_0;

static int g_scriptSpeedPct = 100;
static int g_userSpeedPct   = 100;
static int g_speedPercent   = 100;

static const int I2S_DMA_BUF_SIZE  = 2048;
static const int I2S_DMA_BUF_COUNT = 8;

struct ClipState {
  File f;
  WavFmt fmt{};
  uint32_t dataBytes = 0;
  uint32_t dataStart = 0;
  uint32_t remaining = 0;
  uint32_t bytesPlayed = 0;
  bool active = false;
  bool paused = false;
  String token = "";
  String path = "";
};
static ClipState clip;

static int clampSpeed(int p) {
  if (p < SPEED_MIN_PCT) p = SPEED_MIN_PCT;
  if (p > SPEED_MAX_PCT) p = SPEED_MAX_PCT;
  return p;
}

static void recomputeEffectiveSpeed() {
  int eff = (g_scriptSpeedPct * g_userSpeedPct) / 100;
  g_speedPercent = clampSpeed(eff);
}

static String speedLabel() {
  char b[16];
  int whole = g_userSpeedPct / 100;
  int frac  = (g_userSpeedPct % 100) / 10;
  snprintf(b, sizeof(b), "SPD %d.%dx", whole, frac);
  return String(b);
}

static void ensureI2S_TX(uint32_t out_rate_hz) {
  if (g_i2sRole == I2S_TX && g_i2sRate == out_rate_hz) return;

  if (g_i2sRole == I2S_RX) {
    if (g_i2sLegacyRxActive) {
      i2s_driver_uninstall(MIC_I2S_PORT);
      g_i2sLegacyRxActive = false;
    }
  } else if (g_i2sRole != I2S_NONE) {
    i2s.end();
    delay(2);
  }

  auto cfg = i2s.defaultConfig(TX_MODE);
  cfg.sample_rate     = out_rate_hz;
  cfg.channels        = 2;
  cfg.bits_per_sample = 16;
  cfg.i2s_format      = I2S_STD_FORMAT;
  cfg.pin_bck         = I2S_AMP_BCLK_PIN;
  cfg.pin_ws          = I2S_AMP_WS_PIN;
  cfg.pin_data        = I2S_AMP_DOUT_PIN;
  cfg.is_master       = true;
  cfg.buffer_size     = I2S_DMA_BUF_SIZE;
  cfg.buffer_count    = I2S_DMA_BUF_COUNT;

  i2s.begin(cfg);
  g_i2sRole = I2S_TX;
  g_i2sRate = out_rate_hz;
  delay(2);
}

static void ensureI2S_RX(uint32_t in_rate_hz) {
  if (g_i2sRole == I2S_RX && g_i2sRate == in_rate_hz) return;

  if (g_i2sRole == I2S_TX) {
    i2s.end();
    delay(2);
  }

  if (g_i2sLegacyRxActive) {
    i2s_driver_uninstall(MIC_I2S_PORT);
    g_i2sLegacyRxActive = false;
  }

  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = in_rate_hz;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = 0;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = I2S_MIC_BCLK_PIN;
  pins.ws_io_num = I2S_MIC_WS_PIN;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = I2S_MIC_DIN_PIN;

  i2s_driver_install(MIC_I2S_PORT, &cfg, 0, nullptr);
  i2s_set_pin(MIC_I2S_PORT, &pins);
  i2s_zero_dma_buffer(MIC_I2S_PORT);
  g_i2sLegacyRxActive = true;

  g_i2sRole = I2S_RX;
  g_i2sRate = in_rate_hz;
  delay(2);
}

static void retuneI2SForCurrentClip() {
  if (!clip.active || clip.fmt.sampleRate == 0) return;
  uint32_t out_rate = (uint32_t)((clip.fmt.sampleRate * (uint32_t)g_speedPercent) / 100);
  out_rate = constrain(out_rate, 8000U, 96000U);
  ensureI2S_TX(out_rate);
}

static void setSpeedPercent(int p) {
  g_scriptSpeedPct = clampSpeed(p);
  recomputeEffectiveSpeed();
  retuneI2SForCurrentClip();
}

static void setUserSpeedPercent(int p) {
  g_userSpeedPct = clampSpeed(p);
  recomputeEffectiveSpeed();
  retuneI2SForCurrentClip();
}

static void nudgeUserSpeed(int deltaPct) {
  setUserSpeedPercent(g_userSpeedPct + deltaPct);
  setStatus(speedLabel(), 650);
}

static bool wav_seek_data(File &f, WavFmt &fmt, uint32_t &dataBytes, uint32_t &dataStartPos) {
  uint8_t h12[12];
  if (f.read(h12, 12) != 12) return false;
  if (memcmp(h12, "RIFF", 4) || memcmp(h12 + 8, "WAVE", 4)) return false;

  bool haveFmt=false, haveData=false;
  dataBytes = 0;
  dataStartPos = 0;

  while (f.available()) {
    uint8_t sub[8];
    if (f.read(sub, 8) != 8) return false;
    uint32_t sz = (uint32_t)sub[4] | ((uint32_t)sub[5]<<8) | ((uint32_t)sub[6]<<16) | ((uint32_t)sub[7]<<24);

    if (!memcmp(sub, "fmt ", 4)) {
      uint8_t buf[32];
      size_t n = min<size_t>(sz, sizeof(buf));
      if (f.read(buf, n) != n) return false;
      if (sz > n) f.seek((uint32_t)f.position() + (uint32_t)(sz - n));

      fmt.audioFormat   = (uint16_t)buf[0] | ((uint16_t)buf[1]<<8);
      fmt.numChannels   = (uint16_t)buf[2] | ((uint16_t)buf[3]<<8);
      fmt.sampleRate    = (uint32_t)buf[4] | ((uint32_t)buf[5]<<8) | ((uint32_t)buf[6]<<16) | ((uint32_t)buf[7]<<24);
      fmt.bitsPerSample = (uint16_t)buf[14] | ((uint16_t)buf[15]<<8);
      haveFmt = true;
    }
    else if (!memcmp(sub, "data", 4)) {
      haveData = true;
      dataBytes = sz;
      dataStartPos = (uint32_t)f.position();
      break;
    }
    else {
      f.seek((uint32_t)f.position() + sz);
    }
  }

  if (!haveFmt || !haveData) return false;
  if (fmt.audioFormat != 1) return false;
  if (fmt.numChannels != 1) return false;
  if (fmt.bitsPerSample != 16) return false;
  return true;
}

static void stopClip() {
  if (clip.f) clip.f.close();
  clip = ClipState{};
}

static bool startClipFile(const String &path, const String &tokenLabel) {
  stopClip();

  File f = SD.open(path.c_str(), FILE_READ);
  if (!f) return false;

  WavFmt fmt{}; uint32_t dataBytes=0, dataStart=0;
  if (!wav_seek_data(f, fmt, dataBytes, dataStart)) {
    f.close();
    return false;
  }

  uint32_t out_rate = (uint32_t)((fmt.sampleRate * (uint32_t)g_speedPercent) / 100);
  out_rate = constrain(out_rate, 8000U, 96000U);
  ensureI2S_TX(out_rate);

  f.seek(dataStart);

  clip.f = f;
  clip.fmt = fmt;
  clip.dataBytes = dataBytes;
  clip.dataStart = dataStart;
  clip.remaining = dataBytes;
  clip.bytesPlayed = 0;
  clip.active = true;
  clip.paused = false;
  clip.path = path;
  clip.token = tokenLabel;

  return true;
}

static bool serviceClip() {
  if (!clip.active) return false;
  if (clip.paused) return true;

  static uint8_t inBuf[1024];
  static uint8_t outBuf[2048];

  size_t toRead = min<size_t>(sizeof(inBuf), clip.remaining);
  if (toRead == 0) {
    stopClip();
    return false;
  }

  size_t n = clip.f.read(inBuf, toRead);
  if (n == 0) {
    stopClip();
    return false;
  }

  clip.remaining -= n;
  clip.bytesPlayed += n;

  const int16_t* in16 = (const int16_t*)inBuf;
  size_t monoSamples = n / 2;

  int16_t* out16 = (int16_t*)outBuf;
  for (size_t i = 0; i < monoSamples; i++) {
    int32_t s = in16[i];
    s = (s * (int32_t)volumePct) / 100;
    if (s > 32767) s = 32767;
    if (s < -32768) s = -32768;
    int16_t ss = (int16_t)s;
    *out16++ = ss;
    *out16++ = ss;
  }

  size_t outBytes = monoSamples * 4;
  i2s.write(outBuf, outBytes);

  if (clip.remaining == 0) {
    stopClip();
    return false;
  }
  return true;
}

// ============================================================================
// SCRIPT ENGINE (LESSON PLAYBACK)
// ============================================================================
enum EventType : uint8_t { EVT_CLIP, EVT_PAUSE, EVT_SPEED };

struct ScriptEvent {
  EventType type;
  String token;
  uint32_t pauseMs = 0;
  int speedPct = 100;

  int speedAtStart = 100;
  uint32_t durMsEff = 0;
  uint32_t clipSampleRate = 0;
  uint32_t clipDataBytes  = 0;
};

static String scriptPath = "";
static String lexiconBase = "";
static std::vector<ScriptEvent> events;
static std::vector<uint32_t> eventStartMs;
static uint32_t totalLessonMs = 0;
static int eventIndex = 0;

static bool inPause = false;
static uint32_t pauseUntil = 0;

static uint32_t lastPlaybackUiMs = 0;
static const uint32_t PLAYBACK_UI_PERIOD_MS = 120;

// Lesson navigation
static String currentLessonFolderPath = "";
static String currentLangPath = "";
static std::vector<String> lessonFolders;
static int lessonIdx = 0;
static String browserReturnPath = "";
static String browserReturnSel  = "";

// L/R gesture state (listening mode)
static ActionKey lrActiveKey = AK_NONE;
static uint32_t  lrPressMs = 0;
static bool      lrSeeking = false;
static uint32_t  lrLastSeekTickMs = 0;
static ActionKey lastStableKey = AK_NONE;

// TEACH MODE DATA
static std::vector<String> teachWords;
static int teachWordIdx = 0;
static bool teachResumeWasPaused = false;
static uint32_t teachResumeTimeMs = 0;
static bool teachResumeValid = false;

static ActionKey teachLrActiveKey = AK_NONE;
static uint32_t  teachLrPressMs   = 0;
static bool      teachLrSeeking   = false;
static uint32_t  teachLrLastTick  = 0;

// RECORDINGS BROWSER/PLAYBACK
static String recordingsBrowserReturnPath = "";
static String recordingsBrowserReturnSel  = "";
static String currentRecordingFolderPath  = "";
static String currentRecordingFilePath    = "";

// Recording playback loop mode
static bool     recLoopMode = false;
static uint32_t recLoopStartMsEff = 0;
static uint32_t recLoopLenMsEff   = 5000;

// RECORD CAPTURE
enum RecState : uint8_t { REC_IDLE=0, REC_RECORDING, REC_STOPPED };
static RecState recState = REC_IDLE;

static File   recFile;
static String recFilePath = "";
static uint32_t recDataBytes = 0;
static uint32_t recLastLevelPct = 0;
static uint32_t recLastLevelHoldMs = 0;

static uint8_t  recMicChannel = 0;
static bool     recMicChannelKnown = false;

// Playback snapshot for return after recording
static bool     recPrevTeacherMode = false;
static uint32_t recPrevLessonPosMs = 0;
static bool     recPrevLessonPaused = false;

static int      recPrevTeachWordIdx = 0;
static uint32_t recPrevTeachPosMsEff = 0;
static bool     recPrevTeachPaused = false;

// --------------------------- SCRIPT helpers ----------------------
static String lessonsRelative(const String &path) {
  String root = String(LESSONS_ROOT);
  if (!path.startsWith(root)) return "";
  String rest = path.substring(root.length());
  if (rest.startsWith("/")) rest.remove(0,1);
  return rest;
}

static String lexiconBaseForScript(const String &scriptPath_) {
  String folder = parentPathOf(scriptPath_);
  String rel = lessonsRelative(folder);
  if (!rel.length()) return String(LEXICON_ROOT);
  return String(LEXICON_ROOT) + "/" + rel;
}

static bool folderHasScriptTxt(const String &folderPath) {
  String p = folderPath;
  if (p.endsWith("/")) p.remove(p.length()-1);
  String s = p + "/script.txt";
  return SD.exists(s.c_str());
}

static String tokenToWavPath(const String &token) {
  String t = token;
  t.trim();
  if (!t.length()) return "";
  if (!endsWithIgnoreCase(t, ".wav")) t += ".wav";
  return lexiconBase + "/" + t;
}

static bool probeWav(const String &path, uint32_t &sampleRate, uint32_t &dataBytes) {
  File f = SD.open(path.c_str(), FILE_READ);
  if (!f) return false;
  WavFmt fmt{}; uint32_t db=0, ds=0;
  bool ok = wav_seek_data(f, fmt, db, ds);
  f.close();
  if (!ok) return false;
  sampleRate = fmt.sampleRate;
  dataBytes  = db;
  return true;
}

static void clearScript() {
  events.clear();
  eventStartMs.clear();
  totalLessonMs = 0;
  eventIndex = 0;
  inPause = false;
  pauseUntil = 0;
  stopClip();
  teachWords.clear();
  teachWordIdx = 0;
  teachResumeValid = false;

  g_scriptSpeedPct = 100;
  recomputeEffectiveSpeed();
}

static void addEventClip(const String &tok) {
  ScriptEvent e; e.type = EVT_CLIP; e.token = tok;
  events.push_back(e);
}
static void addEventPause(uint32_t ms) {
  ScriptEvent e; e.type = EVT_PAUSE; e.pauseMs = ms; e.durMsEff = ms;
  events.push_back(e);
}
static void addEventSpeed(int pct) {
  ScriptEvent e; e.type = EVT_SPEED; e.speedPct = clampSpeed(pct);
  events.push_back(e);
}

static void parseScriptLineTags(const String &lineIn) {
  String line = lineIn;
  int pos = 0;
  while (true) {
    int open = line.indexOf('[', pos);
    if (open < 0) break;
    int close = line.indexOf(']', open + 1);
    if (close < 0) break;

    String tag = line.substring(open + 1, close);
    tag.trim();

    if (tag.startsWith("clip:")) {
      String tok = tag.substring(5); tok.trim();
      if (tok.length()) addEventClip(tok);
    } else if (tag.startsWith("pause:")) {
      uint32_t ms = (uint32_t) tag.substring(6).toInt();
      if (ms > 0) addEventPause(ms);
    } else if (tag.startsWith("speed:")) {
      int p = tag.substring(6).toInt();
      addEventSpeed(p);
    }

    pos = close + 1;
  }
}

static void computeTimelineMeta() {
  int curSpeed = 100;
  totalLessonMs = 0;
  eventStartMs.assign(events.size(), 0);

  uint32_t acc = 0;
  for (size_t i = 0; i < events.size(); i++) {
    eventStartMs[i] = acc;

    ScriptEvent &e = events[i];
    if (e.type == EVT_SPEED) {
      curSpeed = clampSpeed(e.speedPct);
      e.durMsEff = 0;
      continue;
    }

    if (e.type == EVT_PAUSE) {
      acc += e.durMsEff;
      continue;
    }

    if (e.type == EVT_CLIP) {
      e.speedAtStart = curSpeed;

      String wavPath = tokenToWavPath(e.token);
      uint32_t sr=0, db=0;
      if (!wavPath.length() || !SD.exists(wavPath.c_str()) || !probeWav(wavPath, sr, db) || sr == 0) {
        e.clipSampleRate = sr;
        e.clipDataBytes  = db;
        e.durMsEff = 0;
        continue;
      }

      e.clipSampleRate = sr;
      e.clipDataBytes  = db;

      uint64_t origMs = (uint64_t)db * 1000ULL / (uint64_t)(sr * 2UL);
      uint64_t effMs  = origMs * 100ULL / (uint64_t)max(1, e.speedAtStart);

      e.durMsEff = (uint32_t)min<uint64_t>(effMs, 0xFFFFFFFFULL);
      acc += e.durMsEff;
      continue;
    }
  }

  totalLessonMs = acc;
}

static bool isLineTokenName(const String &tok) {
  String t = tok; t.toLowerCase();
  return t.startsWith("line");
}

static void buildTeachWordList() {
  teachWords.clear();
  auto existsTok = [&](const String &s)->bool {
    for (auto &x : teachWords) if (x == s) return true;
    return false;
  };

  for (auto &e : events) {
    if (e.type != EVT_CLIP) continue;
    String tok = e.token; tok.trim();
    if (!tok.length()) continue;
    if (isLineTokenName(tok)) continue;
    if (!existsTok(tok)) teachWords.push_back(tok);
  }
}

static bool loadScript(const String &fullScriptPath) {
  clearScript();
  scriptPath = fullScriptPath;
  lexiconBase = lexiconBaseForScript(fullScriptPath);

  File f = SD.open(fullScriptPath.c_str(), FILE_READ);
  if (!f) return false;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line = trimLine(line);
    if (!line.length()) continue;
    parseScriptLineTags(line);
  }
  f.close();

  if (events.empty()) return false;
  computeTimelineMeta();
  buildTeachWordList();
  return true;
}

static uint32_t clipPlayedMsEffective() {
  if (!clip.active || clip.fmt.sampleRate == 0) return 0;
  uint64_t origMs = (uint64_t)clip.bytesPlayed * 1000ULL / (uint64_t)(clip.fmt.sampleRate * 2UL);
  uint64_t effMs  = origMs * 100ULL / (uint64_t)max(1, g_speedPercent);
  return (uint32_t)min<uint64_t>(effMs, 0xFFFFFFFFULL);
}

static uint32_t clipTotalMsEffective() {
  if (!clip.active || clip.fmt.sampleRate == 0) return 0;
  uint64_t origMs = (uint64_t)clip.dataBytes * 1000ULL / (uint64_t)(clip.fmt.sampleRate * 2UL);
  uint64_t effMs  = origMs * 100ULL / (uint64_t)max(1, g_speedPercent);
  return (uint32_t)min<uint64_t>(effMs, 0xFFFFFFFFULL);
}

static uint32_t currentLessonPosMs() {
  if (eventIndex < 0 || eventIndex >= (int)events.size()) return 0;
  uint32_t base = eventStartMs.empty() ? 0 : eventStartMs[eventIndex];

  if (inPause) {
    uint32_t remain = (pauseUntil > millis()) ? (pauseUntil - millis()) : 0;
    uint32_t dur = events[eventIndex].durMsEff;
    uint32_t elapsed = (dur > remain) ? (dur - remain) : 0;
    return base + elapsed;
  }

  if (clip.active) return base + clipPlayedMsEffective();
  return base;
}

static bool jumpToLessonTime(uint32_t tMs) {
  if (totalLessonMs == 0 || events.empty()) return false;
  if (tMs > totalLessonMs) tMs = totalLessonMs;

  stopClip();
  inPause = false;
  pauseUntil = 0;

  uint32_t acc = 0;
  int curSpeed = 100;

  for (int i = 0; i < (int)events.size(); i++) {
    ScriptEvent &e = events[i];

    if (e.type == EVT_SPEED) { curSpeed = clampSpeed(e.speedPct); continue; }

    uint32_t dur = e.durMsEff;
    if (dur == 0) continue;

    if (tMs <= acc + dur) {
      eventIndex = i;

      if (e.type == EVT_PAUSE) {
        uint32_t offset = (tMs > acc) ? (tMs - acc) : 0;
        uint32_t remain = (dur > offset) ? (dur - offset) : 0;
        inPause = true;
        pauseUntil = millis() + remain;
        return true;
      }

      if (e.type == EVT_CLIP) {
        setSpeedPercent(curSpeed);

        String wavPath = tokenToWavPath(e.token);
        if (!wavPath.length() || !SD.exists(wavPath.c_str())) { setStatus("MISSING", 600); return false; }
        if (!startClipFile(wavPath, e.token)) { setStatus("BAD WAV", 600); return false; }

        uint32_t offsetEff = (tMs > acc) ? (tMs - acc) : 0;
        float frac = (dur > 0) ? (float)offsetEff / (float)dur : 0.0f;
        frac = constrain(frac, 0.0f, 1.0f);

        uint32_t byteOff = (uint32_t)((double)clip.dataBytes * (double)frac);
        byteOff &= ~1UL;

        uint32_t absPos = clip.dataStart + byteOff;
        clip.f.seek(absPos);

        clip.bytesPlayed = byteOff;
        clip.remaining   = (clip.dataBytes > byteOff) ? (clip.dataBytes - byteOff) : 0;
        return true;
      }
    }

    acc += dur;
    if (acc >= totalLessonMs) break;
  }

  eventIndex = (int)events.size() - 1;
  return false;
}

static void startCurrentEventOrAdvance() {
  while (eventIndex < (int)events.size()) {
    ScriptEvent &e = events[eventIndex];

    if (e.type == EVT_SPEED) { setSpeedPercent(e.speedPct); eventIndex++; continue; }

    if (e.type == EVT_PAUSE) { inPause = true; pauseUntil = millis() + e.pauseMs; return; }

    if (e.type == EVT_CLIP) {
      String wavPath = tokenToWavPath(e.token);
      if (!wavPath.length() || !SD.exists(wavPath.c_str())) { setStatus("MISSING", 600); eventIndex++; continue; }
      if (!startClipFile(wavPath, e.token)) { setStatus("BAD WAV", 600); eventIndex++; continue; }
      return;
    }

    eventIndex++;
  }

  setStatus("END", 900);
}

static void togglePauseResume() {
  if (inPause) {
    inPause = false;
    pauseUntil = 0;
    eventIndex++;
    startCurrentEventOrAdvance();
    return;
  }
  if (clip.active) {
    clip.paused = !clip.paused;
    setStatus(clip.paused ? "PAUSE" : "PLAY", 250);
  }
}

// --------------------------- LESSON NEXT/PREV -------------------
static int lessonNumberKey(const String &folderName) {
  if (folderName.length() < 2) return 999999;
  if (folderName[0] != 'L' && folderName[0] != 'l') return 999999;
  int val = 0;
  int i = 1;
  int any = 0;
  while (i < (int)folderName.length() && isdigit((unsigned char)folderName[i]) && any < 6) {
    val = val * 10 + (folderName[i] - '0');
    i++; any++;
  }
  if (any == 0) return 999999;
  return val;
}

static void refreshLessonFolderList() {
  lessonFolders.clear();
  lessonIdx = 0;

  if (!currentLangPath.length()) return;
  File dir = SD.open(currentLangPath.c_str());
  if (!dir || !dir.isDirectory()) return;

  File f = dir.openNextFile();
  while (f) {
    if (f.isDirectory()) {
      String full = String(f.name());
      int slash = full.lastIndexOf('/');
      String leaf = (slash >= 0) ? full.substring(slash + 1) : full;

      String p = joinPath(currentLangPath, leaf);
      if (folderHasScriptTxt(p)) lessonFolders.push_back(leaf);
    }
    f.close();
    f = dir.openNextFile();
  }
  dir.close();

  std::sort(lessonFolders.begin(), lessonFolders.end(), [](const String &a, const String &b) {
    int ka = lessonNumberKey(a);
    int kb = lessonNumberKey(b);
    if (ka != kb) return ka < kb;
    String an=a; an.toLowerCase();
    String bn=b; bn.toLowerCase();
    return an < bn;
  });

  String curLeaf = leafName(currentLessonFolderPath);
  for (int i = 0; i < (int)lessonFolders.size(); i++) {
    if (lessonFolders[i] == curLeaf) { lessonIdx = i; break; }
  }
}

static bool startLessonByFolder(const String &lessonFolderPath) {
  String folder = lessonFolderPath;
  if (folder.endsWith("/")) folder.remove(folder.length()-1);

  String scr = folder + "/script.txt";
  if (!SD.exists(scr.c_str())) return false;

  currentLessonFolderPath = folder;
  currentLangPath = parentPathOf(folder);

  if (!loadScript(scr)) return false;

  uiMode = UI_PLAYBACK;
  playbackType = PB_LESSON;
  setStatus("START", 250);

  teacherMode = false;
  teachResumeValid = false;

  lrActiveKey = AK_NONE;
  lrSeeking = false;
  lrLastSeekTickMs = 0;
  lastStableKey = AK_NONE;

  eventIndex = 0;
  inPause = false;
  pauseUntil = 0;

  setSpeedPercent(100);
  refreshLessonFolderList();
  startCurrentEventOrAdvance();
  return true;
}

static void startNextLesson() {
  if (lessonFolders.empty()) refreshLessonFolderList();
  if (lessonFolders.empty()) { setStatus("NO LESSONS", 700); return; }
  if (lessonIdx + 1 >= (int)lessonFolders.size()) { setStatus("NO NEXT", 600); return; }

  lessonIdx++;
  String folder = joinPath(currentLangPath, lessonFolders[lessonIdx]);
  if (!startLessonByFolder(folder)) setStatus("START FAIL", 700);
}

static void startPrevLesson() {
  if (lessonFolders.empty()) refreshLessonFolderList();
  if (lessonFolders.empty()) { setStatus("NO LESSONS", 700); return; }
  if (lessonIdx - 1 < 0) { setStatus("NO PREV", 600); return; }

  lessonIdx--;
  String folder = joinPath(currentLangPath, lessonFolders[lessonIdx]);
  if (!startLessonByFolder(folder)) setStatus("START FAIL", 700);
}

// ============================================================================
// TEACH MODE ENGINE (LESSON)
// ============================================================================
static void restartCurrentClipFromStart(bool forcePlay) {
  if (!clip.active) return;
  clip.f.seek(clip.dataStart);
  clip.bytesPlayed = 0;
  clip.remaining   = clip.dataBytes;
  if (forcePlay) clip.paused = false;
}

static int findNearestLastWordIndex(String &pickedTok) {
  pickedTok = "";

  if (clip.active) {
    String ct = clip.token; ct.trim();
    if (ct.length() && !isLineTokenName(ct)) pickedTok = ct;
  }

  if (!pickedTok.length() && !events.empty()) {
    int i = constrain(eventIndex, 0, (int)events.size() - 1);
    for (; i >= 0; --i) {
      if (events[i].type == EVT_CLIP) {
        String tok = events[i].token; tok.trim();
        if (tok.length() && !isLineTokenName(tok)) { pickedTok = tok; break; }
      }
    }
  }

  if (teachWords.empty()) return -1;
  if (!pickedTok.length()) return 0;

  for (int i = 0; i < (int)teachWords.size(); i++) {
    if (teachWords[i] == pickedTok) return i;
  }
  return 0;
}

static bool startTeachWordByIndex(int idx) {
  if (teachWords.empty()) return false;
  idx = constrain(idx, 0, (int)teachWords.size() - 1);

  const int N = (int)teachWords.size();
  for (int tries = 0; tries < N; tries++) {
    int j = (idx + tries) % N;
    String tok = teachWords[j];
    String wavPath = tokenToWavPath(tok);
    if (!wavPath.length() || !SD.exists(wavPath.c_str())) continue;

    teachWordIdx = j;
    if (startClipFile(wavPath, tok)) {
      restartCurrentClipFromStart(true);
      return true;
    }
  }
  return false;
}

static void seekTeachToEffMs(uint32_t targetEffMs) {
  if (!clip.active || clip.fmt.sampleRate == 0) return;
  uint32_t totalEff = clipTotalMsEffective();
  if (targetEffMs > totalEff) targetEffMs = totalEff;

  uint64_t speed = (uint64_t)max(1, g_speedPercent);
  uint64_t origMs = (uint64_t)targetEffMs * speed / 100ULL;
  uint64_t byteOff = origMs * (uint64_t)clip.fmt.sampleRate * 2ULL / 1000ULL;
  if (byteOff > clip.dataBytes) byteOff = clip.dataBytes;
  uint32_t off = (uint32_t)byteOff;
  off &= ~1UL;

  clip.f.seek(clip.dataStart + off);
  clip.bytesPlayed = off;
  clip.remaining   = (clip.dataBytes > off) ? (clip.dataBytes - off) : 0;
}

static void enterTeachMode() {
  if (uiMode != UI_PLAYBACK || playbackType != PB_LESSON || events.empty() || !scriptPath.length()) {
    setStatus("PLAY FIRST", 700);
    return;
  }
  if (teachWords.empty()) buildTeachWordList();
  if (teachWords.empty()) {
    setStatus("NO WORDS", 800);
    return;
  }

  teachResumeTimeMs = currentLessonPosMs();
  teachResumeWasPaused = (inPause || (clip.active && clip.paused));
  teachResumeValid = true;

  String picked;
  int idx = findNearestLastWordIndex(picked);
  if (idx < 0) idx = 0;

  stopClip();
  inPause = false;
  pauseUntil = 0;

  setSpeedPercent(100);

  teacherMode = true;
  teachLrActiveKey = AK_NONE;
  teachLrSeeking = false;
  teachLrLastTick = 0;

  if (!startTeachWordByIndex(idx)) {
    teacherMode = false;
    setStatus("MISSING", 900);
  } else {
    char b[20];
    snprintf(b, sizeof(b), "WORD %d/%d", teachWordIdx + 1, (int)teachWords.size());
    setStatus(String(b), 650);
  }
}

static void exitTeachMode() {
  if (!teacherMode) return;

  teacherMode = false;
  stopClip();

  if (teachResumeValid && !events.empty() && totalLessonMs > 0) {
    jumpToLessonTime(teachResumeTimeMs);
    if (teachResumeWasPaused && clip.active) clip.paused = true;
  }
  setStatus("BACK", 250);
}

static void restartTeachLoop() {
  if (!teacherMode) return;
  if (!clip.active) {
    if (!startTeachWordByIndex(teachWordIdx)) { setStatus("MISSING", 700); return; }
  }
  restartCurrentClipFromStart(true);
  setStatus("RESTART", 250);
}

static void stepTeachWord(int dir) {
  if (!teacherMode || teachWords.empty()) return;
  int n = (int)teachWords.size();
  int next = teachWordIdx + dir;
  if (next < 0) next = n - 1;
  if (next >= n) next = 0;

  if (startTeachWordByIndex(next)) {
    char b[20];
    snprintf(b, sizeof(b), "WORD %d/%d", teachWordIdx + 1, n);
    setStatus(String(b), 650);
  } else {
    setStatus("MISSING", 650);
  }
}

static void seekTeachByEffMs(int32_t deltaEffMs) {
  if (!teacherMode || !clip.active || clip.fmt.sampleRate == 0) return;

  int64_t sr = (int64_t)clip.fmt.sampleRate;
  int64_t speed = (int64_t)max(1, g_speedPercent);

  int64_t origMs = ((int64_t)deltaEffMs * speed) / 100;
  int64_t dBytes = (origMs * sr * 2) / 1000;

  int64_t cur = (int64_t)clip.bytesPlayed;
  int64_t target = cur + dBytes;

  if (target < 0) target = 0;
  if (target > (int64_t)clip.dataBytes) target = (int64_t)clip.dataBytes;

  target &= ~1LL;

  uint32_t newOff = (uint32_t)target;
  clip.f.seek(clip.dataStart + newOff);
  clip.bytesPlayed = newOff;
  clip.remaining   = (clip.dataBytes > newOff) ? (clip.dataBytes - newOff) : 0;
}

static void serviceTeachLrGesture() {
  ActionKey cur = keyState.stableKey;
  uint32_t now = millis();

  if ((cur == AK_LEFT || cur == AK_RIGHT) && cur != teachLrActiveKey) {
    teachLrActiveKey = cur;
    teachLrPressMs = now;
    teachLrSeeking = false;
    teachLrLastTick = 0;
  }

  if ((cur == AK_LEFT || cur == AK_RIGHT) && keyState.held && teachLrActiveKey == cur) {
    uint32_t heldFor = now - teachLrPressMs;
    if (heldFor >= LR_LONGPRESS_MS) {
      teachLrSeeking = true;

      if (teachLrLastTick == 0 || (now - teachLrLastTick) >= SEEK_TICK_MS) {
        teachLrLastTick = now;

        uint32_t step = (heldFor >= SEEK_RAMP_MS) ? SEEK_STEP_MS_FAST : SEEK_STEP_MS_SLOW;
        int32_t delta = (cur == AK_RIGHT) ? (int32_t)step : -(int32_t)step;

        seekTeachByEffMs(delta);
        setStatus(cur == AK_RIGHT ? ">>" : "<<", 120);
      }
    }
  }

  if (cur == AK_NONE && (teachLrActiveKey == AK_LEFT || teachLrActiveKey == AK_RIGHT)) {
    if (!teachLrSeeking) {
      if (teachLrActiveKey == AK_RIGHT) stepTeachWord(+1);
      else                              stepTeachWord(-1);
    }
    teachLrActiveKey = AK_NONE;
    teachLrSeeking = false;
    teachLrLastTick = 0;
  }
}

// ============================================================================
// RECORD CAPTURE ENGINE
// ============================================================================
static bool mkdirs(const String &path) {
  if (path.length() == 0) return false;
  if (SD.exists(path.c_str())) return true;

  String p = path;
  if (!p.startsWith("/")) p = "/" + p;

  String build = "";
  int start = 1;
  while (start < (int)p.length()) {
    int slash = p.indexOf('/', start);
    if (slash < 0) slash = p.length();
    String part = p.substring(start, slash);
    if (part.length()) {
      build += "/" + part;
      if (!SD.exists(build.c_str())) {
        if (!SD.mkdir(build.c_str())) return false;
      }
    }
    start = slash + 1;
  }
  return true;
}

// NEW: short recording naming index resolver
// returns 0 => use base "rec_<lesson>.wav", otherwise returns suffix number (>=2) to use "rec_<lesson>_NN.wav"
static int nextRecordingIndexShort(const String &folder, const String &lessonFolderLeaf) {
  String lesson = lessonFolderLeaf; lesson.toLowerCase();

  String base = "rec_" + lesson + ".wav";
  bool baseExists = false;
  int maxSuffix = 0;

  File dir = SD.open(folder.c_str());
  if (!dir || !dir.isDirectory()) return 0;

  File f = dir.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      String nm = String(f.name());
      int slash = nm.lastIndexOf('/');
      if (slash >= 0) nm = nm.substring(slash + 1);

      String low = nm; low.toLowerCase();

      if (low == base) {
        baseExists = true;
      } else {
        String prefix = "rec_" + lesson + "_";    // rec_<lesson>_
        if (low.startsWith(prefix) && low.endsWith(".wav")) {
          String tail = low.substring(prefix.length());  // "nn.wav"
          int dot = tail.indexOf('.');
          if (dot > 0) {
            String num = tail.substring(0, dot);
            int v = num.toInt();
            if (v > maxSuffix) maxSuffix = v;
          }
        }
      }
    }
    f.close();
    f = dir.openNextFile();
  }
  dir.close();

  if (!baseExists && maxSuffix == 0) return 0;
  if (maxSuffix < 2) return 2;
  return maxSuffix + 1;
}

static void writeWavHeader(File &f, uint32_t sampleRate, uint16_t bitsPerSample, uint16_t channels, uint32_t dataBytes) {
  uint32_t byteRate = sampleRate * channels * (bitsPerSample / 8);
  uint16_t blockAlign = channels * (bitsPerSample / 8);
  uint32_t riffSize = 36 + dataBytes;

  uint8_t h[44];
  memset(h, 0, sizeof(h));

  memcpy(h + 0,  "RIFF", 4);
  h[4]  = (uint8_t)(riffSize & 0xFF);
  h[5]  = (uint8_t)((riffSize >> 8) & 0xFF);
  h[6]  = (uint8_t)((riffSize >> 16) & 0xFF);
  h[7]  = (uint8_t)((riffSize >> 24) & 0xFF);
  memcpy(h + 8,  "WAVE", 4);

  memcpy(h + 12, "fmt ", 4);
  h[16] = 16;
  h[20] = 1;
  h[22] = (uint8_t)(channels & 0xFF);
  h[23] = (uint8_t)((channels >> 8) & 0xFF);

  h[24] = (uint8_t)(sampleRate & 0xFF);
  h[25] = (uint8_t)((sampleRate >> 8) & 0xFF);
  h[26] = (uint8_t)((sampleRate >> 16) & 0xFF);
  h[27] = (uint8_t)((sampleRate >> 24) & 0xFF);

  h[28] = (uint8_t)(byteRate & 0xFF);
  h[29] = (uint8_t)((byteRate >> 8) & 0xFF);
  h[30] = (uint8_t)((byteRate >> 16) & 0xFF);
  h[31] = (uint8_t)((byteRate >> 24) & 0xFF);

  h[32] = (uint8_t)(blockAlign & 0xFF);
  h[33] = (uint8_t)((blockAlign >> 8) & 0xFF);

  h[34] = (uint8_t)(bitsPerSample & 0xFF);
  h[35] = (uint8_t)((bitsPerSample >> 8) & 0xFF);

  memcpy(h + 36, "data", 4);
  h[40] = (uint8_t)(dataBytes & 0xFF);
  h[41] = (uint8_t)((dataBytes >> 8) & 0xFF);
  h[42] = (uint8_t)((dataBytes >> 16) & 0xFF);
  h[43] = (uint8_t)((dataBytes >> 24) & 0xFF);

  f.seek(0);
  f.write(h, sizeof(h));
  f.flush();
}

static String currentLessonLeafSafe() {
  String leaf = leafName(currentLessonFolderPath);
  if (!leaf.length()) leaf = "Lesson";
  return leaf;
}

static String currentLangLeafSafe() {
  String leaf = leafName(currentLangPath);
  if (!leaf.length()) leaf = "Lang";
  return leaf;
}

static bool openNewRecordingFile() {
  if (!currentLessonFolderPath.length() || !currentLangPath.length()) return false;

  String lang   = currentLangLeafSafe();
  String lesson = currentLessonLeafSafe();

  String folder = String(RECORDINGS_ROOT) + "/" + lang + "/" + lesson;
  if (!mkdirs(folder)) return false;

  int idx = nextRecordingIndexShort(folder, lesson);

  // Build filename (short)
  String fn;
  if (idx == 0) {
    fn = "rec_" + lesson + ".wav";
  } else {
    char suf[8];
    snprintf(suf, sizeof(suf), "_%02d", idx);
    fn = "rec_" + lesson + String(suf) + ".wav";
  }

  recFilePath = folder + "/" + fn;

  recFile = SD.open(recFilePath.c_str(), FILE_WRITE);
  if (!recFile) return false;

  uint8_t z[44] = {0};
  recFile.write(z, sizeof(z));
  recFile.flush();

  recDataBytes = 0;
  recLastLevelPct = 0;
  recLastLevelHoldMs = millis();
  recMicChannelKnown = false;

  return true;
}

static void closeRecordingFileFinalize(bool keepFile) {
  if (recFile) {
    writeWavHeader(recFile, REC_SAMPLE_RATE_HZ, REC_BITS_PER_SAMPLE, REC_CHANNELS_FILE, recDataBytes);
    recFile.close();
  }

  if (!keepFile && recFilePath.length()) {
    SD.remove(recFilePath.c_str());
  }
}

static uint32_t recDurationMs() {
  uint64_t samples = (recDataBytes / 2);
  uint64_t ms = samples * 1000ULL / (uint64_t)REC_SAMPLE_RATE_HZ;
  return (uint32_t)min<uint64_t>(ms, 0xFFFFFFFFULL);
}

static void updateLevelHold(uint32_t lvlPct) {
  uint32_t now = millis();
  if (lvlPct >= recLastLevelPct) {
    recLastLevelPct = lvlPct;
    recLastLevelHoldMs = now;
  } else {
    if (now - recLastLevelHoldMs > 140) {
      if (recLastLevelPct > 2) recLastLevelPct -= 2;
      else recLastLevelPct = lvlPct;
      recLastLevelHoldMs = now;
    }
  }
}

static void serviceRecordingCapture() {
  if (recState != REC_RECORDING) return;
  if (!recFile) return;

  ensureI2S_RX(REC_SAMPLE_RATE_HZ);

  static uint8_t raw[2048];
  static int16_t mono[512];

  size_t got = 0;
  i2s_read(MIC_I2S_PORT, raw, sizeof(raw), &got, pdMS_TO_TICKS(20));
  if (got == 0) return;

  int frames = (int)(got / 8);
  if (frames <= 0) return;
  if (frames > (int)(sizeof(mono)/sizeof(mono[0]))) frames = (int)(sizeof(mono)/sizeof(mono[0]));

  const int32_t* p = (const int32_t*)raw;

  if (!recMicChannelKnown) {
    int64_t sumL=0, sumR=0;
    int testFrames = min(frames, 64);
    for (int i=0;i<testFrames;i++) {
      int32_t l = p[i*2 + 0];
      int32_t r = p[i*2 + 1];
      sumL += llabs((int64_t)l);
      sumR += llabs((int64_t)r);
    }
    recMicChannel = (sumR > sumL) ? 1 : 0;
    recMicChannelKnown = true;
  }

  int32_t peak = 0;
  for (int i=0;i<frames;i++) {
    int32_t s32 = p[i*2 + recMicChannel];
    int32_t s16 = (s32 >> REC_SHIFT_TO_16);
    if (s16 > 32767) s16 = 32767;
    if (s16 < -32768) s16 = -32768;
    mono[i] = (int16_t)s16;

    int32_t a = (s16 < 0) ? -s16 : s16;
    if (a > peak) peak = a;
  }

  size_t bytesToWrite = (size_t)frames * 2;
  size_t written = recFile.write((const uint8_t*)mono, bytesToWrite);
  if (written == bytesToWrite) {
    recDataBytes += (uint32_t)written;
  }

  uint32_t lvlPct = (uint32_t)((peak * 100) / 32768);
  if (lvlPct > 100) lvlPct = 100;
  updateLevelHold(lvlPct);
}

// ============================================================================
// RECORDINGS PLAYBACK (looped 5 sec window service)
// ============================================================================
static bool serviceClipLoopWindow(uint32_t loopStartMsEff, uint32_t loopLenMsEff) {
  if (!clip.active) return false;
  if (clip.paused) return true;

  uint32_t totalEff = clipTotalMsEffective();
  if (totalEff == 0) return true;

  uint32_t startEff = (loopStartMsEff > totalEff) ? totalEff : loopStartMsEff;
  uint32_t endEff   = startEff + loopLenMsEff;
  if (endEff > totalEff) endEff = totalEff;

  auto effMsToByteOff = [&](uint32_t effMs)->uint32_t {
    uint64_t speed = (uint64_t)max(1, g_speedPercent);
    uint64_t origMs = (uint64_t)effMs * speed / 100ULL;
    uint64_t off = origMs * (uint64_t)clip.fmt.sampleRate * 2ULL / 1000ULL;
    if (off > clip.dataBytes) off = clip.dataBytes;
    uint32_t o = (uint32_t)off;
    o &= ~1UL;
    return o;
  };

  uint32_t startByte = effMsToByteOff(startEff);
  uint32_t endByte   = effMsToByteOff(endEff);
  if (endByte <= startByte) endByte = min<uint32_t>(clip.dataBytes, startByte + 2);

  if (clip.bytesPlayed >= endByte) {
    clip.f.seek(clip.dataStart + startByte);
    clip.bytesPlayed = startByte;
    clip.remaining = clip.dataBytes - clip.bytesPlayed;
  }

  uint32_t bytesLeftToEnd = endByte - clip.bytesPlayed;
  if (bytesLeftToEnd == 0) {
    clip.f.seek(clip.dataStart + startByte);
    clip.bytesPlayed = startByte;
    clip.remaining = clip.dataBytes - clip.bytesPlayed;
    bytesLeftToEnd = endByte - clip.bytesPlayed;
  }

  static uint8_t inBuf[1024];
  static uint8_t outBuf[2048];

  size_t toRead = min<size_t>(sizeof(inBuf), bytesLeftToEnd);
  if (toRead == 0) return true;

  size_t n = clip.f.read(inBuf, toRead);
  if (n == 0) return true;

  clip.bytesPlayed += (uint32_t)n;
  clip.remaining = (clip.dataBytes > clip.bytesPlayed) ? (clip.dataBytes - clip.bytesPlayed) : 0;

  const int16_t* in16 = (const int16_t*)inBuf;
  size_t monoSamples = n / 2;

  int16_t* out16 = (int16_t*)outBuf;
  for (size_t i = 0; i < monoSamples; i++) {
    int32_t s = in16[i];
    s = (s * (int32_t)volumePct) / 100;
    if (s > 32767) s = 32767;
    if (s < -32768) s = -32768;
    int16_t ss = (int16_t)s;
    *out16++ = ss;
    *out16++ = ss;
  }

  size_t outBytes = monoSamples * 4;
  i2s.write(outBuf, outBytes);

  return true;
}

// ============================================================================
// RECORDINGS PLAYBACK NAV - seeking and loop window movement
// ============================================================================
static void seekClipToEffMs(uint32_t effMs) {
  if (!clip.active || clip.fmt.sampleRate == 0) return;
  uint32_t totalEff = clipTotalMsEffective();
  if (effMs > totalEff) effMs = totalEff;

  uint64_t speed = (uint64_t)max(1, g_speedPercent);
  uint64_t origMs = (uint64_t)effMs * speed / 100ULL;
  uint64_t off = origMs * (uint64_t)clip.fmt.sampleRate * 2ULL / 1000ULL;
  if (off > clip.dataBytes) off = clip.dataBytes;

  uint32_t o = (uint32_t)off;
  o &= ~1UL;

  clip.f.seek(clip.dataStart + o);
  clip.bytesPlayed = o;
  clip.remaining   = (clip.dataBytes > o) ? (clip.dataBytes - o) : 0;
}

static void recordingLoopEnterDefaultLast5s() {
  if (!clip.active) return;
  uint32_t totalEff = clipTotalMsEffective();
  if (totalEff <= 5000) recLoopStartMsEff = 0;
  else recLoopStartMsEff = totalEff - 5000;
  recLoopLenMsEff = 5000;
  recLoopMode = true;
  seekClipToEffMs(recLoopStartMsEff);
  clip.paused = false;
  setStatus("LOOP 5s", 650);
}

static void recordingLoopMoveWindow(int dir) {
  if (!clip.active) return;
  uint32_t totalEff = clipTotalMsEffective();
  if (totalEff == 0) return;

  int32_t delta = dir * (int32_t)5000;
  int32_t ns = (int32_t)recLoopStartMsEff + delta;
  if (ns < 0) ns = 0;
  if ((uint32_t)ns > totalEff) ns = (int32_t)totalEff;

  recLoopStartMsEff = (uint32_t)ns;
  if (recLoopStartMsEff + recLoopLenMsEff > totalEff) {
    if (totalEff > recLoopLenMsEff) recLoopStartMsEff = totalEff - recLoopLenMsEff;
    else recLoopStartMsEff = 0;
  }

  seekClipToEffMs(recLoopStartMsEff);
  clip.paused = false;
  setStatus(dir > 0 ? "LOOP >>" : "LOOP <<", 220);
}

// ============================================================================
// UI helpers
// ============================================================================
static String fmtTime(uint32_t sec) {
  uint32_t m = sec / 60;
  uint32_t s = sec % 60;
  char buf[12];
  snprintf(buf, sizeof(buf), "%02lu:%02lu", (unsigned long)m, (unsigned long)s);
  return String(buf);
}

static void drawVolumeBar(int x, int y, int w, int h, int pct) {
  u8g2.drawFrame(x, y, w, h);
  pct = constrain(pct, 0, 100);
  int fillH = (h - 2) * pct / 100;
  if (fillH > 0) u8g2.drawBox(x + 1, y + (h - 1 - fillH), w - 2, fillH);
}

static void drawIconPrev(int cx, int cy, int size) {
  int s = size;
  u8g2.drawTriangle(cx + s/2, cy - s/2, cx + s/2, cy + s/2, cx, cy);
  u8g2.drawTriangle(cx,       cy - s/2, cx,       cy + s/2, cx - s/2, cy);
}
static void drawIconNext(int cx, int cy, int size) {
  int s = size;
  u8g2.drawTriangle(cx - s/2, cy - s/2, cx - s/2, cy + s/2, cx, cy);
  u8g2.drawTriangle(cx,       cy - s/2, cx,       cy + s/2, cx + s/2, cy);
}
static void drawIconPlay(int cx, int cy, int size) {
  int s = size;
  u8g2.drawTriangle(cx - s/2, cy - s/2, cx - s/2, cy + s/2, cx + s/2, cy);
}
static void drawIconPause(int cx, int cy, int size) {
  int s = size;
  int bw = s/4;
  u8g2.drawBox(cx - bw - 2, cy - s/2, bw, s);
  u8g2.drawBox(cx + 2,      cy - s/2, bw, s);
}

static void drawSeekBar(int x, int y, int w, int h, uint32_t posMs, uint32_t totalMs) {
  u8g2.drawFrame(x, y, w, h);
  if (totalMs == 0) return;

  float frac = (float)posMs / (float)totalMs;
  frac = constrain(frac, 0.0f, 1.0f);
  int fillW = (int)((w - 2) * frac);
  if (fillW > 0) u8g2.drawBox(x + 1, y + 1, fillW, h - 2);

  int knobX = x + 1 + fillW;
  knobX = constrain(knobX, x + 1, x + w - 2);
  u8g2.drawBox(knobX - 1, y - 1, 3, h + 2);
}

// FIX #2: Mode letter logic
static char currentModeLetter() {
  if (uiMode == UI_RECORD_CAPTURE) return 'R';
  if (uiMode == UI_BROWSER && browserIsRecordings) return 'R';
  if (uiMode == UI_PLAYBACK && playbackType == PB_RECORDING_FILE) return 'R';
  if (teacherMode && playbackType == PB_LESSON) return 'T';
  return 'L';
}

static void drawBatteryIcon(int x, int y, int w, int h, int pct) {
  u8g2.drawFrame(x, y, w, h);
  u8g2.drawBox(x + w, y + (h/3), 2, h/3);
  if (pct < 0) return;
  pct = constrain(pct, 0, 100);
  int innerW = w - 2;
  int fillW  = (innerW * pct) / 100;
  if (fillW > 0) u8g2.drawBox(x + 1, y + 1, fillW, h - 2);
}

static void drawNavBar(const String &centerText) {
  u8g2.drawHLine(0, NAV_H - 1, DISP_W);

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();

  char m[2] = { currentModeLetter(), 0 };
  u8g2.drawStr(0, 1, m);

  drawBatteryIcon(DISP_W - 18, 1, 14, 7, batteryPct);

  String t = centerText;
  const int leftPad = 10;
  const int rightLimit = DISP_W - 22;
  int maxW = rightLimit - leftPad;
  while (u8g2.getStrWidth(t.c_str()) > maxW && t.length() > 1) t.remove(t.length()-1);

  int tw = u8g2.getStrWidth(t.c_str());
  int tx = (DISP_W / 2) - (tw / 2);
  tx = constrain(tx, leftPad, rightLimit - tw);
  u8g2.drawStr(tx, 1, t.c_str());

  u8g2.setFontPosBaseline();
}

// ============================================================================
// Browser marquee (FIX #3)
// ============================================================================
static String g_browserLastPath = "";
static int    g_browserLastSel  = -1;
static bool   g_browserLastRoot = false;
static uint32_t g_browserHoldStartMs = 0;

static void browserHoldUpdate() {
  if (currentPath != g_browserLastPath || selIndex != g_browserLastSel || browserIsRecordings != g_browserLastRoot) {
    g_browserLastPath = currentPath;
    g_browserLastSel  = selIndex;
    g_browserLastRoot = browserIsRecordings;
    g_browserHoldStartMs = millis();
  }
}

static String fitToPixelWidth(String s, int maxPixel) {
  while (u8g2.getStrWidth(s.c_str()) > maxPixel && s.length() > 1) s.remove(s.length()-1);
  return s;
}

static String marqueeLine(const String &full, int maxPixel, uint32_t heldMs) {
  // start scrolling after 2000ms stable
  if (heldMs < 2000) return fitToPixelWidth(full, maxPixel);
  if (u8g2.getStrWidth(full.c_str()) <= maxPixel) return full;

  int len = full.length();
  int maxOffset = len - 1;
  for (int off = 0; off < len; off++) {
    String sub = full.substring(off);
    if (u8g2.getStrWidth(sub.c_str()) <= maxPixel) { maxOffset = off; break; }
  }
  if (maxOffset < 1) return fitToPixelWidth(full, maxPixel);

  // slow scroll: 250ms per char "step"
  uint32_t t = (heldMs - 2000);
  uint32_t step = t / 250;

  // pause at ends
  const uint32_t pauseSteps = 6;
  uint32_t travel = (uint32_t)maxOffset;
  uint32_t cycle = pauseSteps + travel + pauseSteps + travel;

  uint32_t p = (cycle > 0) ? (step % cycle) : 0;
  int off = 0;
  if (p < pauseSteps) {
    off = 0;
  } else if (p < pauseSteps + travel) {
    off = (int)(p - pauseSteps);
  } else if (p < pauseSteps + travel + pauseSteps) {
    off = (int)travel;
  } else {
    uint32_t back = p - (pauseSteps + travel + pauseSteps);
    off = (int)(travel - min<uint32_t>(back, travel));
  }

  String view = full.substring(off);
  return fitToPixelWidth(view, maxPixel);
}

// ============================================================================
// OLED UI: Browser (LESSONS/RECORDINGS)
// ============================================================================
static void drawBrowserUI() {
  u8g2.clearBuffer();

  String root = activeBrowserRoot();
  String navTitle;
  if (currentPath == root) {
    navTitle = browserIsRecordings ? "RECORDS" : "LESSONS";
  } else {
    navTitle = leafName(currentPath);
  }
  drawNavBar(navTitle);

  if (statusMsg.length()) {
    u8g2.setFont(u8g2_font_6x10_tf);
    String sm = statusMsg;
    while (u8g2.getStrWidth(sm.c_str()) > DISP_W && sm.length() > 1) sm.remove(sm.length()-1);
    u8g2.drawStr(0, NAV_H + 12, sm.c_str());

    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.setFontPosTop();
    char buf[16];
    snprintf(buf, sizeof(buf), "%d/%d", entries.empty() ? 0 : (selIndex + 1), (int)entries.size());
    u8g2.drawStr(0, DISP_H - FOOTER_H + 1, buf);
    u8g2.setFontPosBaseline();

    u8g2.sendBuffer();
    return;
  }

  if (!entries.empty()) selIndex = constrain(selIndex, 0, (int)entries.size() - 1);
  else selIndex = 0;

  if (selIndex < scrollTop) scrollTop = selIndex;
  if (selIndex >= scrollTop + LINES) scrollTop = selIndex - LINES + 1;
  if (scrollTop < 0) scrollTop = 0;

  // Update hold timer for marquee
  browserHoldUpdate();
  uint32_t heldMs = millis() - g_browserHoldStartMs;

  u8g2.setFont(u8g2_font_6x10_tf);

  const int maxLinePixel = DISP_W - 8;

  for (int i = 0; i < LINES; i++) {
    int idx = scrollTop + i;
    if (idx >= (int)entries.size()) break;

    int yTop = LIST_TOP + i * LINE_H;
    int yBase = yTop + LINE_H - 2;

    const Entry &e = entries[idx];

    if (idx == selIndex) {
      u8g2.drawBox(0, yTop + 1, DISP_W, LINE_H);
      u8g2.setDrawColor(0);
    } else {
      u8g2.setDrawColor(1);
    }

    String line = e.name;
    if (e.isDir) line += "/";
    if (e.isScript) line = withoutExtension(line);
    if (e.isWav)    line = withoutExtension(line);

    if (!browserIsRecordings && e.isDir) {
      String next = joinPath(currentPath, e.name);
      if (folderHasScriptTxt(next)) line += " >";
    }

    // FIX #3: marquee only for selected line, only if too long
    if (idx == selIndex && u8g2.getStrWidth(line.c_str()) > maxLinePixel) {
      line = marqueeLine(line, maxLinePixel, heldMs);
    } else {
      line = fitToPixelWidth(line, maxLinePixel);
    }

    u8g2.drawStr(2, yBase, line.c_str());
    u8g2.setDrawColor(1);
  }

  u8g2.drawHLine(0, DISP_H - FOOTER_H, DISP_W);
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();

  char buf[16];
  snprintf(buf, sizeof(buf), "%d/%d", entries.empty() ? 0 : (selIndex + 1), (int)entries.size());
  u8g2.drawStr(0, DISP_H - FOOTER_H + 1, buf);

  u8g2.setFontPosBaseline();
  u8g2.sendBuffer();
}

// ============================================================================
// OLED UI: Lesson Playback + Teach UI
// ============================================================================
static void drawEyesAnimated(int cx, int cy) {
  const float PI_F = 3.1415926f;
  uint32_t t = millis();

  bool blink = (t % 3200) < 110;
  bool winkL = ((t % 8700) > 2100 && (t % 8700) < 2230);
  bool winkR = ((t % 9100) > 3100 && (t % 9100) < 3230);

  float ph = (float)(t % 2400) / 2400.0f;
  int px = (int)(sinf(2.0f * PI_F * ph) * 4.0f);
  int py = (int)(sinf(2.0f * PI_F * ph * 0.7f) * 2.0f);

  const int eyeW = 26;
  const int eyeH = 14;
  const int gap  = 10;

  int leftCx  = cx - (gap/2) - (eyeW/2);
  int rightCx = cx + (gap/2) + (eyeW/2);

  auto drawOneEye = [&](int ex, int ey, bool winkThis) {
    const int r = 3;
    u8g2.drawRFrame(ex - eyeW/2, ey - eyeH/2, eyeW, eyeH, r);

    bool closed = blink || winkThis;
    if (closed) {
      int y = ey;
      u8g2.drawHLine(ex - (eyeW/2) + 4, y, eyeW - 8);
      u8g2.drawHLine(ex - (eyeW/2) + 5, y + 1, eyeW - 10);
      return;
    }

    int pupR = 2;
    int pupX = ex + px;
    int pupY = ey + py;
    pupX = constrain(pupX, ex - (eyeW/2) + 6, ex + (eyeW/2) - 6);
    pupY = constrain(pupY, ey - (eyeH/2) + 5, ey + (eyeH/2) - 5);
    u8g2.drawDisc(pupX, pupY, pupR, U8G2_DRAW_ALL);
  };

  drawOneEye(leftCx,  cy, winkL);
  drawOneEye(rightCx, cy, winkR);
}

static void drawPlaybackUI_Lesson() {
  u8g2.clearBuffer();

  String lessonTitle = currentLessonFolderPath.length() ? leafName(currentLessonFolderPath) : "LESSON";
  drawNavBar(lessonTitle);

  const int seekH = SEEK_H;
  const int seekY = DISP_H - seekH;
  const int margin = 1;

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();
  const int smallH = (int)u8g2.getAscent() - (int)u8g2.getDescent();
  const int timeTop   = seekY - smallH - margin;
  const int statusTop = timeTop - smallH - margin;

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontPosTop();
  const int bigH = (int)u8g2.getAscent() - (int)u8g2.getDescent();
  const int fileTop = statusTop - bigH - margin;

  const int controlsTop = NAV_H + 2;
  const int controlsBot = fileTop - 2;
  const int iconCy = (controlsTop + controlsBot) / 2;

  const int volX = 12;
  const int volW = 6;
  const int volY = controlsTop;
  const int volBottomLimit = timeTop - 2;
  int volH = volBottomLimit - volY;
  if (volH < 10) volH = 10;

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();
  u8g2.drawStr(0, volY, "+");
  u8g2.drawStr(0, volY + volH - smallH, "-");
  drawVolumeBar(volX, volY, volW, volH, volumePct);

  const int playX = DISP_W / 2;
  const int spacing = 22;
  const int prevX = playX - spacing;
  const int nextX = playX + spacing;

  const bool playing = (clip.active && !clip.paused && !inPause);
  const bool paused  = !playing;

  drawIconPrev(prevX, iconCy, 14);
  if (paused) drawIconPlay(playX, iconCy, 16);
  else        drawIconPause(playX, iconCy, 16);
  drawIconNext(nextX, iconCy, 14);

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontPosTop();
  String fname = clip.token.length() ? withoutExtension(clip.token) : "";
  while (u8g2.getStrWidth(fname.c_str()) > 108 && fname.length() > 1) fname.remove(fname.length() - 1);
  int fw = u8g2.getStrWidth(fname.c_str());
  int fx = (DISP_W / 2) - (fw / 2);
  if (fx < 18) fx = 18;
  u8g2.drawStr(fx, fileTop, fname.c_str());

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();
  String sm = statusMsg.length() ? statusMsg : speedLabel();
  while (u8g2.getStrWidth(sm.c_str()) > 108 && sm.length() > 1) sm.remove(sm.length() - 1);
  int sw = u8g2.getStrWidth(sm.c_str());
  int sx = (DISP_W / 2) - (sw / 2);
  if (sx < 18) sx = 18;
  if (sm.length()) u8g2.drawStr(sx, statusTop, sm.c_str());

  const uint32_t posMs   = currentLessonPosMs();
  const uint32_t totalMs = totalLessonMs;

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();

  String leftT  = fmtTime(posMs / 1000);
  String rightT = fmtTime((totalMs > 0 ? totalMs : 0) / 1000);

  u8g2.drawStr(0, timeTop, leftT.c_str());
  int rw = u8g2.getStrWidth(rightT.c_str());
  u8g2.drawStr(DISP_W - rw, timeTop, rightT.c_str());

  drawSeekBar(0, seekY, DISP_W, seekH, posMs, totalMs);

  u8g2.setFontPosBaseline();
  u8g2.sendBuffer();
}

static void drawTeachUI_Lesson() {
  u8g2.clearBuffer();

  String lessonTitle = currentLessonFolderPath.length() ? leafName(currentLessonFolderPath) : "LESSON";
  drawNavBar(lessonTitle);

  const int seekH = SEEK_H;
  const int seekY = DISP_H - seekH;
  const int margin = 1;

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();
  int smallH = (int)u8g2.getAscent() - (int)u8g2.getDescent();

  int timeTop   = seekY - smallH - margin;
  int statusTop = timeTop - smallH - margin;

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontPosTop();
  int bigH = (int)u8g2.getAscent() - (int)u8g2.getDescent();
  int fileTop = statusTop - bigH - margin;

  int controlsTop = NAV_H + 2;
  int controlsBot = fileTop - 2;

  const int volX = 12, volW = 6, volY = controlsTop;
  const int volBottomLimit = timeTop - 2;
  int volH = volBottomLimit - volY;
  if (volH < 10) volH = 10;

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();
  u8g2.drawStr(0, volY, "+");
  u8g2.drawStr(0, volY + volH - smallH, "-");
  drawVolumeBar(volX, volY, volW, volH, volumePct);

  int eyesCx = DISP_W / 2;
  int eyesCy = (controlsTop + controlsBot) / 2;
  drawEyesAnimated(eyesCx, eyesCy);

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontPosTop();
  String fname = clip.token.length() ? withoutExtension(clip.token) : " ";
  while (u8g2.getStrWidth(fname.c_str()) > 108 && fname.length() > 1) fname.remove(fname.length() - 1);
  int fw = u8g2.getStrWidth(fname.c_str());
  int fx = (DISP_W / 2) - (fw / 2);
  if (fx < 18) fx = 18;
  u8g2.drawStr(fx, fileTop, fname.c_str());

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();
  String sm = statusMsg;
  if (!sm.length() && !teachWords.empty()) {
    char b[20];
    snprintf(b, sizeof(b), "WORD %d/%d", teachWordIdx + 1, (int)teachWords.size());
    sm = b;
  }
  while (u8g2.getStrWidth(sm.c_str()) > 108 && sm.length() > 1) sm.remove(sm.length() - 1);
  int sw = u8g2.getStrWidth(sm.c_str());
  int sx = (DISP_W / 2) - (sw / 2);
  if (sx < 18) sx = 18;
  if (sm.length()) u8g2.drawStr(sx, statusTop, sm.c_str());

  uint32_t posMs   = clipPlayedMsEffective();
  uint32_t totalMs = clipTotalMsEffective();

  String leftT  = fmtTime(posMs / 1000);
  String rightT = fmtTime(totalMs / 1000);

  u8g2.drawStr(0, timeTop, leftT.c_str());
  int rw = u8g2.getStrWidth(rightT.c_str());
  u8g2.drawStr(DISP_W - rw, timeTop, rightT.c_str());

  drawSeekBar(0, seekY, DISP_W, seekH, posMs, totalMs);

  u8g2.setFontPosBaseline();
  u8g2.sendBuffer();
}

// ============================================================================
// OLED UI: Record Capture Screen (FIX #1)
// ============================================================================
static void drawRecordCaptureUI() {
  u8g2.clearBuffer();

  String title = currentLessonFolderPath.length() ? leafName(currentLessonFolderPath) : "RECORD";
  drawNavBar(title);

  // FIX #1: move dot up + move bar/time up to avoid overlap with footer hint
  int cx = DISP_W / 2;
  int cy = NAV_H + 12;  // was NAV_H+18

  bool dotOn = ((millis() / 300) % 2) == 0;
  if (dotOn && (recState == REC_RECORDING)) u8g2.drawDisc(cx, cy, 4, U8G2_DRAW_ALL);
  else u8g2.drawCircle(cx, cy, 4);

  // Level meter (moved up)
  int barW = 96, barH = 8;
  int barX = (DISP_W - barW) / 2;
  int barY = cy + 8;   // was cy+10

  u8g2.drawFrame(barX, barY, barW, barH);
  uint32_t lvl = constrain(recLastLevelPct, 0U, 100U);
  int fillW = (int)((barW - 2) * lvl / 100);
  if (fillW > 0) u8g2.drawBox(barX + 1, barY + 1, fillW, barH - 2);

  // Time (moved up)
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontPosTop();
  String t = fmtTime(recDurationMs() / 1000);
  int tw = u8g2.getStrWidth(t.c_str());
  u8g2.drawStr((DISP_W - tw)/2, barY + 10, t.c_str());

  // Footer hints pinned at the very bottom line (no overlap)
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();

  String st;
  if (statusMsg.length()) st = statusMsg;
  else {
    if (recState == REC_IDLE) st = "OK:START  MIC:SAVE  EXIT:DEL";
    else if (recState == REC_RECORDING) st = "OK:STOP   MIC:SAVE  EXIT:DEL";
    else st = "OK:START  MIC:SAVE  EXIT:DEL";
  }

  while (u8g2.getStrWidth(st.c_str()) > DISP_W && st.length() > 1) st.remove(st.length()-1);

  // ensure it stays below the time line
  int hintY = DISP_H - 8;   // was DISP_H-9
  u8g2.drawStr(0, hintY, st.c_str());

  u8g2.setFontPosBaseline();
  u8g2.sendBuffer();
}

// ============================================================================
// OLED UI: Recording Playback
// ============================================================================
static void drawPlaybackUI_RecordingFile() {
  u8g2.clearBuffer();

  String title = currentRecordingFolderPath.length() ? leafName(currentRecordingFolderPath) : "RECORD";
  drawNavBar(title);

  const int seekH = SEEK_H;
  const int seekY = DISP_H - seekH;
  const int margin = 1;

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();
  const int smallH = (int)u8g2.getAscent() - (int)u8g2.getDescent();
  const int timeTop   = seekY - smallH - margin;
  const int statusTop = timeTop - smallH - margin;

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontPosTop();
  const int bigH = (int)u8g2.getAscent() - (int)u8g2.getDescent();
  const int fileTop = statusTop - bigH - margin;

  const int controlsTop = NAV_H + 2;
  const int controlsBot = fileTop - 2;
  const int iconCy = (controlsTop + controlsBot) / 2;

  const int volX = 12, volW = 6, volY = controlsTop;
  const int volBottomLimit = timeTop - 2;
  int volH = volBottomLimit - volY;
  if (volH < 10) volH = 10;

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();
  u8g2.drawStr(0, volY, "+");
  u8g2.drawStr(0, volY + volH - smallH, "-");
  drawVolumeBar(volX, volY, volW, volH, volumePct);

  const int playX = DISP_W / 2;
  const int spacing = 22;
  const int prevX = playX - spacing;
  const int nextX = playX + spacing;

  const bool playing = (clip.active && !clip.paused);
  const bool paused  = !playing;

  drawIconPrev(prevX, iconCy, 14);
  if (paused) drawIconPlay(playX, iconCy, 16);
  else        drawIconPause(playX, iconCy, 16);
  drawIconNext(nextX, iconCy, 14);

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontPosTop();
  String fname = clip.token.length() ? withoutExtension(clip.token) : "";
  while (u8g2.getStrWidth(fname.c_str()) > 108 && fname.length() > 1) fname.remove(fname.length() - 1);
  int fw = u8g2.getStrWidth(fname.c_str());
  int fx = (DISP_W / 2) - (fw / 2);
  if (fx < 18) fx = 18;
  u8g2.drawStr(fx, fileTop, fname.c_str());

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();
  String sm;
  if (statusMsg.length()) sm = statusMsg;
  else sm = recLoopMode ? "LOOP 5s (L/R move)" : "TEACH=LOOP  EXIT=BACK";
  while (u8g2.getStrWidth(sm.c_str()) > 108 && sm.length() > 1) sm.remove(sm.length() - 1);
  int sw = u8g2.getStrWidth(sm.c_str());
  int sx = (DISP_W / 2) - (sw / 2);
  if (sx < 18) sx = 18;
  if (sm.length()) u8g2.drawStr(sx, statusTop, sm.c_str());

  uint32_t posMs   = clipPlayedMsEffective();
  uint32_t totalMs = clipTotalMsEffective();

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setFontPosTop();

  String leftT  = fmtTime(posMs / 1000);
  String rightT = fmtTime(totalMs / 1000);

  u8g2.drawStr(0, timeTop, leftT.c_str());
  int rw = u8g2.getStrWidth(rightT.c_str());
  u8g2.drawStr(DISP_W - rw, timeTop, rightT.c_str());

  drawSeekBar(0, seekY, DISP_W, seekH, posMs, totalMs);

  u8g2.setFontPosBaseline();
  u8g2.sendBuffer();
}

// ============================================================================
// NAVIGATION helpers
// ============================================================================
static void enterPath(const String &path) {
  currentPath = path;
  loadDirectory(currentPath);
}

static void enterPathSelect(const String &path, const String &selName) {
  enterPath(path);
  if (selName.length()) {
    for (int i = 0; i < (int)entries.size(); i++) {
      if (entries[i].name == selName) { selIndex = i; break; }
    }
  }
}

static void goUpLocked() {
  String root = activeBrowserRoot();

  if (currentPath == root) {
    setStatus("HOME", 350);
    return;
  }
  String p = parentPathOf(currentPath);
  if (!isInsideRoot(p, root)) {
    enterPath(root);
    setStatus("HOME", 350);
    return;
  }
  enterPath(p);
}

// ============================================================================
// MODE TRANSITIONS
// ============================================================================
static void enterPlaybackFromScript(const String &fullScriptPath) {
  String folder = parentPathOf(fullScriptPath);
  if (!startLessonByFolder(folder)) setStatus("START FAIL", 900);
}

static void exitLessonPlaybackToBrowser() {
  clearScript();
  teacherMode = false;
  uiMode = UI_BROWSER;
  playbackType = PB_LESSON;

  if (browserReturnPath.length()) {
    enterPathSelect(browserReturnPath, browserReturnSel);
  } else if (currentLangPath.length()) {
    enterPathSelect(currentLangPath, leafName(currentLessonFolderPath));
  } else {
    enterPath(String(LESSONS_ROOT));
  }
}

static bool startRecordingPlaybackFile(const String &wavPath, const String &tokenLabel) {
  teacherMode = false;
  inPause = false;
  pauseUntil = 0;

  stopClip();

  setSpeedPercent(100);

  playbackType = PB_RECORDING_FILE;
  uiMode = UI_PLAYBACK;
  recLoopMode = false;

  currentRecordingFilePath = wavPath;
  currentRecordingFolderPath = parentPathOf(wavPath);

  bool ok = startClipFile(wavPath, tokenLabel);
  if (!ok) {
    setStatus("BAD WAV", 800);
    return false;
  }
  setStatus("PLAY", 250);
  return true;
}

static void exitRecordingPlaybackToBrowser() {
  stopClip();
  recLoopMode = false;
  playbackType = PB_LESSON;
  uiMode = UI_BROWSER;

  if (recordingsBrowserReturnPath.length()) {
    enterPathSelect(recordingsBrowserReturnPath, recordingsBrowserReturnSel);
  } else {
    enterPath(String(RECORDINGS_ROOT));
  }
}

// ============================================================================
// LISTEN MODE L/R GESTURE (LESSONS)
// ============================================================================
static void servicePlaybackLrGesture_Lesson() {
  ActionKey cur = keyState.stableKey;
  uint32_t now = millis();

  if ((cur == AK_LEFT || cur == AK_RIGHT) && lastStableKey != cur) {
    lrActiveKey = cur;
    lrPressMs = now;
    lrSeeking = false;
    lrLastSeekTickMs = 0;
  }

  if ((cur == AK_LEFT || cur == AK_RIGHT) && keyState.held && lrActiveKey == cur) {
    uint32_t heldFor = now - lrPressMs;
    if (heldFor >= LR_LONGPRESS_MS) {
      lrSeeking = true;

      if (lrLastSeekTickMs == 0 || (now - lrLastSeekTickMs) >= SEEK_TICK_MS) {
        lrLastSeekTickMs = now;

        uint32_t step = (heldFor >= SEEK_RAMP_MS) ? SEEK_STEP_MS_FAST : SEEK_STEP_MS_SLOW;
        int32_t delta = (cur == AK_RIGHT) ? (int32_t)step : -(int32_t)step;

        uint32_t pos = currentLessonPosMs();
        int32_t target = (int32_t)pos + delta;
        if (target < 0) target = 0;
        if (totalLessonMs > 0 && (uint32_t)target > totalLessonMs) target = (int32_t)totalLessonMs;

        jumpToLessonTime((uint32_t)target);
        setStatus(cur == AK_RIGHT ? ">>" : "<<", 120);
      }
    }
  }

  if (cur == AK_NONE && (lastStableKey == AK_LEFT || lastStableKey == AK_RIGHT)) {
    if (!lrSeeking) {
      if (lastStableKey == AK_RIGHT) startNextLesson();
      else                          startPrevLesson();
    }
    lrActiveKey = AK_NONE;
    lrSeeking = false;
    lrLastSeekTickMs = 0;
  }

  lastStableKey = cur;
}

// ============================================================================
// RECORDING PLAYBACK L/R GESTURE
// ============================================================================
static ActionKey recPbLrActiveKey = AK_NONE;
static uint32_t  recPbLrPressMs = 0;
static bool      recPbLrSeeking = false;
static uint32_t  recPbLastTick = 0;

static void servicePlaybackLrGesture_RecordingFile() {
  ActionKey cur = keyState.stableKey;
  uint32_t now = millis();

  if ((cur == AK_LEFT || cur == AK_RIGHT) && cur != recPbLrActiveKey) {
    recPbLrActiveKey = cur;
    recPbLrPressMs = now;
    recPbLrSeeking = false;
    recPbLastTick = 0;
  }

  if ((cur == AK_LEFT || cur == AK_RIGHT) && keyState.held && recPbLrActiveKey == cur) {
    uint32_t heldFor = now - recPbLrPressMs;
    if (heldFor >= LR_LONGPRESS_MS) {
      recPbLrSeeking = true;

      if (recPbLastTick == 0 || (now - recPbLastTick) >= SEEK_TICK_MS) {
        recPbLastTick = now;

        if (recLoopMode) {
          recordingLoopMoveWindow(cur == AK_RIGHT ? +1 : -1);
        } else {
          uint32_t step = (heldFor >= SEEK_RAMP_MS) ? SEEK_STEP_MS_FAST : SEEK_STEP_MS_SLOW;
          int32_t delta = (cur == AK_RIGHT) ? (int32_t)step : -(int32_t)step;
          uint32_t pos = clipPlayedMsEffective();
          int32_t target = (int32_t)pos + delta;
          if (target < 0) target = 0;
          uint32_t totalEff = clipTotalMsEffective();
          if (totalEff > 0 && (uint32_t)target > totalEff) target = (int32_t)totalEff;
          seekClipToEffMs((uint32_t)target);
          setStatus(cur == AK_RIGHT ? ">>" : "<<", 120);
        }
      }
    }
  }

  if (cur == AK_NONE && (recPbLrActiveKey == AK_LEFT || recPbLrActiveKey == AK_RIGHT)) {
    if (!recPbLrSeeking) {
      if (recLoopMode) {
        recordingLoopMoveWindow(recPbLrActiveKey == AK_RIGHT ? +1 : -1);
      } else {
        int32_t delta = (recPbLrActiveKey == AK_RIGHT) ? +5000 : -5000;
        uint32_t pos = clipPlayedMsEffective();
        int32_t target = (int32_t)pos + delta;
        if (target < 0) target = 0;
        uint32_t totalEff = clipTotalMsEffective();
        if (totalEff > 0 && (uint32_t)target > totalEff) target = (int32_t)totalEff;
        seekClipToEffMs((uint32_t)target);
      }
    }
    recPbLrActiveKey = AK_NONE;
    recPbLrSeeking = false;
    recPbLastTick = 0;
  }
}

// ============================================================================
// RECORD CAPTURE mode entry/exit
// ============================================================================
static void enterRecordCaptureFromLessonPlayback() {
  if (uiMode != UI_PLAYBACK || playbackType != PB_LESSON) return;
  if (!currentLessonFolderPath.length() || !currentLangPath.length()) {
    setStatus("NO LESSON", 800);
    return;
  }

  recPrevTeacherMode = teacherMode;

  if (teacherMode) {
    recPrevTeachWordIdx = teachWordIdx;
    recPrevTeachPosMsEff = clipPlayedMsEffective();
    recPrevTeachPaused = (clip.active && clip.paused);
  } else {
    recPrevLessonPosMs = currentLessonPosMs();
    recPrevLessonPaused = (inPause || (clip.active && clip.paused));
  }

  stopClip();
  inPause = false;
  pauseUntil = 0;

  recordingMode = true;
  uiMode = UI_RECORD_CAPTURE;
  recState = REC_IDLE;

  ensureI2S_RX(REC_SAMPLE_RATE_HZ);

  setStatus("REC READY", 600);
}

static void returnToLessonPlaybackAfterRecord(bool resumePlay) {
  recordingMode = false;
  uiMode = UI_PLAYBACK;
  playbackType = PB_LESSON;

  if (recPrevTeacherMode) {
    teacherMode = true;

    if (teachWords.empty()) buildTeachWordList();
    setSpeedPercent(100);

    if (!startTeachWordByIndex(recPrevTeachWordIdx)) {
      teacherMode = false;
      setStatus("MISSING", 900);
      return;
    }

    seekTeachToEffMs(recPrevTeachPosMsEff);
    if (recPrevTeachPaused || !resumePlay) clip.paused = true;
    else clip.paused = false;

  } else {
    teacherMode = false;

    jumpToLessonTime(recPrevLessonPosMs);

    if (recPrevLessonPaused || !resumePlay) {
      if (inPause) {
        // script pause stays
      } else if (clip.active) {
        clip.paused = true;
      }
    }
  }
}

// ============================================================================
// ACTION HANDLING
// ============================================================================
static void handleBrowserAction(ActionKey k) {
  switch (k) {
    case AK_UP:    if (selIndex > 0) selIndex--; break;
    case AK_DOWN:  if (selIndex < (int)entries.size() - 1) selIndex++; break;

    case AK_LEFT:
      goUpLocked();
      break;

    case AK_MIC:
      browserIsRecordings = !browserIsRecordings;
      enterPath(activeBrowserRoot());
      setStatus(browserIsRecordings ? "RECORDS" : "LESSONS", 450);
      break;

    case AK_RIGHT:
    case AK_OK:
      if (entries.empty()) break;
      if (selIndex < 0 || selIndex >= (int)entries.size()) break;

      if (entries[selIndex].isDir) {
        String next = joinPath(currentPath, entries[selIndex].name);

        if (!isInsideRoot(next, activeBrowserRoot())) { setStatus("DENIED", 700); break; }

        if (!browserIsRecordings) {
          if (folderHasScriptTxt(next)) {
            browserReturnPath = currentPath;
            browserReturnSel  = entries[selIndex].name;
            enterPlaybackFromScript(next + "/script.txt");
          } else {
            enterPath(next);
          }
        } else {
          enterPath(next);
        }
      }
      else if (!browserIsRecordings && entries[selIndex].isScript) {
        String fullScript = joinPath(currentPath, entries[selIndex].name);
        browserReturnPath = currentPath;
        browserReturnSel  = entries[selIndex].name;
        enterPlaybackFromScript(fullScript);
      }
      else if (browserIsRecordings && entries[selIndex].isWav) {
        String fullWav = joinPath(currentPath, entries[selIndex].name);
        recordingsBrowserReturnPath = currentPath;
        recordingsBrowserReturnSel  = entries[selIndex].name;
        startRecordingPlaybackFile(fullWav, entries[selIndex].name);
      }
      break;

    case AK_VOL_UP:
      volumePct = min(100, volumePct + 5);
      setStatus("VOL+", 220);
      break;

    case AK_VOL_DOWN:
      volumePct = max(0, volumePct - 5);
      setStatus("VOL-", 220);
      break;

    case AK_TEACHER:
      setStatus("PLAY FIRST", 650);
      break;

    case AK_EXIT:
      enterPath(activeBrowserRoot());
      setStatus("HOME", 350);
      break;

    default:
      break;
  }
}

static void handleRecordCaptureAction(ActionKey k) {
  switch (k) {
    case AK_OK:
      if (recState == REC_IDLE) {
        if (!openNewRecordingFile()) {
          setStatus("REC FAIL", 900);
          return;
        }
        recState = REC_RECORDING;
        setStatus("REC", 350);
      } else if (recState == REC_RECORDING) {
        recState = REC_STOPPED;
        setStatus("STOP", 350);
      } else {
        recState = REC_RECORDING;
        setStatus("REC", 250);
      }
      break;

    case AK_MIC: {
      if (recState == REC_RECORDING) recState = REC_STOPPED;

      if (!recFile || recDataBytes < 200) {
        if (recFile) closeRecordingFileFinalize(false);
        setStatus("NO DATA", 700);
        returnToLessonPlaybackAfterRecord(true);
        return;
      }

      closeRecordingFileFinalize(true);
      setStatus("SAVED", 800);
      returnToLessonPlaybackAfterRecord(true);
      break;
    }

    case AK_EXIT:
      if (recFile) {
        closeRecordingFileFinalize(false);
      }
      setStatus("DISCARD", 800);
      returnToLessonPlaybackAfterRecord(true);
      break;

    case AK_VOL_UP:
      volumePct = min(100, volumePct + 5);
      setStatus("VOL+", 200);
      break;

    case AK_VOL_DOWN:
      volumePct = max(0, volumePct - 5);
      setStatus("VOL-", 200);
      break;

    default:
      break;
  }
}

static void handleLessonPlaybackAction(ActionKey k) {
  if (teacherMode) {
    switch (k) {
      case AK_OK:
        if (clip.active) {
          clip.paused = !clip.paused;
          setStatus(clip.paused ? "PAUSE" : "PLAY", 250);
        }
        break;

      case AK_VOL_UP:
        volumePct = min(100, volumePct + 5);
        setStatus("VOL+", 200);
        break;

      case AK_VOL_DOWN:
        volumePct = max(0, volumePct - 5);
        setStatus("VOL-", 200);
        break;

      case AK_UP:
        nudgeUserSpeed(+SPEED_STEP_PCT);
        break;

      case AK_DOWN:
        nudgeUserSpeed(-SPEED_STEP_PCT);
        break;

      case AK_TEACHER:
        restartTeachLoop();
        break;

      case AK_EXIT:
        exitTeachMode();
        break;

      case AK_MIC:
        enterRecordCaptureFromLessonPlayback();
        break;

      default:
        break;
    }
    return;
  }

  switch (k) {
    case AK_OK:
      togglePauseResume();
      break;

    case AK_VOL_UP:
      volumePct = min(100, volumePct + 5);
      setStatus("VOL+", 200);
      break;

    case AK_VOL_DOWN:
      volumePct = max(0, volumePct - 5);
      setStatus("VOL-", 200);
      break;

    case AK_UP:
      nudgeUserSpeed(+SPEED_STEP_PCT);
      break;

    case AK_DOWN:
      nudgeUserSpeed(-SPEED_STEP_PCT);
      break;

    case AK_EXIT:
      setStatus("BACK", 200);
      exitLessonPlaybackToBrowser();
      break;

    case AK_TEACHER:
      enterTeachMode();
      break;

    case AK_MIC:
      enterRecordCaptureFromLessonPlayback();
      break;

    default:
      break;
  }
}

static void handleRecordingPlaybackAction(ActionKey k) {
  switch (k) {
    case AK_OK:
      if (clip.active) {
        clip.paused = !clip.paused;
        setStatus(clip.paused ? "PAUSE" : "PLAY", 250);
      }
      break;

    case AK_VOL_UP:
      volumePct = min(100, volumePct + 5);
      setStatus("VOL+", 200);
      break;

    case AK_VOL_DOWN:
      volumePct = max(0, volumePct - 5);
      setStatus("VOL-", 200);
      break;

    case AK_TEACHER:
      if (!recLoopMode) recordingLoopEnterDefaultLast5s();
      else {
        seekClipToEffMs(recLoopStartMsEff);
        clip.paused = false;
        setStatus("RELOOP", 400);
      }
      break;

    case AK_EXIT:
      if (recLoopMode) {
        recLoopMode = false;
        setStatus("LOOP OFF", 500);
      } else {
        setStatus("BACK", 250);
        exitRecordingPlaybackToBrowser();
      }
      break;

    case AK_UP:
      nudgeUserSpeed(+SPEED_STEP_PCT);
      break;

    case AK_DOWN:
      nudgeUserSpeed(-SPEED_STEP_PCT);
      break;

    default:
      break;
  }
}

// ============================================================================
// BOOT SPLASH
// ============================================================================
static void drawBootSplashFrame(uint32_t t) {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontPosTop();

  const char* title = "AURAL";
  int tw = u8g2.getStrWidth(title);
  int tx = (DISP_W - tw) / 2;
  int ty = 16;
  u8g2.drawStr(tx, ty, title);

  int barY = ty + 16;
  int barW = 60;
  int barX = (DISP_W - barW) / 2;
  u8g2.drawFrame(barX, barY, barW, 6);

  int phase = (t / 18) % (barW - 6);
  u8g2.drawBox(barX + 2 + phase, barY + 2, 6, 2);

  u8g2.setFont(u8g2_font_5x7_tf);
  const char* p = "PRESS ANY KEY";
  int pw = u8g2.getStrWidth(p);
  u8g2.drawStr((DISP_W - pw) / 2, 46, p);

  u8g2.sendBuffer();
}

static void waitForAnyKeyOnBoot() {
  resetKeyState();

  while (true) {
    drawBootSplashFrame(millis());

    float a=0, b=0;
    ActionKey raw = readRawActionKey(a, b, nullptr);
    if (raw != AK_NONE) {
      while (true) {
        float a2=0, b2=0;
        ActionKey r2 = readRawActionKey(a2, b2, nullptr);
        if (r2 == AK_NONE) break;
        delay(12);
      }
      resetKeyState();
      return;
    }
    delay(30);
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(50);

  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(KEYPAD_T1_PIN, ADC_11db);
  analogSetPinAttenuation(KEYPAD_T2_PIN, ADC_11db);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(400000);
  u8g2.setI2CAddress(OLED_I2C_ADDR << 1);
  u8g2.begin();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 12, "AURAL");
  u8g2.drawStr(0, 28, "Mounting SD...");
  u8g2.sendBuffer();

  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  pinMode(SD_CS_PIN, OUTPUT);

  if (!SD.begin(SD_CS_PIN, sdSPI, SD_SPI_HZ)) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 12, "SD init FAILED");
    u8g2.drawStr(0, 24, "Check wiring/card");
    u8g2.sendBuffer();
    while (true) delay(1000);
  }

  if (!SD.exists(LESSONS_ROOT)) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 12, "Missing folder:");
    u8g2.drawStr(0, 24, LESSONS_ROOT);
    u8g2.sendBuffer();
    while (true) delay(1000);
  }

  if (!SD.exists(RECORDINGS_ROOT)) {
    mkdirs(String(RECORDINGS_ROOT));
  }

  waitForAnyKeyOnBoot();

  uiMode = UI_BROWSER;
  browserIsRecordings = false;
  enterPath(String(LESSONS_ROOT));
  setStatus("", 0);
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  ActionKey ev;
  if (pollKeyEvent(ev)) {
    if (uiMode == UI_BROWSER) handleBrowserAction(ev);
    else if (uiMode == UI_RECORD_CAPTURE) handleRecordCaptureAction(ev);
    else {
      if (playbackType == PB_LESSON) handleLessonPlaybackAction(ev);
      else handleRecordingPlaybackAction(ev);
    }
  }

  // Playback / capture servicing
  if (uiMode == UI_RECORD_CAPTURE) {
    serviceRecordingCapture();
  }
  else if (uiMode == UI_PLAYBACK) {

    if (playbackType == PB_RECORDING_FILE) {
      servicePlaybackLrGesture_RecordingFile();

      if (recLoopMode) {
        if (clip.active) {
          serviceClipLoopWindow(recLoopStartMsEff, recLoopLenMsEff);
        }
      } else {
        if (clip.active) {
          serviceClip();
        }
      }
    }
    else {
      if (teacherMode) {
        serviceTeachLrGesture();

        if (clip.active) {
          bool stillActive = serviceClip();
          if (!stillActive) {
            if (!startTeachWordByIndex(teachWordIdx)) {
              setStatus("MISSING", 700);
            }
          }
        } else {
          if (!startTeachWordByIndex(teachWordIdx)) {
            setStatus("MISSING", 700);
          }
        }
      }
      else {
        servicePlaybackLrGesture_Lesson();

        if (inPause) {
          if (millis() >= pauseUntil) {
            inPause = false;
            pauseUntil = 0;
            eventIndex++;
            startCurrentEventOrAdvance();
          }
        } else {
          if (clip.active) {
            bool stillActive = serviceClip();
            if (!stillActive) {
              eventIndex++;
              startCurrentEventOrAdvance();
            }
          } else {
            startCurrentEventOrAdvance();
          }
        }
      }
    }
  }

  maybeClearStatus();

  // Draw UI
  if (uiMode == UI_BROWSER) {
    drawBrowserUI();
  } else if (uiMode == UI_RECORD_CAPTURE) {
    drawRecordCaptureUI();
  } else {
    uint32_t now = millis();
    if (now - lastPlaybackUiMs >= PLAYBACK_UI_PERIOD_MS) {
      lastPlaybackUiMs = now;
      if (playbackType == PB_RECORDING_FILE) {
        drawPlaybackUI_RecordingFile();
      } else {
        if (teacherMode) drawTeachUI_Lesson();
        else             drawPlaybackUI_Lesson();
      }
    }
  }

  delay(2);
}
