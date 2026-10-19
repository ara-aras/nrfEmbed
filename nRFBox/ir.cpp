/* ____________________________
   This software is licensed under the MIT License:
   https://github.com/cifertech/nrfbox
   ________________________________________ */

#include "ir.h"
#include "setting.h"
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <SD.h>
#include <FS.h>
#include <vector>

namespace IrControl {

// Hardware IR objects
static IRsend irsend(IR_TX_PIN);
static IRrecv irrecv(IR_RX_PIN, 1024, 50, true);
static decode_results results;

// -------------------------------------------------------------
// Sub-menu definitions
// -------------------------------------------------------------
// States:
// 0: Root IR Menu
// 1: TV Blaster
// 2: Projector Blaster
// 3: TV Remote
// 4: IR Sniffer & Waveform
// 5: IR Jammer
// 6: SD Library
// 7: IR Replayer
static int irState = 0;
static int menuSelected = 0;
static int menuScrollOffset = 0;
const int IR_MENU_COUNT = 7;
static const char* irMenuTitles[IR_MENU_COUNT] = {
  "TV Blaster",
  "Projector Blast",
  "TV Remote",
  "IR Sniffer",
  "IR Jammer",
  "SD Library",
  "IR Replayer"
};

// Button & timing
static unsigned long lastBtnTime = 0;
static const unsigned long DEBOUNCE_MS = 160;
static bool irInitialized = false;

// -------------------------------------------------------------
// Shared Data Structures
// -------------------------------------------------------------
struct PowerCode {
  const char* brand;
  decode_type_t protocol;
  uint64_t code;
  uint16_t bits;
};

// 1. TV Power Codes (TV-B-Gone)
static const PowerCode tvPowerCodes[] = {
  {"Samsung",   NEC,       0xE0E040BF, 32},
  {"LG",        NEC,       0x20DF10EF, 32},
  {"Sony",      SONY,      0xA90,      12},
  {"Panasonic", PANASONIC, 0x40040100BCBDULL, 48},
  {"Philips",   RC5,       0x000C,     12},
  {"TCL/RCA",   NEC,       0xF1A2E01F, 32},
  {"Toshiba",   NEC,       0x02FD48B7, 32},
  {"Sharp",     SHARP,     0x41A2,     15},
  {"Hisense",   NEC,       0xFD0208F7, 32},
  {"Vizio",     NEC,       0x20DF10EF, 32},
  {"Sanyo",     NEC,       0x383820DF, 32},
  {"Generic",   NEC,       0x00FF00FF, 32}
};
static const int TV_CODES_COUNT = sizeof(tvPowerCodes) / sizeof(tvPowerCodes[0]);

// 2. Projector Power Codes
static const PowerCode projPowerCodes[] = {
  {"Epson",       NEC,       0x41B640BF, 32},
  {"BenQ",        NEC,       0x00FF00FF, 32},
  {"Optoma",      NEC,       0x32CD08F7, 32},
  {"ViewSonic",   NEC,       0x20DF10EF, 32},
  {"Sony Proj",   SONY,      0xA90,      12},
  {"NEC Proj",    NEC,       0x18E708F7, 32},
  {"Hitachi",     NEC,       0x6D9208F7, 32},
  {"Casio",       NEC,       0x08F708F7, 32},
  {"InFocus",     NEC,       0x43BC08F7, 32},
  {"Panasonic P", PANASONIC, 0x40040100BCBDULL, 48}
};
static const int PROJ_CODES_COUNT = sizeof(projPowerCodes) / sizeof(projPowerCodes[0]);

// Blaster state
static bool blastingActive = false;
static int blastIndex = 0;
static bool blastLoopMode = false;
static unsigned long lastBlastTime = 0;
static const unsigned long BLAST_INTERVAL_MS = 140;

// 3. TV Remote Brands & Commands
enum RemoteCmd {
  CMD_POWER = 0,
  CMD_VOL_UP,
  CMD_VOL_DOWN,
  CMD_MUTE,
  CMD_CH_UP,
  CMD_CH_DOWN,
  CMD_INPUT,
  CMD_COUNT
};

static const char* remoteCmdNames[CMD_COUNT] = {
  "POWER", "VOL +", "VOL -", "MUTE", "CH +", "CH -", "INPUT"
};

struct BrandRemote {
  const char* name;
  decode_type_t protocol;
  uint16_t bits;
  uint64_t codes[CMD_COUNT];
};

static const BrandRemote brandRemotes[] = {
  {
    "Samsung", NEC, 32,
    {0xE0E040BF, 0xE0E0E01F, 0xE0E0D02F, 0xE0E0F00F, 0xE0E048B7, 0xE0E008F7, 0xE0E0807F}
  },
  {
    "LG", NEC, 32,
    {0x20DF10EF, 0x20DF40BF, 0x20DFC03F, 0x20DF906F, 0x20DF00FF, 0x20DF807F, 0x20DFD02F}
  },
  {
    "Sony", SONY, 12,
    {0xA90, 0x490, 0xC90, 0x290, 0x090, 0x890, 0xA50}
  },
  {
    "Panasonic", PANASONIC, 48,
    {0x40040100BCBDULL, 0x400401000405ULL, 0x400401008485ULL, 0x400401004C4DULL, 0x400401002C2DULL, 0x40040100ACADULL, 0x40040100A0A1ULL}
  }
};
static const int BRAND_COUNT = sizeof(brandRemotes) / sizeof(brandRemotes[0]);
static int currentBrandIndex = 0;
static int currentCmdIndex = 0;
static String lastSentStatus = "";

// 4. IR Sniffer, Waveform & Storage
// Long frames (AC remotes) need 200+ marks/spaces; 512 ticks covers them.
static const uint16_t kCaptureRawMax = 512;

struct CapturedSignal {
  bool valid;
  decode_type_t protocol;
  uint64_t value;
  uint16_t bits;
  uint16_t address;
  uint16_t command;
  uint16_t rawlen;
  uint16_t rawbuf[kCaptureRawMax];
};

static CapturedSignal savedSignal = {false, UNKNOWN, 0, 0, 0, 0, 0, {0}};
static bool snifferWaveformMode = false;
static int waveformScrollX = 0;
static String snifferMsg = "Waiting for IR...";

// 5. IR Jammer State
static bool jammerActive = false;
static int jammerMode = 0; // 0: Carrier Flood (38kHz), 1: AGC Noise Burst
static unsigned long jammerStartTime = 0;
static unsigned long lastJammerUpdate = 0;

// 6. SD Card IR Library
static bool sdAvailable = false;
static std::vector<String> sdIrFiles;
static int sdFileIndex = 0;
static int sdScrollOffset = 0;
static String sdStatusMsg = "";

// -------------------------------------------------------------
// Helper: Send IR signal by protocol
// -------------------------------------------------------------
static void sendIRSignal(decode_type_t protocol, uint64_t code, uint16_t bits) {
  irrecv.pause();
  setNeoPixelColour("red");
  
  if (protocol == PANASONIC) {
    irsend.sendPanasonic64(code);
  } else {
    irsend.send(protocol, code, bits);
  }
  
  delay(25);
  setNeoPixelColour("0");
  irrecv.resume();
}

// Replay whatever the sniffer holds. Decoded protocols are re-encoded;
// UNKNOWN ones (most ACs, odd TVs) are replayed from the raw mark/space timings.
static bool replayCaptured() {
  if (!savedSignal.valid) return false;

  if (savedSignal.protocol != UNKNOWN) {
    sendIRSignal(savedSignal.protocol, savedSignal.value, savedSignal.bits);
    return true;
  }

  if (savedSignal.rawlen < 3) return false;

  // rawbuf[0] is the leading gap; [1..] alternate mark/space in kRawTick units.
  uint16_t count = savedSignal.rawlen - 1;
  uint16_t* usBuf = (uint16_t*)malloc(count * sizeof(uint16_t));
  if (!usBuf) return false;
  for (uint16_t i = 0; i < count; i++) {
    uint32_t us = (uint32_t)savedSignal.rawbuf[i + 1] * kRawTick;
    usBuf[i] = us > 65535 ? 65535 : (uint16_t)us;
  }

  irrecv.pause();
  setNeoPixelColour("red");
  irsend.sendRaw(usBuf, count, 38);
  delay(25);
  setNeoPixelColour("0");
  irrecv.resume();

  free(usBuf);
  return true;
}

static bool checkBtn(int pin) {
  if (digitalRead(pin) == LOW && (millis() - lastBtnTime > DEBOUNCE_MS)) {
    lastBtnTime = millis();
    return true;
  }
  return false;
}

// -------------------------------------------------------------
// SD Card Functions (Flipper Zero format save & load)
// -------------------------------------------------------------
static bool initSD() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  if (!SD.begin(SD_CS_PIN)) {
    sdAvailable = false;
    return false;
  }
  sdAvailable = true;
  if (!SD.exists("/ir")) {
    SD.mkdir("/ir");
  }
  return true;
}

static void reloadSDFiles() {
  sdIrFiles.clear();
  sdFileIndex = 0;
  sdScrollOffset = 0;
  
  if (!sdAvailable && !initSD()) return;

  File dir = SD.open("/ir");
  if (!dir || !dir.isDirectory()) return;

  File entry = dir.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      String name = String(entry.name());
      if (name.endsWith(".ir") || name.endsWith(".IR")) {
        // Keep filename without leading path if any
        int slash = name.lastIndexOf('/');
        if (slash >= 0) name = name.substring(slash + 1);
        sdIrFiles.push_back(name);
      }
    }
    entry.close();
    entry = dir.openNextFile();
  }
  dir.close();
}

static bool saveSignalToSD() {
  if (!savedSignal.valid) return false;
  if (!sdAvailable && !initSD()) return false;

  // Auto-generate unique filename
  int num = 1;
  String filename;
  do {
    filename = "/ir/capture_" + String(num++) + ".ir";
  } while (SD.exists(filename) && num < 999);

  File f = SD.open(filename, FILE_WRITE);
  if (!f) return false;

  f.println("Filetype: IR library file");
  f.println("Version: 1");
  f.println("# Generated by nRFBox");
  f.println("name: Capture");

  // Undecoded signals have no meaningful protocol/value; keep raw timings only.
  if (savedSignal.protocol != UNKNOWN) {
    f.println("type: parsed");
    f.print("protocol: ");
    f.println(typeToString(savedSignal.protocol));
    f.print("bits: ");
    f.println(savedSignal.bits);

    char buf[32];
    sprintf(buf, "0x%llX", (unsigned long long)savedSignal.value);
    f.print("value: ");
    f.println(buf);

    sprintf(buf, "0x%X", savedSignal.address);
    f.print("address: ");
    f.println(buf);

    sprintf(buf, "0x%X", savedSignal.command);
    f.print("command: ");
    f.println(buf);
  }

  // Raw timings only for undecoded signals. A parsed signal stores its protocol
  // instead; emitting a second "type:" line would make playback ambiguous.
  if (savedSignal.protocol == UNKNOWN && savedSignal.rawlen > 0) {
    f.println("type: raw");
    f.println("frequency: 38000");
    f.print("data: ");
    for (uint16_t i = 1; i < savedSignal.rawlen; i++) {
      f.print(savedSignal.rawbuf[i] * kRawTick);
      f.print(" ");
    }
    f.println();
  }

  f.close();
  return true;
}

static bool transmitSDFile(const String& filename) {
  if (!sdAvailable && !initSD()) return false;

  String path = "/ir/" + filename;
  File f = SD.open(path);
  if (!f) return false;

  String proto = "UNKNOWN";
  uint16_t bits = 32;
  uint64_t val = 0;
  bool isRaw = false;
  std::vector<uint16_t> rawData;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.startsWith("protocol:")) {
      proto = line.substring(9);
      proto.trim();
    } else if (line.startsWith("bits:")) {
      bits = line.substring(5).toInt();
    } else if (line.startsWith("value:")) {
      String vStr = line.substring(6);
      vStr.trim();
      val = strtoull(vStr.c_str(), nullptr, 0);
    } else if (line.startsWith("type: raw")) {
      isRaw = true;
    } else if (line.startsWith("data:") && isRaw) {
      String dataStr = line.substring(5);
      dataStr.trim();
      int start = 0;
      while (start < dataStr.length()) {
        int space = dataStr.indexOf(' ', start);
        if (space == -1) space = dataStr.length();
        uint16_t d = dataStr.substring(start, space).toInt();
        if (d > 0) rawData.push_back(d);
        start = space + 1;
      }
    }
  }
  f.close();

  irrecv.pause();
  setNeoPixelColour("red");

  // Prefer the decoded protocol when the file has one. Older files written before
  // the save-format fix carry both a "type: parsed" and a "type: raw" line; keying
  // off the protocol (not isRaw) makes those replay correctly instead of as raw.
  decode_type_t p = strToDecodeType(proto.c_str());
  if (p == PANASONIC) {
    irsend.sendPanasonic64(val);
  } else if (p != UNKNOWN) {
    irsend.send(p, val, bits);
  } else if (isRaw && rawData.size() > 0) {
    irsend.sendRaw(rawData.data(), rawData.size(), 38);
  } else {
    irsend.sendNEC(val, bits);
  }

  delay(40);
  setNeoPixelColour("0");
  irrecv.resume();
  return true;
}

// -------------------------------------------------------------
// Screen: Root IR Menu (Scrollable)
// -------------------------------------------------------------
static void drawIRMenu() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(22, 10, "[ IR Control ]");
  u8g2.drawHLine(0, 12, 128);

  const int visibleRows = 4;
  if (menuSelected < menuScrollOffset) {
    menuScrollOffset = menuSelected;
  } else if (menuSelected >= menuScrollOffset + visibleRows) {
    menuScrollOffset = menuSelected - visibleRows + 1;
  }

  u8g2.setFont(u8g2_font_5x8_tr);
  for (int i = 0; i < visibleRows; i++) {
    int idx = i + menuScrollOffset;
    if (idx >= IR_MENU_COUNT) break;
    int y = 23 + i * 10;
    if (idx == menuSelected) {
      u8g2.drawStr(2, y, ">");
      u8g2.drawStr(12, y, irMenuTitles[idx]);
    } else {
      u8g2.drawStr(12, y, irMenuTitles[idx]);
    }
  }

  // Scroll indicator bar
  u8g2.drawFrame(125, 14, 3, 40);
  int barH = max(4, 40 / IR_MENU_COUNT);
  int barY = 14 + (menuSelected * (40 - barH)) / (IR_MENU_COUNT - 1);
  u8g2.drawBox(125, barY, 3, barH);

  u8g2.drawStr(0, 64, "U/D:Nav R:Open L:Back");
  u8g2.sendBuffer();
}

// -------------------------------------------------------------
// Screen: TV Blaster
// -------------------------------------------------------------
static void drawBlasterScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(18, 10, "TV-B-Gone Blaster");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 23, blastLoopMode ? "Mode: Continuous Loop" : "Mode: Single Sweep");

  if (blastingActive) {
    u8g2.drawStr(0, 34, "Status: BLASTING...");
    String target = "Brand: " + String(tvPowerCodes[blastIndex].brand);
    u8g2.drawStr(0, 45, target.c_str());

    u8g2.drawFrame(0, 49, 128, 5);
    int barW = map(blastIndex, 0, TV_CODES_COUNT, 0, 126);
    u8g2.drawBox(1, 50, barW, 3);
  } else {
    u8g2.drawStr(0, 34, "Status: Idle / Ready");
    u8g2.drawStr(0, 45, "Press RIGHT to Start");
  }

  u8g2.drawStr(0, 64, "R:Start/Stop L:Back U:Mode");
  u8g2.sendBuffer();
}

static void updateBlaster() {
  if (blastingActive && (millis() - lastBlastTime >= BLAST_INTERVAL_MS)) {
    lastBlastTime = millis();
    const PowerCode& pc = tvPowerCodes[blastIndex];
    sendIRSignal(pc.protocol, pc.code, pc.bits);
    
    blastIndex++;
    if (blastIndex >= TV_CODES_COUNT) {
      blastIndex = 0;
      if (!blastLoopMode) {
        blastingActive = false;
        setNeoPixelColour("0");
      }
    }
    drawBlasterScreen();
  }
}

// -------------------------------------------------------------
// Screen: Projector Blaster
// -------------------------------------------------------------
static void drawProjectorScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(14, 10, "Projector Blaster");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 23, blastLoopMode ? "Mode: Continuous Loop" : "Mode: Single Sweep (2x)");

  if (blastingActive) {
    u8g2.drawStr(0, 34, "Status: BLASTING PROJ...");
    String target = "Proj: " + String(projPowerCodes[blastIndex].brand);
    u8g2.drawStr(0, 45, target.c_str());

    u8g2.drawFrame(0, 49, 128, 5);
    int barW = map(blastIndex, 0, PROJ_CODES_COUNT, 0, 126);
    u8g2.drawBox(1, 50, barW, 3);
  } else {
    u8g2.drawStr(0, 34, "Status: Ready (Epson/BenQ..)");
    u8g2.drawStr(0, 45, "Press RIGHT to Blast");
  }

  u8g2.drawStr(0, 64, "R:Start/Stop L:Back U:Mode");
  u8g2.sendBuffer();
}

static void updateProjector() {
  if (blastingActive && (millis() - lastBlastTime >= BLAST_INTERVAL_MS)) {
    lastBlastTime = millis();
    const PowerCode& pc = projPowerCodes[blastIndex];
    // Projectors require 2 pulses to confirm power toggles
    sendIRSignal(pc.protocol, pc.code, pc.bits);
    delay(30);
    sendIRSignal(pc.protocol, pc.code, pc.bits);

    blastIndex++;
    if (blastIndex >= PROJ_CODES_COUNT) {
      blastIndex = 0;
      if (!blastLoopMode) {
        blastingActive = false;
        setNeoPixelColour("0");
      }
    }
    drawProjectorScreen();
  }
}

// -------------------------------------------------------------
// Screen: TV Remote
// -------------------------------------------------------------
static void drawRemoteScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  String header = "Remote: " + String(brandRemotes[currentBrandIndex].name);
  u8g2.drawStr(2, 10, header.c_str());
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 24, "Command: ");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(55, 24, remoteCmdNames[currentCmdIndex]);

  u8g2.setFont(u8g2_font_5x8_tr);
  char hexBuf[24];
  uint64_t c = brandRemotes[currentBrandIndex].codes[currentCmdIndex];
  sprintf(hexBuf, "HEX: 0x%llX", (unsigned long long)c);
  u8g2.drawStr(0, 36, hexBuf);

  if (lastSentStatus.length() > 0) {
    u8g2.drawStr(0, 48, lastSentStatus.c_str());
  } else {
    u8g2.drawStr(0, 48, "Press RIGHT to Send");
  }

  u8g2.drawStr(0, 64, "U/D:Cmd L:Brand R:Send");
  u8g2.sendBuffer();
}

// -------------------------------------------------------------
// Screen: IR Sniffer & Waveform Visualizer
// -------------------------------------------------------------
static void drawSnifferScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  if (snifferWaveformMode) {
    u8g2.drawStr(12, 10, "[ Waveform View ]");
    u8g2.drawHLine(0, 12, 128);

    if (savedSignal.valid && savedSignal.rawlen > 2) {
      // Draw oscillogram logic analyzer waveform
      const int graphX = 4;
      const int graphW = 120;
      const int highY = 22;
      const int lowY  = 44;
      
      u8g2.drawFrame(graphX - 1, highY - 2, graphW + 2, (lowY - highY) + 5);

      int currentX = graphX;
      int pulseIdx = 1 + waveformScrollX;
      bool stateHigh = true; // First pulse in rawbuf is mark

      while (currentX < graphX + graphW && pulseIdx < savedSignal.rawlen) {
        uint32_t durationUs = savedSignal.rawbuf[pulseIdx] * kRawTick;
        int pWidth = max(2, (int)(durationUs / 90));
        if (currentX + pWidth > graphX + graphW) {
          pWidth = (graphX + graphW) - currentX;
        }

        int yLevel = stateHigh ? highY : lowY;
        u8g2.drawHLine(currentX, yLevel, pWidth);
        // Vertical transition
        u8g2.drawVLine(currentX + pWidth, highY, (lowY - highY) + 1);

        currentX += pWidth;
        stateHigh = !stateHigh;
        pulseIdx++;
      }

      // Legend & scroll
      char posBuf[32];
      sprintf(posBuf, "P:%d/%d  Len:%dus", pulseIdx, savedSignal.rawlen, (int)(savedSignal.rawbuf[1] * kRawTick));
      u8g2.setFont(u8g2_font_4x6_tr);
      u8g2.drawStr(4, 56, posBuf);
    } else {
      u8g2.setFont(u8g2_font_5x8_tr);
      u8g2.drawStr(10, 32, "No pulse data captured");
    }

    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(0, 64, "U/D:Scroll R:Data L:Back");
  } else {
    u8g2.drawStr(18, 10, "IR Sniffer / Rx");
    u8g2.drawHLine(0, 12, 128);

    u8g2.setFont(u8g2_font_5x8_tr);
    if (savedSignal.valid) {
      String proto = "Proto: " + typeToString(savedSignal.protocol) + " (" + String(savedSignal.bits) + "b)";
      u8g2.drawStr(0, 22, proto.c_str());

      char valBuf[32];
      sprintf(valBuf, "Code : 0x%llX", (unsigned long long)savedSignal.value);
      u8g2.drawStr(0, 32, valBuf);

      char addrBuf[32];
      sprintf(addrBuf, "Addr : 0x%X  Cmd: 0x%X", savedSignal.address, savedSignal.command);
      u8g2.drawStr(0, 42, addrBuf);

      if (lastSentStatus.length() > 0) {
        u8g2.drawStr(0, 52, lastSentStatus.c_str());
      } else {
        u8g2.drawStr(0, 52, "U:Waveform  D:Save SD");
      }
    } else {
      u8g2.drawStr(0, 28, snifferMsg.c_str());
      u8g2.drawStr(0, 42, "Aim remote at receiver");
      u8g2.drawStr(0, 52, "Press any remote button");
    }

    u8g2.drawStr(0, 64, "R:Replay  L:Back");
  }

  u8g2.sendBuffer();
}

static void updateSniffer() {
  if (irrecv.decode(&results)) {
    savedSignal.valid = true;
    savedSignal.protocol = results.decode_type;
    savedSignal.value = results.value;
    savedSignal.bits = results.bits;
    savedSignal.address = results.address;
    savedSignal.command = results.command;
    savedSignal.rawlen = min((uint16_t)results.rawlen, kCaptureRawMax);
    for (uint16_t i = 0; i < savedSignal.rawlen; i++) {
      savedSignal.rawbuf[i] = results.rawbuf[i];
    }
    waveformScrollX = 0;
    lastSentStatus = "";

    setNeoPixelColour("green");
    drawSnifferScreen();
    delay(80);
    setNeoPixelColour("0");

    irrecv.resume();
  }
}

// -------------------------------------------------------------
// Screen: IR Jammer / Disruptor
// -------------------------------------------------------------
static void drawJammerScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(12, 10, "IR Jammer / Flood");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 24, jammerMode == 0 ? "Mode: 38kHz Carrier Flood" : "Mode: AGC Noise Barrage");

  if (jammerActive) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(4, 38, "[ JAMMING ACTIVE ]");
    u8g2.setFont(u8g2_font_5x8_tr);
    unsigned long secs = (millis() - jammerStartTime) / 1000;
    char timeBuf[24];
    sprintf(timeBuf, "Blinding: %lus", secs);
    u8g2.drawStr(4, 50, timeBuf);
  } else {
    u8g2.drawStr(0, 36, "Status: Disabled / Ready");
    u8g2.drawStr(0, 48, "Press RIGHT to Activate");
  }

  u8g2.drawStr(0, 64, "R:Toggle  U:Mode  L:Back");
  u8g2.sendBuffer();
}

static void updateJammer() {
  if (jammerActive) {
    irrecv.pause();
    // Pulse barrage: saturates and blinds AGC on VS1838/TSOP receivers
    if (jammerMode == 0) {
      // 38kHz continuous flood with micro-burst gaps to protect LED
      for (int i = 0; i < 8; i++) {
        irsend.mark(random(600, 1500));
        irsend.space(random(100, 300));
      }
    } else {
      // Chaotic mark/space lengths disrupt protocol framing
      for (int i = 0; i < 12; i++) {
        irsend.mark(random(200, 800));
        irsend.space(random(150, 500));
      }
    }
    
    // Status visual update every 500ms
    if (millis() - lastJammerUpdate >= 500) {
      lastJammerUpdate = millis();
      setNeoPixelColour((millis() / 500) % 2 == 0 ? "magenta" : "red");
      drawJammerScreen();
    }
  }
}

// -------------------------------------------------------------
// Screen: SD Card Library (Flipper Zero .ir Browser)
// -------------------------------------------------------------
static void drawSDLibraryScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(12, 10, "SD Card IR Library");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_5x8_tr);
  if (!sdAvailable) {
    u8g2.drawStr(0, 30, "[ SD Init Failed / No Card ]");
    u8g2.drawStr(0, 44, "Insert FAT32 SD to /ir");
  } else if (sdIrFiles.empty()) {
    u8g2.drawStr(0, 30, "No .ir files found in /ir");
    u8g2.drawStr(0, 44, "Save codes in Sniffer");
  } else {
    const int visibleFiles = 3;
    if (sdFileIndex < sdScrollOffset) {
      sdScrollOffset = sdFileIndex;
    } else if (sdFileIndex >= sdScrollOffset + visibleFiles) {
      sdScrollOffset = sdFileIndex - visibleFiles + 1;
    }

    for (int i = 0; i < visibleFiles; i++) {
      int idx = i + sdScrollOffset;
      if (idx >= sdIrFiles.size()) break;
      int y = 24 + i * 10;
      if (idx == sdFileIndex) {
        u8g2.drawStr(2, y, ">");
        u8g2.drawStr(12, y, sdIrFiles[idx].c_str());
      } else {
        u8g2.drawStr(12, y, sdIrFiles[idx].c_str());
      }
    }

    if (sdStatusMsg.length() > 0) {
      u8g2.drawStr(0, 54, sdStatusMsg.c_str());
    } else {
      u8g2.drawStr(0, 54, "R:Transmit File");
    }
  }

  u8g2.drawStr(0, 64, "U/D:Nav  R:Send  L:Back");
  u8g2.sendBuffer();
}

// -------------------------------------------------------------
// Screen: IR Replayer
// -------------------------------------------------------------
static void drawReplayerScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(18, 10, "IR Cloner / Replay");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_5x8_tr);
  if (!savedSignal.valid) {
    u8g2.drawStr(0, 30, "No signal captured yet!");
    u8g2.drawStr(0, 44, "Capture in Sniffer first.");
  } else {
    String proto = "Proto: " + typeToString(savedSignal.protocol) + " (" + String(savedSignal.bits) + "b)";
    u8g2.drawStr(0, 24, proto.c_str());

    char valBuf[32];
    sprintf(valBuf, "Code : 0x%llX", (unsigned long long)savedSignal.value);
    u8g2.drawStr(0, 36, valBuf);

    if (lastSentStatus.length() > 0) {
      u8g2.drawStr(0, 48, lastSentStatus.c_str());
    } else {
      u8g2.drawStr(0, 48, "Press RIGHT to Clone/Send");
    }
  }

  u8g2.drawStr(0, 64, "R:Send  L:Back");
  u8g2.sendBuffer();
}

// -------------------------------------------------------------
// Lifecycle: Setup & Loop
// -------------------------------------------------------------
void irSetup() {
  if (!irInitialized) {
    irsend.begin();
    irrecv.enableIRIn();
    irInitialized = true;
  }
  irState = 0;
  menuSelected = 0;
  menuScrollOffset = 0;
  blastingActive = false;
  jammerActive = false;
  lastSentStatus = "";
  sdStatusMsg = "";
  snifferWaveformMode = false;
  drawIRMenu();
}

void irLoop() {
  switch (irState) {
    // ---------------------------------------------------------
    // 0: Root IR Menu
    // ---------------------------------------------------------
    case 0: {
      if (checkBtn(BUTTON_UP_PIN)) {
        menuSelected = (menuSelected - 1 + IR_MENU_COUNT) % IR_MENU_COUNT;
        drawIRMenu();
      }
      if (checkBtn(BUTTON_DOWN_PIN)) {
        menuSelected = (menuSelected + 1) % IR_MENU_COUNT;
        drawIRMenu();
      }
      if (checkBtn(BTN_PIN_RIGHT)) {
        irState = menuSelected + 1;
        blastingActive = false;
        jammerActive = false;
        lastSentStatus = "";
        sdStatusMsg = "";

        if (irState == 1) drawBlasterScreen();
        else if (irState == 2) drawProjectorScreen();
        else if (irState == 3) drawRemoteScreen();
        else if (irState == 4) {
          snifferWaveformMode = false;
          irrecv.resume();
          drawSnifferScreen();
        }
        else if (irState == 5) drawJammerScreen();
        else if (irState == 6) {
          reloadSDFiles();
          drawSDLibraryScreen();
        }
        else if (irState == 7) drawReplayerScreen();
      }
      break;
    }

    // ---------------------------------------------------------
    // 1: TV Blaster
    // ---------------------------------------------------------
    case 1: {
      updateBlaster();

      if (checkBtn(BUTTON_UP_PIN) || checkBtn(BUTTON_DOWN_PIN)) {
        blastLoopMode = !blastLoopMode;
        drawBlasterScreen();
      }
      if (checkBtn(BTN_PIN_RIGHT)) {
        blastingActive = !blastingActive;
        if (blastingActive) {
          blastIndex = 0;
          lastBlastTime = millis();
        } else {
          setNeoPixelColour("0");
        }
        drawBlasterScreen();
      }
      if (checkBtn(BTN_PIN_LEFT)) {
        blastingActive = false;
        setNeoPixelColour("0");
        irState = 0;
        drawIRMenu();
      }
      break;
    }

    // ---------------------------------------------------------
    // 2: Projector Blaster
    // ---------------------------------------------------------
    case 2: {
      updateProjector();

      if (checkBtn(BUTTON_UP_PIN) || checkBtn(BUTTON_DOWN_PIN)) {
        blastLoopMode = !blastLoopMode;
        drawProjectorScreen();
      }
      if (checkBtn(BTN_PIN_RIGHT)) {
        blastingActive = !blastingActive;
        if (blastingActive) {
          blastIndex = 0;
          lastBlastTime = millis();
        } else {
          setNeoPixelColour("0");
        }
        drawProjectorScreen();
      }
      if (checkBtn(BTN_PIN_LEFT)) {
        blastingActive = false;
        setNeoPixelColour("0");
        irState = 0;
        drawIRMenu();
      }
      break;
    }

    // ---------------------------------------------------------
    // 3: TV Remote
    // ---------------------------------------------------------
    case 3: {
      if (checkBtn(BUTTON_UP_PIN)) {
        currentCmdIndex = (currentCmdIndex - 1 + CMD_COUNT) % CMD_COUNT;
        lastSentStatus = "";
        drawRemoteScreen();
      }
      if (checkBtn(BUTTON_DOWN_PIN)) {
        currentCmdIndex = (currentCmdIndex + 1) % CMD_COUNT;
        lastSentStatus = "";
        drawRemoteScreen();
      }
      if (checkBtn(BTN_PIN_LEFT)) {
        currentBrandIndex = (currentBrandIndex + 1) % BRAND_COUNT;
        lastSentStatus = "";
        drawRemoteScreen();
      }
      if (checkBtn(BTN_PIN_RIGHT)) {
        const BrandRemote& br = brandRemotes[currentBrandIndex];
        uint64_t code = br.codes[currentCmdIndex];
        sendIRSignal(br.protocol, code, br.bits);
        lastSentStatus = ">> SENT: " + String(br.name) + " " + String(remoteCmdNames[currentCmdIndex]);
        drawRemoteScreen();
      }
      break;
    }

    // ---------------------------------------------------------
    // 4: IR Sniffer & Waveform Visualizer
    // ---------------------------------------------------------
    case 4: {
      updateSniffer();

      if (checkBtn(BUTTON_UP_PIN)) {
        if (snifferWaveformMode) {
          waveformScrollX = max(0, waveformScrollX - 2);
        } else {
          snifferWaveformMode = true;
        }
        drawSnifferScreen();
      }

      if (checkBtn(BUTTON_DOWN_PIN)) {
        if (snifferWaveformMode) {
          if (savedSignal.valid && waveformScrollX < savedSignal.rawlen - 5) {
            waveformScrollX += 2;
          }
        } else {
          // Save to SD card in Flipper Zero format
          if (savedSignal.valid) {
            bool ok = saveSignalToSD();
            lastSentStatus = ok ? "Saved to /ir/ SD!" : "SD Save Failed!";
          }
        }
        drawSnifferScreen();
      }

      if (checkBtn(BTN_PIN_RIGHT)) {
        if (snifferWaveformMode) {
          snifferWaveformMode = false;
          drawSnifferScreen();
        } else if (savedSignal.valid) {
          bool ok = replayCaptured();
          lastSentStatus = ok ? (savedSignal.protocol == UNKNOWN ? "Replayed (raw)!" : "Replayed Signal!")
                              : "Replay failed!";
          drawSnifferScreen();
        }
      }

      if (checkBtn(BTN_PIN_LEFT)) {
        if (snifferWaveformMode) {
          snifferWaveformMode = false;
          drawSnifferScreen();
        } else {
          irState = 0;
          drawIRMenu();
        }
      }
      break;
    }

    // ---------------------------------------------------------
    // 5: IR Jammer / Disruptor
    // ---------------------------------------------------------
    case 5: {
      updateJammer();

      if (checkBtn(BUTTON_UP_PIN) || checkBtn(BUTTON_DOWN_PIN)) {
        jammerMode = (jammerMode + 1) % 2;
        drawJammerScreen();
      }

      if (checkBtn(BTN_PIN_RIGHT)) {
        jammerActive = !jammerActive;
        if (jammerActive) {
          jammerStartTime = millis();
          lastJammerUpdate = millis();
          setNeoPixelColour("red");
        } else {
          setNeoPixelColour("0");
          irrecv.resume();
        }
        drawJammerScreen();
      }

      if (checkBtn(BTN_PIN_LEFT)) {
        jammerActive = false;
        setNeoPixelColour("0");
        irrecv.resume();
        irState = 0;
        drawIRMenu();
      }
      break;
    }

    // ---------------------------------------------------------
    // 6: SD Library
    // ---------------------------------------------------------
    case 6: {
      if (checkBtn(BUTTON_UP_PIN)) {
        if (!sdIrFiles.empty()) {
          sdFileIndex = (sdFileIndex - 1 + sdIrFiles.size()) % sdIrFiles.size();
          sdStatusMsg = "";
          drawSDLibraryScreen();
        }
      }

      if (checkBtn(BUTTON_DOWN_PIN)) {
        if (!sdIrFiles.empty()) {
          sdFileIndex = (sdFileIndex + 1) % sdIrFiles.size();
          sdStatusMsg = "";
          drawSDLibraryScreen();
        }
      }

      if (checkBtn(BTN_PIN_RIGHT)) {
        if (!sdIrFiles.empty() && sdFileIndex < sdIrFiles.size()) {
          sdStatusMsg = "Transmitting...";
          drawSDLibraryScreen();
          bool ok = transmitSDFile(sdIrFiles[sdFileIndex]);
          sdStatusMsg = ok ? "Sent: " + sdIrFiles[sdFileIndex] : "Failed to Send!";
          drawSDLibraryScreen();
        }
      }

      if (checkBtn(BTN_PIN_LEFT)) {
        irState = 0;
        drawIRMenu();
      }
      break;
    }

    // ---------------------------------------------------------
    // 7: IR Replayer
    // ---------------------------------------------------------
    case 7: {
      if (checkBtn(BTN_PIN_RIGHT)) {
        if (savedSignal.valid) {
          lastSentStatus = replayCaptured() ? "Signal Cloned & Sent!" : "Replay failed!";
          drawReplayerScreen();
        }
      }
      if (checkBtn(BTN_PIN_LEFT)) {
        irState = 0;
        drawIRMenu();
      }
      break;
    }
  }
}

} // namespace IrControl
