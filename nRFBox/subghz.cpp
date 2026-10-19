/* ____________________________
   This software is licensed under the MIT License:
   https://github.com/cifertech/nrfbox
   ________________________________________ */

#include "subghz.h"
#include "setting.h"
#include "icon.h"
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <vector>

namespace SubGhz {

// -------------------------------------------------------------
// CC1101 Hardware Register Map
// -------------------------------------------------------------
#define CC1101_IOCFG2      0x00
#define CC1101_IOCFG1      0x01
#define CC1101_IOCFG0      0x02
#define CC1101_FIFOTHR     0x03
#define CC1101_SYNC1       0x04
#define CC1101_SYNC0       0x05
#define CC1101_PKTLEN      0x06
#define CC1101_PKTCTRL1    0x07
#define CC1101_PKTCTRL0    0x08
#define CC1101_ADDR        0x09
#define CC1101_CHANNR      0x0A
#define CC1101_FSCTRL1     0x0B
#define CC1101_FSCTRL0     0x0C
#define CC1101_FREQ2       0x0D
#define CC1101_FREQ1       0x0E
#define CC1101_FREQ0       0x0F
#define CC1101_MDMCFG4     0x10
#define CC1101_MDMCFG3     0x11
#define CC1101_MDMCFG2     0x12
#define CC1101_MDMCFG1     0x13
#define CC1101_MDMCFG0     0x14
#define CC1101_DEVIATN     0x15
#define CC1101_MCSM2       0x16
#define CC1101_MCSM1       0x17
#define CC1101_MCSM0       0x18
#define CC1101_FOCCFG      0x19
#define CC1101_BSCFG       0x1A
#define CC1101_AGCCTRL2    0x1B
#define CC1101_AGCCTRL1    0x1C
#define CC1101_AGCCTRL0    0x1D
#define CC1101_WOREVT1     0x1E
#define CC1101_WOREVT0     0x1F
#define CC1101_WORCTRL     0x20
#define CC1101_FREND1      0x21
#define CC1101_FREND0      0x22
#define CC1101_FSCAL3      0x23
#define CC1101_FSCAL2      0x24
#define CC1101_FSCAL1      0x25
#define CC1101_FSCAL0      0x26
#define CC1101_RCCTRL1     0x27
#define CC1101_RCCTRL0     0x28
#define CC1101_FSTEST      0x29
#define CC1101_PTEST       0x2A
#define CC1101_AGCTEST     0x2B
#define CC1101_TEST2       0x2C
#define CC1101_TEST1       0x2D
#define CC1101_TEST0       0x2E

// Command Strobes
#define CC1101_SRES        0x30
#define CC1101_SFSTXON     0x31
#define CC1101_SXOFF       0x32
#define CC1101_SCAL        0x33
#define CC1101_SRX         0x34
#define CC1101_STX         0x35
#define CC1101_SIDLE       0x36
#define CC1101_SWOR        0x38
#define CC1101_SPWD        0x39
#define CC1101_SFRX        0x3A
#define CC1101_SFTX        0x3B
#define CC1101_SWORRST     0x3C
#define CC1101_SNOP        0x3D

// Status Registers (Read with 0xC0)
#define CC1101_PARTNUM     (0x30 | 0xC0)
#define CC1101_VERSION     (0x31 | 0xC0)
#define CC1101_FREQEST     (0x32 | 0xC0)
#define CC1101_LQI         (0x33 | 0xC0)
#define CC1101_RSSI        (0x34 | 0xC0)
#define CC1101_MARCSTATE   (0x35 | 0xC0)
#define CC1101_PATABLE     0x3E

// -------------------------------------------------------------
// Expanded Frequencies (Automotive, Gates, Security, ISM)
// -------------------------------------------------------------
struct SubGhzChannel {
  uint32_t freqHz;
  const char* name;
  const char* bandLabel;
};

static const SubGhzChannel channels[] = {
  { 300000000, "300.00M", "Generic ISM" },
  { 310000000, "310.00M", "US Automotive" },
  { 315000000, "315.00M", "US Fobs/TPMS" },
  { 318000000, "318.00M", "Linear/MegaCode" },
  { 330000000, "330.00M", "Asia Remotes" },
  { 345000000, "345.00M", "Honeywell Sec" },
  { 390000000, "390.00M", "Chamberlain/Lift" },
  { 418000000, "418.00M", "UK Remotes" },
  { 433075000, "433.07M", "LPD433 Ch01" },
  { 433920000, "433.92M", "Global Fobs/Gates" },
  { 434420000, "434.42M", "LPD433 Mid" },
  { 434775000, "434.77M", "LPD433 Ch69" },
  { 868350000, "868.35M", "EU Car/Alarms" },
  { 868850000, "868.85M", "Sommer/EU Gate" },
  { 868950000, "868.95M", "Security/Sensors" },
  { 915000000, "915.00M", "US 915M ISM" },
  { 928000000, "928.00M", "ISM Upper" }
};

static const int CHANNEL_COUNT = sizeof(channels) / sizeof(channels[0]);
static int currentChannelIdx = 9; // Default 433.92 MHz

// -------------------------------------------------------------
// Driver State & Buffers
// -------------------------------------------------------------
static bool cc1101Present = false;
static uint8_t cc1101Version = 0;
static uint32_t activeFreqHz = 433920000;

// Pulse Capture Buffer (ISR-safe)
static const uint16_t MAX_EDGES = 768;
static volatile uint32_t isrTimestamps[MAX_EDGES];
static volatile uint16_t isrEdgeCount = 0;
static volatile bool isrCapturing = false;
static volatile uint32_t isrLastEdgeUs = 0;

// Processed Raw Frame Buffer
struct RawSubSignal {
  bool valid;
  uint32_t frequency;
  uint16_t count;
  int32_t pulses[MAX_EDGES]; // positive = mark, negative = space (Flipper format)
  uint32_t totalDurationUs;
};

static RawSubSignal capturedSubSignal = {false, 433920000, 0, {0}, 0};

// UI & Menu States
// 0: Menu
// 1: Freq Analyzer (Peak Fob Detector)
// 2: Spectrum Sweep (Band graph)
// 3: RAW Sniffer & Waveform
// 4: Replay & SD Library
// 5: Multi-MCU / BW16 Bridge
static int subghzState = 0;
static int menuSelected = 0;
const int SUBGHZ_MENU_COUNT = 5;
static const char* subghzMenuTitles[SUBGHZ_MENU_COUNT] = {
  "Freq Analyzer",
  "Spectrum Sweep",
  "RAW Sniffer",
  "Replay / SD",
  "Dual-MCU Bridge"
};

static unsigned long lastBtnTime = 0;
static const unsigned long DEBOUNCE_MS = 160;
static int waveformScrollX = 0;
static String subghzStatusMsg = "";

// SD Card storage
static bool sdAvailable = false;
static std::vector<String> sdSubFiles;
static int sdFileIndex = 0;
static int sdScrollOffset = 0;

// Spectrum sweep buffers
static int8_t spectrumRssi[64];
static int spectrumBand = 1; // 0: 300-348, 1: 387-464, 2: 779-928

// -------------------------------------------------------------
// Hardware SPI CC1101 Low-Level Access
// -------------------------------------------------------------
static inline void ccSelect() {
  digitalWrite(CC1101_CSN_PIN, LOW);
}

static inline void ccDeselect() {
  digitalWrite(CC1101_CSN_PIN, HIGH);
}

static bool waitMiso() {
  uint32_t start = micros();
  while (digitalRead(19) == HIGH) { // MISO is GPIO 19
    if (micros() - start > 2000) return false;
  }
  return true;
}

static uint8_t ccStrobe(uint8_t cmd) {
  ccSelect();
  waitMiso();
  uint8_t res = SPI.transfer(cmd);
  ccDeselect();
  return res;
}

static void ccWriteReg(uint8_t addr, uint8_t val) {
  ccSelect();
  waitMiso();
  SPI.transfer(addr);
  SPI.transfer(val);
  ccDeselect();
}

static uint8_t ccReadReg(uint8_t addr) {
  ccSelect();
  waitMiso();
  SPI.transfer(addr | 0x80);
  uint8_t val = SPI.transfer(0x00);
  ccDeselect();
  return val;
}

static uint8_t ccReadStatus(uint8_t addr) {
  ccSelect();
  waitMiso();
  SPI.transfer(addr | 0xC0);
  uint8_t val = SPI.transfer(0x00);
  ccDeselect();
  return val;
}

static int ccReadRssiDbm() {
  uint8_t raw = ccReadStatus(CC1101_RSSI);
  int rssi;
  if (raw >= 128) {
    rssi = ((int)raw - 256) / 2 - 74;
  } else {
    rssi = (int)raw / 2 - 74;
  }
  return rssi;
}

static void ccSetFrequency(uint32_t freqHz) {
  activeFreqHz = freqHz;
  // Freq = (freqHz * 65536) / 26000000
  uint64_t num = ((uint64_t)freqHz) << 16;
  uint32_t reg = (uint32_t)((num + 13000000ULL) / 26000000ULL);

  ccStrobe(CC1101_SIDLE);
  ccWriteReg(CC1101_FREQ2, (reg >> 16) & 0xFF);
  ccWriteReg(CC1101_FREQ1, (reg >> 8) & 0xFF);
  ccWriteReg(CC1101_FREQ0, reg & 0xFF);
  ccStrobe(CC1101_SCAL);
  delayMicroseconds(250);
}

static void ccConfigureOOK(uint32_t freqHz) {
  ccStrobe(CC1101_SRES);
  delay(10);

  // Configure GDO pins:
  // GDO2: 0x0D = Serial Data Output in asynchronous OOK mode
  ccWriteReg(CC1101_IOCFG2, 0x0D);
  // GDO0: 0x2E = High impedance / Serial Data In for TX
  ccWriteReg(CC1101_IOCFG0, 0x0D);

  // FIFO Threshold & packet handling
  ccWriteReg(CC1101_FIFOTHR, 0x47);
  ccWriteReg(CC1101_PKTCTRL0, 0x32); // Asynchronous serial mode, infinite packet length
  ccWriteReg(CC1101_PKTCTRL1, 0x00);

  // Modulation: ASK/OOK (0x30), no Manchester, no preamble/sync
  ccWriteReg(CC1101_MDMCFG2, 0x30);
  // Filter bandwidth: wideband ~270 kHz for robust remote capture
  ccWriteReg(CC1101_MDMCFG4, 0xF8);
  ccWriteReg(CC1101_MDMCFG3, 0x93);
  ccWriteReg(CC1101_MDMCFG1, 0x00);
  ccWriteReg(CC1101_MDMCFG0, 0x00);

  // Frequency Synthesizer & Auto-Calibrate
  ccWriteReg(CC1101_FSCTRL1, 0x06);
  ccWriteReg(CC1101_FSCTRL0, 0x00);
  ccWriteReg(CC1101_MCSM0, 0x18);   // Auto calibrate when going from IDLE to RX/TX
  ccWriteReg(CC1101_MCSM1, 0x30);

  // AGC configuration
  ccWriteReg(CC1101_AGCCTRL2, 0x03);
  ccWriteReg(CC1101_AGCCTRL1, 0x00);
  ccWriteReg(CC1101_AGCCTRL0, 0x91);

  // PA Table: 10 dBm transmit power for OOK (index 0 = 0x00, index 1 = 0xC0)
  ccSelect();
  waitMiso();
  SPI.transfer(CC1101_PATABLE | 0x40);
  SPI.transfer(0x00);
  SPI.transfer(0xC0);
  ccDeselect();

  ccSetFrequency(freqHz);
  ccStrobe(CC1101_SRX);
}

// -------------------------------------------------------------
// Interrupt Service Routine for RAW Pulse Capture
// -------------------------------------------------------------
static void IRAM_ATTR subghzEdgeISR() {
  if (!isrCapturing) return;
  uint32_t now = micros();
  if (isrEdgeCount < MAX_EDGES) {
    isrTimestamps[isrEdgeCount++] = now;
    isrLastEdgeUs = now;
  }
}

static void startCapture() {
  isrEdgeCount = 0;
  isrLastEdgeUs = micros();
  isrCapturing = true;
  attachInterrupt(digitalPinToInterrupt(CC1101_GDO2_PIN), subghzEdgeISR, CHANGE);
  ccStrobe(CC1101_SRX);
}

static void stopCapture() {
  isrCapturing = false;
  detachInterrupt(digitalPinToInterrupt(CC1101_GDO2_PIN));
}

// -------------------------------------------------------------
// Button Debounce Helper
// -------------------------------------------------------------
static bool checkBtn(int pin) {
  if (digitalRead(pin) == LOW && (millis() - lastBtnTime > DEBOUNCE_MS)) {
    lastBtnTime = millis();
    return true;
  }
  return false;
}

// -------------------------------------------------------------
// SD Card Sub-GHz Storage (Flipper Zero .sub RAW format)
// -------------------------------------------------------------
static bool initSD() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  if (!SD.begin(SD_CS_PIN)) {
    sdAvailable = false;
    return false;
  }
  sdAvailable = true;
  if (!SD.exists("/subghz")) {
    SD.mkdir("/subghz");
  }
  return true;
}

static void reloadSDSubFiles() {
  sdSubFiles.clear();
  sdFileIndex = 0;
  sdScrollOffset = 0;
  if (!sdAvailable && !initSD()) return;

  File dir = SD.open("/subghz");
  if (!dir || !dir.isDirectory()) return;

  File entry = dir.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      String name = String(entry.name());
      if (name.endsWith(".sub") || name.endsWith(".SUB")) {
        int slash = name.lastIndexOf('/');
        if (slash >= 0) name = name.substring(slash + 1);
        sdSubFiles.push_back(name);
      }
    }
    entry.close();
    entry = dir.openNextFile();
  }
  dir.close();
}

static bool saveSubToSD() {
  if (!capturedSubSignal.valid || capturedSubSignal.count < 4) return false;
  if (!sdAvailable && !initSD()) return false;

  int num = 1;
  String filename;
  do {
    filename = "/subghz/fob_" + String(num++) + ".sub";
  } while (SD.exists(filename) && num < 999);

  File f = SD.open(filename, FILE_WRITE);
  if (!f) return false;

  f.println("Filetype: Flipper SubGhz RAW File");
  f.println("Version: 1");
  f.println("# Generated by nRFBox Sub-GHz Module");
  f.print("Frequency: ");
  f.println(capturedSubSignal.frequency);
  f.println("Preset: FuriHalSubGhzPresetOokAsync");
  f.println("Protocol: RAW");
  f.print("RAW_Data: ");

  for (uint16_t i = 0; i < capturedSubSignal.count; i++) {
    f.print(capturedSubSignal.pulses[i]);
    f.print(" ");
    if ((i + 1) % 16 == 0 && (i + 1) < capturedSubSignal.count) {
      f.println();
      f.print("RAW_Data: ");
    }
  }
  f.println();
  f.close();
  return true;
}

// -------------------------------------------------------------
// Transmit / Replay Captured Pulse Train
// -------------------------------------------------------------
static void transmitPulses(uint32_t frequency, const int32_t* pulses, uint16_t count) {
  if (count == 0) return;

  setNeoPixelColour("red");
  ccSetFrequency(frequency);

  // Put CC1101 in asynchronous TX mode
  ccStrobe(CC1101_SIDLE);
  ccWriteReg(CC1101_IOCFG0, 0x2E); // GDO0 as input
  ccStrobe(CC1101_STX);

  pinMode(CC1101_GDO0_PIN, OUTPUT);
  digitalWrite(CC1101_GDO0_PIN, LOW);

  // Modulate carrier according to mark (+) and space (-) durations
  for (uint16_t i = 0; i < count; i++) {
    int32_t p = pulses[i];
    if (p > 0) {
      digitalWrite(CC1101_GDO0_PIN, HIGH);
      delayMicroseconds(p);
    } else {
      digitalWrite(CC1101_GDO0_PIN, LOW);
      delayMicroseconds(-p);
    }
  }

  digitalWrite(CC1101_GDO0_PIN, LOW);
  ccStrobe(CC1101_SIDLE);
  setNeoPixelColour("0");
  ccStrobe(CC1101_SRX);
}

// -------------------------------------------------------------
// Screen 0: Root Sub-GHz Menu
// -------------------------------------------------------------
static void drawSubGhzMenu() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(14, 10, "[ Sub-GHz CC1101 ]");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_5x8_tr);

  if (!cc1101Present) {
    u8g2.drawStr(4, 26, "! CC1101 NOT DETECTED !");
    u8g2.drawStr(4, 38, "Check CS:12 GDO2:35 GDO0:0");
    u8g2.drawStr(4, 50, "Shared SPI: 18/19/23");
    u8g2.drawStr(0, 64, "Press RIGHT to Retry Init");
  } else {
    char statusBuf[32];
    sprintf(statusBuf, "Chip: CC1101 v%d  Freq:%s", cc1101Version, channels[currentChannelIdx].name);
    u8g2.drawStr(0, 22, statusBuf);

    for (int i = 0; i < SUBGHZ_MENU_COUNT; i++) {
      int y = 32 + i * 8;
      if (y > 58) break;
      if (i == menuSelected) {
        u8g2.drawStr(4, y, ">");
        u8g2.drawStr(14, y, subghzMenuTitles[i]);
      } else {
        u8g2.drawStr(14, y, subghzMenuTitles[i]);
      }
    }
    u8g2.drawStr(0, 64, "U/D:Nav R:Select L:Exit");
  }
  u8g2.sendBuffer();
}

// -------------------------------------------------------------
// Screen 1: Frequency Analyzer (Peak Fob Detector)
// -------------------------------------------------------------
static int analyzerPeakRssi = -120;
static int analyzerPeakChannel = 9;
static unsigned long analyzerLastSweep = 0;

static void drawAnalyzerScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(16, 10, "Sub-GHz Analyzer");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_5x8_tr);

  // Peak channel and target description
  const SubGhzChannel& peakCh = channels[analyzerPeakChannel];
  char fBuf[32];
  sprintf(fBuf, "Peak: %s", peakCh.name);
  u8g2.drawStr(0, 24, fBuf);

  u8g2.drawStr(70, 24, peakCh.bandLabel);

  char rssiBuf[32];
  sprintf(rssiBuf, "RSSI: %d dBm", analyzerPeakRssi);
  u8g2.drawStr(0, 36, rssiBuf);

  // Visual RSSI analog meter bar
  u8g2.drawFrame(0, 40, 128, 9);
  int meterWidth = map(constrain(analyzerPeakRssi, -100, -20), -100, -20, 0, 124);
  if (meterWidth > 0) {
    u8g2.drawBox(2, 42, meterWidth, 5);
  }

  if (analyzerPeakRssi > -65) {
    u8g2.drawStr(0, 56, "* SIGNAL DETECTED *");
  } else {
    u8g2.drawStr(0, 56, "Press remote near box...");
  }

  u8g2.drawStr(0, 64, "R:Capture Freq  L:Back");
  u8g2.sendBuffer();
}

static void updateAnalyzer() {
  if (millis() - analyzerLastSweep < 40) return;
  analyzerLastSweep = millis();

  // Sweep across channels, measure RSSI
  for (int i = 0; i < CHANNEL_COUNT; i++) {
    ccSetFrequency(channels[i].freqHz);
    ccStrobe(CC1101_SRX);
    delayMicroseconds(400);

    int rssi = ccReadRssiDbm();
    if (rssi > analyzerPeakRssi) {
      analyzerPeakRssi = rssi;
      analyzerPeakChannel = i;
      currentChannelIdx = i;
      setNeoPixelColour(rssi > -65 ? "green" : "0");
    }
  }

  // Decay peak slowly if no active signal
  if (analyzerPeakRssi > -110) {
    analyzerPeakRssi -= 2;
  }

  drawAnalyzerScreen();
}

// -------------------------------------------------------------
// Screen 2: Spectrum Sweep (Band graph)
// -------------------------------------------------------------
static void drawSpectrumScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tr);

  const char* bandName = "387-464 MHz";
  if (spectrumBand == 0) bandName = "300-348 MHz";
  else if (spectrumBand == 2) bandName = "779-928 MHz";

  char title[32];
  sprintf(title, "Sweep: %s", bandName);
  u8g2.drawStr(0, 8, title);
  u8g2.drawHLine(0, 10, 128);

  // Draw 64 vertical bars (2 pixels wide each)
  for (int i = 0; i < 64; i++) {
    int h = map(constrain((int)spectrumRssi[i], -105, -25), -105, -25, 1, 40);
    int x = i * 2;
    int y = 52 - h;
    u8g2.drawVLine(x, y, h);
  }

  u8g2.drawHLine(0, 53, 128);
  u8g2.drawStr(0, 63, "U/D:Band  L:Back");
  u8g2.sendBuffer();
}

static void updateSpectrum() {
  uint32_t startFreq = 387000000;
  uint32_t step = (464000000 - 387000000) / 64;
  if (spectrumBand == 0) {
    startFreq = 300000000;
    step = (348000000 - 300000000) / 64;
  } else if (spectrumBand == 2) {
    startFreq = 779000000;
    step = (928000000 - 779000000) / 64;
  }

  for (int i = 0; i < 64; i++) {
    uint32_t f = startFreq + (i * step);
    ccSetFrequency(f);
    ccStrobe(CC1101_SRX);
    delayMicroseconds(350);
    spectrumRssi[i] = (int8_t)ccReadRssiDbm();
  }

  drawSpectrumScreen();
}

// -------------------------------------------------------------
// Screen 3: RAW Sniffer & Waveform Visualizer
// -------------------------------------------------------------
static void drawSnifferScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(12, 10, "RAW OOK Sniffer");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_5x8_tr);

  // Current tuning
  char fBuf[32];
  sprintf(fBuf, "Freq: %s  Ch:%d/%d", channels[currentChannelIdx].name, currentChannelIdx + 1, CHANNEL_COUNT);
  u8g2.drawStr(0, 22, fBuf);

  if (capturedSubSignal.valid && capturedSubSignal.count > 4) {
    // Waveform view
    const int graphX = 4;
    const int graphW = 120;
    const int highY = 28;
    const int lowY  = 42;
    u8g2.drawFrame(graphX - 1, highY - 1, graphW + 2, (lowY - highY) + 3);

    int curX = graphX;
    int idx = waveformScrollX;
    while (curX < graphX + graphW && idx < capturedSubSignal.count) {
      int32_t p = capturedSubSignal.pulses[idx];
      bool isHigh = (p > 0);
      uint32_t dur = isHigh ? p : -p;
      int w = max(2, (int)(dur / 80));
      if (curX + w > graphX + graphW) w = (graphX + graphW) - curX;

      int y = isHigh ? highY : lowY;
      u8g2.drawHLine(curX, y, w);
      u8g2.drawVLine(curX + w, highY, (lowY - highY) + 1);

      curX += w;
      idx++;
    }

    char infBuf[32];
    sprintf(infBuf, "Edges:%d  Dur:%dms", capturedSubSignal.count, (int)(capturedSubSignal.totalDurationUs / 1000));
    u8g2.drawStr(0, 52, infBuf);

    if (subghzStatusMsg.length() > 0) {
      u8g2.drawStr(0, 62, subghzStatusMsg.c_str());
    } else {
      u8g2.drawStr(0, 62, "R:Replay  D:Save SD");
    }
  } else {
    u8g2.drawStr(0, 32, isrCapturing ? "Listening for RF carrier..." : "Ready to sniff");
    u8g2.drawStr(0, 44, "Aim key fob & press button");
    u8g2.drawStr(0, 54, "U/D:Change Frequency");
    u8g2.drawStr(0, 64, "R:Capture  L:Back");
  }

  u8g2.sendBuffer();
}

static void updateSniffer() {
  if (isrCapturing) {
    uint32_t now = micros();
    // If we have received edges and no transition for > 35ms, the transmission is complete
    if (isrEdgeCount > 8 && (now - isrLastEdgeUs > 35000)) {
      stopCapture();

      // Convert timestamps into mark/space pulse array
      capturedSubSignal.valid = true;
      capturedSubSignal.frequency = channels[currentChannelIdx].freqHz;
      capturedSubSignal.count = isrEdgeCount - 1;
      capturedSubSignal.totalDurationUs = 0;

      // First state read from GDO2
      bool state = (digitalRead(CC1101_GDO2_PIN) == LOW); // State during first duration

      for (uint16_t i = 0; i < capturedSubSignal.count; i++) {
        uint32_t delta = isrTimestamps[i + 1] - isrTimestamps[i];
        capturedSubSignal.totalDurationUs += delta;
        capturedSubSignal.pulses[i] = state ? (int32_t)delta : -(int32_t)delta;
        state = !state;
      }

      waveformScrollX = 0;
      subghzStatusMsg = "Captured! R:Replay";
      setNeoPixelColour("green");
      drawSnifferScreen();
      delay(80);
      setNeoPixelColour("0");
    }
  }
}

// -------------------------------------------------------------
// Screen 4: Replay & SD File Library
// -------------------------------------------------------------
static void drawSDLibraryScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(8, 10, "Sub-GHz SD Library");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_5x8_tr);
  if (!sdAvailable) {
    u8g2.drawStr(0, 30, "[ SD Init Failed / No Card ]");
    u8g2.drawStr(0, 44, "Insert FAT32 SD to /subghz");
  } else if (sdSubFiles.empty()) {
    u8g2.drawStr(0, 30, "No .sub files in /subghz");
    u8g2.drawStr(0, 44, "Capture in Sniffer first");
  } else {
    const int visibleFiles = 3;
    if (sdFileIndex < sdScrollOffset) {
      sdScrollOffset = sdFileIndex;
    } else if (sdFileIndex >= sdScrollOffset + visibleFiles) {
      sdScrollOffset = sdFileIndex - visibleFiles + 1;
    }

    for (int i = 0; i < visibleFiles; i++) {
      int idx = i + sdScrollOffset;
      if (idx >= sdSubFiles.size()) break;
      int y = 24 + i * 10;
      if (idx == sdFileIndex) {
        u8g2.drawStr(2, y, ">");
        u8g2.drawStr(12, y, sdSubFiles[idx].c_str());
      } else {
        u8g2.drawStr(12, y, sdSubFiles[idx].c_str());
      }
    }

    if (subghzStatusMsg.length() > 0) {
      u8g2.drawStr(0, 54, subghzStatusMsg.c_str());
    } else {
      u8g2.drawStr(0, 54, "R:Transmit File");
    }
  }

  u8g2.drawStr(0, 64, "U/D:Nav R:Send L:Back");
  u8g2.sendBuffer();
}

static bool transmitSDSubFile(const String& filename) {
  if (!sdAvailable && !initSD()) return false;

  String path = "/subghz/" + filename;
  File f = SD.open(path);
  if (!f) return false;

  uint32_t freq = 433920000;
  std::vector<int32_t> pulses;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.startsWith("Frequency:")) {
      freq = line.substring(10).toInt();
    } else if (line.startsWith("RAW_Data:")) {
      String dataStr = line.substring(9);
      dataStr.trim();
      int start = 0;
      while (start < dataStr.length()) {
        int space = dataStr.indexOf(' ', start);
        if (space == -1) space = dataStr.length();
        int32_t val = dataStr.substring(start, space).toInt();
        if (val != 0) pulses.push_back(val);
        start = space + 1;
      }
    }
  }
  f.close();

  if (pulses.empty()) return false;
  transmitPulses(freq, pulses.data(), pulses.size());
  return true;
}

// -------------------------------------------------------------
// Screen 5: Dual-MCU & BW16 Communication Bridge
// -------------------------------------------------------------
static void drawBridgeScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(6, 10, "Dual-MCU / BW16 Hub");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.drawStr(0, 20, "Offload Sub-GHz/5G to 2nd MCU:");
  u8g2.drawStr(0, 28, "Master ESP32  <--UART2-->  BW16/ESP32");
  u8g2.drawStr(0, 36, "TX: Pin 17                RX: Pin 16");
  u8g2.drawStr(0, 44, "Baud: 115200 (Auto-Sync Mode)");
  u8g2.drawStr(0, 52, "Conflict Fixed: Pin 5 SD CS isolated");

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 64, "R:Send Ping  L:Back");
  u8g2.sendBuffer();
}

// -------------------------------------------------------------
// Lifecycle: Setup, Loop & Teardown
// -------------------------------------------------------------
void subghzSetup() {
  pinMode(CC1101_CSN_PIN, OUTPUT);
  digitalWrite(CC1101_CSN_PIN, HIGH);
  pinMode(CC1101_GDO2_PIN, INPUT);
  pinMode(CC1101_GDO0_PIN, OUTPUT);
  digitalWrite(CC1101_GDO0_PIN, LOW);

  // Check CC1101 Version Register
  cc1101Version = ccReadStatus(CC1101_VERSION);
  uint8_t part = ccReadStatus(CC1101_PARTNUM);

  // Valid CC1101 chips return part == 0x00 and version 0x04 or 0x14
  if (part == 0x00 && (cc1101Version == 0x04 || cc1101Version == 0x14)) {
    cc1101Present = true;
    ccConfigureOOK(channels[currentChannelIdx].freqHz);
  } else {
    // If not detected, test with soft-init for breadboard testing
    cc1101Present = (cc1101Version != 0xFF && cc1101Version != 0x00);
    if (cc1101Present) {
      ccConfigureOOK(channels[currentChannelIdx].freqHz);
    }
  }

  subghzState = 0;
  menuSelected = 0;
  subghzStatusMsg = "";
  drawSubGhzMenu();
}

void subghzStop() {
  stopCapture();
  digitalWrite(CC1101_CSN_PIN, HIGH);
  ccStrobe(CC1101_SIDLE);
  ccStrobe(CC1101_SPWD); // Ultra low power sleep
}

void subghzLoop() {
  switch (subghzState) {
    // ---------------------------------------------------------
    // 0: Sub-GHz Main Menu
    // ---------------------------------------------------------
    case 0: {
      if (checkBtn(BUTTON_UP_PIN)) {
        menuSelected = (menuSelected - 1 + SUBGHZ_MENU_COUNT) % SUBGHZ_MENU_COUNT;
        drawSubGhzMenu();
      }
      if (checkBtn(BUTTON_DOWN_PIN)) {
        menuSelected = (menuSelected + 1) % SUBGHZ_MENU_COUNT;
        drawSubGhzMenu();
      }
      if (checkBtn(BTN_PIN_RIGHT)) {
        if (!cc1101Present) {
          subghzSetup();
          return;
        }
        subghzState = menuSelected + 1;
        subghzStatusMsg = "";

        if (subghzState == 1) {
          analyzerPeakRssi = -120;
          drawAnalyzerScreen();
        } else if (subghzState == 2) {
          drawSpectrumScreen();
        } else if (subghzState == 3) {
          startCapture();
          drawSnifferScreen();
        } else if (subghzState == 4) {
          reloadSDSubFiles();
          drawSDLibraryScreen();
        } else if (subghzState == 5) {
          drawBridgeScreen();
        }
      }
      break;
    }

    // ---------------------------------------------------------
    // 1: Frequency Analyzer
    // ---------------------------------------------------------
    case 1: {
      updateAnalyzer();

      if (checkBtn(BUTTON_UP_PIN)) {
        currentChannelIdx = (currentChannelIdx - 1 + CHANNEL_COUNT) % CHANNEL_COUNT;
        analyzerPeakChannel = currentChannelIdx;
        drawAnalyzerScreen();
      }
      if (checkBtn(BUTTON_DOWN_PIN)) {
        currentChannelIdx = (currentChannelIdx + 1) % CHANNEL_COUNT;
        analyzerPeakChannel = currentChannelIdx;
        drawAnalyzerScreen();
      }
      if (checkBtn(BTN_PIN_RIGHT)) {
        // Jump directly to sniffer locked onto peak frequency!
        subghzState = 3;
        startCapture();
        drawSnifferScreen();
      }
      if (checkBtn(BTN_PIN_LEFT)) {
        subghzState = 0;
        drawSubGhzMenu();
      }
      break;
    }

    // ---------------------------------------------------------
    // 2: Spectrum Sweep
    // ---------------------------------------------------------
    case 2: {
      updateSpectrum();

      if (checkBtn(BUTTON_UP_PIN) || checkBtn(BUTTON_DOWN_PIN)) {
        spectrumBand = (spectrumBand + 1) % 3;
        drawSpectrumScreen();
      }
      if (checkBtn(BTN_PIN_LEFT)) {
        subghzState = 0;
        drawSubGhzMenu();
      }
      break;
    }

    // ---------------------------------------------------------
    // 3: RAW Sniffer & Waveform
    // ---------------------------------------------------------
    case 3: {
      updateSniffer();

      if (checkBtn(BUTTON_UP_PIN)) {
        if (capturedSubSignal.valid) {
          waveformScrollX = max(0, waveformScrollX - 4);
        } else {
          currentChannelIdx = (currentChannelIdx - 1 + CHANNEL_COUNT) % CHANNEL_COUNT;
          ccSetFrequency(channels[currentChannelIdx].freqHz);
          startCapture();
        }
        drawSnifferScreen();
      }

      if (checkBtn(BUTTON_DOWN_PIN)) {
        if (capturedSubSignal.valid) {
          bool ok = saveSubToSD();
          subghzStatusMsg = ok ? "Saved to /subghz!" : "SD Save Failed!";
        } else {
          currentChannelIdx = (currentChannelIdx + 1) % CHANNEL_COUNT;
          ccSetFrequency(channels[currentChannelIdx].freqHz);
          startCapture();
        }
        drawSnifferScreen();
      }

      if (checkBtn(BTN_PIN_RIGHT)) {
        if (capturedSubSignal.valid) {
          subghzStatusMsg = "Transmitting...";
          drawSnifferScreen();
          transmitPulses(capturedSubSignal.frequency, capturedSubSignal.pulses, capturedSubSignal.count);
          subghzStatusMsg = "Replayed Signal!";
          drawSnifferScreen();
        } else {
          startCapture();
          drawSnifferScreen();
        }
      }

      if (checkBtn(BTN_PIN_LEFT)) {
        stopCapture();
        subghzState = 0;
        drawSubGhzMenu();
      }
      break;
    }

    // ---------------------------------------------------------
    // 4: Replay & SD Library
    // ---------------------------------------------------------
    case 4: {
      if (checkBtn(BUTTON_UP_PIN)) {
        if (!sdSubFiles.empty()) {
          sdFileIndex = (sdFileIndex - 1 + sdSubFiles.size()) % sdSubFiles.size();
          subghzStatusMsg = "";
          drawSDLibraryScreen();
        }
      }

      if (checkBtn(BUTTON_DOWN_PIN)) {
        if (!sdSubFiles.empty()) {
          sdFileIndex = (sdFileIndex + 1) % sdSubFiles.size();
          subghzStatusMsg = "";
          drawSDLibraryScreen();
        }
      }

      if (checkBtn(BTN_PIN_RIGHT)) {
        if (!sdSubFiles.empty() && sdFileIndex < sdSubFiles.size()) {
          subghzStatusMsg = "Transmitting...";
          drawSDLibraryScreen();
          bool ok = transmitSDSubFile(sdSubFiles[sdFileIndex]);
          subghzStatusMsg = ok ? "Sent: " + sdSubFiles[sdFileIndex] : "Failed to Send!";
          drawSDLibraryScreen();
        }
      }

      if (checkBtn(BTN_PIN_LEFT)) {
        subghzState = 0;
        drawSubGhzMenu();
      }
      break;
    }

    // ---------------------------------------------------------
    // 5: Dual-MCU Bridge
    // ---------------------------------------------------------
    case 5: {
      if (checkBtn(BTN_PIN_RIGHT)) {
        // Send test ping packet over Bridge Serial
        Serial2.begin(115200, SERIAL_8N1, 16, 17);
        Serial2.println("{\"cmd\":\"PING\",\"src\":\"nRFBox-ESP32\",\"ts\":" + String(millis()) + "}");
        setNeoPixelColour("blue");
        delay(50);
        setNeoPixelColour("0");
        drawBridgeScreen();
      }
      if (checkBtn(BTN_PIN_LEFT)) {
        subghzState = 0;
        drawSubGhzMenu();
      }
      break;
    }
  }
}

} // namespace SubGhz
