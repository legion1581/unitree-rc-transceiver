// =============================================================================
//  Unitree 2.4 GHz RC  <->  LilyGO T3-S3 (SX1280)   —   receiver + transmitter
//
//  The Unitree handheld remote and the robot's RF module (SX1280) speak
//  LoRa at 2.4 GHz. This firmware turns a LilyGO T3-S3 (SX1280PA) into either end:
//    • RECEIVE  — decode a real remote's uplink (sticks/buttons/checksum)
//    • DRIVE  — transmit as a remote: beacon the uplink so a searching robot bonds to
//               us and acts on our sticks/buttons. The robot answers with an ACK
//               (its downlink), which we show as link state + RSSI, exactly like a
//               real remote's "DL" LED and signal bars.
//
//  Everything (air-id, home channel, hop set) is derived from the remote's printed
//  10-digit ID — see docs/PROTOCOL.md. Verified end-to-end against a Go2: driving
//  makes rt/wirelesscontroller track our emulated sticks.
//
//  Two board notes that make or break TX on the T3-S3 SX1280PA:
//    1. RF switch (FEM): TX=GPIO10, RX=GPIO21 must be driven (setRfSwitchPins), or
//       the front end stays in RX and nothing is transmitted.
//    2. Crystal: the SX1280 assumes an exact 52 MHz reference; a plain crystal is
//       off by tens of ppm, so commanded frequencies miss the module's narrow RX
//       window. We correct in software (ppm), auto-measured by 'calibrate'.
//
//  Serial @115200, newline-terminated. Type 'help' for the command list.
// =============================================================================
#include <RadioLib.h>
#include <U8g2lib.h>
#include <cstring>

// ---- Onboard SSD1306 128x64 OLED (I2C 0x3C, SDA=18 SCL=17) ----
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, /*SCL=*/17, /*SDA=*/18);

// ---- LilyGO T3-S3 SX1280 pin map ----
#define PIN_SCK  5
#define PIN_MISO 3
#define PIN_MOSI 6
#define PIN_CS   7
#define PIN_DIO1 9
#define PIN_BUSY 36
#define PIN_RST  8
#define PIN_LED  37     // onboard user LED — lit while the link is alive
#define PIN_BTN  0      // BOOT button — cycles the active remote profile
// SX1280PA RF switch (FEM). LilyGO docs: TX=1 -> PA (transmit), TX=0/RX=1 -> LNA
// (receive). On the plain SX1280 board (no FEM) these GPIOs are unused, so driving
// them is harmless. Without this the FEM stays in RX and TX never leaves the chip.
#define PIN_RF_RXEN 21
#define PIN_RF_TXEN 10

SX1280 radio = new Module(PIN_CS, PIN_DIO1, PIN_RST, PIN_BUSY);

// ---- radio config (the link settings: SF5 / BW406 / CR 4-6 LI /
//      preamble 12 / explicit header / CRC on / default sync 0x1424) ----
struct Cfg {
  float    freq     = 2437.900; // MHz (nominal; the live channel is set per profile)
  float    bw       = 406.25;   // kHz
  uint8_t  sf       = 5;
  uint8_t  cr       = 6;        // 4/6; long-interleaving forced in applyConfig()
  uint16_t preamble = 12;
  uint8_t  syncByte = 0x12;     // -> LoRa sync 0x1424 (SX1280 default; module uses it)
  uint8_t  syncCtl  = 0x44;
  int8_t   power    = 3;        // dBm. The SX1280PA FEM turns ~3 dBm into ~20 dBm;
                                // >~5 dBm into the FEM can DAMAGE it. Raise with 'pwr'
                                // only on the non-PA board.
} cfg;
bool rfSwitchEnabled = true;    // drive the FEM switch (toggle with 'rfsw on|off')

// Crystal error correction (ppm). The SX1280 assumes a perfect 52 MHz reference;
// this board's crystal reads high, so commanded frequencies come out high and miss
// the module's narrow receive window. We subtract the error in software.
// 'calibrate' measures it against the live robot; 'ppm <v>' sets it by hand.
float crystalPpm = -49.0f;

enum Mode { IDLE, RECEIVE };
Mode mode = IDLE;
volatile bool rxFlag = false;
bool verbose = false;           // 'verbose on' -> print each received frame as hex
uint32_t rxAckMs = 0, rxAckCount = 0;   // last downlink ACK we sent back to the remote
IRAM_ATTR void onDio1() { rxFlag = true; }

uint16_t rawSync() {
  uint8_t msb = (cfg.syncByte & 0xF0) | ((cfg.syncCtl & 0xF0) >> 4);
  uint8_t lsb = ((cfg.syncByte & 0x0F) << 4) | (cfg.syncCtl & 0x0F);
  return ((uint16_t)msb << 8) | lsb;
}

int applyConfig() {
  // Set the FEM switch pins BEFORE begin so TX/RX routing is right from the start.
  if (rfSwitchEnabled) radio.setRfSwitchPins(PIN_RF_RXEN, PIN_RF_TXEN);
  int st = radio.begin(cfg.freq, cfg.bw, cfg.sf, cfg.cr, cfg.syncByte, cfg.power, cfg.preamble);
  if (st != RADIOLIB_ERR_NONE) return st;
  radio.setSyncWord(cfg.syncByte, cfg.syncCtl);
  // The module uses SX1280 CR 0x06 = 4/6 LONG INTERLEAVING. RadioLib's plain cr=6 is
  // 0x02 (short); force LI so our TX matches what the module's receiver expects.
  radio.setCodingRate(6, true);
  radio.setDio1Action(onDio1);
  return RADIOLIB_ERR_NONE;
}
void startRx() { rxFlag = false; radio.startReceive(); }

// ===== link math (see docs/PROTOCOL.md) =====================================
// CHAN[] is the fixed 100-entry channel map.
static const uint8_t CHAN[100] = {
  26,10,19,56,15,24,53,62,21,58,17, 5,39,23, 2,61,20, 7,41,25, 4,63,22, 9,43,
  51,35,69, 6,65,49,28,37,71, 8,67,30,64,48,52,11,70,32,66,50,54,13,72,34,68,
   1,60,44,31,40,74, 3,62,46,33,42,55,14,73,27,36,45,57,16,75,29,38,47,59,18,76,
  85,94,81,90,99,78,87,96,83,92,80,89,98,77,86,95,82,91,100,79,88,97,84,93 };
struct Derived { uint8_t airA, airB, n; float homeMHz, hopMHz[4]; };
static Derived derive(uint64_t id) {
  Derived d; uint8_t n = id % 100, col = n % 25, row = n / 25;
  d.n   = n;
  d.airA = (uint8_t)((id + (id / 1000) * 24) & 0xFF);                               // last 3 digits mod 256
  d.airB = (uint8_t)((((id / 10000) % 100) + ((id / 1000000) % 100) * 18) & 0xFF);  // (month + 18*day) mod 256
  for (int r = 0; r < 4; r++)
    d.hopMHz[r] = (CHAN[r * 25 + col] * 900000.0f + 2401000000.0f) / 1e6f;          // 2401.0 + 0.9*CHAN
  d.homeMHz = d.hopMHz[row];
  return d;
}

// Default remote profiles. These are EXAMPLE placeholders — set your own remote at
// runtime with 'setid <6-char code>' (the code printed on the remote), or edit these.
// The 10-digit ID is derived from the printed code (see docs/PROTOCOL.md §2).
struct RemoteID { const char* code; uint64_t id; };
static const RemoteID REMOTES[] = {
  {"RC-1", 2500010001ULL},
  {"RC-2", 2500010002ULL},
  {"RC-3", 2500010003ULL},
};
struct RemProfile { char code[12]; uint64_t id; float home; uint8_t a, b; };
static const int NPROF = sizeof(REMOTES) / sizeof(REMOTES[0]);
static RemProfile PROFILES[NPROF];
int activeProf = 0;

// The 6-char code printed on the remote is base 36 (0-9, A-Z). Convert it to the
// 10-digit ID (docs/PROTOCOL.md §2.1). Returns 0 if the string is not 6 valid chars.
static int b36(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'Z') return c - 'A' + 10;
  if (c >= 'a' && c <= 'z') return c - 'a' + 10;
  return -1;
}
static uint64_t codeToId(const char* s) {
  int v[6];
  for (int i = 0; i < 6; i++) { if (!s[i]) return 0; v[i] = b36(s[i]); if (v[i] < 0) return 0; }
  if (s[6]) return 0;                                      // must be exactly 6 chars
  uint64_t f0 = (v[1] + 21) % 100, f1 = v[2] % 100, f2 = v[3] % 100, f3 = (v[4] * 36 + v[5]) % 10000;
  return f0 * 100000000ULL + f1 * 1000000ULL + f2 * 10000ULL + f3;  // "%02d%02d%02d%04d"
}
static void setProfile(int i, const char* code, uint64_t id) {
  Derived d = derive(id);
  strncpy(PROFILES[i].code, code, sizeof(PROFILES[i].code) - 1);
  PROFILES[i].code[sizeof(PROFILES[i].code) - 1] = 0;
  PROFILES[i].id = id; PROFILES[i].home = d.homeMHz; PROFILES[i].a = d.airA; PROFILES[i].b = d.airB;
}
static void buildProfiles() {
  for (int i = 0; i < NPROF; i++) setProfile(i, REMOTES[i].code, REMOTES[i].id);
}
// Camp frequency for a profile, crystal-corrected.
static float profileFreq(int i) { return PROFILES[i].home * (1.0f + crystalPpm * 1e-6f); }

const char* BTN1[8] = {"R1","L1","St","Se","R2","L2","F1","F2"};
const char* BTN2[8] = {"A","B","X","Y","Up","Ri","Dn","Lf"};

// ---- injected remote state (what DRIVE transmits) ----
struct Inj { uint8_t id0 = 0xda, id1 = 0xf1, b1 = 0, b2 = 0;
             float lx = -0.0f, rx = -0.0f, ry = -0.0f, ly = 0.0f; } inj;
void buildPkt(uint8_t* p) {
  memset(p, 0, 42);
  p[0]=inj.id0; p[1]=inj.id1; p[2]=0x55; p[3]=0x51; p[4]=inj.b1; p[5]=inj.b2;
  memcpy(p+6,&inj.lx,4); memcpy(p+10,&inj.rx,4); memcpy(p+14,&inj.ry,4); memcpy(p+22,&inj.ly,4);
  uint16_t s=0; for (int i=4;i<40;i++) s+=p[i];
  p[40]=(s>>8)&0xFF; p[41]=s&0xFF;                       // checksum: BE sum over [4:40]
}

// ---- decoded RX state ----
struct Decoded { uint8_t id0, id1, b1, b2; float lx, rx, ry, ly; bool ok; } dec;
float lastRSSI = -120, lastSNR = 0; uint32_t pktCount = 0;

// ---- DRIVE / bond state ----
bool     driveOn = false, driveWave = false;
uint32_t driveLastTx = 0, driveT0 = 0;
uint32_t DRIVE_GAP_MS = 90;           // beacon period (~11 Hz; real remote ~10-17 Hz)
uint32_t ackCount = 0, lastAckMs = 0; // robot downlink (ACK) -> "bonded"/DL + RSSI
float    lastAckRSSI = -120;

// ===================== OLED (core-0 render task) ============================
struct Snap {
  float freq; int prof; bool driving;
  // receive (RX)
  Decoded rx; float rxRssi; uint32_t pkt;
  // drive (TX + downlink)
  float lx, ly, rrx, rry; uint8_t b1, b2; bool bonded; float ackRssi;
};
static Snap gSnap;
static SemaphoreHandle_t stateMux;
static char gMsgA[24] = "", gMsgB[24] = "";
static volatile uint32_t gMsgUntil = 0;

void setBanner(const char* a, const char* b, uint32_t ms) {
  if (!stateMux) return;
  xSemaphoreTake(stateMux, portMAX_DELAY);
  strncpy(gMsgA, a, sizeof(gMsgA)-1); gMsgA[sizeof(gMsgA)-1]=0;
  strncpy(gMsgB, b, sizeof(gMsgB)-1); gMsgB[sizeof(gMsgB)-1]=0;
  gMsgUntil = millis() + ms;
  xSemaphoreGive(stateMux);
}
void publishSnap() {
  if (!stateMux) return;
  xSemaphoreTake(stateMux, portMAX_DELAY);
  gSnap.freq = cfg.freq; gSnap.prof = activeProf; gSnap.driving = driveOn;
  gSnap.rx = dec; gSnap.rxRssi = lastRSSI; gSnap.pkt = pktCount;
  gSnap.lx = inj.lx; gSnap.ly = inj.ly; gSnap.rrx = inj.rx; gSnap.rry = inj.ry;
  gSnap.b1 = inj.b1; gSnap.b2 = inj.b2;
  gSnap.bonded = lastAckMs && (millis() - lastAckMs < 400); gSnap.ackRssi = lastAckRSSI;
  xSemaphoreGive(stateMux);
}

static float rdF(const uint8_t* p, int o) { float f; memcpy(&f, p + o, 4); return f; }

void storeDecoded(const uint8_t* b, size_t len, int st) {
  pktCount++;   // RSSI/SNR are captured by the caller BEFORE any ACK transmit
  bool crcOk = (st == RADIOLIB_ERR_NONE);
  bool magic = (len >= 4 && b[2] == 0x55 && b[3] == 0x51);
  if (len >= 42 && magic) {
    dec.id0=b[0]; dec.id1=b[1]; dec.b1=b[4]; dec.b2=b[5];
    dec.lx=rdF(b,6); dec.rx=rdF(b,10); dec.ry=rdF(b,14); dec.ly=rdF(b,22);
    uint16_t s=0; for (int i=4;i<40;i++) s+=b[i];
    dec.ok = ((s & 0xFFFF) == (uint16_t)((b[40]<<8)|b[41])) && crcOk;
  }
  publishSnap();
}

void drawStick(int x0, int y0, int w, float ax, float ay) {
  oled.drawFrame(x0, y0, w, w);
  int cx = x0 + w/2, cy = y0 + w/2, r = w/2 - 2;
  oled.drawHLine(x0, cy, w); oled.drawVLine(cx, y0, w);
  if (ax < -1) ax = -1; if (ax > 1) ax = 1;
  if (ay < -1) ay = -1; if (ay > 1) ay = 1;
  oled.drawDisc(cx + (int)(ax * r), cy - (int)(ay * r), 2);
}
void drawButtons(int y, uint8_t b1, uint8_t b2) {
  // Full-width row below the sticks. Lay labels out one at a time and stop before the
  // right edge so any number of simultaneous presses clips cleanly instead of overflowing.
  const int x0 = 2, maxX = 127;
  int x = x0; bool any = false, more = false;
  for (int k = 0; k < 16; k++) {
    const char* lbl = (k < 8) ? ((b1 & (1 << k))     ? BTN1[k]     : nullptr)
                              : ((b2 & (1 << (k-8)))  ? BTN2[k-8]   : nullptr);
    if (!lbl) continue;
    int w = oled.getStrWidth(lbl);
    if (x + w > maxX) { more = true; break; }     // no room for this one -> clip here
    oled.drawStr(x, y, lbl);
    x += w + 4;                                    // 4px gap between labels
    any = true;
  }
  if (!any) oled.drawStr(x0, y, "-");
  else if (more && x + 5 <= 128) oled.drawStr(x, y, ">");   // more are pressed than fit
}
void drawRssiBars(int x, int y, float rssi) {    // -40..-100 dBm -> 5..0 bars
  int bars = (int)((rssi + 100) / 12); if (bars < 0) bars = 0; if (bars > 5) bars = 5;
  for (int i = 0; i < 5; i++) {
    int h = 3 + i*2;
    if (i < bars) oled.drawBox(x + i*4, y - h, 3, h);
    else          oled.drawFrame(x + i*4, y - h, 3, h);
  }
}

void renderDrive(const Snap& s) {
  char l[40];
  oled.setFont(u8g2_font_5x7_tf);
  snprintf(l, sizeof(l), "DRIVE %s %02x%02x", PROFILES[s.prof].code, inj.id0, inj.id1);
  oled.drawStr(0, 7, l);
  // bond state from the robot's downlink (ACK) = a real remote's "DL" indicator
  if (s.bonded) { oled.drawStr(92, 7, "DL"); drawRssiBars(104, 8, s.ackRssi); }
  else            oled.drawStr(92, 7, "....");
  drawStick(2, 11, 34, s.lx, s.ly);      // left stick
  drawStick(92, 11, 34, s.rrx, s.rry);   // right stick
  oled.drawStr(14, 62, "L"); oled.drawStr(110, 62, "R");
  snprintf(l, sizeof(l), "%.3f", s.freq); oled.drawStr(40, 22, l);
  snprintf(l, sizeof(l), s.bonded ? "BONDED %ddBm" : "searching...", (int)s.ackRssi);
  oled.drawStr(40, 34, l);
  drawButtons(55, s.b1, s.b2);
}
void renderReceive(const Snap& s) {
  char l[40];
  oled.setFont(u8g2_font_5x7_tf);
  snprintf(l, sizeof(l), "RX %s %ddBm #%lu", PROFILES[s.prof].code, (int)s.rxRssi,
           (unsigned long)s.pkt);
  oled.drawStr(0, 7, l);
  { bool lk = rxAckMs && (millis()-rxAckMs < 400);
    oled.drawStr(116, 7, lk ? "DL" : "dl"); }   // we are the robot: DL bright while ACKs flow
  drawStick(2, 11, 34, s.rx.lx, s.rx.ly);
  drawStick(92, 11, 34, s.rx.rx, s.rx.ry);
  oled.drawStr(14, 62, "L"); oled.drawStr(110, 62, "R");
  snprintf(l, sizeof(l), "%.3f", s.freq); oled.drawStr(40, 22, l);
  oled.drawStr(40, 34, s.rx.ok ? "CKSUM OK" : "no link");
  drawButtons(55, s.rx.b1, s.rx.b2);
}
void renderState(const Snap& s) {
  oled.clearBuffer();
  if (s.driving) renderDrive(s); else renderReceive(s);
  oled.sendBuffer();
}
void oledMsg(const char* a, const char* b2) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 12, a); if (b2) oled.drawStr(0, 26, b2);
  oled.sendBuffer();
}
void renderTask(void*) {
  for (;;) {
    Snap s; char a[24], b[24]; uint32_t until;
    xSemaphoreTake(stateMux, portMAX_DELAY);
    s = gSnap; until = gMsgUntil;
    strncpy(a, gMsgA, sizeof(a)); strncpy(b, gMsgB, sizeof(b));
    xSemaphoreGive(stateMux);
    if (millis() < until) oledMsg(a, b); else renderState(s);
    vTaskDelay(pdMS_TO_TICKS(12));          // ~60 Hz redraw (I2C at 1 MHz keeps it quick)
  }
}

// ===================== connection LED =======================================
uint32_t lastLinkMs = 0;   // last good RX (receive) OR last ACK (drive)
void updateLinkLed() {
  bool alive = (driveOn && lastAckMs && millis()-lastAckMs < 600) ||
               (!driveOn && lastLinkMs && millis()-lastLinkMs < 600);
  digitalWrite(PIN_LED, alive ? HIGH : LOW);
}

// ===================== RECEIVE ================================================
void applyProfile(int i) {
  i = ((i % NPROF) + NPROF) % NPROF;
  activeProf = i;
  inj.id0 = PROFILES[i].a; inj.id1 = PROFILES[i].b;
  cfg.freq = profileFreq(i);
  mode = RECEIVE; driveOn = false;
  applyConfig(); startRx();
  if (Serial) Serial.printf("[rem] #%d %s  air=%02x%02x  camp=%.3f MHz\n",
                            i+1, PROFILES[i].code, PROFILES[i].a, PROFILES[i].b, cfg.freq);
  char a[24], b[24];
  snprintf(a, sizeof(a), "Remote %d/%d", i+1, NPROF);
  snprintf(b, sizeof(b), "%s %.1f", PROFILES[i].code, cfg.freq);
  setBanner(a, b, 700);
}
void pollButton() {
  static int prev = HIGH; static uint32_t lastMs = 0;
  int cur = digitalRead(PIN_BTN);
  if (cur == LOW && prev == HIGH && millis()-lastMs > 220) { lastMs = millis(); applyProfile(activeProf+1); }
  prev = cur;
}
void reportPacket() {
  size_t len = radio.getPacketLength();
  uint8_t buf[64]; if (len > sizeof(buf)) len = sizeof(buf);
  int st = radio.readData(buf, len);
  lastRSSI = radio.getRSSI(); lastSNR = radio.getSNR();   // grab NOW: an ACK TX clobbers these
  // Emulate the robot's downlink: the instant we accept the remote's addressed frame, fire
  // the ACK straight back on this same channel. That ACK is the remote's "DL" LED + RSSI, and
  // it makes a SEARCHING remote stop channel-hopping and LOCK to us -> frames stop being choppy
  // and the update rate jumps. The ACK must land in the remote's short post-TX window, so this
  // runs before the (slower) decode/publish below and keeps the radio on cfg.freq (ppm-corrected).
  if (st == RADIOLIB_ERR_NONE) {
    uint8_t a0 = PROFILES[activeProf].a, a1 = PROFILES[activeProf].b;
    if (len >= 4 && buf[0]==0xFF && buf[1]==0xFF && buf[2]==a0 && buf[3]==a1) {
      uint8_t ack[4] = { 0xFF, 0xFF, a0, a1 };   // bonded-data ACK (PROTOCOL.md 5.2)
      radio.transmit(ack, 4); rxAckMs = millis(); rxAckCount++;
    } else if (len >= 2 && buf[0]==a0 && buf[1]==a1) {
      uint8_t ack[3] = { a0, a1, 0x50 };         // unicast ACK (PROTOCOL.md 5.2)
      radio.transmit(ack, 3); rxAckMs = millis(); rxAckCount++;
    }
  }
  storeDecoded(buf, len, st);
  rxFlag = false; radio.startReceive();
  if (dec.ok) lastLinkMs = millis();
  if (!Serial || !verbose) return;        // per-frame dump only in verbose mode
  Serial.print("RX: ");
  for (size_t i=0;i<len;i++) Serial.printf("%02x ", buf[i]);
  if (st == RADIOLIB_ERR_CRC_MISMATCH) Serial.print(" [CRC BAD]");
  Serial.printf(" [RSSI %.1f SNR %.1f]\n", lastRSSI, lastSNR);
}

// ===================== channel-finding helpers (blocking) ===================
bool serialAbort() { if (Serial.available()) { while (Serial.available()) Serial.read(); return true; } return false; }
void fscan() {
  applyConfig();
  Serial.println("[fscan] 2400..2490 MHz, listening for a remote beacon (any key aborts)");
  for (float f = 2400.0; f <= 2490.0; f += cfg.bw/1000.0) {
    if (serialAbort()) { Serial.println("[fscan] aborted"); break; }
    cfg.freq = f; radio.standby(); radio.setFrequency(f); startRx();
    uint32_t t = millis();
    while (millis()-t < 250) {
      if (rxFlag) { rxFlag=false; size_t len=radio.getPacketLength(); uint8_t b[64];
        if (len>sizeof(b)) len=sizeof(b);
        if (radio.readData(b,len)==RADIOLIB_ERR_NONE)
          Serial.printf("[fscan] HIT %.3f MHz  %02x %02x %02x %02x ...  rssi=%.0f\n",
                        f, b[0],b[1],b[2],b[3], radio.getRSSI());
        radio.startReceive();
      }
    }
  }
  Serial.println("[fscan] done");
}
void hunt() {
  applyConfig();
  Serial.println("[hunt] CAD preamble sweep 2400..2490 MHz (any key aborts)");
  float step = cfg.bw/1000.0;
  for (float f = 2400.0; f <= 2490.0; f += step) {
    if (serialAbort()) { Serial.println("[hunt] aborted"); break; }
    radio.standby(); radio.setFrequency(f);
    int hits=0, tot=0; uint32_t t=millis();
    while (millis()-t < 300) { if (radio.scanChannel()==RADIOLIB_LORA_DETECTED) hits++; tot++; }
    if (hits) Serial.printf("[hunt] *HIT* %.3f MHz  %d/%d\n", f, hits, tot);
  }
  Serial.println("[hunt] done");
}

// ===================== DRIVE (emulator) =====================================
// Camp on the profile's crystal-corrected home channel and beacon the uplink. A
// searching robot (no remote bonded) bonds on the first addressed frame and acts
// on our sticks/buttons; it answers with an ACK ('<air_A> <air_B> 50') which we
// receive between beacons -> bonded/DL + downlink RSSI.
void applyDrive(int i) {
  i = ((i % NPROF) + NPROF) % NPROF; activeProf = i;
  inj.id0 = PROFILES[i].a; inj.id1 = PROFILES[i].b;
  cfg.freq = profileFreq(i);
  mode = IDLE; driveOn = true; driveLastTx = 0; driveT0 = millis();
  ackCount = 0; lastAckMs = 0;
  applyConfig();
  Serial.printf("[drive] %s air=%02x%02x camp=%.3f MHz (home %.3f, %.0f ppm) beacon %lu ms\n",
                PROFILES[i].code, inj.id0, inj.id1, cfg.freq, PROFILES[i].home, crystalPpm,
                (unsigned long)DRIVE_GAP_MS);
  setBanner("DRIVE", PROFILES[i].code, 1000);
}
void driveOnRx() {
  uint8_t b[64]; size_t len = radio.getPacketLength(); if (len > sizeof(b)) len = sizeof(b);
  int st = radio.readData(b, len);
  // The robot's downlink to us takes two forms: the unicast ACK '<air_A> <air_B> 50'
  // (while searching/locking) and the bonded-data frame 'FF FF <air_A> <air_B> ...'
  // (once the link is established). Either one means the robot hears us -> this is the
  // real remote's "DL" indicator + its RSSI.
  bool dl = st == RADIOLIB_ERR_NONE &&
            ((len >= 3 && b[0]==inj.id0 && b[1]==inj.id1) ||
             (len >= 4 && b[0]==0xFF && b[1]==0xFF && b[2]==inj.id0 && b[3]==inj.id1));
  if (dl) {
    ackCount++; lastAckMs = millis(); lastAckRSSI = radio.getRSSI();
    if (Serial && (ackCount < 4 || (ackCount % 20)==0))
      Serial.printf("[drive] <== DL %u B rssi=%.0f (bonded, dl=%lu)\n",
                    (unsigned)len, lastAckRSSI, (unsigned long)ackCount);
  }
  radio.startReceive();
}
void driveStep() {
  uint32_t now = millis();
  if (now - driveLastTx < DRIVE_GAP_MS) return;
  driveLastTx = now;
  if (driveWave) inj.rx = 0.8f * sinf((((now - driveT0) % 4000) / 4000.0f) * 6.2831853f);
  uint8_t p[42]; buildPkt(p);
  radio.transmit(p, 42);
  rxFlag = false; radio.startReceive();     // listen for the robot's ACK until the next beacon
  publishSnap();
}

// ===================== calibrate (crystal ppm) ==============================
// Sweep the crystal correction and keep the value where the robot's ACKs are
// strongest. Needs a robot that is SEARCHING (its real remote off) on the active
// profile's channel. This measures THIS board's crystal error end-to-end.
void calibrate() {
  if (!driveOn) { inj.id0 = PROFILES[activeProf].a; inj.id1 = PROFILES[activeProf].b; }
  Serial.printf("[calib] %s: sweeping ppm, robot must be searching (its remote OFF)\n",
                PROFILES[activeProf].code);
  float home = PROFILES[activeProf].home;
  float bestPpm = crystalPpm; int bestAcks = -1; float bestRssi = -999;
  uint8_t p[42];
  for (float ppm = -90; ppm <= 30.0f; ppm += 4.0f) {
    float f = home * (1.0f + ppm * 1e-6f);
    radio.standby(); radio.setFrequency(f); startRx();
    int acks = 0; float rsum = 0; uint32_t t0 = millis();
    while (millis() - t0 < 1200) {
      if (serialAbort()) { Serial.println("[calib] aborted"); return; }
      buildPkt(p); radio.transmit(p, 42);
      rxFlag = false; radio.startReceive();
      uint32_t w = millis();
      while (millis() - w < 80) {
        if (rxFlag) { rxFlag=false; uint8_t b[64]; size_t len=radio.getPacketLength();
          if (len>sizeof(b)) len=sizeof(b);
          if (radio.readData(b,len)==RADIOLIB_ERR_NONE && len>=3 && b[0]==inj.id0 && b[1]==inj.id1)
            { acks++; rsum += radio.getRSSI(); }
          radio.startReceive();
        }
      }
    }
    float ar = acks ? rsum/acks : -999;
    Serial.printf("[calib] %+5.0f ppm (%.3f MHz): acks=%2d rssi=%.0f %s\n",
                  ppm, f, acks, ar, acks?"":" .");
    if (acks > bestAcks || (acks == bestAcks && acks && ar > bestRssi))
      { bestAcks = acks; bestRssi = ar; bestPpm = ppm; }
  }
  if (bestAcks > 0) {
    crystalPpm = bestPpm;
    Serial.printf("[calib] BEST %+.0f ppm (%.3f MHz) acks=%d rssi=%.0f -> saved\n",
                  bestPpm, home*(1.0f+bestPpm*1e-6f), bestAcks, bestRssi);
    setBanner("CALIBRATED", "", 1500);
  } else {
    Serial.println("[calib] no ACKs at any offset — is the robot searching (remote OFF)?");
  }
  if (driveOn) { cfg.freq = profileFreq(activeProf); radio.standby(); radio.setFrequency(cfg.freq); startRx(); }
}

// ===================== console ==============================================
void printCfg() {
  Serial.printf("CFG freq=%.3f bw=%.3f sf=%u cr=4/6LI pre=%u sync=0x%04X pwr=%d ppm=%.0f rfsw=%s\n",
                cfg.freq, cfg.bw, cfg.sf, cfg.preamble, rawSync(), cfg.power, crystalPpm,
                rfSwitchEnabled?"on":"off");
}
void printHelp() {
  Serial.println(F(
    "commands:\n"
    "  REMOTE:  setid <6-char code> | rem N | next (BOOT btn)\n"
    "  RECEIVE: receive | fscan | hunt\n"
    "  DRIVE:  drive [N] | drivestop | stick lx rx ry ly | btn h1 h2 | wave on|off | gap ms\n"
    "  CAL:    calibrate | ppm <value> | pwr <dBm> | rfsw on|off\n"
    "  low:    f MHz | b kHz | sf N | p N | verbose on|off | info | help | stop\n"
    "  setid takes the 6-char code printed on the remote. N = profile 1..3. live hex/float."));
}
void handleLine(String s) {
  s.trim(); if (!s.length()) return;
  if (s == "help")  { printHelp(); return; }
  if (s == "info")  { printCfg(); return; }
  if (s.startsWith("verbose")) { char o[8]={0}; sscanf(s.c_str(),"verbose %7s",o); verbose=(String(o)!="off"); Serial.printf("[verbose] %s\n", verbose?"on (per-frame dump)":"off"); return; }
  if (s == "stop")  { driveOn=false; mode=IDLE; radio.standby(); digitalWrite(PIN_LED,LOW); Serial.println("[idle]"); return; }
  if (s.startsWith("drivestop")) { driveOn=false; mode=IDLE; radio.standby(); digitalWrite(PIN_LED,LOW); Serial.println("[drive] stopped"); return; }
  if (s.startsWith("drive")) { int i=activeProf+1; sscanf(s.c_str(),"drive %d",&i); applyDrive(i-1); return; }
  if (s.startsWith("wave"))  { char o[8]={0}; sscanf(s.c_str(),"wave %7s",o); driveWave=(String(o)!="off"); driveT0=millis(); Serial.printf("[drive] wave=%s\n", driveWave?"on":"off"); return; }
  if (s.startsWith("gap"))   { int ms; if (sscanf(s.c_str(),"gap %d",&ms)==1 && ms>=10) DRIVE_GAP_MS=ms; Serial.printf("[drive] gap=%lu ms\n",(unsigned long)DRIVE_GAP_MS); return; }
  if (s.startsWith("calib")) { calibrate(); return; }
  if (s.startsWith("ppm"))   { float v; if (sscanf(s.c_str(),"ppm %f",&v)==1){ crystalPpm=v; if(driveOn){cfg.freq=profileFreq(activeProf); radio.standby(); radio.setFrequency(cfg.freq); startRx();}} Serial.printf("[cal] ppm=%.0f -> camp=%.3f\n", crystalPpm, profileFreq(activeProf)); return; }
  if (s.startsWith("rfsw"))  { char o[8]={0}; sscanf(s.c_str(),"rfsw %7s",o); rfSwitchEnabled=(String(o)!="off"); applyConfig(); Serial.printf("[rf] FEM switch %s\n", rfSwitchEnabled?"ON":"OFF"); return; }
  if (s.startsWith("pwr"))   { int p; if (sscanf(s.c_str(),"pwr %d",&p)==1){ cfg.power=p; applyConfig(); } Serial.printf("pwr=%d dBm\n",cfg.power); return; }
  if (s.startsWith("btn"))   { unsigned a=0,b=0; sscanf(s.c_str(),"btn %x %x",&a,&b); inj.b1=a; inj.b2=b; Serial.printf("btn=%02x %02x\n",inj.b1,inj.b2); return; }
  if (s.startsWith("stick")) { float a=0,b=0,c=0,d=0; sscanf(s.c_str(),"stick %f %f %f %f",&a,&b,&c,&d); inj.lx=a; inj.rx=b; inj.ry=c; inj.ly=d; Serial.printf("LX%.2f RX%.2f RY%.2f LY%.2f\n",inj.lx,inj.rx,inj.ry,inj.ly); return; }
  if (s.startsWith("setid")) {                 // set the active profile from a printed 6-char code
    char code[16]={0};
    if (sscanf(s.c_str(),"setid %15s",code)!=1) { Serial.println("usage: setid <6-char code>"); return; }
    for (char* p=code; *p; p++) if (*p>='a'&&*p<='z') *p-=32;      // uppercase
    uint64_t id = codeToId(code);
    if (!id) { Serial.println("[setid] need exactly 6 chars, 0-9 / A-Z"); return; }
    setProfile(activeProf, code, id);
    inj.id0 = PROFILES[activeProf].a; inj.id1 = PROFILES[activeProf].b;
    cfg.freq = profileFreq(activeProf);
    radio.standby(); radio.setFrequency(cfg.freq); if (mode==RECEIVE && !driveOn) startRx();
    Serial.printf("[setid] %s  air=%02x%02x  home=%.3f MHz (camp %.3f)\n",
                  code, PROFILES[activeProf].a, PROFILES[activeProf].b, PROFILES[activeProf].home, cfg.freq);
    setBanner("SET ID", code, 1200);
    return;
  }
  if (s == "next")  { applyProfile(activeProf+1); return; }
  if (s.startsWith("rem")) { int i; if (sscanf(s.c_str(),"rem %d",&i)==1) applyProfile(i-1); else Serial.printf("[rem] #%d %s\n",activeProf+1,PROFILES[activeProf].code); return; }
  if (s == "receive" || s == "recv" || s == "cap") { mode=RECEIVE; driveOn=false; rxAckCount=0; cfg.freq=profileFreq(activeProf); applyConfig(); startRx(); Serial.println("[receive] robot emulation: ACKing every addressed frame -> remote locks"); return; }
  if (s == "fscan") { fscan(); return; }
  if (s == "hunt")  { hunt(); return; }
  int sp = s.indexOf(' '); if (sp < 0) { Serial.println("? (help)"); return; }
  String c = s.substring(0,sp), a = s.substring(sp+1); a.trim();
  if      (c=="f")  { cfg.freq=a.toFloat(); radio.standby(); radio.setFrequency(cfg.freq); if(mode==RECEIVE) startRx(); }
  else if (c=="b")  { cfg.bw=a.toFloat(); applyConfig(); if(mode==RECEIVE) startRx(); }
  else if (c=="sf") { cfg.sf=(uint8_t)a.toInt(); applyConfig(); if(mode==RECEIVE) startRx(); }
  else if (c=="p")  { cfg.preamble=(uint16_t)a.toInt(); applyConfig(); if(mode==RECEIVE) startRx(); }
  else { Serial.println("? (help)"); return; }
  printCfg();
}
String lineBuf;
void pollSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch=='\n' || ch=='\r') { if (lineBuf.length()) { handleLine(lineBuf); lineBuf=""; } }
    else lineBuf += ch;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);     // non-blocking USB-CDC: don't stall when no host reads
  delay(2500);
  pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, LOW);
  pinMode(PIN_BTN, INPUT_PULLUP);
  stateMux = xSemaphoreCreateMutex();
  Serial.println("\n== Unitree LoRa RC receiver + transmitter (T3-S3 / SX1280PA) ==");
  oled.begin(); oled.setBusClock(1000000); oledMsg("Unitree RC", "booting");
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  buildProfiles();
  int st = applyConfig();
  Serial.printf("radio.begin -> %d %s\n", st, st==RADIOLIB_ERR_NONE?"(ok)":"(FAIL - check wiring)");
  for (int i=0;i<NPROF;i++)
    Serial.printf("  %d %s  air=%02x%02x  home=%.3f MHz\n",
                  i+1, PROFILES[i].code, PROFILES[i].a, PROFILES[i].b, PROFILES[i].home);
  printCfg(); printHelp();
  if (st == RADIOLIB_ERR_NONE) {
    applyProfile(0);            // default: first profile, receiving
    xTaskCreatePinnedToCore(renderTask, "render", 4096, nullptr, 1, nullptr, 0);
  } else { mode = IDLE; oledMsg("RADIO FAIL", "check wiring"); }
}

void loop() {
  pollSerial();
  pollButton();
  updateLinkLed();
  if (driveOn) { if (rxFlag) { rxFlag=false; driveOnRx(); } driveStep(); }
  else if (rxFlag && mode == RECEIVE) { rxFlag=false; reportPacket(); }
}
