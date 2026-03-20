#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

// Build identifier (shown in UI)
static const char* BUILD_TAG = "RHXpreS_V208_ledpower_sync_fix";


// ============================================================
// TRAIN MASTER SKETCH (Waveshare ESP32-S3-ETH-8DI-8RO / TCA9554)
// ------------------------------------------------------------
// UI ADDITIONS (this version):
// - Dark theme UI
// - Tabbed UI (Dashboard / WiFi Settings)
// - WiFi Settings: set SSID+PASS, optional Static IP, persists in NVS
// - SEQ stats:
//    * Counter per SEQ (how many times it became active)
//    * Last duration (ms) for last time each SEQ ran
//    * Current SEQ elapsed timer (live)
// ============================================================

// ============================================================
// FUTURE PREP: Ethernet W5500 (disabled)
// ============================================================
#define ENABLE_ETH 0

// ============================================================
// PINS (placeholders included)
// ============================================================
// Confirmed: DI1=4, DI2=5, DI7=10 (pressed=closed)
static const uint8_t PIN_DI[9] = {
  0,
  4,   // DI1  Sensor 1 (visible near EndA)
  5,   // DI2  Sensor 2 (visible near EndB)
  6,   // DI3  Sensor 3 (hidden near EndB)
  7,   // DI4  Sensor 4 (hidden near EndA)
  8,   // DI5  placeholder
  9,   // DI6  cycle button (pressed=closed)
  10,  // DI7  MODE button (pressed=closed)
  11   // DI8  placeholder
};

static const uint8_t PIN_PIXEL   = 38;  // onboard WS2812
static const uint8_t PIN_STRIP   = 21;  // shared external WS2812 data (15px sweep + 40px full-spec)
static const uint8_t PIN_BEACON  = 47;  // 4px beacon mirror strip
static const uint8_t PIN_BUZZER  = 46;  // onboard buzzer
static const uint8_t I2C_SCL_PIN = 41;
static const uint8_t I2C_SDA_PIN = 42;

// ============================================================
// TCA9554
// ============================================================
static const uint8_t TCA_ADDR     = 0x20;
static const uint8_t REG_INPUT    = 0x00;
static const uint8_t REG_OUTPUT   = 0x01;
static const uint8_t REG_POLARITY = 0x02;
static const uint8_t REG_CONFIG   = 0x03;

// ============================================================
// RELAY CHANNELS (EXIO 1..8 maps to bits 0..7 on TCA9554 output)
// (placeholders included)
// ============================================================
static const uint8_t EXIO_CH1_PWR_HI_GATE  = 1; // HI transformer gate/cutoff
static const uint8_t EXIO_CH2_PWR_LO_GATE  = 2; // LO transformer gate/cutoff
static const uint8_t EXIO_CH3_SPEED_SELECT = 3; // HI/LO selector
static const uint8_t EXIO_CH4_DIR_SELECT   = 4; // Polarity REV/FWD (meaning inversion configurable)
static const uint8_t EXIO_CH5_TURNOUT_PULSE = 5; // shared turnout pulse
static const uint8_t EXIO_CH6_TRACK_SELECT  = 6; // hidden/visible selector
static const uint8_t EXIO_CH7_WLED_PULSE   = 7; // LED rail power (CH7): closed/on = power enabled
static const uint8_t EXIO_CH8_UV_LIGHT     = 8; // UV ON/OFF

// ============================================================
// INPUT / RELAY POLARITIES
// ============================================================
// DI input logic (true means input reads LOW when âactive/closedâ)
static bool USE_PULLUP    = true;
static bool DI_ACTIVE_LOW = true;

// DI7 pressed polarity: **pressed closes DI7**
static const bool DI7_PRESSED_IS_OPEN = false;

// Relay electrical polarity (true means output bit LOW = relay ON)
// NOTE: set these TRUE per-channel if your relay board is active-low.
// Defaults are all false (active-high).
static bool EXIO_ACTIVE_LOW[9] = {
  false,
  false,false,false,false,false,false,false,false
};

// Meaning inversions (not electrical)
static bool INVERT_SPEED_LOGIC = false;
static bool INVERT_DIR_LOGIC   = false;

// ============================================================
// TIMINGS
// ============================================================
static uint16_t SAFE_SWITCH_MS      = 50;
static uint16_t TURN_DWELL_MS       = 750;
static uint16_t COOLDOWN_MS         = 75;
static uint16_t TURNOUT_PULSE_MS    = 350;
static uint16_t TURNOUT_SETTLE_MS   = 50;

// Train transition shaping (UI tunables)
static uint16_t ACCEL_EXTRA_MS     = 0;    // extra delay when switching LO->HI
static uint16_t DECEL_EXTRA_MS     = 0;    // extra delay when switching HI->LO



// Minimum enforced cooldown after leaving STOP (prevents immediate sensor edges)
static const uint16_t STOP_EXIT_MIN_COOLDOWN_MS = 250;
static uint16_t DI7_DEBOUNCE_MS     = 50;
static uint16_t DI7_HOLD_MIN_MS     = 1100;
static uint16_t DI7_IGNORE_AFTER_RESUME_MS = 50;

// Sensor stability
static uint16_t SENSOR_SEEN_STABLE_MS  = 50;   // âseenâ must be stable this long
static uint16_t SENSOR_CLEAR_STABLE_MS = 199;  // âclearâ must be stable this long


// Failsafe: if no sensor edge (SEEN/CLEAR) happens for this long while not STOP, enter STOP.
// Set to 0 to disable.
static uint16_t SENSOR_GAP_TIMEOUT_MS   = 10000;
// Web UI toggle for the NoSignal (sensor gap) fault.
static bool SENSOR_GAP_ENABLE = true;

// Stall/derail detection: if a sensor remains "seen" for too long,
// trip a stuck fault. Adjustable 1s..25s (default 5s).
static uint16_t SENSOR_STUCK_TIMEOUT_MS = 10000;
// UI toggle in case you want to disable during testing
static bool SENSOR_STUCK_ENABLE = true;
// Minimum time between accepted âSEENâ hits across sensors
static uint16_t MIN_BETWEEN_SENSORS_MS = 50;

// UI
static uint16_t FLASH_MS         = 50;
static uint8_t  PIXEL_BRIGHTNESS = 80;
static uint32_t RUN_GREEN        = 0x00FF00;

// WLED pulse durations
static uint16_t WLED_PULSE_SHORT_MS = 150;
static uint16_t WLED_PULSE_LONG_MS  = 1200;

static uint16_t WLED_PULSE_GAP_MS   = 120;  // gap between pulses in double-pulse
static uint16_t WLED_CHAIN_GAP_MS  = 10;   // OFF gap between short->long chain
// Manual override pulse durations (for CH5..CH8 override buttons)
static uint16_t OVR_PULSE_SHORT_MS = 150;
static uint16_t OVR_PULSE_LONG_MS  = 1200;

// Buzzer tone controls (mode transition beeps)
static uint16_t BEEP_LEN1_MS = 220;      // 10..2000
static uint16_t BEEP_LEN2_MS = 220;      // 10..2000
static uint16_t BEEP_BASE_HZ = 440;      // baseline LOW note (Hz)
static uint16_t BEEP_OCTAVE_GAP = 1;     // 1..3 octaves above base
static uint16_t BEEP_GAP_MS = 140;        // silence between beeps
static uint8_t  BEEP_DUTY = 200;         // 0..255 (volume-ish)

// ============================================================
// WiFi / Web UI
// ============================================================
static String AP_SSID = "TrainCtrl";
static String AP_PASS = "RocksRock";

static String STA_SSID = "";
static String STA_PASS = "";

// Optional static IP support (persisted)
static bool      STA_USE_STATIC = false;
static IPAddress STA_IP(0,0,0,0);
static IPAddress STA_GW(0,0,0,0);
static IPAddress STA_MASK(0,0,0,0);
static IPAddress STA_DNS(0,0,0,0);

static uint32_t  g_lastWifiChangeMs = 0;

WebServer server(80);
Preferences prefs;

// ============================================================
// ENUMS
// ============================================================
enum SeqId : uint8_t { SEQ1=0, SEQ2, SEQ3, SEQ4, SEQ5, SEQ6, SEQ_COUNT };
enum Speed : uint8_t { SPEED_LO=0, SPEED_HI=1 };
enum Dir   : uint8_t { DIR_FWD=0,  DIR_REV=1 };
enum Mode  : uint8_t { MODE_RUN=0, MODE_RETURN, MODE_STOP };

enum ApplyPhase : uint8_t {
  AP_IDLE=0,
  AP_PWR_SAFE,
  AP_TURN_DWELL,
  AP_TURN_DIR_SAFE,
  AP_DIR_SAFE,
  AP_SPEED_SAFE,
  AP_PWR_ON
};

enum FaultCode : uint8_t {
  FAULT_NONE=0,
  FAULT_PWR_BOTH_ON,
  FAULT_SENSOR_GAP,
  FAULT_SENSOR_STUCK_A,
  FAULT_SENSOR_STUCK_B,
  FAULT_I2C_FAIL
};

enum LastEvent : uint8_t {
  EVT_NONE=0,
  EVT_DI7_SHORT,
  EVT_DI7_HOLD,
  EVT_MODE_RUN,
  EVT_MODE_RETURN,
  EVT_MODE_STOP,
  EVT_APPLY_START,
  EVT_APPLY_FINISH,
  EVT_A_SEEN,
  EVT_A_CLEAR,
  EVT_B_SEEN,
  EVT_B_CLEAR,
  EVT_A_IGNORED,
  EVT_B_IGNORED,
  EVT_FAULT,
  EVT_FAULT_CLEAR,
  EVT_WIFI_UPDATE
};

enum Di7Event : uint8_t { DI7_NONE=0, DI7_SHORT, DI7_HOLD };
enum RouteSel : uint8_t { ROUTE_VISIBLE=0, ROUTE_HIDDEN=1 };

// ============================================================
// AUX RELAY OVERRIDES (CH5..CH8)
// ============================================================
enum AuxOverrideMode : uint8_t {
  AUX_AUTO=0,
  AUX_FORCE_ON,
  AUX_FORCE_OFF,
  AUX_PULSE_SHORT,
  AUX_PULSE_LONG
};

static const char* auxModeStr(AuxOverrideMode m){
  switch(m){
    case AUX_AUTO:        return "AUTO";
    case AUX_FORCE_ON:    return "ON";
    case AUX_FORCE_OFF:   return "OFF";
    case AUX_PULSE_SHORT: return "SHORT";
    case AUX_PULSE_LONG:  return "LONG";
    default:              return "?";
  }
}

// Auto requested states (from SEQ logic / internal rules)
static bool g_auxAutoClosed[9] = {false,false,false,false,false,false,false,false,false};

// Operator override modes
static AuxOverrideMode g_auxOvrMode[9] = {AUX_AUTO,AUX_AUTO,AUX_AUTO,AUX_AUTO,AUX_AUTO,AUX_AUTO,AUX_AUTO,AUX_AUTO,AUX_AUTO};

// Pulse bookkeeping (used when mode is AUX_PULSE_*)
static uint32_t g_auxPulseOffAt[9] = {0,0,0,0,0,0,0,0,0};

static inline bool auxIsPulseMode(AuxOverrideMode m){
  return (m == AUX_PULSE_SHORT) || (m == AUX_PULSE_LONG);
}

static inline uint16_t auxPulseMsForMode(AuxOverrideMode m){
  return (m == AUX_PULSE_LONG) ? OVR_PULSE_LONG_MS : OVR_PULSE_SHORT_MS;
}

static inline void auxSetAuto(uint8_t exio, bool closed){
  if (exio < 1 || exio > 8) return;
  g_auxAutoClosed[exio] = closed;
}

static inline void auxStartPulse(uint8_t exio, AuxOverrideMode m, uint32_t now){
  if (exio < 1 || exio > 8) return;
  uint16_t ms = auxPulseMsForMode(m);
  g_auxOvrMode[exio] = m;
  g_auxPulseOffAt[exio] = now + ms;
}

static void ledRailManualRun(uint32_t now);
static void ledRailManualOff(uint32_t now);

static inline void auxSetOverride(uint8_t exio, AuxOverrideMode m, uint32_t now){
  if (exio < 1 || exio > 8) return;

  // CH7 is now a simple power gate for the GPIO21 LED load.
  // Only AUTO / ON / OFF are valid on CH7.
  if (exio == EXIO_CH7_WLED_PULSE) {
    if (m != AUX_AUTO && m != AUX_FORCE_ON && m != AUX_FORCE_OFF) return;
    g_auxOvrMode[exio] = m;
    g_auxPulseOffAt[exio] = 0;
    return;
  }

  if (m == AUX_AUTO || m == AUX_FORCE_ON || m == AUX_FORCE_OFF) {
    g_auxOvrMode[exio] = m;
    g_auxPulseOffAt[exio] = 0;
    return;
  }
  auxStartPulse(exio, m, now);
}

static inline void auxService(uint32_t now){
  for (uint8_t ch=5; ch<=8; ch++){
    if (!auxIsPulseMode(g_auxOvrMode[ch])) continue;
    uint32_t offAt = g_auxPulseOffAt[ch];
    if (offAt != 0 && (int32_t)(now - offAt) >= 0) {
      g_auxOvrMode[ch] = AUX_AUTO;
      g_auxPulseOffAt[ch] = 0;
    }
  }
}

static inline bool auxEffectiveClosed(uint8_t exio, uint32_t now){
  if (exio < 1 || exio > 8) return false;
  AuxOverrideMode m = g_auxOvrMode[exio];
  if (m == AUX_FORCE_ON) return true;
  if (m == AUX_FORCE_OFF) return false;
  if (auxIsPulseMode(m)) {
    // Safety: if pulse expired but not yet serviced, treat as AUTO
    uint32_t offAt = g_auxPulseOffAt[exio];
    if (offAt != 0 && (int32_t)(now - offAt) >= 0) return g_auxAutoClosed[exio];
    return true;
  }
  return g_auxAutoClosed[exio];
}

// ============================================================
// SEQ PLAN
// ============================================================
struct SeqPlan { Dir dir; Speed speed; bool isTurnaround; };

static const SeqPlan PLAN[SEQ_COUNT] = {
  /*SEQ1*/ { DIR_FWD, SPEED_LO, false },
  /*SEQ2*/ { DIR_REV, SPEED_LO, true  },
  /*SEQ3*/ { DIR_REV, SPEED_HI, false },
  /*SEQ4*/ { DIR_REV, SPEED_LO, false },
  /*SEQ5*/ { DIR_FWD, SPEED_LO, true  },
  /*SEQ6*/ { DIR_FWD, SPEED_HI, false }
};

// ============================================================
// SEQ STATS (counters + last duration + current elapsed)
// ============================================================
static uint32_t g_seqCount[SEQ_COUNT]   = {0};
static uint32_t g_seqLastMs[SEQ_COUNT]  = {0};
static uint32_t g_seqStartMs            = 0;

// ============================================================
// STRING HELPERS
// ============================================================
static const char* modeStr(Mode m){
  switch(m){
    case MODE_RUN:    return "RUN";
    case MODE_RETURN: return "RETURN";
    case MODE_STOP:   return "STOP";
    default:          return "?";
  }
}
static const char* seqStr(SeqId s){
  switch(s){
    case SEQ1: return "SEQ1";
    case SEQ2: return "SEQ2";
    case SEQ3: return "SEQ3";
    case SEQ4: return "SEQ4";
    case SEQ5: return "SEQ5";
    case SEQ6: return "SEQ6";
    default:   return "?";
  }
}
static const char* dirStr(Dir d){ return d==DIR_FWD ? "FWD" : "REV"; }
static const char* spdStr(Speed s){ return s==SPEED_LO ? "LO" : "HI"; }
static const char* routeStr(RouteSel r){ return r==ROUTE_HIDDEN ? "HIDDEN" : "VISIBLE"; }

static const char* applyPhaseStr(ApplyPhase p){
  switch(p){
    case AP_IDLE:          return "IDLE";
    case AP_PWR_SAFE:      return "PWR_SAFE";
    case AP_TURN_DWELL:    return "TURN_DWELL";
    case AP_TURN_DIR_SAFE: return "TURN_DIR_SAFE";
    case AP_DIR_SAFE:      return "DIR_SAFE";
    case AP_SPEED_SAFE:    return "SPEED_SAFE";
    case AP_PWR_ON:        return "PWR_ON";
    default:               return "?";
  }
}

static const char* faultStr(FaultCode f){
  switch(f){
    case FAULT_NONE:           return "NONE";
    case FAULT_PWR_BOTH_ON:    return "PWR_BOTH_ON";
    case FAULT_SENSOR_GAP:     return "SENSOR_GAP";
    case FAULT_SENSOR_STUCK_A: return "SENSOR_STUCK_A";
    case FAULT_SENSOR_STUCK_B: return "SENSOR_STUCK_B";
    case FAULT_I2C_FAIL:       return "I2C_FAIL";
    default:                   return "?";
  }
}

static const char* evtStr(LastEvent e){
  switch(e){
    case EVT_NONE:         return "NONE";
    case EVT_DI7_SHORT:    return "DI7_SHORT";
    case EVT_DI7_HOLD:     return "DI7_HOLD";
    case EVT_MODE_RUN:     return "MODE_RUN";
    case EVT_MODE_RETURN:  return "MODE_RETURN";
    case EVT_MODE_STOP:    return "MODE_STOP";
    case EVT_APPLY_START:  return "APPLY_START";
    case EVT_APPLY_FINISH: return "APPLY_FINISH";
    case EVT_A_SEEN:       return "A_SEEN";
    case EVT_A_CLEAR:      return "A_CLEAR";
    case EVT_B_SEEN:       return "B_SEEN";
    case EVT_B_CLEAR:      return "B_CLEAR";
    case EVT_A_IGNORED:    return "A_IGNORED";
    case EVT_B_IGNORED:    return "B_IGNORED";
    case EVT_FAULT:        return "FAULT";
    case EVT_FAULT_CLEAR:  return "FAULT_CLEAR";
    case EVT_WIFI_UPDATE:  return "WIFI_UPDATE";
    default:               return "?";
  }
}

static const char* wlStr(wl_status_t st){
  switch(st){
    case WL_NO_SHIELD: return "NO_SHIELD";
    case WL_IDLE_STATUS: return "IDLE";
    case WL_NO_SSID_AVAIL: return "NO_SSID";
    case WL_SCAN_COMPLETED: return "SCAN_DONE";
    case WL_CONNECTED: return "CONNECTED";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "LOST";
    case WL_DISCONNECTED: return "DISCONNECTED";
    default: return "?";
  }
}

// ============================================================
// PIXEL
// ============================================================
Adafruit_NeoPixel pixel(1, PIN_PIXEL, NEO_RGB + NEO_KHZ800);

static inline void pixelSet(uint32_t rgb) {
  pixel.setPixelColor(0, rgb);
  pixel.show();
}
static inline void pixelBlink(uint32_t now, uint32_t color) {
  bool on = ((now / FLASH_MS) & 1) == 0;
  pixelSet(on ? color : 0x000000);
}


// ============================================================
// EXTERNAL WS2812 LED BOX (GPIO21 data / CH7 power)
// ------------------------------------------------------------
// Behavior (requested):
// - On debounced Sensor A SEEN (rising edge), turn rail power ON (CH7 auto)
//   and run: SweepForward -> Pause -> SweepBack.
// - All timings are user-set in UI (no measurement-based scaling):
//     Sweep ms, Pause ms, Back ms (range 1..10000, default 2000)
// - User adjustable: segment length, pattern, RGB color, global brightness.
// - UI also provides manual Run/Off controls.
// ============================================================

// Physical strip length: additive map on one shared GPIO21 strip.
//   pixels 0..14   => tunnel sweep
//   pixels 15..55  => cave full-spec reveal
static const uint16_t LED_SWEEP_N    = 15;
static const uint16_t LED_FULLSPEC_N = 41;
static const uint16_t LED_PHYS_N     = LED_SWEEP_N + LED_FULLSPEC_N;
static const uint16_t LED_SEG_MIN    = 1;
static const uint16_t LED_SEG_MAX    = LED_SWEEP_N;
static const uint16_t FULLSPEC_SEG_MIN = 1;
static const uint16_t FULLSPEC_SEG_MAX = LED_FULLSPEC_N;
static const uint16_t CAVE_PHYS_N = FULLSPEC_SEG_MAX;

Adafruit_NeoPixel ledStrip(LED_PHYS_N, PIN_STRIP, NEO_GRB + NEO_KHZ800);

// ------------------------------------------------------------
// ADDITIVE LED MAP
// GPIO21:
//   - pixels 0..14   => tunnel sweep
//   - pixels 15..55  => cave full-spec reveal
// GPIO47:
//   - pixels 0..3    => beacon mirror
// ------------------------------------------------------------
static const uint16_t BEACON_N = 4;
Adafruit_NeoPixel beaconStrip(BEACON_N, PIN_BEACON, NEO_GRB + NEO_KHZ800);

static uint8_t  BEACON_BRIGHTNESS = 96;
static bool     BEACON_FAULT_ONLY = false;
static uint16_t BEACON_EVENT_MS   = 600;
static uint32_t g_beaconEventUntil = 0;

static inline void stripClearRange(uint16_t start, uint16_t count){
  uint16_t end = start + count;
  if (end > LED_PHYS_N) end = LED_PHYS_N;
  for (uint16_t i = start; i < end; i++) ledStrip.setPixelColor(i, 0);
}

static inline uint32_t scaleRgbColor(uint8_t r, uint8_t g, uint8_t b, uint8_t bri255){
  uint8_t rr = (uint8_t)(((uint16_t)r * bri255) / 255u);
  uint8_t gg = (uint8_t)(((uint16_t)g * bri255) / 255u);
  uint8_t bb = (uint8_t)(((uint16_t)b * bri255) / 255u);
  return ledStrip.Color(rr, gg, bb);
}

static inline uint32_t beaconScaleColor(uint8_t r, uint8_t g, uint8_t b){
  uint8_t rr = (uint8_t)(((uint16_t)r * BEACON_BRIGHTNESS) / 255u);
  uint8_t gg = (uint8_t)(((uint16_t)g * BEACON_BRIGHTNESS) / 255u);
  uint8_t bb = (uint8_t)(((uint16_t)b * BEACON_BRIGHTNESS) / 255u);
  return beaconStrip.Color(rr, gg, bb);
}

static inline void beaconShowColor(uint32_t c){
  for (uint16_t i=0; i<BEACON_N; i++) beaconStrip.setPixelColor(i, c);
  beaconStrip.show();
}

static inline void beaconOff(){
  beaconStrip.clear();
  beaconStrip.show();
}

static inline void beaconBegin(){
  beaconStrip.begin();
  beaconStrip.clear();
  beaconStrip.show();
}

static inline void beaconEventS2(uint32_t now){
  if (!BEACON_FAULT_ONLY) g_beaconEventUntil = now + BEACON_EVENT_MS;
}

static inline void beaconService(uint32_t now, Mode mode, FaultCode fault){
  bool on = ((now / FLASH_MS) & 1u) == 0u;

  if (fault != FAULT_NONE || mode == MODE_STOP) {
    beaconShowColor(on ? beaconScaleColor(255, 0, 0) : 0);
    return;
  }
  if (BEACON_FAULT_ONLY) {
    beaconOff();
    return;
  }
  if ((int32_t)(g_beaconEventUntil - now) > 0) {
    beaconShowColor(beaconScaleColor(0, 255, 255)); // cyan sensor-2 event
    return;
  }
  if (mode == MODE_RETURN) {
    beaconShowColor(on ? beaconScaleColor(255, 180, 0) : 0);
    return;
  }
  beaconShowColor(beaconScaleColor(0, 255, 0));
}

// ------------------------------------------------------------
// UV timing controller (CH8 relay)
// ------------------------------------------------------------
enum UvDelayState : uint8_t {
  UV_DELAY_IDLE = 0,
  UV_DELAY_WAIT_OFF,
  UV_DELAY_WAIT_ON
};

struct UvController {
  uint16_t offDelayMs = 500;
  uint16_t onDelayMs  = 500;
  UvDelayState st = UV_DELAY_IDLE;
  uint32_t dueMs = 0;
  bool currentOn = true;

  void begin() {
    currentOn = true;
    auxSetAuto(EXIO_CH8_UV_LIGHT, true);
    st = UV_DELAY_IDLE;
    dueMs = 0;
  }

  void forceOn() {
    currentOn = true;
    auxSetAuto(EXIO_CH8_UV_LIGHT, true);
    st = UV_DELAY_IDLE;
    dueMs = 0;
  }

  void forceOff() {
    currentOn = false;
    auxSetAuto(EXIO_CH8_UV_LIGHT, false);
    st = UV_DELAY_IDLE;
    dueMs = 0;
  }

  void requestOff(uint32_t now) {
    if (offDelayMs == 0) {
      forceOff();
      return;
    }
    st = UV_DELAY_WAIT_OFF;
    dueMs = now + offDelayMs;
  }

  void requestOn(uint32_t now) {
    if (onDelayMs == 0) {
      forceOn();
      return;
    }
    st = UV_DELAY_WAIT_ON;
    dueMs = now + onDelayMs;
  }

  void service(uint32_t now) {
    if (st == UV_DELAY_IDLE) return;
    if ((int32_t)(now - dueMs) < 0) return;

    if (st == UV_DELAY_WAIT_OFF) {
      currentOn = false;
      auxSetAuto(EXIO_CH8_UV_LIGHT, false);
    } else if (st == UV_DELAY_WAIT_ON) {
      currentOn = true;
      auxSetAuto(EXIO_CH8_UV_LIGHT, true);
    }
    st = UV_DELAY_IDLE;
    dueMs = 0;
  }

  const char* stateStr() const {
    switch (st) {
      case UV_DELAY_WAIT_OFF: return "WAIT_OFF";
      case UV_DELAY_WAIT_ON:  return "WAIT_ON";
      default:                return "IDLE";
    }
  }
} uvCtrl;

// ------------------------------------------------------------
// CAVE FULL-SPEC section on GPIO21 pixels 15..55
// Fade pattern is progressive by 5-pixel bands:
//   20->15, 25->21, 30->26, 35->31, 40->36, 45->41, 50->46, 55->51
// Entry:
//   - preglow starts when tunnel sweep window reaches pixel 12
//   - end of sweep interrupts preglow and jumps cave to hold brightness
// Exit:
//   - after dwell and post-dwell LED delay, cave jumps back down to the
//     remembered pre-jump fade state, then fades back to dark
// ------------------------------------------------------------
enum CaveLightState : uint8_t {
  CAVE_OFF = 0,
  CAVE_FADE_IN,
  CAVE_ON,
  CAVE_FADE_OUT
};

struct CaveFullSpec {
  bool enabled = true;
  uint16_t segN = 41;
  uint8_t  r = 255, g = 255, b = 255;
  uint8_t  briPct = 75;          // hold brightness while cave is revealed
  uint16_t triggerDelayMs = 0;   // reserved / legacy; not used by V202 cave logic
  uint16_t fadeMs = 1500;        // entry preglow speed and exit fade speed
  CaveLightState st = CAVE_OFF;
  uint32_t stStart = 0;
  bool dwellHold = false;

  bool preglowStarted = false;
  uint8_t savedJumpProg255 = 0;

  void powerReqAuto(bool on) { auxSetAuto(EXIO_CH7_WLED_PULSE, on); }

  void begin() {
    stripClearRange(LED_SWEEP_N, LED_FULLSPEC_N);
  }

  uint16_t activeCount() const {
    return (uint16_t)constrain((int)segN, (int)FULLSPEC_SEG_MIN, (int)FULLSPEC_SEG_MAX);
  }

  uint8_t targetBri255() const {
    return (uint8_t)(((uint16_t)constrain((int)briPct, 0, 100) * 255u) / 100u);
  }

  static uint8_t clamp255(int32_t v) {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
  }

  uint8_t currentPreglowProg255(uint32_t now) const {
    if (st != CAVE_FADE_IN || fadeMs == 0) return 0;
    uint32_t el = now - stStart;
    if (el >= fadeMs) return 255;
    return (uint8_t)((el * 255u) / fadeMs);
  }

  // Render progressive fill pattern into cave segment only.
  void renderPreglow(uint8_t prog255) {
    uint16_t n = activeCount();
    uint8_t maxBri255 = targetBri255();
    stripClearRange(LED_SWEEP_N, LED_FULLSPEC_N);

    uint16_t idx = 0;
    while (idx < n) {
      uint16_t remaining = (uint16_t)(n - idx);
      uint16_t len = (remaining >= 6) ? 6 : remaining;
      if (len == 0) break;

      // Each band fills from highest address toward lowest address.
      // Next lower pixel starts when prior pixel reaches 50%.
      // Model using stage spacing of 0.25 band-progress and ramp span of 0.5.
      uint16_t totalSpanQ100 = (uint16_t)(50 + (len > 0 ? (len - 1) * 25 : 0)); // 50..175
      uint16_t tQ100 = (uint16_t)(((uint32_t)prog255 * totalSpanQ100 + 127u) / 255u);

      for (uint16_t d = 0; d < len; d++) {
        int32_t startQ100 = (int32_t)d * 25;      // 0.00,0.25,0.50...
        int32_t numerQ100 = (int32_t)tQ100 - startQ100;
        uint8_t level255 = 0;
        if (numerQ100 > 0) {
          int32_t lv = (numerQ100 * 255) / 50;    // 0.5 span to full
          level255 = clamp255(lv);
        }

        uint16_t phys = (uint16_t)(LED_SWEEP_N + idx + (len - 1 - d)); // high addr first
        uint16_t finalBri255 = (uint16_t)((uint32_t)maxBri255 * (uint32_t)level255 / 255u);
        uint8_t rr = (uint8_t)(((uint16_t)r * finalBri255) / 255u);
        uint8_t gg = (uint8_t)(((uint16_t)g * finalBri255) / 255u);
        uint8_t bb = (uint8_t)(((uint16_t)b * finalBri255) / 255u);
        ledStrip.setPixelColor(phys, ledStrip.Color(rr, gg, bb));
      }

      idx = (uint16_t)(idx + len);
    }
    ledStrip.show();
  }

  void renderUniform(uint8_t scale255) {
    uint16_t n = activeCount();
    uint16_t maxBri255 = targetBri255();
    uint16_t outBri255 = (uint16_t)((uint32_t)maxBri255 * (uint32_t)scale255 / 255u);
    stripClearRange(LED_SWEEP_N, LED_FULLSPEC_N);
    uint8_t rr = (uint8_t)(((uint16_t)r * outBri255) / 255u);
    uint8_t gg = (uint8_t)(((uint16_t)g * outBri255) / 255u);
    uint8_t bb = (uint8_t)(((uint16_t)b * outBri255) / 255u);
    uint32_t c = ledStrip.Color(rr, gg, bb);
    for (uint16_t i = 0; i < n; i++) ledStrip.setPixelColor(LED_SWEEP_N + i, c);
    ledStrip.show();
  }

  void forceOff() {
    st = CAVE_OFF;
    dwellHold = false;
    preglowStarted = false;
    stripClearRange(LED_SWEEP_N, LED_FULLSPEC_N);
    ledStrip.show();
  }

  void startPreglow(uint32_t now) {
    if (!enabled) return;
    if (preglowStarted) return;
    powerReqAuto(true);
    auxApplyOutputs(now);
    delayMicroseconds(300);
    preglowStarted = true;
    st = CAVE_FADE_IN;
    stStart = now;
    renderPreglow(0);
  }

  void jumpToHold(uint32_t now) {
    if (!enabled) return;
    powerReqAuto(true);
    auxApplyOutputs(now);
    delayMicroseconds(300);
    if (preglowStarted && st == CAVE_FADE_IN) {
      savedJumpProg255 = currentPreglowProg255(now);
    } else {
      savedJumpProg255 = 0;
    }
    if (savedJumpProg255 > 255) savedJumpProg255 = 255;
    st = CAVE_ON;
    dwellHold = true;
    renderUniform(255); // all cave pixels ON at hold brightness percent
  }

  void onDwellStart(uint32_t now) {
    (void)now;
    dwellHold = true;
    if (st == CAVE_ON) renderUniform(255);
  }

  void onDwellEnd(uint32_t now) {
    (void)now;
    dwellHold = false;
  }

  void startExitFromSaved(uint32_t now) {
    if (!enabled) return;
    powerReqAuto(true);
    auxApplyOutputs(now);
    delayMicroseconds(300);
    st = CAVE_FADE_OUT;
    stStart = now;
    if (savedJumpProg255 == 0 || fadeMs == 0) {
      forceOff();
    } else {
      renderPreglow(savedJumpProg255);
    }
  }

  void manualRun(uint32_t now) {
    startPreglow(now);
  }

  void manualOff(uint32_t now) {
    startExitFromSaved(now);
  }

  bool isActive() const {
    return st != CAVE_OFF;
  }

  void service(uint32_t now) {
    segN = (uint16_t)constrain((int)segN, (int)FULLSPEC_SEG_MIN, (int)FULLSPEC_SEG_MAX);
    briPct = (uint8_t)constrain((int)briPct, 0, 100);
    fadeMs = (uint16_t)constrain((int)fadeMs, 0, 15000);

    if (!enabled) {
      forceOff();
      return;
    }

    switch (st) {
      case CAVE_OFF:
        break;

      case CAVE_FADE_IN: {
        if (fadeMs == 0) {
          renderPreglow(255);
          st = CAVE_ON;
          dwellHold = false;
          break;
        }
        uint8_t p = currentPreglowProg255(now);
        if (p >= 255) {
          // Preglow reached full pattern; keep it there until sweep-end jump or exit.
          renderPreglow(255);
        } else {
          renderPreglow(p);
        }
      } break;

      case CAVE_ON:
        renderUniform(255);
        break;

      case CAVE_FADE_OUT: {
        if (fadeMs == 0 || savedJumpProg255 == 0) {
          forceOff();
          break;
        }
        uint32_t el = now - stStart;
        if (el >= fadeMs) {
          forceOff();
        } else {
          uint8_t p = (uint8_t)(((uint32_t)savedJumpProg255 * (uint32_t)(fadeMs - el)) / fadeMs);
          renderPreglow(p);
        }
      } break;
    }
  }

  const char* stateStr() const {
    switch (st) {
      case CAVE_FADE_IN:  return "FADE_IN";
      case CAVE_ON:       return "ON";
      case CAVE_FADE_OUT: return "FADE_OUT";
      default:            return "OFF";
    }
  }
} caveFullSpec;

static void caveManualRun(uint32_t now){ caveFullSpec.manualRun(now); }
static void caveManualOff(uint32_t now){ caveFullSpec.manualOff(now); }

enum LedAnimState : uint8_t {
  LED_IDLE=0,
  LED_SWEEP_FWD,
  LED_PAUSE,      // repurposed as cave hold / post-dwell LED delay
  LED_SWEEP_BACK
};

enum LedPattern : uint8_t {
  LED_PAT_TAIL4 = 0,
  LED_PAT_SOLID = 1,
  LED_PAT_DOT   = 2
};

static struct LedRail {
  uint16_t segN = 15;
  uint8_t  pattern = LED_PAT_TAIL4;
  uint8_t  r = 255, g = 255, b = 255;
  uint8_t  bri = 230;
  uint16_t sweepMs = 2000; // entry sweep forward timing
  uint16_t pauseMs = 1200; // post-dwell LED delay before reverse sweep
  uint16_t backMs  = 2000; // reverse sweep timing
  bool     enabled = true;

  LedAnimState st = LED_IDLE;
  uint32_t stStart = 0;
  uint16_t stDur = 0;
  int16_t  pos = -1;           // current lowest lit pixel address in tunnel sweep window
  bool cavePreglowTriggered = false;
  bool exitDelayStarted = false;

  void begin() {
    ledStrip.begin();
    ledStrip.clear();
  }

  void powerReqAuto(bool on) { auxSetAuto(EXIO_CH7_WLED_PULSE, on); }

  static inline uint8_t clampU8(int v){ if (v<0) return 0; if (v>255) return 255; return (uint8_t)v; }

  void clearSweepOnly() {
    stripClearRange(0, LED_SWEEP_N);
    ledStrip.show();
  }

  void renderSolid() {
    uint16_t n = (uint16_t)constrain((int)segN, (int)LED_SEG_MIN, (int)LED_SEG_MAX);
    uint32_t c = scaleRgbColor(r, g, b, bri);
    stripClearRange(0, LED_SWEEP_N);
    for (uint16_t i=0; i<n; i++) ledStrip.setPixelColor(i, c);
    ledStrip.show();
  }

  void renderDot(int16_t p) {
    uint16_t n = (uint16_t)constrain((int)segN, (int)LED_SEG_MIN, (int)LED_SEG_MAX);
    if (p < 0) p = 0;
    if (p > (int16_t)n-1) p = (int16_t)n-1;

    uint32_t c = scaleRgbColor(r, g, b, bri);
    stripClearRange(0, LED_SWEEP_N);
    ledStrip.setPixelColor((uint16_t)p, c);
    ledStrip.show();
  }

  // Fixed tunnel window pattern: lowest lit pixel is always brightest.
  // Window = [p, p+1, p+2] => 75%, 50%, 25%
  void renderTail4(int16_t p) {
    uint16_t n = (uint16_t)constrain((int)segN, (int)LED_SEG_MIN, (int)LED_SEG_MAX);
    if (n < 3) {
      renderDot(p);
      return;
    }

    int16_t maxStart = (int16_t)n - 3;
    if (p < 0) p = 0;
    if (p > maxStart) p = maxStart;

    stripClearRange(0, LED_SWEEP_N);

    static const uint8_t pct[3] = { 75, 50, 25 };
    for (uint8_t k = 0; k < 3; k++) {
      int16_t idx = (int16_t)(p + k);
      if (idx < 0 || idx >= (int16_t)n) continue;

      uint16_t rb = (uint16_t)r * (uint16_t)bri / 255u;
      uint16_t gb = (uint16_t)g * (uint16_t)bri / 255u;
      uint16_t bb = (uint16_t)b * (uint16_t)bri / 255u;
      rb = (rb * (uint16_t)pct[k]) / 100u;
      gb = (gb * (uint16_t)pct[k]) / 100u;
      bb = (bb * (uint16_t)pct[k]) / 100u;

      ledStrip.setPixelColor((uint16_t)idx, ledStrip.Color(clampU8(rb), clampU8(gb), clampU8(bb)));
    }
    ledStrip.show();
  }

  void renderAt(int16_t p) {
    switch ((LedPattern)pattern) {
      case LED_PAT_SOLID: renderSolid(); break;
      case LED_PAT_DOT:   renderDot(p);  break;
      case LED_PAT_TAIL4:
      default:            renderTail4(p); break;
    }
  }

  void startAnim(uint32_t now) {
    if (!enabled) return;

    segN = (uint16_t)constrain((int)segN, (int)LED_SEG_MIN, (int)LED_SEG_MAX);
    if (segN < 3) segN = 3;
    if (sweepMs < 1) sweepMs = 1; if (sweepMs > 10000) sweepMs = 10000;
    if (pauseMs > 15000) pauseMs = 15000;
    if (backMs  < 1) backMs  = 1; if (backMs  > 10000) backMs  = 10000;

    powerReqAuto(true);
    auxApplyOutputs(now);
    delayMicroseconds(300);
    st = LED_SWEEP_FWD;
    stStart = now;
    stDur = sweepMs;
    pos = 0;
    cavePreglowTriggered = false;
    exitDelayStarted = false;
    renderAt(pos);
  }

  void stopAnim(uint32_t now) {
    (void)now;
    st = LED_IDLE;
    pos = -1;
    stripClearRange(0, LED_SWEEP_N);
    ledStrip.show();
    if (!caveFullSpec.isActive()) powerReqAuto(false);
  }

  void onSensor2Approach(uint32_t now) {
    if (st == LED_IDLE) startAnim(now);
  }

  void onSensor2Exit(uint32_t now) {
    (void)now;
  }

  void onSensorASeen(uint32_t now) {
    (void)now;
  }

  void onDwellStart(uint32_t now) {
    caveFullSpec.onDwellStart(now);
  }

  void onDwellEnd(uint32_t now) {
    caveFullSpec.onDwellEnd(now);
    // Revised show rule:
    // - dwell completion no longer starts the exit lighting sequence
    // - exit sweep begins on S3 SEEN
    // - cave restore happens on S3 CLEAR
    (void)now;
    exitDelayStarted = false;
  }

  void onSensor3Seen(uint32_t now) {
    if (st != LED_PAUSE) return;
    uint16_t n = (uint16_t)constrain((int)segN, (int)LED_SEG_MIN, (int)LED_SEG_MAX);
    if (n < 3) return;
    int16_t maxStart = (int16_t)n - 3;
    // Keep UV OFF and cave full-spec ON while train begins leaving hidden track.
    st = LED_SWEEP_BACK;
    stStart = now;
    stDur = backMs;
    pos = maxStart;
    renderAt(pos);
  }

  void onSensor3Clear(uint32_t now) {
    // S3 CLEAR is the hard handoff back to idle cave lighting.
    uvCtrl.requestOn(now);
    caveFullSpec.forceOff();
    stopAnim(now);
    exitDelayStarted = false;
  }

  void manualRun(uint32_t now) {
    startAnim(now);
  }

  void manualOff(uint32_t now) {
    exitDelayStarted = true;
    if (st != LED_PAUSE) {
      st = LED_PAUSE;
      stStart = now;
      stDur = 0;
    }
    caveFullSpec.onDwellEnd(now);
  }

  void service(uint32_t now) {
    if (st == LED_IDLE) return;

    uint16_t n = (uint16_t)constrain((int)segN, (int)LED_SEG_MIN, (int)LED_SEG_MAX);
    if (n < 3) {
      stopAnim(now);
      return;
    }
    int16_t maxStart = (int16_t)n - 3;

    switch (st) {
      case LED_SWEEP_FWD: {
        uint32_t dur = (stDur < 1) ? 1 : stDur;
        uint32_t el = now - stStart;

        if (el >= dur) {
          stripClearRange(0, LED_SWEEP_N);
          ledStrip.show();
          caveFullSpec.jumpToHold(now);   // interrupt preglow and jump cave to hold brightness
          uvCtrl.requestOff(now);         // UV OFF at jump-up
          st = LED_PAUSE;
          stStart = now;
          stDur = 0;
          pos = maxStart;
          return;
        }

        int16_t p = (int16_t)((el * (uint32_t)maxStart + (dur/2)) / dur);
        if (p < 0) p = 0;
        if (p > maxStart) p = maxStart;
        if (!cavePreglowTriggered && p >= 12) {
          cavePreglowTriggered = true;
          caveFullSpec.startPreglow(now);
        }
        if (p != pos) {
          pos = p;
          renderAt(pos);
        }
      } break;

      case LED_PAUSE: {
        // Hold cave reveal during dwell. Exit is now started explicitly by S3 SEEN,
        // and restore back to UV happens at S3 CLEAR.
      } break;

      case LED_SWEEP_BACK: {
        uint32_t dur = (stDur < 1) ? 1 : stDur;
        uint32_t el = now - stStart;
        if (el >= dur) {
          pos = 0;
          renderAt(pos);
          stopAnim(now);
          return;
        }
        int16_t down = (int16_t)((el * (uint32_t)maxStart + (dur/2)) / dur);
        if (down < 0) down = 0;
        if (down > maxStart) down = maxStart;
        int16_t p = (int16_t)(maxStart - down);
        if (p != pos) { pos = p; renderAt(pos); }
      } break;

      default:
        st = LED_IDLE;
        break;
    }
  }
} ledRail;

static void ledRailManualRun(uint32_t now){ ledRail.manualRun(now); }
static void ledRailManualOff(uint32_t now){ ledRail.manualOff(now); }

// ------------------------------------------------------------
// Sensor-2 cave event orchestrator
// V202 show rule:
//   - first S2 SEEN starts the timed tunnel/cave show
//   - cave/tunnel/UV logic then runs from timers and dwell callbacks
//   - additional S2 edges do not directly drive the show
// ------------------------------------------------------------
enum Sensor2CavePhase : uint8_t {
  S2CAVE_IDLE = 0,
  S2CAVE_RUNNING
};

struct Sensor2CaveController {
  Sensor2CavePhase phase = S2CAVE_IDLE;

  void reset() { phase = S2CAVE_IDLE; }

  void onSeen(uint32_t now) {
    if (phase == S2CAVE_IDLE) {
      ledRail.onSensor2Approach(now);
      beaconEventS2(now);
      phase = S2CAVE_RUNNING;
    }
  }

  void onClear(uint32_t now) {
    (void)now;
  }

  const char* stateStr() const {
    switch (phase) {
      case S2CAVE_RUNNING: return "RUNNING";
      default:             return "IDLE";
    }
  }
} s2Cave;
// ============================================================
// End external WS2812 LED box
// ============================================================
// ============================================================
// I2C / TCA9554 HELPERS
// ============================================================
static inline bool tcaWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
static inline bool tcaRead(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)TCA_ADDR, 1) != 1) return false;
  val = Wire.read();
  return true;
}
static inline uint8_t exioBit(uint8_t exio1to8) {
  exio1to8 = (uint8_t)constrain(exio1to8, 1, 8);
  return (uint8_t)(exio1to8 - 1);
}
static inline bool readDI(uint8_t gpioPin) {
  bool rawHigh = (digitalRead(gpioPin) == HIGH);
  return DI_ACTIVE_LOW ? !rawHigh : rawHigh; // true = closed/active
}

// ============================================================
// GLOBAL DIAG + LOG (ring buffer)
// ============================================================
static volatile LastEvent g_lastEvent = EVT_NONE;
static volatile uint32_t  g_lastEventAt = 0;
static volatile FaultCode g_lastFault = FAULT_NONE;
static volatile uint32_t  g_lastFaultAt = 0;

struct LogEntry {
  uint32_t ms;
  LastEvent evt;
  FaultCode fault;
  Mode mode;
  SeqId activeSeq;
  SeqId pendingSeq;
  ApplyPhase applyPhase;
};

static const uint16_t LOG_CAP = 80;
static LogEntry g_log[LOG_CAP];
static uint16_t g_logHead = 0;
static uint16_t g_logCount = 0;

static inline void logPush(uint32_t now, LastEvent evt, FaultCode fault,
                           Mode mode, SeqId activeSeq, SeqId pendingSeq, ApplyPhase ph)
{
  g_lastEvent = evt;
  g_lastEventAt = now;
  if (fault != FAULT_NONE) {
    g_lastFault = fault;
    g_lastFaultAt = now;
  }
  LogEntry &e = g_log[g_logHead];
  e.ms = now;
  e.evt = evt;
  e.fault = fault;
  e.mode = mode;
  e.activeSeq = activeSeq;
  e.pendingSeq = pendingSeq;
  e.applyPhase = ph;

  g_logHead = (uint16_t)((g_logHead + 1) % LOG_CAP);
  if (g_logCount < LOG_CAP) g_logCount++;
}

static inline void setFault(uint32_t now, FaultCode f,
                            Mode mode, SeqId activeSeq, SeqId pendingSeq, ApplyPhase ph)
{
  g_lastFault = f;
  g_lastFaultAt = now;
  logPush(now, EVT_FAULT, f, mode, activeSeq, pendingSeq, ph);
}

// ============================================================
// RELAY OUTPUT
// ============================================================
static inline void relaySetClosed(uint8_t exioPin, bool closed) {
  uint8_t out;
  if (!tcaRead(REG_OUTPUT, out)) return;

  uint8_t bit = exioBit(exioPin);
  bool activeLow = EXIO_ACTIVE_LOW[exioPin];

  // closed=true means relay ON
  bool outHigh = activeLow ? !closed : closed;

  if (outHigh) out |=  (1u << bit);
  else         out &= ~(1u << bit);

  (void)tcaWrite(REG_OUTPUT, out);
}


// Helper: compute relay CLOSED state from an output register byte
static inline bool exioClosedFromOutByte(uint8_t exioPin, uint8_t outByte){
  uint8_t bit = exioBit(exioPin);
  bool outHigh = ((outByte >> bit) & 1u) != 0;
  bool activeLow = EXIO_ACTIVE_LOW[exioPin];
  return activeLow ? !outHigh : outHigh;
}

// Helper: true if BOTH track-power gates are OFF (HI gate + LO gate)
static inline bool powerGatesBothOff(){
  uint8_t out=0;
  if (!tcaRead(REG_OUTPUT, out)) return true; // assume OFF if we can't read
  bool lo = exioClosedFromOutByte(EXIO_CH2_PWR_LO_GATE, out);
  bool hi = exioClosedFromOutByte(EXIO_CH1_PWR_HI_GATE, out);
  return (!lo && !hi);
}

// ============================================================
// TURNOUT PULSE ENGINE
// ============================================================
struct TurnoutPulseEngine {
  bool active=false;
  RouteSel targetRoute=ROUTE_VISIBLE;
  uint8_t phase=0;
  uint32_t dueMs=0;
  void start(RouteSel r, uint32_t now){ active=true; targetRoute=r; phase=0; dueMs=now; }
  void service(uint32_t now){
    if(!active) return;
    if((int32_t)(now-dueMs) < 0) return;
    if(targetRoute==ROUTE_VISIBLE){
      switch(phase){
        case 0: relaySetClosed(EXIO_CH6_TRACK_SELECT,false); relaySetClosed(EXIO_CH5_TURNOUT_PULSE,true); phase=1; dueMs=now+TURNOUT_PULSE_MS; break;
        default: relaySetClosed(EXIO_CH5_TURNOUT_PULSE,false); active=false; break;
      }
    } else {
      switch(phase){
        case 0: relaySetClosed(EXIO_CH6_TRACK_SELECT,true); phase=1; dueMs=now+TURNOUT_SETTLE_MS; break;
        case 1: relaySetClosed(EXIO_CH5_TURNOUT_PULSE,true); phase=2; dueMs=now+TURNOUT_PULSE_MS; break;
        case 2: relaySetClosed(EXIO_CH5_TURNOUT_PULSE,false); phase=3; dueMs=now+TURNOUT_SETTLE_MS; break;
        default: relaySetClosed(EXIO_CH6_TRACK_SELECT,false); active=false; break;
      }
    }
  }
} turnoutPulse;

static inline void requestTurnoutRoute(RouteSel r, uint32_t now){
  turnoutPulse.start(r, now);
}

// ============================================================
// AUX OUTPUT APPLY (CH5..CH8)
// ============================================================
static inline void auxApplyOutputs(uint32_t now) {
  // CH7 is a simple LED power gate. In AUTO it follows whether either
  // the 15-pixel rail animation or the 40-pixel full-spec cave lights
  // are currently active. Manual ON/OFF overrides still win via auxEffectiveClosed().
  auxSetAuto(EXIO_CH7_WLED_PULSE, (ledRail.st != LED_IDLE) || (caveFullSpec.st != CAVE_OFF));

  auxService(now);
  for (uint8_t ch=7; ch<=8; ch++){
    relaySetClosed(ch, auxEffectiveClosed(ch, now));
  }
}

// WLED pulse engine (AUTO logic on CH7)
// Supports:
//  - single pulse (short or long)
//  - double pulse (short, gap, short)
//  - chained short then long (short, gap, long)
// (CH7 repurposed to LED rail power; legacy WLED pulse engine removed)

// UV is CH8 now (AUTO logic)
static inline void uvSetAuto(bool on) { auxSetAuto(EXIO_CH8_UV_LIGHT, on); }

// Legacy light rules removed (UV coupling removed)
static inline void applyLightRulesForSeq(SeqId /*s*/) { /* no-op */ }

static inline void setSpeedSelect(Speed s) {
  Speed eff = INVERT_SPEED_LOGIC ? (s==SPEED_LO ? SPEED_HI : SPEED_LO) : s;
  relaySetClosed(EXIO_CH3_SPEED_SELECT, (eff == SPEED_HI));
}

// Direction control with WLED pulses on polarity change
static Dir  g_lastEffDir = DIR_FWD;
static bool g_lastEffDirValid = false;

static inline Dir effDir(Dir d) {
  return INVERT_DIR_LOGIC ? (d==DIR_FWD ? DIR_REV : DIR_FWD) : d;
}

static inline void setDirection(Dir d, uint32_t now, bool allowPulse=true) {
  // WLED CH7 pulses are no longer tied to direction changes; they are triggered by SEQ1..SEQ3-side sensor/dwell events.
  (void)now;
  (void)allowPulse;

  Dir eff = effDir(d);

  // Apply relay state: REV=closed, FWD=open
  relaySetClosed(EXIO_CH4_DIR_SELECT, (eff == DIR_REV));

  g_lastEffDir = eff;
  g_lastEffDirValid = true;
}


// aOn = LO gate (CH2), bOn = HI gate (CH1)
static inline void powerSetSafe(uint32_t now, bool aOn, bool bOn,
                                Mode mode, SeqId activeSeq, SeqId pendingSeq, ApplyPhase ph)
{
  if (aOn && bOn) {
    relaySetClosed(EXIO_CH2_PWR_LO_GATE, false);
    relaySetClosed(EXIO_CH1_PWR_HI_GATE, false);
    setFault(now, FAULT_PWR_BOTH_ON, mode, activeSeq, pendingSeq, ph);
    return;
  }
  relaySetClosed(EXIO_CH2_PWR_LO_GATE, aOn);
  relaySetClosed(EXIO_CH1_PWR_HI_GATE, bOn);
}

static inline void powerCut(uint32_t now, Mode m, SeqId a, SeqId p, ApplyPhase ph) {
  powerSetSafe(now, false, false, m, a, p, ph);
}
static inline void powerOnLO(uint32_t now, Mode m, SeqId a, SeqId p, ApplyPhase ph) {
  powerSetSafe(now, true, false, m, a, p, ph);
}
static inline void powerOnHI(uint32_t now, Mode m, SeqId a, SeqId p, ApplyPhase ph) {
  powerSetSafe(now, false, true, m, a, p, ph);
}
static inline void powerOnForSpeed(uint32_t now, Speed s, Mode m, SeqId a, SeqId p, ApplyPhase ph) {
  if (s == SPEED_HI) powerOnHI(now, m, a, p, ph);
  else              powerOnLO(now, m, a, p, ph);
}

// ============================================================
// BUZZER (same as before, kept)
// ============================================================
static const uint8_t BUZZ_CH   = 7;
static const uint8_t BUZZ_BITS = 8;

struct BuzzNote { uint16_t hz; uint16_t ms; uint8_t duty; };

struct BuzzerPlayer {
  bool active=false;
  uint32_t nextAt=0;
  uint8_t idx=0;
  const BuzzNote* seq=nullptr;
  uint8_t len=0;

  void begin() {
    ledcSetup(BUZZ_CH, 2000, BUZZ_BITS);
    ledcAttachPin(PIN_BUZZER, BUZZ_CH);
    stop();
  }
  void stop() {
    ledcWrite(BUZZ_CH, 0);
    ledcWriteTone(BUZZ_CH, 0);
    active=false; seq=nullptr; len=0; idx=0; nextAt=0;
  }
  void play(const BuzzNote* s, uint8_t n, uint32_t /*now*/) {
    seq=s; len=n; idx=0;
    active=true;
    nextAt=millis();
  }
  void service(uint32_t now) {
    if (!active) return;
    if ((int32_t)(now - nextAt) < 0) return;
    if (idx >= len) { stop(); return; }

    BuzzNote note = seq[idx++];
    if (note.hz == 0 || note.duty == 0) {
      ledcWrite(BUZZ_CH, 0);
      ledcWriteTone(BUZZ_CH, 0);
    } else {
      ledcWriteTone(BUZZ_CH, note.hz);
      ledcWrite(BUZZ_CH, note.duty);
    }
    nextAt = now + note.ms;
  }
} buzzer;

static const uint8_t R2D2_MAX_NOTES = 32;
static BuzzNote r2d2Buf[R2D2_MAX_NOTES];

static inline uint16_t rng16() {
  static uint32_t s = 0xA5A5A5A5;
  s = s * 1664525UL + 1013904223UL;
  return (uint16_t)(s >> 16);
}

enum SoundEvent : uint8_t {
  SND_RUN_TO_STOP=0,
  SND_RUN_TO_RETURN,
  SND_RETURN_TO_RUN,
  SND_RETURN_TO_STOP,
  SND_STOP_TO_RUN,
  SND_STOP_TO_RETURN
};

struct ISoundGen { virtual void play(SoundEvent e, uint32_t now) = 0; };

static BuzzNote beepBuf[4];

static inline uint16_t beepHzLow(){
  uint32_t hz = (uint32_t)BEEP_BASE_HZ;
  if (hz < 50) hz = 50;
  if (hz > 12000) hz = 12000;
  return (uint16_t)hz;
}
static inline uint16_t beepHzHigh(){
  uint32_t oct = (uint32_t)constrain((int)BEEP_OCTAVE_GAP, 1, 3);
  uint32_t mul = 1u << oct; // 2^oct
  uint32_t hz = (uint32_t)BEEP_BASE_HZ * mul;
  if (hz < 50) hz = 50;
  if (hz > 12000) hz = 12000;
  return (uint16_t)hz;
}

struct BeepSoundGen : ISoundGen {
  void play(SoundEvent e, uint32_t now) override {
    uint16_t lo = beepHzLow();
    uint16_t hi = beepHzHigh();
    uint8_t n = 0;

    auto tone = [&](uint16_t hz, uint16_t ms){
      if (n < 4) beepBuf[n++] = { hz, ms, BEEP_DUTY };
    };
    auto gap = [&](uint16_t ms){
      if (n < 4) beepBuf[n++] = { 0, ms, 0 };
    };

    buzzer.stop();

    switch(e){
      case SND_RUN_TO_STOP:    tone(hi, BEEP_LEN1_MS); gap(BEEP_GAP_MS); tone(lo, BEEP_LEN2_MS); break;
      case SND_STOP_TO_RUN:    tone(lo, BEEP_LEN1_MS); gap(BEEP_GAP_MS); tone(hi, BEEP_LEN2_MS); break;
      case SND_RUN_TO_RETURN:  tone(lo, BEEP_LEN1_MS); gap(BEEP_GAP_MS); tone(lo, BEEP_LEN2_MS); break;
      case SND_STOP_TO_RETURN: tone(hi, BEEP_LEN1_MS); gap(BEEP_GAP_MS); tone(hi, BEEP_LEN2_MS); break;
      case SND_RETURN_TO_RUN:  tone(lo, BEEP_LEN1_MS); gap(BEEP_GAP_MS); tone(hi, BEEP_LEN2_MS); break;
      case SND_RETURN_TO_STOP: tone(hi, BEEP_LEN1_MS); gap(BEEP_GAP_MS); tone(lo, BEEP_LEN2_MS); break;
      default: break;
    }

    if (n) buzzer.play(beepBuf, n, now);
  }
} soundGen;

struct R2D2SoundGen : ISoundGen {
  uint8_t buildPhrase(float pitchMul) {
    uint8_t n = 0;
    uint8_t notes = 10 + (rng16() % 10); // 10..19
    for (uint8_t i=0; i<notes && n < (R2D2_MAX_NOTES-2); i++) {
      uint16_t baseHz = 900 + (rng16() % 1700);
      int tmp = (int)(baseHz * pitchMul);
      if (tmp < 120) tmp = 120;
      baseHz = (uint16_t)tmp;

      uint8_t  duty   = 120 + (rng16() % 100);
      uint16_t durMs  = 35  + (rng16() % 110);

      if ((rng16() % 3) == 0 && (n + 3) < R2D2_MAX_NOTES) {
        uint16_t delta = 40 + (rng16() % 180);
        int tmp2 = (int)((baseHz + delta) * pitchMul);
        if (tmp2 < 120) tmp2 = 120;
        uint16_t h2 = (uint16_t)tmp2;

        r2d2Buf[n++] = { baseHz, (uint16_t)(durMs/2), duty };
        r2d2Buf[n++] = { h2,     (uint16_t)(durMs/2), duty };
        r2d2Buf[n++] = { 0,      (uint16_t)(15 + (rng16()%30)), 0 };
      } else {
        r2d2Buf[n++] = { baseHz, durMs, duty };
        r2d2Buf[n++] = { 0, (uint16_t)(20 + (rng16()%70)), 0 };
      }
    }
    r2d2Buf[n++] = { 0, 70, 0 };
    return n;
  }

  void play(SoundEvent e, uint32_t now) override {
    float mul = (e == SND_RUN_TO_STOP || e == SND_RETURN_TO_STOP) ? 0.5f : 1.0f;
    buzzer.stop();
    uint8_t len = buildPhrase(mul);
    buzzer.play(r2d2Buf, len, now);
  }
} r2d2Gen;

// ============================================================
// APPLY ENGINE
// ============================================================
struct ApplyEngine {
  bool active=false;
  SeqId target=SEQ1;
  ApplyPhase phase=AP_IDLE;
  uint32_t dueMs=0;

  bool justFinished=false;
  SeqId finishedSeq=SEQ1;

  void abort() {
    active=false; phase=AP_IDLE; justFinished=false;
  }
  bool isActive() const { return active; }

  void start(SeqId s, uint32_t now, Mode mode, SeqId activeSeq, SeqId pendingSeq) {
    target = s;
    active = true;
    justFinished = false;

    // Always cut power first
    powerCut(now, mode, activeSeq, pendingSeq, phase);

    const SeqPlan &p = PLAN[target];
    if (p.isTurnaround) {
      phase = AP_TURN_DWELL;
      dueMs = now + TURN_DWELL_MS;
      // LED rail hooks track train-control phases (not raw sensor edges)
      if (target == SEQ2) {
        ledRail.onDwellStart(now);
      }
    } else {
      phase = AP_PWR_SAFE;
      dueMs = now + SAFE_SWITCH_MS;

      // Treat entry into the approach / run sequences as the LED trigger.
      // (This makes the SEQ buttons in the UI drive the LED behavior too.)
      if (target == SEQ1) {
        ledRail.onSensorASeen(now);
      }
    }
  }

  void service(uint32_t now, Mode mode, SeqId activeSeq, SeqId pendingSeq) {
    justFinished = false;
    if (!active) return;
    if ((int32_t)(now - dueMs) < 0) return;

    const SeqPlan &p = PLAN[target];

    switch (phase) {
      case AP_PWR_SAFE:
        phase = AP_DIR_SAFE;
        dueMs = now;
        break;

      case AP_TURN_DWELL:
        if (target == SEQ2) {
          ledRail.onDwellEnd(now);
        }
        requestTurnoutRoute(turnoutRouteForSeq(target), now);
        setDirection(p.dir, now, true);
        phase = AP_TURN_DIR_SAFE;
        dueMs = now + SAFE_SWITCH_MS + TURNOUT_SETTLE_MS + TURNOUT_PULSE_MS + TURNOUT_SETTLE_MS;
        break;

      case AP_TURN_DIR_SAFE:
        setSpeedSelect(SPEED_LO);
        powerOnLO(now, mode, activeSeq, pendingSeq, phase);
        phase = AP_PWR_ON;
        dueMs = now;
        break;

      case AP_DIR_SAFE:
        setDirection(p.dir, now, true);
        phase = AP_SPEED_SAFE;
        dueMs = now + SAFE_SWITCH_MS;
        break;

      case AP_SPEED_SAFE:
        setSpeedSelect(p.speed);
        phase = AP_PWR_ON;
        {
          uint16_t extra = 0;
          Speed fromSpd = PLAN[activeSeq].speed;
          Speed toSpd   = p.speed;
          if (fromSpd == SPEED_LO && toSpd == SPEED_HI) extra = ACCEL_EXTRA_MS;
          else if (fromSpd == SPEED_HI && toSpd == SPEED_LO) extra = DECEL_EXTRA_MS;
          dueMs = now + SAFE_SWITCH_MS + (uint32_t)extra;
        }
        break;

      case AP_PWR_ON:
      default:
        if (p.isTurnaround) {
          setSpeedSelect(SPEED_LO);
          powerOnLO(now, mode, activeSeq, pendingSeq, phase);
        } else {
          powerOnForSpeed(now, p.speed, mode, activeSeq, pendingSeq, phase);
        }

        active = false;
        phase = AP_IDLE;
        justFinished = true;
        finishedSeq = target;
        break;
    }
  }
} apply;

// ============================================================
// DI7 DECODER (pressed fires hold ASAP)
// ============================================================
struct Di7Decoder {
  bool pressed=false;
  bool holdFired=false;
  uint32_t t0=0;
  uint32_t ignoreUntil=0;

  void ignoreFor(uint32_t now, uint16_t ms){ ignoreUntil = now + ms; }

  Di7Event update(bool isPressed, uint32_t now) {
    if (now < ignoreUntil) {
      // Ignore DI7 during cooldown without re-triggering when held through the ignore window.
      // If the button is held during ignore, treat it as already-consumed (no short/hold events)
      // until it is released and pressed again.
      if (!isPressed) {
        pressed = false;
        holdFired = false;
      } else {
        pressed = true;
        holdFired = true;
        t0 = now;
      }
      return DI7_NONE;
    }

    if (!pressed && isPressed) {
      pressed=true;
      holdFired=false;
      t0=now;
      return DI7_NONE;
    }

    if (pressed && isPressed && !holdFired) {
      uint32_t dt = now - t0;
      if (dt >= DI7_HOLD_MIN_MS) {
        holdFired = true;
        return DI7_HOLD;
      }
      return DI7_NONE;
    }

    if (pressed && !isPressed) {
      pressed=false;
      uint32_t dt = now - t0;
      if (dt < DI7_DEBOUNCE_MS) { holdFired=false; return DI7_NONE; }
      if (holdFired) { holdFired=false; return DI7_NONE; }
      return DI7_SHORT;
    }

    return DI7_NONE;
  }
} di7;

// ============================================================
// VIRTUAL DI7 (UI buttons emulate physical DI7 press)
// - "short" press: held just over DI7_DEBOUNCE_MS to generate DI7_SHORT on release
// - "hold"  press: held just over DI7_HOLD_MIN_MS to generate DI7_HOLD ASAP, then safely released
// ============================================================
static uint32_t g_vdi7_until_ms = 0;

static inline void vdi7PressMs(uint32_t now, uint16_t holdMs){
  // Extend (never shorten) an in-flight virtual press
  uint32_t until = now + (uint32_t)holdMs;
  if (until > g_vdi7_until_ms) g_vdi7_until_ms = until;
}

static inline bool vdi7IsPressed(uint32_t now){
  return (now < g_vdi7_until_ms);
}


// ============================================================
// SENSOR STABILITY
// ============================================================
enum SensorEdge : uint8_t { EDGE_NONE=0, EDGE_SEEN, EDGE_CLEAR };

struct StableSensor {
  bool raw=false;
  bool stable=false;
  uint32_t lastRawFlip=0;
  uint32_t lastStableFlip=0;

  SensorEdge update(bool rawSeen, uint32_t now,
                    uint16_t seenStableMs, uint16_t clearStableMs)
  {
    if (rawSeen != raw) {
      raw = rawSeen;
      lastRawFlip = now;
    }

    if (stable != raw) {
      uint32_t dt = now - lastRawFlip;
      uint16_t need = raw ? seenStableMs : clearStableMs;
      if (dt >= need) {
        stable = raw;
        lastStableFlip = now;
        return stable ? EDGE_SEEN : EDGE_CLEAR;
      }
    }
    return EDGE_NONE;
  }
};

StableSensor sens1;
StableSensor sens2;
StableSensor sens3;
StableSensor sens4;
StableSensor di6Cycle;


// Sensor activity tracking for failsafe
static uint32_t g_lastSensorActivityAt = 0;
// ============================================================
// SENSOR RING HELPERS
// ============================================================
static inline uint8_t nextSensorInRing(uint8_t s){
  switch(s){
    case 1: return 2;
    case 2: return 3;
    case 3: return 4;
    default: return 1;
  }
}
static inline bool sensorIsSpeedUp(uint8_t s){ return s==1 || s==3; }
static inline bool sensorIsDwell(uint8_t s){ return s==2 || s==4; }
static inline SeqId seqForSensorHit(uint8_t s){
  switch(s){
    case 1: return SEQ6; // visible departure speed-up
    case 2: return SEQ2; // EndB dwell then depart hidden REV LO
    case 3: return SEQ3; // hidden departure speed-up
    case 4: return SEQ5; // EndA dwell then depart visible FWD LO
    default: return SEQ1;
  }
}
static inline RouteSel turnoutRouteForSeq(SeqId s){
  return (s == SEQ2 || s == SEQ3) ? ROUTE_HIDDEN : ROUTE_VISIBLE;
}

// ============================================================
// CONTROLLER
// ============================================================
struct Controller {
  Mode mode = MODE_STOP;

  SeqId activeSeq = SEQ1;
  SeqId pendingSeq = SEQ1;

  bool stopPowerLatched=false;
  uint32_t cooldownUntil=0;

  // RETURN mode behavior: stop only after we have progressed away from SEQ1
  // and then SEQ1 is applied again ("stop on next SEQ1").
  bool returnStopArmed=false;
  bool returnSawNonSeq1=false;

  uint32_t lastAcceptedSensorAt=0;
  uint8_t  lastAcceptedSensor=255;

  uint8_t expectedSensor = 1;
  RouteSel currentRoute = ROUTE_VISIBLE;
  RouteSel desiredRoute = ROUTE_VISIBLE;
  bool emergencyTurnoutRequested = false;
  bool handoffActive = false;
  uint8_t handoffSensor = 0;
  bool handoffSeenArmed = false;

  // WLED (CH7) one-shots for SEQ1..SEQ3 side


  Dir currentDir() const { return PLAN[activeSeq].dir; }

  void render(uint32_t now, bool trig) {
    if (mode == MODE_STOP)   { pixelBlink(now, 0xFF0000); return; }
    if (mode == MODE_RETURN) { pixelBlink(now, 0xFFFF00); return; }
    pixelSet(trig ? 0x004040 : RUN_GREEN);
  }

  void playModeSound(Mode from, Mode to, uint32_t now) {
    if (from == MODE_RUN    && to == MODE_STOP)   soundGen.play(SND_RUN_TO_STOP, now);
    if (from == MODE_RUN    && to == MODE_RETURN) soundGen.play(SND_RUN_TO_RETURN, now);
    if (from == MODE_RETURN && to == MODE_RUN)    soundGen.play(SND_RETURN_TO_RUN, now);
    if (from == MODE_RETURN && to == MODE_STOP)   soundGen.play(SND_RETURN_TO_STOP, now);
    if (from == MODE_STOP   && to == MODE_RUN)    soundGen.play(SND_STOP_TO_RUN, now);
    if (from == MODE_STOP   && to == MODE_RETURN) soundGen.play(SND_STOP_TO_RETURN, now);
  }

  void enterStop(uint32_t now) {
    Mode prev = mode;
    mode = MODE_STOP;

    // Clear RETURN latch
    returnStopArmed = false;
    returnSawNonSeq1 = false;

    apply.abort();
    powerCut(now, mode, activeSeq, pendingSeq, apply.phase);
    stopPowerLatched = true;
    // Stop forces AUTO aux relays off; operator overrides (ON/OFF) can still hold them
    uvCtrl.forceOff();   handoffActive = false;
    handoffSeenArmed = false;

    playModeSound(prev, MODE_STOP, now);
    logPush(now, EVT_MODE_STOP, FAULT_NONE, mode, activeSeq, pendingSeq, apply.phase);
  }

  void restoreActivePower(uint32_t now) {
    stopPowerLatched = false;
    g_lastSensorActivityAt = now;
    pendingSeq = activeSeq;
    apply.start(activeSeq, now, mode, activeSeq, pendingSeq);
    uint32_t cd = (COOLDOWN_MS < STOP_EXIT_MIN_COOLDOWN_MS) ? STOP_EXIT_MIN_COOLDOWN_MS : COOLDOWN_MS;
    cooldownUntil = now + cd;
    di7.ignoreFor(now, DI7_IGNORE_AFTER_RESUME_MS);
    applyLightRulesForSeq(activeSeq);
    uvCtrl.forceOn();
    s2Cave.reset();
    expectedSensor = 1;
    desiredRoute = turnoutRouteForSeq(activeSeq);
  }

  void setModeRun(uint32_t now, bool fromStop) {
    Mode prev = mode;
    mode = MODE_RUN;

    // Clear RETURN latch
    returnStopArmed = false;
    returnSawNonSeq1 = false;
    playModeSound(prev, MODE_RUN, now);
    logPush(now, EVT_MODE_RUN, FAULT_NONE, mode, activeSeq, pendingSeq, apply.phase);
    if (fromStop) restoreActivePower(now);
  }

  void setModeReturn(uint32_t now, bool fromStop) {
    Mode prev = mode;
    mode = MODE_RETURN;

    // Arm "stop on next SEQ1" latch.
    // We only stop after we have first applied at least one non-SEQ1 state.
    returnStopArmed = true;
    returnSawNonSeq1 = false;
    playModeSound(prev, MODE_RETURN, now);
    logPush(now, EVT_MODE_RETURN, FAULT_NONE, mode, activeSeq, pendingSeq, apply.phase);
    if (fromStop) restoreActivePower(now);
  }

  void onDi7(Di7Event e, uint32_t now) {
    if (e == DI7_NONE) return;

    if (e == DI7_SHORT) logPush(now, EVT_DI7_SHORT, FAULT_NONE, mode, activeSeq, pendingSeq, apply.phase);
    if (e == DI7_HOLD)  logPush(now, EVT_DI7_HOLD,  FAULT_NONE, mode, activeSeq, pendingSeq, apply.phase);

    if (mode == MODE_RUN) {
      if (e == DI7_SHORT) enterStop(now);
      else                setModeReturn(now, false);
      return;
    }
    if (mode == MODE_RETURN) {
      if (e == DI7_SHORT) enterStop(now);
      else                setModeRun(now, false);
      return;
    }
    if (e == DI7_SHORT) setModeReturn(now, true);
    else                setModeRun(now, true);
  }

  void onApplyFinished(uint32_t now) {
    if (!apply.justFinished) return;

    // SEQ stats: close out previous active seq duration, start new
    SeqId prev = activeSeq;
    uint32_t dt = (g_seqStartMs == 0) ? 0 : (now - g_seqStartMs);
    if (g_seqStartMs != 0) g_seqLastMs[prev] = dt;

    activeSeq = apply.finishedSeq;

    g_seqCount[activeSeq] += 1;
    g_seqStartMs = now;

    applyLightRulesForSeq(activeSeq);
    logPush(now, EVT_APPLY_FINISH, FAULT_NONE, mode, activeSeq, pendingSeq, apply.phase);

    // RETURN MODE: stop on the next time SEQ1 is applied.
    // "Next" means: we must first observe at least one non-SEQ1 apply while in RETURN,
    // then when SEQ1 is applied again we enter STOP.
    if (mode == MODE_RETURN && returnStopArmed) {
      if (activeSeq != SEQ1) {
        returnSawNonSeq1 = true;
      } else if (returnSawNonSeq1) {
        enterStop(now);
        return; // enterStop already logged and cut power
      }
    }

    expectedSensor = nextSensorInRing(expectedSensor);
    currentRoute = turnoutRouteForSeq(activeSeq);
    desiredRoute = currentRoute;
    handoffActive = false;
    handoffSeenArmed = false;
  }

  bool acceptSensor(uint32_t now, uint8_t sensorId, LastEvent ignoredEvt) {
    if (lastAcceptedSensorAt != 0 && (now - lastAcceptedSensorAt) < MIN_BETWEEN_SENSORS_MS) {
      logPush(now, ignoredEvt, FAULT_NONE, mode, activeSeq, pendingSeq, apply.phase);
      return false;
    }
    lastAcceptedSensorAt = now;
    lastAcceptedSensor = sensorId;
    return true;
  }

  void applyFromSensor(uint32_t now, uint8_t sensorId, SensorEdge edge) {
    if (mode == MODE_STOP) return;
    if (apply.isActive()) return;
    if (now < cooldownUntil) return;
    if (edge != EDGE_SEEN) return;
    if (sensorId != expectedSensor) {
      logPush(now, EVT_A_IGNORED, FAULT_NONE, mode, activeSeq, pendingSeq, apply.phase);
      return;
    }
    if (!acceptSensor(now, sensorId, EVT_A_IGNORED)) return;
    pendingSeq = seqForSensorHit(sensorId);
    desiredRoute = turnoutRouteForSeq(pendingSeq);
    logPush(now, EVT_APPLY_START, FAULT_NONE, mode, activeSeq, pendingSeq, apply.phase);
    apply.start(pendingSeq, now, mode, activeSeq, pendingSeq);
    cooldownUntil = now + COOLDOWN_MS;
  }
} ctrl;

// ============================================================
// PARAM SAFETY (kept)
// ============================================================
static inline uint16_t clampU16(uint16_t v, uint16_t lo, uint16_t hi){
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static inline uint16_t quantizeU16(uint16_t v, uint16_t step){
  if (step <= 1) return v;
  return (uint16_t)((v / step) * step);
}
static inline bool isStopLocked() {
  return (ctrl.mode != MODE_STOP) || apply.isActive();
}
static inline uint16_t argU16NonNeg(const String& s) {
  long v = s.toInt();
  if (v < 0) v = 0;
  if (v > 65535) v = 65535;
  return (uint16_t)v;
}

enum ParamId : uint8_t {
  P_SAFE=0, P_DWELL, P_COOL, P_SEEN, P_CLEAR, P_MINBETWEEN, P_GAP, P_STUCK,
  P_WSHORT, P_WLONG, P_WGAP,
  P_OVRSHORT, P_OVRLONG,
  P_BEEP1, P_BEEP2, P_BEEPBASE, P_BEEPOCT,
  P_ACCELX_MS, P_DECELX_MS,
  P_PBRIGHT,
  P_COUNT
};

struct ParamMeta {
  const char* key;
  uint16_t hardMin, hardMax;
  uint16_t uiMin, uiMax;
  uint16_t step;
  bool requireStop;
  const char* units;
};

static const ParamMeta PARAMS[P_COUNT] = {
  {"safety",         10,   500,     15,  450,   5,   true,   "ms"},
  {"dwell",         250,  5000,    300, 4500,  50,   true,   "ms"},
  {"cool",            0,  5000,     25, 3000,  25,   true,   "ms"},
  {"seen",            0,  1000,     10,  600,   5,   true,   "ms"},
  {"clear",           0,  2000,     50, 1200,  10,   true,   "ms"},
  {"minbetween",      0, 30000,     50, 20000,  50,  true,   "ms"},
  {"gap",            50, 30000,     50, 20000,  50,  true,   "ms"},
  {"stuck",          50, 60000,     50, 20000,  50,  true,   "ms"},
  {"wshort",         10,  2000,    100,  800,  10,  false,  "ms"},
  {"wlong",         100,  8000,   1000, 5000,  50,  false,  "ms"},
  {"wgap",           1,  2000,     10,  800,  10,  false,  "ms"},
  {"ovrshort",       10,  2000,     50,  800,  10,  false,  "ms"},
  {"ovrlong",       100,  8000,    250, 5000,  50,  false,  "ms"},
  {"beep1",          10,  2000,     10, 2000,  10,  false,  "ms"},
  {"beep2",          10,  2000,     10, 2000,  10,  false,  "ms"},
  {"beepbase",      100,  5000,    200, 2000,  10,  false,  "Hz"},
  {"beepoct",         1,     3,      1,    3,   1,  false,  "oct"},

  {"accelx",        0,  5000,      0,  1500,  25, true,   "ms"},
  {"decelx",        0,  5000,      0,  1500,  25, true,   "ms"},
{"pbright",         0,   255,      0,  255,   1,  false,  ""}
};

// ============================================================
// WIFI PERSISTENCE + CONNECT
// ============================================================
static bool parseIP(const String& s, IPAddress &out) {
  int a,b,c,d;
  if (sscanf(s.c_str(), "%d.%d.%d.%d", &a,&b,&c,&d) != 4) return false;
  if (a<0||a>255||b<0||b>255||c<0||c>255||d<0||d>255) return false;
  out = IPAddress((uint8_t)a,(uint8_t)b,(uint8_t)c,(uint8_t)d);
  return true;
}

static void loadWifiPrefs() {
  AP_SSID = prefs.getString("ap_ssid", AP_SSID);
  AP_PASS = prefs.getString("ap_pass", AP_PASS);
  STA_SSID = prefs.getString("ssid", "");
  STA_PASS = prefs.getString("pass", "");
  STA_USE_STATIC = prefs.getBool("stastatic", false);

  String ip  = prefs.getString("ip",  "");
  String gw  = prefs.getString("gw",  "");
  String msk = prefs.getString("mask","");
  String dns = prefs.getString("dns","");

  if (!parseIP(ip, STA_IP))   STA_IP = IPAddress(0,0,0,0);
  if (!parseIP(gw, STA_GW))   STA_GW = IPAddress(0,0,0,0);
  if (!parseIP(msk, STA_MASK))STA_MASK = IPAddress(0,0,0,0);
  if (!parseIP(dns, STA_DNS)) STA_DNS = IPAddress(0,0,0,0);
}

static void saveWifiPrefs() {
  prefs.putString("ap_ssid", AP_SSID);
  prefs.putString("ap_pass", AP_PASS);
  prefs.putString("ssid", STA_SSID);
  prefs.putString("pass", STA_PASS);
  prefs.putBool("stastatic", STA_USE_STATIC);
  prefs.putString("ip",   STA_IP.toString());
  prefs.putString("gw",   STA_GW.toString());
  prefs.putString("mask", STA_MASK.toString());
  prefs.putString("dns",  STA_DNS.toString());
}

// ============================================================
// LED BOX PERSISTENCE
// ============================================================
static void loadLedPrefs() {
  ledRail.enabled = prefs.getBool("len", true);
  ledRail.segN    = (uint16_t)prefs.getUShort("lseg", 15);
  ledRail.pattern = (uint8_t)prefs.getUChar("lpat", (uint8_t)LED_PAT_TAIL4);
  ledRail.r       = (uint8_t)prefs.getUChar("lr", 255);
  ledRail.g       = (uint8_t)prefs.getUChar("lg", 255);
  ledRail.b       = (uint8_t)prefs.getUChar("lb", 255);
  ledRail.bri     = (uint8_t)prefs.getUChar("lbri", 230);
  ledRail.sweepMs = (uint16_t)prefs.getUShort("lsw", 2000);
  ledRail.pauseMs = (uint16_t)prefs.getUShort("lpa", 2000);
  ledRail.backMs  = (uint16_t)prefs.getUShort("lba", 2000);

  // sanitize
  ledRail.segN = (uint16_t)constrain((int)ledRail.segN, (int)LED_SEG_MIN, (int)LED_SEG_MAX);
  if (ledRail.segN > LED_PHYS_N) ledRail.segN = LED_PHYS_N;
  if (ledRail.sweepMs < 1) ledRail.sweepMs = 1; if (ledRail.sweepMs > 10000) ledRail.sweepMs = 10000;
  if (ledRail.pauseMs < 1) ledRail.pauseMs = 1; if (ledRail.pauseMs > 10000) ledRail.pauseMs = 10000;
  if (ledRail.backMs  < 1) ledRail.backMs  = 1; if (ledRail.backMs  > 10000) ledRail.backMs  = 10000;
}

static void saveLedPrefs() {
  prefs.putBool("len", ledRail.enabled);
  prefs.putUShort("lseg", ledRail.segN);
  prefs.putUChar("lpat", ledRail.pattern);
  prefs.putUChar("lr", ledRail.r);
  prefs.putUChar("lg", ledRail.g);
  prefs.putUChar("lb", ledRail.b);
  prefs.putUChar("lbri", ledRail.bri);
  prefs.putUShort("lsw", ledRail.sweepMs);
  prefs.putUShort("lpa", ledRail.pauseMs);
  prefs.putUShort("lba", ledRail.backMs);
}


static void loadCavePrefs() {
  caveFullSpec.enabled        = prefs.getBool("c_en", true);
  caveFullSpec.segN           = (uint16_t)prefs.getUShort("c_seg", 41);
  caveFullSpec.r              = (uint8_t)prefs.getUChar("c_r", 255);
  caveFullSpec.g              = (uint8_t)prefs.getUChar("c_g", 255);
  caveFullSpec.b              = (uint8_t)prefs.getUChar("c_b", 255);
  caveFullSpec.briPct         = (uint8_t)prefs.getUChar("c_bp", 75);
  caveFullSpec.triggerDelayMs = (uint16_t)prefs.getUShort("c_del", 500);
  caveFullSpec.fadeMs         = (uint16_t)prefs.getUShort("c_fad", 1500);
  uvCtrl.offDelayMs           = (uint16_t)prefs.getUShort("uv_off", 500);
  uvCtrl.onDelayMs            = (uint16_t)prefs.getUShort("uv_on", 500);

  caveFullSpec.segN           = (uint16_t)constrain((int)caveFullSpec.segN, 1, (int)FULLSPEC_SEG_MAX);
  caveFullSpec.briPct         = (uint8_t)constrain((int)caveFullSpec.briPct, 0, 100);
  caveFullSpec.triggerDelayMs = (uint16_t)constrain((int)caveFullSpec.triggerDelayMs, 0, 15000);
  caveFullSpec.fadeMs         = (uint16_t)constrain((int)caveFullSpec.fadeMs, 0, 15000);
  uvCtrl.offDelayMs           = (uint16_t)constrain((int)uvCtrl.offDelayMs, 0, 15000);
  uvCtrl.onDelayMs            = (uint16_t)constrain((int)uvCtrl.onDelayMs, 0, 15000);
}

static void saveCavePrefs() {
  prefs.putBool("c_en", caveFullSpec.enabled);
  prefs.putUShort("c_seg", caveFullSpec.segN);
  prefs.putUChar("c_r", caveFullSpec.r);
  prefs.putUChar("c_g", caveFullSpec.g);
  prefs.putUChar("c_b", caveFullSpec.b);
  prefs.putUChar("c_bp", caveFullSpec.briPct);
  prefs.putUShort("c_del", caveFullSpec.triggerDelayMs);
  prefs.putUShort("c_fad", caveFullSpec.fadeMs);
  prefs.putUShort("uv_off", uvCtrl.offDelayMs);
  prefs.putUShort("uv_on", uvCtrl.onDelayMs);
}

static void loadBeaconPrefs() {
  BEACON_BRIGHTNESS = (uint8_t)prefs.getUChar("b_bri", 96);
  BEACON_FAULT_ONLY = prefs.getBool("b_fault", false);
  BEACON_EVENT_MS   = (uint16_t)prefs.getUShort("b_evt", 600);

  BEACON_BRIGHTNESS = (uint8_t)constrain((int)BEACON_BRIGHTNESS, 0, 255);
  BEACON_EVENT_MS   = (uint16_t)constrain((int)BEACON_EVENT_MS, 50, 5000);
}

static void saveBeaconPrefs() {
  prefs.putUChar("b_bri", BEACON_BRIGHTNESS);
  prefs.putBool("b_fault", BEACON_FAULT_ONLY);
  prefs.putUShort("b_evt", BEACON_EVENT_MS);
}

static void handleBeaconSet() {
  bool any = false;

  if (server.hasArg("bri"))   { BEACON_BRIGHTNESS = (uint8_t)server.arg("bri").toInt(); any = true; }
  if (server.hasArg("fault")) { BEACON_FAULT_ONLY = (server.arg("fault").toInt() != 0); any = true; }
  if (server.hasArg("evt"))   { BEACON_EVENT_MS = (uint16_t)server.arg("evt").toInt(); any = true; }

  BEACON_BRIGHTNESS = (uint8_t)constrain((int)BEACON_BRIGHTNESS, 0, 255);
  BEACON_EVENT_MS   = (uint16_t)constrain((int)BEACON_EVENT_MS, 50, 5000);

  if (any) saveBeaconPrefs();

  String s="{";
  s += "\"ok\":true,";
  s += "\"bri\":" + String((int)BEACON_BRIGHTNESS) + ",";
  s += "\"fault\":" + String(BEACON_FAULT_ONLY ? "true":"false") + ",";
  s += "\"evt\":" + String((int)BEACON_EVENT_MS);
  s += "}";
  server.send(200, "application/json", s);
}


// ============================================================
// /led_set (writes LED box params, persists)
// ============================================================
static void handleLedSet() {
  bool any = false;

  if (server.hasArg("en"))   { ledRail.enabled = (server.arg("en").toInt() != 0); any = true; }
  if (server.hasArg("seg"))  { ledRail.segN = (uint16_t)server.arg("seg").toInt(); any = true; }
  if (server.hasArg("pat"))  { ledRail.pattern = (uint8_t)server.arg("pat").toInt(); any = true; }
  if (server.hasArg("r"))    { ledRail.r = (uint8_t)server.arg("r").toInt(); any = true; }
  if (server.hasArg("g"))    { ledRail.g = (uint8_t)server.arg("g").toInt(); any = true; }
  if (server.hasArg("b"))    { ledRail.b = (uint8_t)server.arg("b").toInt(); any = true; }
  if (server.hasArg("bri"))  { ledRail.bri = (uint8_t)server.arg("bri").toInt(); any = true; }
  if (server.hasArg("sweep")){ ledRail.sweepMs = (uint16_t)server.arg("sweep").toInt(); any = true; }
  if (server.hasArg("pause")){ ledRail.pauseMs = (uint16_t)server.arg("pause").toInt(); any = true; }
  if (server.hasArg("back")) { ledRail.backMs  = (uint16_t)server.arg("back").toInt(); any = true; }

  // sanitize
  ledRail.segN = (uint16_t)constrain((int)ledRail.segN, (int)LED_SEG_MIN, (int)LED_SEG_MAX);
  if (ledRail.segN > LED_PHYS_N) ledRail.segN = LED_PHYS_N;
  ledRail.pattern = (uint8_t)constrain((int)ledRail.pattern, 0, 2);
  ledRail.r = (uint8_t)constrain((int)ledRail.r, 0, 255);
  ledRail.g = (uint8_t)constrain((int)ledRail.g, 0, 255);
  ledRail.b = (uint8_t)constrain((int)ledRail.b, 0, 255);
  ledRail.bri = (uint8_t)constrain((int)ledRail.bri, 0, 255);
  ledRail.sweepMs = (uint16_t)constrain((int)ledRail.sweepMs, 1, 10000);
  ledRail.pauseMs = (uint16_t)constrain((int)ledRail.pauseMs, 1, 10000);
  ledRail.backMs  = (uint16_t)constrain((int)ledRail.backMs,  1, 10000);

  if (any) saveLedPrefs();

  String s="{";
  s += "\"ok\":true,";
  s += "\"en\":" + String(ledRail.enabled?"true":"false") + ",";
  s += "\"seg\":" + String((int)ledRail.segN) + ",";
  s += "\"pat\":" + String((int)ledRail.pattern) + ",";
  s += "\"r\":" + String((int)ledRail.r) + ",";
  s += "\"g\":" + String((int)ledRail.g) + ",";
  s += "\"b\":" + String((int)ledRail.b) + ",";
  s += "\"bri\":" + String((int)ledRail.bri) + ",";
  s += "\"sweep\":" + String((int)ledRail.sweepMs) + ",";
  s += "\"pause\":" + String((int)ledRail.pauseMs) + ",";
  s += "\"back\":" + String((int)ledRail.backMs);
  s += "}";
  server.send(200, "application/json", s);
}

static void handleCaveSet() {
  bool any = false;

  if (server.hasArg("en"))    { caveFullSpec.enabled = (server.arg("en").toInt() != 0); any = true; }
  if (server.hasArg("seg"))   { caveFullSpec.segN = (uint16_t)server.arg("seg").toInt(); any = true; }
  if (server.hasArg("r"))     { caveFullSpec.r = (uint8_t)server.arg("r").toInt(); any = true; }
  if (server.hasArg("g"))     { caveFullSpec.g = (uint8_t)server.arg("g").toInt(); any = true; }
  if (server.hasArg("b"))     { caveFullSpec.b = (uint8_t)server.arg("b").toInt(); any = true; }
  if (server.hasArg("bp"))    { caveFullSpec.briPct = (uint8_t)server.arg("bp").toInt(); any = true; }
  if (server.hasArg("delay")) { caveFullSpec.triggerDelayMs = (uint16_t)server.arg("delay").toInt(); any = true; }
  if (server.hasArg("fade"))  { caveFullSpec.fadeMs = (uint16_t)server.arg("fade").toInt(); any = true; }
  if (server.hasArg("uvoff")) { uvCtrl.offDelayMs = (uint16_t)server.arg("uvoff").toInt(); any = true; }
  if (server.hasArg("uvon"))  { uvCtrl.onDelayMs  = (uint16_t)server.arg("uvon").toInt(); any = true; }

  caveFullSpec.segN           = (uint16_t)constrain((int)caveFullSpec.segN, 1, (int)FULLSPEC_SEG_MAX);
  caveFullSpec.r              = (uint8_t)constrain((int)caveFullSpec.r, 0, 255);
  caveFullSpec.g              = (uint8_t)constrain((int)caveFullSpec.g, 0, 255);
  caveFullSpec.b              = (uint8_t)constrain((int)caveFullSpec.b, 0, 255);
  caveFullSpec.briPct         = (uint8_t)constrain((int)caveFullSpec.briPct, 0, 100);
  caveFullSpec.triggerDelayMs = (uint16_t)constrain((int)caveFullSpec.triggerDelayMs, 0, 15000);
  caveFullSpec.fadeMs         = (uint16_t)constrain((int)caveFullSpec.fadeMs, 0, 15000);
  uvCtrl.offDelayMs           = (uint16_t)constrain((int)uvCtrl.offDelayMs, 0, 15000);
  uvCtrl.onDelayMs            = (uint16_t)constrain((int)uvCtrl.onDelayMs, 0, 15000);

  if (any) saveCavePrefs();

  String s="{";
  s += "\"ok\":true,";
  s += "\"en\":" + String(caveFullSpec.enabled?"true":"false") + ",";
  s += "\"seg\":" + String((int)caveFullSpec.segN) + ",";
  s += "\"r\":" + String((int)caveFullSpec.r) + ",";
  s += "\"g\":" + String((int)caveFullSpec.g) + ",";
  s += "\"b\":" + String((int)caveFullSpec.b) + ",";
  s += "\"bp\":" + String((int)caveFullSpec.briPct) + ",";
  s += "\"delay\":" + String((int)caveFullSpec.triggerDelayMs) + ",";
  s += "\"fade\":" + String((int)caveFullSpec.fadeMs) + ",";
  s += "\"uvoff\":" + String((int)uvCtrl.offDelayMs) + ",";
  s += "\"uvon\":" + String((int)uvCtrl.onDelayMs);
  s += "}";
  server.send(200, "application/json", s);
}


static void wifiApplyStaConfig() {
  if (STA_USE_STATIC && STA_IP != IPAddress(0,0,0,0) && STA_GW != IPAddress(0,0,0,0) && STA_MASK != IPAddress(0,0,0,0)) {
    IPAddress dns = (STA_DNS == IPAddress(0,0,0,0)) ? STA_GW : STA_DNS;
    WiFi.config(STA_IP, STA_GW, STA_MASK, dns);
  } else {
    // reset to DHCP
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  }
}

static void wifiConnectNow() {
  WiFi.disconnect(false, true);
  delay(20);

  if (STA_SSID.length() == 0) return;

  wifiApplyStaConfig();
  WiFi.begin(STA_SSID.c_str(), STA_PASS.c_str());
  g_lastWifiChangeMs = millis();
  logPush(g_lastWifiChangeMs, EVT_WIFI_UPDATE, FAULT_NONE, ctrl.mode, ctrl.activeSeq, ctrl.pendingSeq, apply.phase);
}

// ============================================================
// WEB UI (Dark Theme + Tabs + WiFi Settings + SEQ stats)
// ============================================================
static String htmlRow(const String& k, const String& v){
  return "<tr><td class='k'>" + k + "</td><td class='v'><b>" + v + "</b></td></tr>";
}

static String htmlPage() {
  String s;
  s.reserve(52000);

  s += "<!doctype html><html><head><meta charset='utf-8'/>";
  s += "<meta name='viewport' content='width=device-width,initial-scale=1'/>";
  s += "<title>TrainCtrl</title>";

  // Dark theme
  s += "<style>";
  s += ":root{--bg:#0b0d10;--card:#12151a;--card2:#0f1216;--txt:#e8edf2;--mut:#9aa6b2;--br:#26303a;--btn:#1a2027;--btnh:#202834;--ok:#11331c;--warn:#332a11;--bad:#3a1414}";
  s += "body{font-family:system-ui,Segoe UI,Roboto,Arial,sans-serif;background:var(--bg);color:var(--txt);max-width:1020px;margin:18px auto;padding:0 14px}";
  s += "@media(min-width:760px){body{padding:0 220px 0 14px}.stickycol{display:block}}@media(max-width:759px){.stickycol{display:none}}";
  s += "a{color:#8ab4ff;text-decoration:none} a:hover{text-decoration:underline}";
  s += "code{background:rgba(255,255,255,.06);padding:2px 6px;border-radius:8px;border:1px solid var(--br)}";
  s += "table{border-collapse:collapse;width:100%;max-width:980px}";
  s += "tr{border-bottom:1px solid var(--br)}";
  s += "td.k{padding:7px 10px;opacity:.9;width:44%;color:var(--mut)}";
  s += "td.v{padding:7px 10px}";
  s += ".row{display:flex;flex-wrap:wrap;gap:10px;align-items:center}";
  s += ".topbar{display:flex;gap:10px;align-items:center;justify-content:space-between;margin-bottom:10px}";
  s += ".tabs{display:flex;gap:8px;flex-wrap:wrap}";
  s += ".tabbtn{border:1px solid var(--br);background:var(--btn);color:var(--txt);padding:10px 12px;border-radius:12px;cursor:pointer}";
  s += ".tabbtn:hover{background:var(--btnh)}";
  s += ".tabbtn.on{outline:2px solid rgba(138,180,255,.35)}";
  s += ".pill{display:inline-block;padding:5px 10px;border-radius:999px;border:1px solid var(--br);font-size:12px;color:var(--txt)}";
  s += ".pill.ok{background:var(--ok)} .pill.warn{background:var(--warn)} .pill.bad{background:var(--bad)}";
  s += ".card{border:1px solid var(--br);border-radius:16px;padding:12px;margin:12px 0;background:var(--card);box-shadow:0 1px 0 rgba(0,0,0,.25)}";
  s += ".grid{display:grid;grid-template-columns:1fr;gap:12px}";
  s += "@media(min-width:760px){.grid{grid-template-columns:1fr 1fr}}";
  s += ".dashgrid{display:grid;grid-template-columns:1fr;gap:12px}.sortable{grid-auto-flow:row dense;align-items:start}.box{margin:0}.handle{cursor:grab;user-select:none}.box.dragging{opacity:.6}.box.drop-hint{outline:2px dashed rgba(138,180,255,.35)}";
  s += "@media(min-width:760px){.dashgrid{grid-template-columns:1fr 1fr;align-items:start}}";
  s += ".dashgrid .left{grid-column:1} .dashgrid .right{grid-column:2}";
  s += "@media(max-width:759px){.dashgrid .left,.dashgrid .right{grid-column:1}}";
  s += ".btn{display:inline-block;padding:10px 14px;border:1px solid var(--br);border-radius:12px;background:var(--btn);color:var(--txt);cursor:pointer}";
  s += ".btn.sm{padding:7px 10px;border-radius:10px;font-size:12px}";
  s += ".btn.on{outline:2px solid rgba(138,180,255,.35)}";
  s += ".btn:hover{background:var(--btnh)}";
  s += ".btn:active{transform:translateY(1px)}";
  s += ".muted{color:var(--mut)} .small{font-size:12px}";
  s += ".kv{display:flex;justify-content:space-between;gap:10px}";
  s += ".slider{width:100%}";
  s += "input,select{background:var(--card2);color:var(--txt);border:1px solid var(--br);border-radius:12px;padding:10px 10px;outline:none}";
  s += "input:focus,select:focus{outline:2px solid rgba(138,180,255,.25)}";
  s += ".h3{margin:4px 0 10px}";
  s += ".sec{display:none} .sec.on{display:block}";
  s += ".topctrl{display:grid;grid-template-columns:1fr 1fr;gap:10px}.toprow{display:flex;gap:10px;align-items:center;justify-content:space-between}.topinfo{color:var(--mut)}.stickycol{position:fixed;top:96px;right:12px;width:180px;border:1px solid var(--br);border-radius:14px;background:rgba(18,21,26,.92);backdrop-filter:blur(4px);padding:10px;z-index:20}.stickline{display:flex;justify-content:space-between;gap:10px;padding:6px 0;border-bottom:1px solid var(--br)}.stickline:last-child{border-bottom:none}.stickk{font-size:12px;color:var(--mut)}@media(max-width:1100px){body{padding:0 14px}.stickycol{display:none}}";
  s += "</style></head><body>";

  // Header + tabs
  s += "<div class='topbar'>";
  s += "<div><div style='font-size:20px;font-weight:700'>TrainCtrl</div>";
  s += "<div class='small muted'>AP <code>"; s += AP_SSID; s += "</code> / Pass <code>"; s += AP_PASS; s += "</code> | Build <code>"; s += BUILD_TAG; s += "</code></div></div>";
  s += "<div class='tabs'>";
  s += "<button id='tabDash' class='tabbtn on' onclick=\"setTab('dash')\">Dashboard</button>";
  s += "<button id='tabWifi' class='tabbtn' onclick=\"setTab('wifi')\">WiFi Settings</button>";
  s += "<span id='lockPill' class='pill warn'>LOCK?</span>";
  s += "</div>";
  s += "</div>";

  // DASH SECTION
  s += "<div id='secDash' class='sec on'>";

  // TOP CONTROL BOX (buttons + key live values)
  s += "<div class='card'><div class='h3'>Controls</div>";
  s += "<div class='topctrl'>";
  s += "<div class='toprow'><button id='btnStop2' class='btn' onclick=\"cmdMode('stop')\">STOP</button><div class='topinfo'>MODE: <b id='st_mode_top'>-</b></div></div>";
  s += "<div class='toprow'><button id='btnReturn2' class='btn' onclick=\"cmdMode('return')\">RETURN</button><div class='topinfo'>DIR: <b id='st_dir_top'>-</b></div></div>";
  s += "<div class='toprow'><button id='btnRun2' class='btn' onclick=\"cmdMode('run')\">RUN</button><div class='topinfo'>Active SEQ: <b id='st_seq_top'>-</b></div></div>";
  s += "<div class='toprow'><button class='btn' onclick=\"cmdFaultClear()\">Clear Fault</button><div class='topinfo'>LastFault: <b id='st_fault_top'>-</b></div></div>";
  s += "</div>";
  s += "</div>";

  // Sticky right-side summary column
  s += "<div id='sticky' class='stickycol'>";
  s += "<div class='stickline'><span class='stickk'>MODE</span><b id='sc_mode'>-</b></div>";
  s += "<div class='stickline'><span class='stickk'>DIR</span><b id='sc_dir'>-</b></div>";
  s += "<div class='stickline'><span class='stickk'>SEQ</span><b id='sc_seq'>-</b></div>";
  s += "<div class='stickline'><span class='stickk'>EXP</span><b id='sc_exp'>-</b></div>";
  s += "<div class='stickline'><span class='stickk'>FAULT</span><b id='sc_fault'>-</b></div>";
  s += "</div>";

  s += "<div id='dashSort' class='dashgrid sortable'>";

  // Live State
  s += "<div class='card box' draggable='true' data-box='live'><div class='h3 handle'>Live State</div>";
  s += "<table>";
  s += htmlRow("Mode", "<span id='st_mode'>-</span>");
  s += htmlRow("Active Seq", "<span id='st_seq'>-</span>");
  s += htmlRow("Current SEQ elapsed", "<span id='st_seq_elapsed'>-</span> ms");
  s += htmlRow("Dir", "<span id='st_dir'>-</span>");
  s += htmlRow("Expected sensor", "<span id='st_exp'>-</span>");
  s += htmlRow("Handoff active", "<span id='st_handoff'>-</span>");
  s += htmlRow("Apply active", "<span id='st_apply'>-</span>");
  s += htmlRow("Apply phase", "<span id='st_phase'>-</span>");
  s += htmlRow("PWR LO gate", "<span id='st_pwrlo'>-</span>");
  s += htmlRow("PWR HI gate", "<span id='st_pwrhi'>-</span>");
  s += htmlRow("Last event", "<span id='st_evt'>-</span>");
  s += htmlRow("Last fault", "<span id='st_fault'>-</span>");
  s += "</table>";
  s += "<div id='toast' class='small muted' style='margin-top:10px'></div><div id='conn' class='small muted' style='margin-top:6px;opacity:.9'></div>";
  s += "</div>";

  // Controls
  s += "<div class='card box' draggable='true' data-box='controls'><div class='h3 handle'>Controls</div>";
  s += "<div class='row'>";
  s += "<button id='btnRun' class='btn' onclick=\"cmdMode('run')\">RUN</button>";
  s += "<button id='btnReturn' class='btn' onclick=\"cmdMode('return')\">RETURN</button>";
  s += "<button id='btnStop' class='btn' onclick=\"cmdMode('stop')\">STOP</button>";
  s += "<button class='btn' onclick=\"cmdFaultClear()\">Clear Fault</button>";
  s += "<button class='btn' onclick=\"cmdSound('r2d2')\">R2D2</button>";
  s += "</div>";
  s += "<div class='row' style='margin-top:8px'>";
  s += "<button class='btn' onclick=\"cmdSeq(1)\">Apply SEQ1</button>";
  s += "<button class='btn' onclick=\"cmdSeq(2)\">Apply SEQ2</button>";
  s += "<button class='btn' onclick=\"cmdSeq(3)\">Apply SEQ3</button>";
  s += "<button class='btn' onclick=\"cmdSeq(4)\">Apply SEQ4</button>";
  s += "<button class='btn' onclick=\"cmdSeq(5)\">Apply SEQ5</button>";
  s += "<button class='btn' onclick=\"cmdSeq(6)\">Apply SEQ6</button>";
  s += "</div>";
  s += "<div class='small muted' style='margin-top:10px'>Buttons do not navigate; state updates live.</div>";
  s += "</div>";

  // Relay overrides
  s += "<div class='card box' draggable='true' data-box='relayovr'><div class='h3 handle'>Relay Overrides (CH7-CH8) + Turnouts</div><div class='row' style='gap:8px;margin-bottom:8px'><button class='btn' onclick=\"tget('/cmd?cycle_expected=1')\">Cycle Expected Sensor</button><button class='btn' onclick=\"tget('/cmd?turnout=visible')\">Turnout Visible</button><button class='btn' onclick=\"tget('/cmd?turnout=hidden')\">Turnout Hidden</button></div>";
  s += "<div class='small muted'>Auto follows SEQ/internal rules. On/Off locks ignoring SEQs. Short/Long are momentary then return to Auto.</div>";
  s += "<table style='margin-top:10px'>";
  s += "<!-- CH5 moved to turnout control --><tr style='display:none'><td class='k'>CH5 Pulse</td><td class='v'>";
  s += "<div class='row'>";
  s += "<button id='ov5_auto' class='btn sm' onclick=\"cmdOvr(5,'auto')\">Auto</button>";
  s += "<button id='ov5_on' class='btn sm' onclick=\"cmdOvr(5,'on')\">On</button>";
  s += "<button id='ov5_off' class='btn sm' onclick=\"cmdOvr(5,'off')\">Off</button>";
  s += "<button id='ov5_sp' class='btn sm' onclick=\"cmdOvr(5,'sp')\">ShortPulse</button>";
  s += "<button id='ov5_lp' class='btn sm' onclick=\"cmdOvr(5,'lp')\">LongPulse</button>";
  s += "</div>";
  s += "<div class='small muted' style='margin-top:6px'>Mode: <b id='ovm5'>-</b> &nbsp; State: <b id='ovs5'>-</b> <span id='ovp5' class='small muted'></span></div>";
  s += "</td></tr>";
  s += "<tr style='display:none'><td class='k'>CH6 Selector</td><td class='v'>";
  s += "<div class='row'>";
  s += "<button id='ov6_auto' class='btn sm' onclick=\"cmdOvr(6,'auto')\">Auto</button>";
  s += "<button id='ov6_on' class='btn sm' onclick=\"cmdOvr(6,'on')\">On</button>";
  s += "<button id='ov6_off' class='btn sm' onclick=\"cmdOvr(6,'off')\">Off</button>";
  s += "<button id='ov6_sp' class='btn sm' onclick=\"cmdOvr(6,'sp')\">ShortPulse</button>";
  s += "<button id='ov6_lp' class='btn sm' onclick=\"cmdOvr(6,'lp')\">LongPulse</button>";
  s += "</div>";
  s += "<div class='small muted' style='margin-top:6px'>Mode: <b id='ovm6'>-</b> &nbsp; State: <b id='ovs6'>-</b> <span id='ovp6' class='small muted'></span></div>";
  s += "</td></tr>";
  s += "<tr><td class='k'>CH7</td><td class='v'>";
  s += "<div class='row'>";
  s += "<button id='ov7_auto' class='btn sm' onclick=\"cmdOvr(7,'auto')\">Auto</button>";
  s += "<button id='ov7_on' class='btn sm' onclick=\"cmdOvr(7,'on')\">On</button>";
  s += "<button id='ov7_off' class='btn sm' onclick=\"cmdOvr(7,'off')\">Off</button>";
  s += "";
  s += "";
  s += "</div>";
  s += "<div class='small muted' style='margin-top:6px'>Mode: <b id='ovm7'>-</b> &nbsp; State: <b id='ovs7'>-</b> <span id='ovp7' class='small muted'></span></div>";
  s += "</td></tr>";
  s += "<tr><td class='k'>CH8</td><td class='v'>";
  s += "<div class='row'>";
  s += "<button id='ov8_auto' class='btn sm' onclick=\"cmdOvr(8,'auto')\">Auto</button>";
  s += "<button id='ov8_on' class='btn sm' onclick=\"cmdOvr(8,'on')\">On</button>";
  s += "<button id='ov8_off' class='btn sm' onclick=\"cmdOvr(8,'off')\">Off</button>";
  s += "<button id='ov8_sp' class='btn sm' onclick=\"cmdOvr(8,'sp')\">ShortPulse</button>";
  s += "<button id='ov8_lp' class='btn sm' onclick=\"cmdOvr(8,'lp')\">LongPulse</button>";
  s += "</div>";
  s += "<div class='small muted' style='margin-top:6px'>Mode: <b id='ovm8'>-</b> &nbsp; State: <b id='ovs8'>-</b> <span id='ovp8' class='small muted'></span></div>";
  s += "</td></tr>";
  s += "</table>";
  s += "</div>";

  // WS2812 LED Box (dedicated UI)
  s += "<div class='card box' draggable='true' data-box='ws2812'><div class='h3 handle'>WS2812 LED Box</div>";
  s += "<div class='small muted'>Data GPIO: <b>21</b> &nbsp; Power gate: <b>CH7</b></div>";

  s += "<table style='margin-top:10px'>";
  s += htmlRow("Power (CH7)", "<b id='led_pwr'>-</b>");
  s += htmlRow("LED state", "<b id='led_state'>-</b>");
  s += htmlRow("Position", "<b id='led_pos'>-</b>");
  s += "</table>";

  s += "<div class='row' style='margin-top:10px;gap:10px;align-items:center'>";
  s += "<label class='small muted' style='display:flex;gap:8px;align-items:center'>"
       "<input id='led_en' type='checkbox' onchange='ledCommit()'/> Enable sensor-trigger sweep"
       "</label>";
  s += "<button class='btn sm' onclick='cmdLedRun()'>Run</button>";
  s += "<button class='btn sm' onclick='cmdLedOff()'>Off</button>";
  s += "</div>";

  s += "<div class='grid' style='margin-top:12px;grid-template-columns:1fr 1fr;gap:12px'>";
  // Segment length
  s += "<div><div class='small muted'>Segment length</div>"
       "<input id='led_seg' class='slider' type='range' min='1' max='60' step='1' oninput='ledTouch(\"seg\")' onchange='ledCommit()'/>"
       "<div class='small'>Value: <b id='led_seg_v'>-</b></div></div>";

  // Pattern select
  s += "<div><div class='small muted'>Pattern</div>"
       "<select id='led_pat' class='btn sm' style='width:100%;text-align:left' onchange='ledCommit()'>"
       "<option value='0'>Tail4 (90/70/60/50)</option>"
       "<option value='1'>Solid</option>"
       "<option value='2'>Dot</option>"
       "</select></div>";

  // Color
  s += "<div><div class='small muted'>Color</div>"
       "<input id='led_color' type='color' value='#ffffff' onchange='ledCommit()' style='width:100%;height:38px;border-radius:10px;border:1px solid rgba(255,255,255,.12);background:#111'/>"
       "</div>";

  // Brightness
  s += "<div><div class='small muted'>Brightness (0..255)</div>"
       "<input id='led_bri' class='slider' type='range' min='0' max='255' step='1' oninput='ledTouch(\"bri\")' onchange='ledCommit()'/>"
       "<div class='small'>Value: <b id='led_bri_v'>-</b></div></div>";

  // Sweep / Pause / Back (1..10000)
  s += "<div><div class='small muted'>Sweep ms</div>"
       "<input id='led_sweep' class='slider' type='range' min='1' max='10000' step='1' oninput='ledTouch(\"sweep\")' onchange='ledCommit()'/>"
       "<div class='small'>Value: <b id='led_sweep_v'>-</b></div></div>";

  s += "<div><div class='small muted'>Pause ms</div>"
       "<input id='led_pause' class='slider' type='range' min='1' max='10000' step='1' oninput='ledTouch(\"pause\")' onchange='ledCommit()'/>"
       "<div class='small'>Value: <b id='led_pause_v'>-</b></div></div>";

  s += "<div><div class='small muted'>Sweep-back ms</div>"
       "<input id='led_back' class='slider' type='range' min='1' max='10000' step='1' oninput='ledTouch(\"back\")' onchange='ledCommit()'/>"
       "<div class='small'>Value: <b id='led_back_v'>-</b></div></div>";

  s += "</div>"; // inner grid
  s += "<div class='small muted' style='margin-top:10px'>Sweep runs automatically on debounced Sensor A SEEN (rising). All speeds are manual.</div>";
  s += "</div>";

  // Cave full-spec WS2815 strip
  s += "<div class='card box' draggable='true' data-box='cavews'><div class='h3 handle'>Cave Full-Spec WS2815</div>";
  s += "<div class='small muted'>Data GPIO: <b>47</b> &nbsp; Pixels: <b>40</b> &nbsp; Trigger: same approach event as tunnel sweep</div>";
  s += "<table style='margin-top:10px'>";
  s += htmlRow("State", "<b id='cave_state'>-</b>");
  s += htmlRow("Dwell hold", "<b id='cave_hold'>-</b>");
  s += "</table>";
  s += "<div class='row' style='margin-top:10px;gap:10px;align-items:center'>";
  s += "<label class='small muted' style='display:flex;gap:8px;align-items:center'><input id='cave_en' type='checkbox' onchange='caveCommit()'/> Enable cave dwell reveal</label>";
  s += "<button class='btn sm' onclick='cmdCaveRun()'>Run</button>";
  s += "<button class='btn sm' onclick='cmdCaveOff()'>Off</button>";
  s += "</div>";
  s += "<div class='grid' style='margin-top:12px;grid-template-columns:1fr 1fr;gap:12px'>";
  s += "<div><div class='small muted'>Color</div><input id='cave_color' type='color' value='#ffffff' onchange='caveCommit()' style='width:100%;height:38px;border-radius:10px;border:1px solid rgba(255,255,255,.12);background:#111'/></div>";
  s += "<div><div class='small muted'>Brightness % (0..100)</div><input id='cave_bri' class='slider' type='range' min='0' max='100' step='1' oninput='caveTouch('bri')' onchange='caveCommit()'/><div class='small'>Value: <b id='cave_bri_v'>-</b></div></div>";
  s += "<div><div class='small muted'>Trigger delay ms (0..15000)</div><input id='cave_delay' class='slider' type='range' min='0' max='15000' step='1' oninput='caveTouch('delay')' onchange='caveCommit()'/><div class='small'>Value: <b id='cave_delay_v'>-</b></div></div>";
  s += "<div><div class='small muted'>Fade ms (0..15000)</div><input id='cave_fade' class='slider' type='range' min='0' max='15000' step='1' oninput='caveTouch('fade')' onchange='caveCommit()'/><div class='small'>Value: <b id='cave_fade_v'>-</b></div></div>";
  s += "</div>";
  s += "<div class='small muted' style='margin-top:10px'>Default: white, 75% brightness, 500ms trigger delay, 1500ms fade, stays on during cave dwell.</div>";
  s += "</div>";

  s += "<div class='row' style='margin-top:10px'>";

  s += "</div>"; // grid

  // SEQ Stats
  s += "<div class='card box' draggable='true' data-box='seqstats'><div class='h3 handle'>SEQ Stats</div>";
  s += "<div class='small muted'>Count = times a SEQ became active. Last(ms) = last duration for that SEQ. Current(ms) updates for active SEQ.</div>";
  s += "<table id='seqTable' style='margin-top:10px'>";
  s += "<tr><td class='k'>SEQ</td><td class='v'><b>Count</b></td><td class='v'><b>Last (ms)</b></td><td class='v'><b>Current (ms)</b></td></tr>";
  s += "<tr><td class='k'>SEQ1</td><td class='v' id='c0'>-</td><td class='v' id='l0'>-</td><td class='v' id='x0'>-</td></tr>";
  s += "<tr><td class='k'>SEQ2</td><td class='v' id='c1'>-</td><td class='v' id='l1'>-</td><td class='v' id='x1'>-</td></tr>";
  s += "<tr><td class='k'>SEQ3</td><td class='v' id='c2'>-</td><td class='v' id='l2'>-</td><td class='v' id='x2'>-</td></tr>";
  s += "<tr><td class='k'>SEQ4</td><td class='v' id='c3'>-</td><td class='v' id='l3'>-</td><td class='v' id='x3'>-</td></tr>";
  s += "<tr><td class='k'>SEQ5</td><td class='v' id='c4'>-</td><td class='v' id='l4'>-</td><td class='v' id='x4'>-</td></tr>";
  s += "<tr><td class='k'>SEQ6</td><td class='v' id='c5'>-</td><td class='v' id='l5'>-</td><td class='v' id='x5'>-</td></tr>";
  s += "</table>";
  s += "</div>";

  // Tunables
  s += "<div class='card box' draggable='true' data-box='tunables'><div class='h3 handle'>Tunables</div>";
  s += "<div class='small muted'>Timing sliders require STOP. WLED pulse durations can change anytime.</div>";
  s += "<div class='row' style='margin-top:10px;gap:10px;align-items:center'>";
  s += "<label class='small muted' style='display:flex;gap:8px;align-items:center'>"
       "<input id='gap_en' type='checkbox' onchange='setGapEn()'/> Enable NoSignal failsafe"       "</label>";
  s += "<span id='gapPill' class='pill warn'>-</span>";
  s += "</div>";
  s += "<div class='row' style='margin-top:10px;gap:10px;align-items:center'>";
  s += "<label class='small muted' style='display:flex;gap:8px;align-items:center'>"
       "<input id='stuck_en' type='checkbox' onchange='setStuckEn()'/> Enable Stuck-sensor failsafe"       "</label>";
  s += "<span id='stuckPill' class='pill warn'>-</span>";
  s += "</div>";
  s += "<div class='small muted'>NoSignal = FAULT_SENSOR_GAP (timeout since last sensor SEEN/CLEAR edge).</div>";
  s += "<div class='small muted'>Stuck = FAULT_SENSOR_STUCK_A/B (sensor held active too long).</div>";
  s += "<div id='sliders'></div>";
  s += "<div class='small muted' style='margin-top:10px'>Endpoints: <a href='/diag'>/diag</a>, <a href='/get'>/get</a>, <a href='/meta'>/meta</a>, <a href='/log'>/log</a>, <a href='/wifi_get'>/wifi_get</a></div>";
  s += "</div>";

  // Close the dashboard sortable grid AND the dashboard section.
  // (Bugfix) Previously we only closed one div, which caused the WiFi tab to be nested
  // inside the dashboard section. Then switching tabs would hide the WiFi UI.
  s += "</div>"; // dashSort
  s += "</div>"; // secDash

  // WIFI SECTION
  s += "<div id='secWifi' class='sec'>";
  s += "<div class='card'><div class='h3 handle'>WiFi Router Connection</div>";
  s += "<div class='small muted'>AP remains active so you cannot lock yourself out. Saving applies immediately.</div>";
  s += "<div class='grid' style='margin-top:10px'>";

  // Status
  s += "<div class='card' style='margin:0;background:var(--card2)'><div class='h3 handle'>Status</div>";
  s += "<table>";
  s += htmlRow("STA status", "<span id='wf_status'>-</span>");
  s += htmlRow("STA SSID", "<span id='wf_ssid'>-</span>");
  s += htmlRow("STA IP", "<span id='wf_ip'>-</span>");
  s += htmlRow("RSSI", "<span id='wf_rssi'>-</span>");
  s += htmlRow("AP IP", "<span id='wf_apip'>-</span>");
  s += "</table>";
  s += "</div>";

  // Settings form
  s += "<div class='card' style='margin:0;background:var(--card2)'><div class='h3 handle'>Settings</div>";
  s += "<div class='row' style='gap:8px;align-items:stretch'>";
  s += "<button class='btn' onclick='wifiScan()'>Scan</button>";
  s += "<select id='wf_scan' style='min-width:220px' onchange='pickScan()'><option value=''>Select SSID...</option></select>";
  s += "</div>";
  s += "<div class='row' style='margin-top:10px;gap:10px'>";
  s += "<div style='flex:1'><div class='small muted'>SSID</div><input id='wf_in_ssid' style='width:100%' placeholder='Your WiFi SSID'/></div>";
  s += "<div style='flex:1'><div class='small muted'>Password</div><input id='wf_in_pass' type='password' style='width:100%' placeholder='WiFi password'/></div>";
  s += "</div>";
  // AP credentials (editable) + QR code
  s += "<div class='row' style='margin-top:10px;gap:10px'>";
  s += "<div style='flex:1'><div class='small muted'>TrainCtrl AP SSID</div><input id='wf_ap_ssid' style='width:100%' placeholder='TrainCtrl'/></div>";
  s += "<div style='flex:1'><div class='small muted'>TrainCtrl AP Password</div><input id='wf_ap_pass' type='password' style='width:100%' placeholder='AP password'/></div>";
  s += "</div>";
  s += "<div class='card' style='margin-top:12px;margin-bottom:0;background:var(--card2)'><div class='h3 handle'>QR Code (TrainCtrl AP)</div>";
  s += "<div class='small muted'>Scan to join the ESP32 AP. Updates when SSID/PASS are saved.</div>";
  s += "<img id='wf_qr_img' width='220' height='220' style='margin-top:10px;background:#fff;border-radius:12px;border:1px solid var(--br)'/>";
  s += "<div class='small muted' style='margin-top:8px'>Payload: <code id='wf_qr_payload'>-</code></div>";
  s += "</div>";
  s += "<div class='row' style='margin-top:10px;gap:10px;align-items:center'>";
  s += "<label class='small muted' style='display:flex;gap:8px;align-items:center'><input id='wf_static' type='checkbox'/> Use static IP</label>";
  s += "</div>";
  s += "<div class='row' style='margin-top:8px;gap:10px'>";
  s += "<div style='flex:1'><div class='small muted'>IP</div><input id='wf_ip_in' style='width:100%' placeholder='192.168.x.x'/></div>";
  s += "<div style='flex:1'><div class='small muted'>Gateway</div><input id='wf_gw_in' style='width:100%' placeholder='192.168.x.1'/></div>";
  s += "</div>";
  s += "<div class='row' style='margin-top:8px;gap:10px'>";
  s += "<div style='flex:1'><div class='small muted'>Mask</div><input id='wf_mask_in' style='width:100%' placeholder='255.255.255.0'/></div>";
  s += "<div style='flex:1'><div class='small muted'>DNS (optional)</div><input id='wf_dns_in' style='width:100%' placeholder='192.168.x.1'/></div>";
  s += "</div>";
  s += "<div class='row' style='margin-top:12px'>";
  s += "<button class='btn' onclick='wifiSave()'>Save & Connect</button>";
  s += "<button class='btn' onclick='wifiClear()'>Clear STA creds</button>";
  s += "</div>";
  s += "<div class='small muted' id='wf_toast' style='margin-top:10px'></div>";
  s += "</div>";

  s += "</div>"; // grid
  s += "</div>"; // card
  s += "</div>"; // secWifi

  // JS
  s += "<script>";
  // Polling updates can fight the user while dragging sliders.
  // Track per-slider edit state so the poller won't snap the slider back mid-adjust.
  s += "const POLL_MS=500;let LAST_MODE=''; let META=null; let SL_EDIT={};";

  s += "function toast(msg,bad=false,id='toast'){"
       "const el=document.getElementById(id);"
       "if(!el) return;"
       "el.textContent=msg;"
       "el.className='small '+(bad?'':'muted');"
       "}";
  s += "window.addEventListener('error',e=>{toast('JS error: '+(e.message||e.type),true,'conn');});";

  s += "function setTab(t){"
       "const d=document.getElementById('secDash');"
       "const w=document.getElementById('secWifi');"
       "d.classList.remove('on'); w.classList.remove('on');"
       "document.getElementById('tabDash').classList.remove('on');"
       "document.getElementById('tabWifi').classList.remove('on');"
       "if(t==='wifi'){w.classList.add('on');document.getElementById('tabWifi').classList.add('on');wifiPollOnce();}"
       "else{d.classList.add('on');document.getElementById('tabDash').classList.add('on');}"
       "}";

  // Drag/snap two-column layout (client-side). Order persisted in localStorage.
  s += "function initSortable(){"
       "const grid=document.getElementById('dashSort'); if(!grid) return;"
       "let dragEl=null;"
       "const key='dash_order_v1';"
       "try{const raw=localStorage.getItem(key);"
       "if(raw){const ids=JSON.parse(raw);"
       "ids.forEach(id=>{const el=grid.querySelector('[data-box=\"'+id+'\"]'); if(el) grid.appendChild(el);});}"
       "}catch(e){}"
       "function save(){const ids=[...grid.querySelectorAll('.box[data-box]')].map(el=>el.getAttribute('data-box'));"
       "try{localStorage.setItem(key,JSON.stringify(ids));}catch(e){}"
       "}"
       "grid.querySelectorAll('.box').forEach(el=>{"
       "el.addEventListener('dragstart',ev=>{dragEl=el;el.classList.add('dragging');ev.dataTransfer.effectAllowed='move';});"
       "el.addEventListener('dragend',()=>{if(dragEl){dragEl.classList.remove('dragging');dragEl=null;}grid.querySelectorAll('.box').forEach(b=>b.classList.remove('drop-hint'));save();});"
       "el.addEventListener('dragover',ev=>{ev.preventDefault();ev.dataTransfer.dropEffect='move';});"
       "el.addEventListener('dragenter',()=>{if(!dragEl||el===dragEl) return; el.classList.add('drop-hint');});"
       "el.addEventListener('dragleave',()=>{el.classList.remove('drop-hint');});"
       "el.addEventListener('drop',ev=>{ev.preventDefault(); if(!dragEl||el===dragEl) return;"
       "el.classList.remove('drop-hint');"
       "const boxes=[...grid.querySelectorAll('.box')];"
       "const dragIdx=boxes.indexOf(dragEl); const dropIdx=boxes.indexOf(el);"
       "if(dragIdx<0||dropIdx<0) return;"
       "if(dragIdx<dropIdx) grid.insertBefore(dragEl, el.nextSibling); else grid.insertBefore(dragEl, el);"
       "});"
       "});"
       "};";
  s += "window.addEventListener('load',()=>{initSortable();});";

  s += "async function jget(url){const r=await fetch(url,{cache:'no-store'});const t=await r.text();if(!r.ok) throw new Error(t||('HTTP '+r.status));try{return JSON.parse(t);}catch(e){throw new Error('Bad JSON from '+url+': '+t.slice(0,120));}}";
  s += "async function tget(url){const r=await fetch(url,{cache:'no-store'});const t=await r.text();if(!r.ok) throw new Error(t||('HTTP '+r.status));return t;}";

  s += "async function cmdMode(m){try{let url='/cmd?mode_set='+encodeURIComponent(m);await tget(url);toast('Cmd: '+m);}catch(e){toast('Command failed: '+(e&&e.message?e.message:e),true);}}";
  s += "async function cmdSeq(n){try{await tget('/cmd?seq='+n);toast('Apply SEQ'+n);}catch(e){toast('Command failed: '+(e&&e.message?e.message:e),true);}}";
  s += "async function cmdOvr(ch,mode){try{await tget('/cmd?ovr_ch='+ch+'&ovr_mode='+mode);toast('CH'+ch+' '+mode);}catch(e){toast('Command failed: '+(e&&e.message?e.message:e),true);}}";
  s += "async function cmdFaultClear(){try{await tget('/cmd?fault=clear');toast('Fault cleared');}catch(e){toast('Command failed: '+(e&&e.message?e.message:e),true);}}";
  s += "async function cmdSound(s){try{await tget('/cmd?sound='+encodeURIComponent(s));toast('Sound: '+s);}catch(e){toast('Command failed: '+(e&&e.message?e.message:e),true);}}";
  s += "async function setGapEn(){const cb=document.getElementById('gap_en');if(!cb) return;const v=cb.checked?1:0;try{await tget('/cmd?gap_en='+v);toast('NoSignal failsafe '+(v?'ENABLED':'DISABLED'));}catch(e){toast('Command failed: '+(e&&e.message?e.message:e),true);}}";
  s += "async function setStuckEn(){const cb=document.getElementById('stuck_en');if(!cb) return;const v=cb.checked?1:0;try{await tget('/cmd?stuck_en='+v);toast('Stuck-sensor failsafe '+(v?'ENABLED':'DISABLED'));}catch(e){toast('Command failed: '+(e&&e.message?e.message:e),true);}}";
  // Tunables
  s += "function buildSliders(meta,current){"
       "const host=document.getElementById('sliders');host.innerHTML='';"
       "Object.keys(meta).forEach(k=>{"
       "const pm=meta[k];"
       "const cur=(current&&current[pm.bind])!=null?current[pm.bind]:null;"
       "const wrap=document.createElement('div');wrap.style.margin='12px 0';"
       "wrap.innerHTML=`"
         "<div class=\"kv\">"
           "<div><b>${pm.label}</b> <span class=\"small muted\">(${pm.units})</span></div>"
           "<div class=\"small\"><span class=\"muted\">Applied:</span> <b id=\"ap_${k}\">-</b> "
           "<span class=\"muted\">Requested:</span> <b id=\"rq_${k}\">-</b></div>"
         "</div>"
         "<input class=\"slider\" id=\"sl_${k}\" type=\"range\"/>"
         "<div class=\"small muted\" id=\"meta_${k}\"></div>"
       "`;"
       "host.appendChild(wrap);"
       "const sl=document.getElementById('sl_'+k);"
       "sl.min=pm.ui_min;sl.max=pm.ui_max;sl.step=pm.step;sl.value=cur!=null?cur:pm.ui_min;"
       "document.getElementById('meta_'+k).textContent="
         "'UI '+pm.ui_min+'..'+pm.ui_max+' step '+pm.step+' | HARD '+pm.hard_min+'..'+pm.hard_max + (pm.require_stop?' | Requires STOP':'');"
       "document.getElementById('rq_'+k).textContent=sl.value;"
       // Keep requested display live while dragging
       "sl.oninput=()=>{SL_EDIT[k]=true;document.getElementById('rq_'+k).textContent=sl.value;};"
       // Mark editing explicitly so the poller never overwrites slider.value while interacting
       "sl.addEventListener('pointerdown',()=>{SL_EDIT[k]=true;});"
       "sl.addEventListener('pointerup',()=>{SL_EDIT[k]=false;});"
       "sl.addEventListener('pointercancel',()=>{SL_EDIT[k]=false;});"
       "sl.addEventListener('touchstart',()=>{SL_EDIT[k]=true;},{passive:true});"
       "sl.addEventListener('touchend',()=>{SL_EDIT[k]=false;},{passive:true});"
       "sl.addEventListener('keydown',()=>{SL_EDIT[k]=true;});"
       "sl.addEventListener('keyup',()=>{SL_EDIT[k]=false;});"
       // Apply only when the user commits the change
       "sl.onchange=()=>{SL_EDIT[k]=false;applyParam(k,sl.value);};"
       "});"
       "}";

  s += "function setLockPill(locked){"
       "const p=document.getElementById('lockPill');"
       "if(locked){p.textContent='LOCKED (STOP required)';p.className='pill warn';}"
       "else{p.textContent='UNLOCKED';p.className='pill ok';}"
       "}";

  s += "function applyLocks(meta,locked){"
       "Object.keys(meta).forEach(k=>{"
       "const pm=meta[k];const sl=document.getElementById('sl_'+k);if(!sl)return;"
       "sl.disabled = (locked && pm.require_stop);"
       "});"
       "}";

  s += "async function applyParam(k,val){"
       "if(!META)return;const pm=META[k];"
       "try{"
       "const res=await jget('/set?'+encodeURIComponent(pm.key)+'='+encodeURIComponent(val));"
       "if(res.locked && pm.require_stop){toast('Refused: STOP required for '+pm.label,true);return;}"
       "if(res[k] && res[k].refused){toast('Refused: STOP required for '+pm.label,true);}"
       "else if(res[k] && res[k].clamped){toast('Clamped '+pm.label+' to '+res[k].applied,true);}"
       "else{toast('Applied '+pm.label+' = '+val);}"
       "}catch(e){toast('Set failed',true);}"
       "}";

  // Mode buttons (RUN/RETURN/STOP) interlock helpers
  s += "function setModeBtn(id,on){ const b=document.getElementById(id); if(!b) return; b.className='btn'+(on?' on':''); }";
  s += "function updModeButtons(m){ setModeBtn('btnRun', m==='RUN'); setModeBtn('btnReturn', m==='RETURN'); setModeBtn('btnStop', m==='STOP'); }";

  s += "function setBtnOn(id,on){ const b=document.getElementById(id); if(!b) return; b.className='btn sm'+(on?' on':''); }";
  s += "function updAux(di){ if(!di.aux) return; [5,6,7,8].forEach(ch=>{ const a=di.aux[String(ch)]; if(!a) return; const mode=(a.mode||'AUTO'); const mEl=document.getElementById('ovm'+ch); const sEl=document.getElementById('ovs'+ch); const pEl=document.getElementById('ovp'+ch); if(mEl) mEl.textContent=mode; if(sEl) sEl.textContent=(a.closed?'ON':'OFF'); if(pEl) pEl.textContent=(a.pulse_rem_ms&&a.pulse_rem_ms>0)?(' ('+a.pulse_rem_ms+'ms)'):''; setBtnOn('ov'+ch+'_auto', mode==='AUTO'); setBtnOn('ov'+ch+'_on', mode==='ON'); setBtnOn('ov'+ch+'_off', mode==='OFF'); setBtnOn('ov'+ch+'_sp', mode==='SHORT'); setBtnOn('ov'+ch+'_lp', mode==='LONG'); }); }";
  // WS2812 LED box JS
  s += "const LED_EDIT={}; let LED_COMMIT_TO=null;";
  s += "function hx2(v){return ('0'+v.toString(16)).slice(-2);}";
  s += "function rgb2hex(r,g,b){return '#'+hx2(r)+hx2(g)+hx2(b);}";
  s += "function ledTouch(k){LED_EDIT[k]=true; if(k===\"seg\")document.getElementById(\"led_seg_v\").textContent=document.getElementById(\"led_seg\").value; if(k===\"bri\")document.getElementById(\"led_bri_v\").textContent=document.getElementById(\"led_bri\").value; if(k===\"sweep\")document.getElementById(\"led_sweep_v\").textContent=document.getElementById(\"led_sweep\").value; if(k===\"pause\")document.getElementById(\"led_pause_v\").textContent=document.getElementById(\"led_pause\").value; if(k===\"back\")document.getElementById(\"led_back_v\").textContent=document.getElementById(\"led_back\").value;";
  s += "}";
  s += "async function cmdLedRun(){try{await tget(\"/cmd?led_run=1\");toast(\"LED: run\");}catch(e){toast(\"Command failed\",true);}}";
  s += "async function cmdLedOff(){try{await tget(\"/cmd?led_off=1\");toast(\"LED: off\");}catch(e){toast(\"Command failed\",true);}}";
  s += "function ledCommit(){ if(LED_COMMIT_TO) clearTimeout(LED_COMMIT_TO); LED_COMMIT_TO=setTimeout(async()=>{const en=document.getElementById(\"led_en\").checked?1:0;const seg=document.getElementById(\"led_seg\").value;const pat=document.getElementById(\"led_pat\").value;const bri=document.getElementById(\"led_bri\").value;const sw=document.getElementById(\"led_sweep\").value;const pa=document.getElementById(\"led_pause\").value;const ba=document.getElementById(\"led_back\").value;const col=document.getElementById(\"led_color\").value||\"#ffffff\";const r=parseInt(col.slice(1,3),16)||0; const g=parseInt(col.slice(3,5),16)||0; const b=parseInt(col.slice(5,7),16)||0;try{await jget(`/led_set?en=${en}&seg=${seg}&pat=${pat}&r=${r}&g=${g}&b=${b}&bri=${bri}&sweep=${sw}&pause=${pa}&back=${ba}`);toast(\"LED settings saved\");}catch(e){toast(\"LED set failed\",true);} LED_EDIT[\"seg\"]=LED_EDIT[\"bri\"]=LED_EDIT[\"sweep\"]=LED_EDIT[\"pause\"]=LED_EDIT[\"back\"]=false;},150); }";
s += "function updLed(di){ if(!di.led) return; const L=di.led;const pwr=document.getElementById(\"led_pwr\"); if(pwr) pwr.textContent=(L.power?\"ON\":\"OFF\");const st=document.getElementById(\"led_state\"); if(st) st.textContent=(L.state||\"-\");const ps=document.getElementById(\"led_pos\"); if(ps) ps.textContent=(L.pos!=null?L.pos:\"-\");const en=document.getElementById(\"led_en\"); if(en && document.activeElement!==en) en.checked=!!L.enabled;const seg=document.getElementById(\"led_seg\"); if(seg && !(LED_EDIT[\"seg\"]||document.activeElement===seg||seg.matches(\":active\"))){ seg.value=L.seg; document.getElementById(\"led_seg_v\").textContent=L.seg; }const pat=document.getElementById(\"led_pat\"); if(pat && document.activeElement!==pat) pat.value=L.pattern;const bri=document.getElementById(\"led_bri\"); if(bri && !(LED_EDIT[\"bri\"]||document.activeElement===bri||bri.matches(\":active\"))){ bri.value=L.bri; document.getElementById(\"led_bri_v\").textContent=L.bri; }const sw=document.getElementById(\"led_sweep\"); if(sw && !(LED_EDIT[\"sweep\"]||document.activeElement===sw||sw.matches(\":active\"))){ sw.value=L.sweep; document.getElementById(\"led_sweep_v\").textContent=L.sweep; }const pa=document.getElementById(\"led_pause\"); if(pa && !(LED_EDIT[\"pause\"]||document.activeElement===pa||pa.matches(\":active\"))){ pa.value=L.pause; document.getElementById(\"led_pause_v\").textContent=L.pause; }const ba=document.getElementById(\"led_back\"); if(ba && !(LED_EDIT[\"back\"]||document.activeElement===ba||ba.matches(\":active\"))){ ba.value=L.back; document.getElementById(\"led_back_v\").textContent=L.back; }const col=document.getElementById(\"led_color\"); if(col && document.activeElement!==col){ col.value=rgb2hex(L.r||255,L.g||255,L.b||255); }}";
s += "const CAVE_EDIT={}; let CAVE_COMMIT_TO=null;";
s += "function caveTouch(k){CAVE_EDIT[k]=true; if(k==='bri')document.getElementById('cave_bri_v').textContent=document.getElementById('cave_bri').value; if(k==='delay')document.getElementById('cave_delay_v').textContent=document.getElementById('cave_delay').value; if(k==='fade')document.getElementById('cave_fade_v').textContent=document.getElementById('cave_fade').value;}";
s += "async function cmdCaveRun(){try{await tget('/cmd?cave_run=1');toast('Cave: run');}catch(e){toast('Command failed',true);}}";
s += "async function cmdCaveOff(){try{await tget('/cmd?cave_off=1');toast('Cave: off');}catch(e){toast('Command failed',true);}}";
s += "function caveCommit(){ if(CAVE_COMMIT_TO) clearTimeout(CAVE_COMMIT_TO); CAVE_COMMIT_TO=setTimeout(async()=>{const en=document.getElementById('cave_en').checked?1:0;const bri=document.getElementById('cave_bri').value;const delay=document.getElementById('cave_delay').value;const fade=document.getElementById('cave_fade').value;const col=document.getElementById('cave_color').value||'#ffffff';const r=parseInt(col.slice(1,3),16)||0; const g=parseInt(col.slice(3,5),16)||0; const b=parseInt(col.slice(5,7),16)||0;try{await jget('/cave_set?en='+en+'&seg=40&r='+r+'&g='+g+'&b='+b+'&bp='+bri+'&delay='+delay+'&fade='+fade);toast('Cave settings saved');}catch(e){toast('Cave set failed',true);} CAVE_EDIT['bri']=CAVE_EDIT['delay']=CAVE_EDIT['fade']=false;},150); }";
s += "function updCave(di){ if(!di.cave) return; const C=di.cave;const st=document.getElementById('cave_state'); if(st) st.textContent=(C.state||'-');const hold=document.getElementById('cave_hold'); if(hold) hold.textContent=(C.dwell_hold?'YES':'NO');const en=document.getElementById('cave_en'); if(en && document.activeElement!==en) en.checked=!!C.enabled;const bri=document.getElementById('cave_bri'); if(bri && !(CAVE_EDIT['bri']||document.activeElement===bri||bri.matches(':active'))){ bri.value=C.bri_pct; document.getElementById('cave_bri_v').textContent=C.bri_pct; }const delay=document.getElementById('cave_delay'); if(delay && !(CAVE_EDIT['delay']||document.activeElement===delay||delay.matches(':active'))){ delay.value=C.delay; document.getElementById('cave_delay_v').textContent=C.delay; }const fade=document.getElementById('cave_fade'); if(fade && !(CAVE_EDIT['fade']||document.activeElement===fade||fade.matches(':active'))){ fade.value=C.fade; document.getElementById('cave_fade_v').textContent=C.fade; }const col=document.getElementById('cave_color'); if(col && document.activeElement!==col){ col.value=rgb2hex(C.r||255,C.g||255,C.b||255); }}";

// SEQ stats UI update
  s += "function updSeqStats(di){"
       "const counts=di.seq_counts||[];"
       "const last=di.seq_last_ms||[];"
       "const elapsed=di.seq_elapsed_ms||0;"
       "const activeIdx=di.active_seq_idx||0;"
       "for(let i=0;i<6;i++){"
       "const c=document.getElementById('c'+i);"
       "const l=document.getElementById('l'+i);"
       "const x=document.getElementById('x'+i);"
       "if(c) c.textContent=(counts[i]!=null?counts[i]:'-');"
       "if(l) l.textContent=(last[i]!=null?last[i]:'-');"
       "if(x) x.textContent=(i===activeIdx?elapsed:'-');"
       "}"
       "}";

  s += "function updState(di){"
       "document.getElementById('st_mode').textContent=di.mode||'-';const m=(di.mode||'-');const d=(di.dir||'-');const sq=(di.active_seq||'-');const ex=(di.expected_sensor||'-');const lf=(di.last_fault||'-');document.getElementById('st_mode_top').textContent=m;document.getElementById('st_dir_top').textContent=d;document.getElementById('st_seq_top').textContent=sq;document.getElementById('st_fault_top').textContent=lf;const scm=document.getElementById('sc_mode'); if(scm) scm.textContent=m;const scd=document.getElementById('sc_dir'); if(scd) scd.textContent=d;const scs=document.getElementById('sc_seq'); if(scs) scs.textContent=sq;const sce=document.getElementById('sc_exp'); if(sce) sce.textContent=ex;const scf=document.getElementById('sc_fault'); if(scf) scf.textContent=lf;LAST_MODE=m;updModeButtons(LAST_MODE);"
       "document.getElementById('st_seq').textContent=di.active_seq||'-';"
       "document.getElementById('st_seq_elapsed').textContent=di.seq_elapsed_ms||0;"
       "document.getElementById('st_dir').textContent=di.dir||'-';"
       "document.getElementById('st_exp').textContent=di.expected_sensor||'-';"
       "document.getElementById('st_handoff').textContent=(di.handoff_active?'YES':'NO');"
       "document.getElementById('st_apply').textContent=(di.apply_active?'YES':'NO');"
       "document.getElementById('st_phase').textContent=di.apply_phase||'-';document.getElementById('st_pwrlo').textContent=(di.pwr_lo===true?'ON':'OFF');document.getElementById('st_pwrhi').textContent=(di.pwr_hi===true?'ON':'OFF');"
       "document.getElementById('st_evt').textContent=(di.last_event||'-')+' @ '+(di.last_event_at||0)+'ms';"
       "document.getElementById('st_fault').textContent=(di.last_fault||'-')+' @ '+(di.last_fault_at||0)+'ms';"
       "updAux(di);updLed(di);updCave(di);updSeqStats(di);"
       "}";

  s += "function updTunables(get){"
       "if(!META)return;"
       "const cb=document.getElementById('gap_en');if(cb && get.SENSOR_GAP_ENABLE!=null) cb.checked=!!get.SENSOR_GAP_ENABLE;"
       "const cb2=document.getElementById('stuck_en');if(cb2 && get.SENSOR_STUCK_ENABLE!=null) cb2.checked=!!get.SENSOR_STUCK_ENABLE;"
       "const pill=document.getElementById('gapPill');if(pill && get.SENSOR_GAP_ENABLE!=null){if(get.SENSOR_GAP_ENABLE){pill.textContent='NoSignal: ON';pill.className='pill ok';}else{pill.textContent='NoSignal: OFF';pill.className='pill warn';}}"
       "const pill2=document.getElementById('stuckPill');if(pill2 && get.SENSOR_STUCK_ENABLE!=null){if(get.SENSOR_STUCK_ENABLE){pill2.textContent='Stuck: ON';pill2.className='pill ok';}else{pill2.textContent='Stuck: OFF';pill2.className='pill warn';}}"
       "Object.keys(META).forEach(k=>{"
       "const pm=META[k];"
       "const ap=document.getElementById('ap_'+k);"
       "const sl=document.getElementById('sl_'+k);"
       "if(!ap||!sl)return;"
       "const v=get[pm.bind];if(v==null)return;"
       "ap.textContent=v;"
       // Do NOT overwrite while user is interacting with this slider (mouse/touch/keyboard/focus).
       "if(!(SL_EDIT[k]||document.activeElement===sl||sl.matches(':active'))){sl.value=v;document.getElementById('rq_'+k).textContent=sl.value;}"
       "});"
       "}";

  // Main poll
  s += "async function pollOnce(){"
       "try{"
       "const [di,ge]=await Promise.all([jget('/diag'),jget('/get')]);"
       "updState(di);updTunables(ge);"
       "const locked=(di.mode!=='STOP')||(di.apply_active===true);"
       "setLockPill(locked);applyLocks(META,locked);"
       "toast('Last update: '+(di.ms!=null?di.ms:'?')+' ms',false,'conn');"
       "}catch(e){toast('Poll error: '+(e&&e.message?e.message:e),true,'conn');}"
       "}";

  // WiFi tab functions
  s += "function wfToast(msg,bad=false){toast(msg,bad,'wf_toast');}"
       "async function wifiPollOnce(){"
       "try{"
       "const st=await jget('/wifi_get');"
       "document.getElementById('wf_status').textContent=st.sta_status||'-';"
       "document.getElementById('wf_ssid').textContent=st.sta_ssid||st.saved_ssid||'-';"
       "document.getElementById('wf_ip').textContent=st.sta_ip||'-';"
       "document.getElementById('wf_rssi').textContent=(st.sta_rssi!=null?st.sta_rssi:'-');"
       "document.getElementById('wf_apip').textContent=st.ap_ip||'-';"
       "document.getElementById('wf_in_ssid').value=st.saved_ssid||'';document.getElementById('wf_ap_ssid').value=st.ap_ssid||'';document.getElementById('wf_ap_pass').value=st.ap_pass||'';renderApQr(st.ap_ssid||'', st.ap_pass||'');"
       "document.getElementById('wf_static').checked=!!st.sta_use_static;"
       "document.getElementById('wf_ip_in').value=st.sta_static_ip||'';"
       "document.getElementById('wf_gw_in').value=st.sta_static_gw||'';"
       "document.getElementById('wf_mask_in').value=st.sta_static_mask||'';"
       "document.getElementById('wf_dns_in').value=st.sta_static_dns||'';"
       "}catch(e){}"
       "}";

  // (Bugfix) WiFi page actions were inert due to a JS syntax error here.
  // Also send AP SSID/PASS so the QR + AP creds actually update.
  s += "async function wifiSave(){"
       "const ssid=document.getElementById('wf_in_ssid').value||'';"
       "const pass=document.getElementById('wf_in_pass').value||'';"
       "const ap_ssid=document.getElementById('wf_ap_ssid').value||'';"
       "const ap_pass=document.getElementById('wf_ap_pass').value||'';"
       "const st=document.getElementById('wf_static').checked?1:0;"
       "const ip=document.getElementById('wf_ip_in').value||'';"
       "const gw=document.getElementById('wf_gw_in').value||'';"
       "const mask=document.getElementById('wf_mask_in').value||'';"
       "const dns=document.getElementById('wf_dns_in').value||'';"
       "try{"
       "const url='/wifi_set?ssid='+encodeURIComponent(ssid)"
              "+'&pass='+encodeURIComponent(pass)"
              "+'&static='+st"
              "+'&ip='+encodeURIComponent(ip)"
              "+'&gw='+encodeURIComponent(gw)"
              "+'&mask='+encodeURIComponent(mask)"
              "+'&dns='+encodeURIComponent(dns)"
              "+'&ap_ssid='+encodeURIComponent(ap_ssid)"
              "+'&ap_pass='+encodeURIComponent(ap_pass);"
       "const res=await jget(url);"
       "wfToast(res.ok?'Saved. Connecting...':'Save failed', !res.ok);"
       "document.getElementById('wf_in_pass').value='';"
       "renderApQr(ap_ssid, ap_pass);"
       "setTimeout(wifiPollOnce,700);"
       "}catch(e){wfToast('Save failed',true);}"
       "}";

  s += "async function wifiClear(){"
       "try{"
       "const res=await jget('/wifi_set?ssid=&pass=&static=0&ip=&gw=&mask=&dns=');"
       "wfToast(res.ok?'Cleared. STA disconnected.':'Clear failed', !res.ok);"
       "setTimeout(wifiPollOnce,500);"
       "}catch(e){wfToast('Clear failed',true);}"
       "}";

  s += "async function wifiScan(){"
       "try{"
       "wfToast('Scanning...');"
       "const res=await jget('/wifi_scan');"
       "const sel=document.getElementById('wf_scan');"
       "sel.innerHTML='<option value=\"\">Select SSID...</option>';"
       "(res.networks||[]).forEach(n=>{"
       "const opt=document.createElement('option');"
       "opt.value=n.ssid; opt.textContent=n.ssid+'  ('+n.rssi+' dBm)';"
       "sel.appendChild(opt);"
       "});"
       "wfToast('Scan complete');"
       "}catch(e){wfToast('Scan failed',true);}"
       "}";

  s += "function pickScan(){"
       "const sel=document.getElementById('wf_scan');"
       "if(!sel.value) return;"
       "document.getElementById('wf_in_ssid').value=sel.value;"
       "}";

  // Init
  s += "function escWifi(v){v=(v||'');const bs=String.fromCharCode(92);v=v.split(bs).join(bs+bs);v=v.split(';').join(bs+';');v=v.split(',').join(bs+',');v=v.split(':').join(bs+':');return v;}function wifiPayload(ssid,pass){const S=escWifi(ssid);const P=escWifi(pass);return 'WIFI:T:WPA;S:'+S+';P:'+P+';;';}function renderApQr(ssid,pass){const payload=wifiPayload(ssid,pass);const el=document.getElementById('wf_qr_payload'); if(el) el.textContent=payload;const img=document.getElementById('wf_qr_img'); if(img){img.alt=payload;img.src='https://api.qrserver.com/v1/create-qr-code/?size=220x220&data='+encodeURIComponent(payload);} }async function init(){"
       "let ge=null;"
       "try{"
       "try{META=await jget('/meta');}catch(e){META=null;toast('Meta failed (sliders disabled): '+(e&&e.message?e.message:e),true);}"
       "ge=await jget('/get');"
       "if(META) buildSliders(META,ge);"
       "await pollOnce();"
       "setInterval(pollOnce,POLL_MS);"
       "}catch(e){toast('UI init failed: '+(e&&e.message?e.message:e),true);}"
       "}"
       "init();";

  s += "</script>";

  s += "</body></html>";
  return s;
}

static void handleRoot() { server.send(200, "text/html", htmlPage()); }

// ============================================================
// /get
// ============================================================
static void handleGet() {
  String s="{";
  s += "\"SAFE_SWITCH_MS\":" + String(SAFE_SWITCH_MS) + ",";
  s += "\"TURN_DWELL_MS\":" + String(TURN_DWELL_MS) + ",";
  s += "\"COOLDOWN_MS\":" + String(COOLDOWN_MS) + ",";
  s += "\"DI7_HOLD_MIN_MS\":" + String(DI7_HOLD_MIN_MS) + ",";
  s += "\"SENSOR_SEEN_STABLE_MS\":" + String(SENSOR_SEEN_STABLE_MS) + ",";
  s += "\"SENSOR_CLEAR_STABLE_MS\":" + String(SENSOR_CLEAR_STABLE_MS) + ",";
  s += "\"MIN_BETWEEN_SENSORS_MS\":" + String(MIN_BETWEEN_SENSORS_MS) + ",";
  s += "\"SENSOR_GAP_TIMEOUT_MS\":" + String(SENSOR_GAP_TIMEOUT_MS) + ",";
  s += "\"SENSOR_GAP_ENABLE\":"; s += (SENSOR_GAP_ENABLE ? "true":"false"); s += ",";
  s += "\"SENSOR_STUCK_TIMEOUT_MS\":" + String(SENSOR_STUCK_TIMEOUT_MS) + ",";
  s += "\"SENSOR_STUCK_ENABLE\":"; s += (SENSOR_STUCK_ENABLE ? "true":"false"); s += ",";

  s += "\"OVR_PULSE_SHORT_MS\":" + String(OVR_PULSE_SHORT_MS) + ",";
  s += "\"OVR_PULSE_LONG_MS\":" + String(OVR_PULSE_LONG_MS) + ",";
  s += "\"BEEP_LEN1_MS\":" + String(BEEP_LEN1_MS) + ",";
  s += "\"BEEP_LEN2_MS\":" + String(BEEP_LEN2_MS) + ",";
  s += "\"BEEP_BASE_HZ\":" + String(BEEP_BASE_HZ) + ",";
  s += "\"BEEP_OCTAVE_GAP\":" + String(BEEP_OCTAVE_GAP) + ",";
  s += "\"ACCEL_EXTRA_MS\":" + String(ACCEL_EXTRA_MS) + ",";
  s += "\"DECEL_EXTRA_MS\":" + String(DECEL_EXTRA_MS) + ",";
  s += "\"UV_OFF_DELAY_MS\":" + String(uvCtrl.offDelayMs) + ",";
  s += "\"UV_ON_DELAY_MS\":" + String(uvCtrl.onDelayMs) + ",";
  s += "\"BEACON_BRIGHTNESS\":" + String((int)BEACON_BRIGHTNESS) + ",";
  s += "\"BEACON_FAULT_ONLY\":"; s += (BEACON_FAULT_ONLY ? "true":"false"); s += ",";
s += "\"PIXEL_BRIGHTNESS\":" + String(PIXEL_BRIGHTNESS);
  s += "}";
  server.send(200, "application/json", s);
}

// ============================================================
// /meta
// ============================================================
static void handleMeta() {
  String s="{";

  auto add = [&](const char* id,
                 const char* key,
                 const char* label,
                 const char* bind,
                 const ParamMeta &pm)
  {
    s += "\""; s += id; s += "\":{";
    s += "\"key\":\""; s += key; s += "\",";
    s += "\"label\":\""; s += label; s += "\",";
    s += "\"bind\":\""; s += bind; s += "\",";
    s += "\"units\":\""; s += pm.units; s += "\",";
    s += "\"hard_min\":"; s += pm.hardMin; s += ",";
    s += "\"hard_max\":"; s += pm.hardMax; s += ",";
    s += "\"ui_min\":";   s += pm.uiMin;   s += ",";
    s += "\"ui_max\":";   s += pm.uiMax;   s += ",";
    s += "\"step\":";     s += pm.step;    s += ",";
    s += "\"require_stop\":"; s += (pm.requireStop ? "true":"false");
    s += "}";
  };

  add("safety",     "safety",     "Safe switch",            "SAFE_SWITCH_MS",          PARAMS[P_SAFE]);       s += ",";
  add("dwell",      "dwell",      "Turn dwell",             "TURN_DWELL_MS",           PARAMS[P_DWELL]);      s += ",";
  add("cool",       "cool",       "Cooldown",               "COOLDOWN_MS",             PARAMS[P_COOL]);       s += ",";
  add("seen",       "seen",       "Seen stable",            "SENSOR_SEEN_STABLE_MS",   PARAMS[P_SEEN]);       s += ",";
  add("clear",      "clear",      "Clear stable",           "SENSOR_CLEAR_STABLE_MS",  PARAMS[P_CLEAR]);      s += ",";
  add("minbetween", "minbetween", "Min between sensors",    "MIN_BETWEEN_SENSORS_MS",  PARAMS[P_MINBETWEEN]); s += ",";
  add("gap",        "gap",        "Sensor gap failsafe",    "SENSOR_GAP_TIMEOUT_MS",     PARAMS[P_GAP]);          s += ",";
  add("stuck",      "stuck",      "Sensor stuck failsafe",  "SENSOR_STUCK_TIMEOUT_MS",   PARAMS[P_STUCK]);        s += ",";

  add("accelx",    "accelx",    "Accel extra (LOâHI)",    "ACCEL_EXTRA_MS",        PARAMS[P_ACCELX_MS]);      s += ",";
  add("decelx",    "decelx",    "Decel extra (HIâLO)",    "DECEL_EXTRA_MS",        PARAMS[P_DECELX_MS]);    s += ",";
  add("ovrshort",   "ovrshort",   "Override pulse short",   "OVR_PULSE_SHORT_MS",      PARAMS[P_OVRSHORT]);   s += ",";
  add("ovrlong",    "ovrlong",    "Override pulse long",    "OVR_PULSE_LONG_MS",       PARAMS[P_OVRLONG]);    s += ",";
  add("beep1",      "beep1",      "Beep length 1",          "BEEP_LEN1_MS",            PARAMS[P_BEEP1]);      s += ",";
  add("beep2",      "beep2",      "Beep length 2",          "BEEP_LEN2_MS",            PARAMS[P_BEEP2]);      s += ",";
  add("beepbase",   "beepbase",   "Beep base (LOW) freq",     "BEEP_BASE_HZ",            PARAMS[P_BEEPBASE]);   s += ",";
  add("beepoct",    "beepoct",    "Beep octave gap",         "BEEP_OCTAVE_GAP",         PARAMS[P_BEEPOCT]);    s += ",";
  add("pbright",    "pbright",    "Pixel brightness",       "PIXEL_BRIGHTNESS",        PARAMS[P_PBRIGHT]);

  s += "}";
  server.send(200, "application/json", s);
}

// ============================================================
// /diag (adds SEQ stats + current seq elapsed)
// ============================================================
static void handleDiag() {
  uint32_t now = millis();
  uint8_t expSen = ctrl.expectedSensor;
  uint32_t elapsed = (g_seqStartMs == 0) ? 0 : (now - g_seqStartMs);

  String s="{";
  s += "\"ms\":" + String(now) + ",";
  s += "\"mode\":\"" + String(modeStr(ctrl.mode)) + "\",";
  s += "\"active_seq\":\"" + String(seqStr(ctrl.activeSeq)) + "\",";
  s += "\"active_seq_idx\":" + String((uint8_t)ctrl.activeSeq) + ",";
  s += "\"seq_elapsed_ms\":" + String(elapsed) + ",";
  s += "\"dir\":\"" + String(dirStr(ctrl.currentDir())) + "\",";
  s += "\"expected_sensor\":\"" + String((expSen==0) ? "A" : "B") + "\",";
  s += "\"handoff_active\":" + String(ctrl.handoffActive ? "true" : "false") + ",";
  s += "\"apply_active\":" + String(apply.isActive() ? "true" : "false") + ",";
  s += "\"apply_phase\":\"" + String(applyPhaseStr(apply.phase)) + "\",";

  // Track-power gate states (read from TCA9554 output register)
  uint8_t out=0;
  bool pwrLo=false, pwrHi=false;
  if (tcaRead(REG_OUTPUT, out)) {
    pwrLo = exioClosedFromOutByte(EXIO_CH2_PWR_LO_GATE, out);
    pwrHi = exioClosedFromOutByte(EXIO_CH1_PWR_HI_GATE, out);
  }
  s += "\"pwr_lo\":" + String(pwrLo ? "true" : "false") + ",";
  s += "\"pwr_hi\":" + String(pwrHi ? "true" : "false") + ",";
  s += "\"last_event\":\"" + String(evtStr(g_lastEvent)) + "\",";
  s += "\"last_event_at\":" + String(g_lastEventAt) + ",";
  s += "\"last_fault\":\"" + String(faultStr(g_lastFault)) + "\",";
  s += "\"last_fault_at\":" + String(g_lastFaultAt) + ",";

  s += "\"seq_counts\":[";
  for (int i=0;i<SEQ_COUNT;i++){ if(i) s += ","; s += String(g_seqCount[i]); }
  s += "],";

  s += "\"seq_last_ms\":[";
  for (int i=0;i<SEQ_COUNT;i++){ if(i) s += ","; s += String(g_seqLastMs[i]); }
  s += "]";

  // AUX relay overrides (CH5..CH8)
  s += ",\"aux\":{";
  for (uint8_t ch=5; ch<=8; ch++){
    if (ch != 5) s += ",";
    uint32_t rem = 0;
    if (auxIsPulseMode(g_auxOvrMode[ch]) && g_auxPulseOffAt[ch] != 0 && (int32_t)(g_auxPulseOffAt[ch] - now) > 0) rem = (uint32_t)(g_auxPulseOffAt[ch] - now);
    s += "\"" + String(ch) + "\":{";
    s += "\"mode\":\"" + String(auxModeStr(g_auxOvrMode[ch])) + "\",";
    s += "\"auto_closed\":" + String(g_auxAutoClosed[ch] ? "true" : "false") + ",";
    s += "\"closed\":" + String(auxEffectiveClosed(ch, now) ? "true" : "false") + ",";
    s += "\"pulse_rem_ms\":" + String(rem);
    s += "}";
  }
  s += "}";

  // WS2812 LED box status
  bool ledPwr = auxEffectiveClosed(EXIO_CH7_WLED_PULSE, now);
  const char* ledStateStr = "IDLE";
  if (ledRail.st == LED_SWEEP_FWD) ledStateStr = "SWEEP";
  else if (ledRail.st == LED_PAUSE) ledStateStr = "PAUSE";
  else if (ledRail.st == LED_SWEEP_BACK) ledStateStr = "BACK";

  s += ",\"led\":{";
  s += "\"gpio\":" + String((int)PIN_STRIP) + ",";
  s += "\"ch\":" + String((int)EXIO_CH7_WLED_PULSE) + ",";
  s += "\"enabled\":" + String(ledRail.enabled ? "true":"false") + ",";
  s += "\"power\":" + String(ledPwr ? "true":"false") + ",";
  s += "\"state\":\"" + String(ledStateStr) + "\",";
  s += "\"pos\":" + String((int)ledRail.pos) + ",";
  s += "\"seg\":" + String((int)ledRail.segN) + ",";
  s += "\"pattern\":" + String((int)ledRail.pattern) + ",";
  s += "\"r\":" + String((int)ledRail.r) + ",";
  s += "\"g\":" + String((int)ledRail.g) + ",";
  s += "\"b\":" + String((int)ledRail.b) + ",";
  s += "\"bri\":" + String((int)ledRail.bri) + ",";
  s += "\"sweep\":" + String((int)ledRail.sweepMs) + ",";
  s += "\"pause\":" + String((int)ledRail.pauseMs) + ",";
  s += "\"back\":" + String((int)ledRail.backMs);
  s += "},";

  s += "\"cave\":{";
  s += "\"gpio\":" + String((int)PIN_STRIP) + ",";
  s += "\"enabled\":" + String(caveFullSpec.enabled ? "true":"false") + ",";
  s += "\"state\":\"" + String(caveFullSpec.stateStr()) + "\",";
  s += "\"dwell_hold\":" + String(caveFullSpec.dwellHold ? "true":"false") + ",";
  s += "\"seg\":" + String((int)caveFullSpec.segN) + ",";
  s += "\"r\":" + String((int)caveFullSpec.r) + ",";
  s += "\"g\":" + String((int)caveFullSpec.g) + ",";
  s += "\"b\":" + String((int)caveFullSpec.b) + ",";
  s += "\"bri_pct\":" + String((int)caveFullSpec.briPct) + ",";
  s += "\"delay\":" + String((int)caveFullSpec.triggerDelayMs) + ",";
  s += "\"fade\":" + String((int)caveFullSpec.fadeMs) + ",";
  s += "\"s2_phase\":\"" + String(s2Cave.stateStr()) + "\",";
  s += "\"uv_state\":\"" + String(uvCtrl.stateStr()) + "\",";
  s += "\"uv_on\":" + String(uvCtrl.currentOn ? "true":"false") + ",";
  s += "\"uv_off_delay\":" + String((int)uvCtrl.offDelayMs) + ",";
  s += "\"uv_on_delay\":" + String((int)uvCtrl.onDelayMs);
  s += "},";

  s += "\"beacon\":{";
  s += "\"gpio\":" + String((int)PIN_BEACON) + ",";
  s += "\"brightness\":" + String((int)BEACON_BRIGHTNESS) + ",";
  s += "\"fault_only\":" + String(BEACON_FAULT_ONLY ? "true":"false") + ",";
  s += "\"event_ms\":" + String((int)BEACON_EVENT_MS);
  s += "}";

  s += "}";
  server.send(200, "application/json", s);
}

// ============================================================
// /log
// ============================================================
static void handleLog() {
  String s="[";
  uint16_t n = g_logCount;
  uint16_t start = (g_logHead + LOG_CAP - n) % LOG_CAP;
  for (uint16_t i=0; i<n; i++) {
    uint16_t idx = (start + i) % LOG_CAP;
    const LogEntry &e = g_log[idx];
    if (i) s += ",";
    s += "{";
    s += "\"ms\":" + String(e.ms) + ",";
    s += "\"evt\":\"" + String(evtStr(e.evt)) + "\",";
    s += "\"fault\":\"" + String(faultStr(e.fault)) + "\",";
    s += "\"mode\":\"" + String(modeStr(e.mode)) + "\",";
    s += "\"active\":\"" + String(seqStr(e.activeSeq)) + "\",";
    s += "\"pending\":\"" + String(seqStr(e.pendingSeq)) + "\",";
    s += "\"phase\":\"" + String(applyPhaseStr(e.applyPhase)) + "\"";
    s += "}";
  }
  s += "]";
  server.send(200, "application/json", s);
}

// ============================================================
// /set
// ============================================================
static void handleSet() {
  const bool locked = isStopLocked();

  String out="{";
  bool first=true;

  auto emit = [&](const char* id, uint16_t req, uint16_t applied, bool clamped, bool refused){
    if (!first) out += ",";
    first=false;
    out += "\""; out += id; out += "\":{";
    out += "\"requested\":"; out += req; out += ",";
    out += "\"applied\":";   out += applied; out += ",";
    out += "\"clamped\":";   out += (clamped ? "true":"false"); out += ",";
    out += "\"refused\":";   out += (refused ? "true":"false");
    out += "}";
  };

  auto doU16 = [&](const char* id, const ParamMeta &pm, uint16_t &var){
    if (!server.hasArg(pm.key)) return;

    uint16_t req = argU16NonNeg(server.arg(pm.key));
    bool refused = (pm.requireStop && locked);

    if (refused) { emit(id, req, var, false, true); return; }

    uint16_t v = quantizeU16(req, pm.step);
    uint16_t applied = clampU16(v, pm.hardMin, pm.hardMax);
    bool clamped = (applied != req) || (applied != v);

    var = applied;
    emit(id, req, var, clamped, false);
  };

  doU16("safety",     PARAMS[P_SAFE],       SAFE_SWITCH_MS);
  doU16("dwell",      PARAMS[P_DWELL],      TURN_DWELL_MS);
  doU16("cool",       PARAMS[P_COOL],       COOLDOWN_MS);
  doU16("seen",       PARAMS[P_SEEN],       SENSOR_SEEN_STABLE_MS);
  doU16("clear",      PARAMS[P_CLEAR],      SENSOR_CLEAR_STABLE_MS);
  doU16("minbetween", PARAMS[P_MINBETWEEN], MIN_BETWEEN_SENSORS_MS);
  doU16("gap",       PARAMS[P_GAP],        SENSOR_GAP_TIMEOUT_MS);
  doU16("stuck",     PARAMS[P_STUCK],      SENSOR_STUCK_TIMEOUT_MS);

  doU16("accelx",    PARAMS[P_ACCELX_MS],     ACCEL_EXTRA_MS);
  doU16("decelx",    PARAMS[P_DECELX_MS],     DECEL_EXTRA_MS);
  doU16("ovrshort",   PARAMS[P_OVRSHORT],   OVR_PULSE_SHORT_MS);
  doU16("ovrlong",    PARAMS[P_OVRLONG],    OVR_PULSE_LONG_MS);
  doU16("beep1",      PARAMS[P_BEEP1],      BEEP_LEN1_MS);
  doU16("beep2",      PARAMS[P_BEEP2],      BEEP_LEN2_MS);
  doU16("beepbase",   PARAMS[P_BEEPBASE],   BEEP_BASE_HZ);
  doU16("beepoct",    PARAMS[P_BEEPOCT],    BEEP_OCTAVE_GAP);

  if (server.hasArg(PARAMS[P_PBRIGHT].key)) {
    uint16_t req = argU16NonNeg(server.arg(PARAMS[P_PBRIGHT].key));
    uint16_t v = quantizeU16(req, PARAMS[P_PBRIGHT].step);
    uint16_t applied = clampU16(v, PARAMS[P_PBRIGHT].hardMin, PARAMS[P_PBRIGHT].hardMax);
    bool clamped = (applied != req) || (applied != v);
    PIXEL_BRIGHTNESS = (uint8_t)applied;
    pixel.setBrightness(PIXEL_BRIGHTNESS);
    pixel.show();
    emit("pbright", req, PIXEL_BRIGHTNESS, clamped, false);
  }

  if (!first) out += ",";
  out += "\"locked\":"; out += (locked ? "true":"false");
  out += "}";

  server.send(200, "application/json", out);
}

// ============================================================
// /cmd
// ============================================================
static void handleCmd() {
  uint32_t now = millis();

  if (server.hasArg("fault") && server.arg("fault") == "clear") {
    g_lastFault = FAULT_NONE;
    g_lastFaultAt = now;
    logPush(now, EVT_FAULT_CLEAR, FAULT_NONE, ctrl.mode, ctrl.activeSeq, ctrl.pendingSeq, apply.phase);
  }


  // NoSignal (sensor gap) failsafe enable/disable (for testing)
  if (server.hasArg("gap_en")) {
    SENSOR_GAP_ENABLE = (server.arg("gap_en").toInt() != 0);
    // reset timer so enabling doesn't instantly trip
    g_lastSensorActivityAt = now;
  }
  if (server.hasArg("stuck_en")) {
    SENSOR_STUCK_ENABLE = (server.arg("stuck_en").toInt() != 0);
    // don't want an immediate fault if we just turned it on
    if (SENSOR_STUCK_ENABLE) {
      // zeroing lastStableFlip ensures grace period
      if (sens1.stable && sens1.raw) sens1.lastStableFlip = now;
      if (sens2.stable && sens2.raw) sens2.lastStableFlip = now;
      if (sens3.stable && sens3.raw) sens3.lastStableFlip = now;
      if (sens4.stable && sens4.raw) sens4.lastStableFlip = now;
    }
  }

  // Explicit target mode buttons (RUN / RETURN / STOP) - interlocked UI
  // Uses DI7 semantics under the hood so physical+UI behavior stays identical.
  if (server.hasArg("mode_set")) {
    String t = server.arg("mode_set");
    t.toLowerCase();

    if (t == "run" || t == "green") {
      // To reach RUN: HOLD from STOP or RETURN; no-op if already RUN.
      if (ctrl.mode != MODE_RUN) {
        vdi7PressMs(now, (uint16_t)(DI7_HOLD_MIN_MS + 10));
      }
    } else if (t == "return" || t == "resume" || t == "yellow") {
      // To reach RETURN:
      // - from STOP => SHORT
      // - from RUN  => HOLD
      // - from RETURN => no-op
      if (ctrl.mode == MODE_STOP) {
        vdi7PressMs(now, (uint16_t)(DI7_DEBOUNCE_MS + 10));
      } else if (ctrl.mode == MODE_RUN) {
        vdi7PressMs(now, (uint16_t)(DI7_HOLD_MIN_MS + 10));
      }
    } else if (t == "stop" || t == "red") {
      // To reach STOP: SHORT from RUN/RETURN; no-op if already STOP.
      if (ctrl.mode != MODE_STOP) {
        vdi7PressMs(now, (uint16_t)(DI7_DEBOUNCE_MS + 10));
      }
    }
  }

  if (server.hasArg("mode")) {
    // UI mode buttons emulate DI7:
    // - RUN   => DI7_HOLD (long press)
    // - RESUME/RETURN => DI7_SHORT (short press)
    // - STOP  => DI7_SHORT, but ignored while already STOP
    String m = server.arg("mode");
    m.toLowerCase();

    if (m == "run" || m == "green") {
      vdi7PressMs(now, (uint16_t)(DI7_HOLD_MIN_MS + 10));
    } else if (m == "resume" || m == "return" || m == "yellow") {
      vdi7PressMs(now, (uint16_t)(DI7_DEBOUNCE_MS + 10));
    } else if (m == "stop" || m == "red") {
      if (ctrl.mode != MODE_STOP) {
        vdi7PressMs(now, (uint16_t)(DI7_DEBOUNCE_MS + 10));
      }
    }
  }


  if (server.hasArg("cycle_expected")) {
    ctrl.applyFromSensor(now, ctrl.expectedSensor, EDGE_SEEN);
    server.send(200, "text/plain", "OK");
    return;
  }
  if (server.hasArg("turnout")) {
    String v = server.arg("turnout"); v.toLowerCase();
    if (v == "hidden") { ctrl.desiredRoute = ROUTE_HIDDEN; requestTurnoutRoute(ROUTE_HIDDEN, now); }
    else { ctrl.desiredRoute = ROUTE_VISIBLE; requestTurnoutRoute(ROUTE_VISIBLE, now); }
    server.send(200, "text/plain", "OK");
    return;
  }

  // Operator override for AUX relays CH5..CH8
  if (server.hasArg("ovr_ch") && server.hasArg("ovr_mode")) {
    int ch = server.arg("ovr_ch").toInt();
    String mo = server.arg("ovr_mode");
    AuxOverrideMode m2 = AUX_AUTO;
    if (mo == "auto") m2 = AUX_AUTO;
    else if (mo == "on") m2 = AUX_FORCE_ON;
    else if (mo == "off") m2 = AUX_FORCE_OFF;
    else if (mo == "sp" || mo == "short") m2 = AUX_PULSE_SHORT;
    else if (mo == "lp" || mo == "long")  m2 = AUX_PULSE_LONG;

    if (ch >= 5 && ch <= 8) {
      if (ch == EXIO_CH7_WLED_PULSE && !(m2 == AUX_AUTO || m2 == AUX_FORCE_ON || m2 == AUX_FORCE_OFF)) {
        // CH7 only supports AUTO / ON / OFF
      } else {
        auxSetOverride((uint8_t)ch, m2, now);
      }
    }
  }

  // WS2812 LED box manual controls
  if (server.hasArg("led_run")) ledRailManualRun(now);
  if (server.hasArg("led_off")) ledRailManualOff(now);
  if (server.hasArg("cave_run")) caveManualRun(now);
  if (server.hasArg("cave_off")) caveManualOff(now);

  if (server.hasArg("sound")) {
    String ss = server.arg("sound");
    if (ss == "beep") {
      // Test: low->high
      soundGen.play(SND_STOP_TO_RUN, now);
    }
    if (ss == "r2d2") {
      r2d2Gen.play(SND_STOP_TO_RUN, now);
    }
  }

  if (server.hasArg("seq")) {
    int q = server.arg("seq").toInt();
    if (q >= 1 && q <= 6) {
      SeqId s2 = (SeqId)(q-1);
      ctrl.pendingSeq = s2;
      logPush(now, EVT_APPLY_START, FAULT_NONE, ctrl.mode, ctrl.activeSeq, ctrl.pendingSeq, apply.phase);
      apply.start(s2, now, ctrl.mode, ctrl.activeSeq, ctrl.pendingSeq);
      ctrl.cooldownUntil = now + COOLDOWN_MS;
    }
  }

  server.send(200, "text/plain", "OK\n");
}


// ============================================================
// WiFi endpoints: /wifi_get /wifi_set /wifi_scan
// ============================================================
static void handleWifiGet() {
  IPAddress apip = WiFi.softAPIP();
  wl_status_t st = WiFi.status();
  bool connected = (st == WL_CONNECTED);

  String s="{";
  s += "\"ok\":true,";
  s += "\"ap_ssid\":\""; s += AP_SSID; s += "\",";
  s += "\"ap_pass\":\""; s += AP_PASS; s += "\",";
  s += "\"ap_ip\":\""; s += apip.toString(); s += "\",";
  s += "\"sta_status\":\""; s += wlStr(st); s += "\",";
  s += "\"sta_connected\":"; s += (connected?"true":"false"); s += ",";
  s += "\"sta_ssid\":\""; s += (connected?WiFi.SSID():String("")); s += "\",";
  s += "\"sta_ip\":\""; s += (connected?WiFi.localIP().toString():String("")); s += "\",";
  s += "\"sta_rssi\":"; s += (connected?String(WiFi.RSSI()):String("null")); s += ",";
  s += "\"saved_ssid\":\""; s += STA_SSID; s += "\",";
  s += "\"sta_use_static\":"; s += (STA_USE_STATIC?"true":"false"); s += ",";
  s += "\"sta_static_ip\":\""; s += STA_IP.toString(); s += "\",";
  s += "\"sta_static_gw\":\""; s += STA_GW.toString(); s += "\",";
  s += "\"sta_static_mask\":\""; s += STA_MASK.toString(); s += "\",";
  s += "\"sta_static_dns\":\""; s += STA_DNS.toString(); s += "\",";
  s += "\"last_change_ms\":"; s += g_lastWifiChangeMs;
  s += "}";
  server.send(200, "application/json", s);
}

static void handleWifiSet() {
  // Accept: ssid, pass, static (0/1), ip, gw, mask, dns, ap_ssid, ap_pass
  bool apChanged=false;
  if (server.hasArg("ap_ssid")) { AP_SSID = server.arg("ap_ssid"); apChanged=true; }
  if (server.hasArg("ap_pass")) { AP_PASS = server.arg("ap_pass"); apChanged=true; }
  STA_SSID = server.hasArg("ssid") ? server.arg("ssid") : "";
  STA_PASS = server.hasArg("pass") ? server.arg("pass") : "";

  STA_USE_STATIC = server.hasArg("static") ? (server.arg("static").toInt() != 0) : false;

  IPAddress ip(0,0,0,0), gw(0,0,0,0), mask(0,0,0,0), dns(0,0,0,0);
  bool okIp=true, okGw=true, okMask=true, okDns=true;

  if (server.hasArg("ip") && server.arg("ip").length())   okIp = parseIP(server.arg("ip"), ip);
  if (server.hasArg("gw") && server.arg("gw").length())   okGw = parseIP(server.arg("gw"), gw);
  if (server.hasArg("mask") && server.arg("mask").length()) okMask = parseIP(server.arg("mask"), mask);
  if (server.hasArg("dns") && server.arg("dns").length()) okDns = parseIP(server.arg("dns"), dns);

  if (okIp)   STA_IP = ip;
  if (okGw)   STA_GW = gw;
  if (okMask) STA_MASK = mask;
  if (okDns)  STA_DNS = dns;

  bool ok = okIp && okGw && okMask && okDns;

  saveWifiPrefs();
  // Apply AP credential changes immediately
  if (apChanged) {
    WiFi.softAPdisconnect(true);
    delay(50);
    WiFi.softAP(AP_SSID.c_str(), AP_PASS.c_str());
  }
  wifiConnectNow();

  String s="{";
  s += "\"ok\":"; s += (ok ? "true":"false");
  s += "}";
  server.send(200, "application/json", s);
}

static void handleWifiScan() {
  // Note: scan can block for a few seconds
  int n = WiFi.scanNetworks(false, true);
  String s="{\"ok\":true,\"networks\":[";
  for (int i=0;i<n;i++){
    if (i) s += ",";
    String ssid = WiFi.SSID(i);
    int32_t rssi = WiFi.RSSI(i);
    s += "{";
    s += "\"ssid\":\""; s += ssid; s += "\",";
    s += "\"rssi\":"; s += rssi;
    s += "}";
  }
  s += "]}";
  server.send(200, "application/json", s);
}

// ============================================================
// WIFI START
// ============================================================
static void wifiBegin() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID.c_str(), AP_PASS.c_str());

  if (STA_SSID.length() > 0) wifiConnectNow();

  server.on("/", handleRoot);
  server.on("/get", handleGet);
  server.on("/meta", handleMeta);
  server.on("/set", handleSet);
  server.on("/diag", handleDiag);
  server.on("/log", handleLog);
  server.on("/cmd", handleCmd);
  server.on("/led_set", handleLedSet);
  server.on("/cave_set", handleCaveSet);
  server.on("/beacon_set", handleBeaconSet);

  server.on("/wifi_get", handleWifiGet);
  server.on("/wifi_set", handleWifiSet);
  server.on("/wifi_scan", handleWifiScan);

  server.begin();
}

// ============================================================
// SETUP / LOOP
// ============================================================
static uint8_t buildAllRelaysOffOutByte() {
  // For each channel OFF (closed=false), set output level according to EXIO_ACTIVE_LOW
  // activeLow: OFF => outHigh=1; activeHigh: OFF => outHigh=0
  uint8_t out = 0;
  for (uint8_t ch=1; ch<=8; ch++){
    bool activeLow = EXIO_ACTIVE_LOW[ch];
    bool outHigh = activeLow ? true : false; // closed=false
    uint8_t bit = exioBit(ch);
    if (outHigh) out |= (1u<<bit);
  }
  return out;
}

void setup() {
  // DI pins
  for (int i = 1; i <= 8; i++) {
    pinMode(PIN_DI[i], USE_PULLUP ? INPUT_PULLUP : INPUT);
  }

  // Pixel
  pixel.begin();
  pixel.setBrightness(PIXEL_BRIGHTNESS);
  pixelSet(0x000000);

  // LED rail / cave strip
  ledRail.begin();
  caveFullSpec.begin();
  beaconBegin();

  // Preferences
  prefs.begin("trainctrl", false);
  loadWifiPrefs();
  loadLedPrefs();
  loadCavePrefs();
  loadBeaconPrefs();

  // I2C / TCA9554
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  (void)tcaWrite(REG_POLARITY, 0x00);
  (void)tcaWrite(REG_CONFIG,   0x00); // all outputs

  // Initialize outputs to "all relays OFF" according to EXIO_ACTIVE_LOW[]
  (void)tcaWrite(REG_OUTPUT, buildAllRelaysOffOutByte());

  // Boot-safe defaults
  ctrl.mode = MODE_STOP;
  ctrl.activeSeq = SEQ1;
  ctrl.pendingSeq = SEQ1;
  ctrl.stopPowerLatched = true;
  ctrl.handoffActive = false;
  ctrl.handoffSeenArmed = false;
  ctrl.expectedSensor = 1;
  ctrl.currentRoute = ROUTE_VISIBLE;
  ctrl.desiredRoute = ROUTE_VISIBLE;

  // Init AUTO aux logic (CH7..CH8) and turnout outputs. Set base direction/speed WITHOUT pulse at boot
  setDirection(PLAN[SEQ1].dir, millis(), false);
  setSpeedSelect(PLAN[SEQ1].speed);
  applyLightRulesForSeq(SEQ1);
  uvCtrl.begin();
  auxApplyOutputs(millis());

  // Power rails OFF
  powerSetSafe(millis(), false, false, ctrl.mode, ctrl.activeSeq, ctrl.pendingSeq, apply.phase);

  // Buzzer
  buzzer.begin();
  buzzer.stop();

  // SEQ stats init: SEQ1 starts "active" at boot
  g_seqStartMs = millis();
  g_seqCount[SEQ1] = 1;

  g_lastSensorActivityAt = millis();

  wifiBegin();

  logPush(millis(), EVT_NONE, FAULT_NONE, ctrl.mode, ctrl.activeSeq, ctrl.pendingSeq, apply.phase);
}

void loop() {
  uint32_t now = millis();

  server.handleClient();
  uvCtrl.service(now);
  ledRail.service(now);
  caveFullSpec.service(now);
  buzzer.service(now);

  // DI7 pressed polarity (physical) + Virtual DI7 (UI)
  bool di7Closed = readDI(PIN_DI[7]);
  bool di7PhysPressed = DI7_PRESSED_IS_OPEN ? (!di7Closed) : di7Closed;
  bool di7VirtPressed = vdi7IsPressed(now);
  bool di7Pressed = di7PhysPressed || di7VirtPressed;
  ctrl.onDi7(di7.update(di7Pressed, now), now);

  // STOP mode: hold power off, red blink
  if (ctrl.mode == MODE_STOP) {
    auxApplyOutputs(now);
    if (!ctrl.stopPowerLatched) {
      powerCut(now, ctrl.mode, ctrl.activeSeq, ctrl.pendingSeq, apply.phase);
      ctrl.stopPowerLatched = true;
    }
    ctrl.render(now, false);
    beaconService(now, ctrl.mode, g_lastFault);
    return;
  } else {
    ctrl.stopPowerLatched = false;
  }

  // Sensors (DI1..DI4) and DI6 cycle button
  bool di1Seen = readDI(PIN_DI[1]);
  bool di2Seen = readDI(PIN_DI[2]);
  bool di3Seen = readDI(PIN_DI[3]);
  bool di4Seen = readDI(PIN_DI[4]);
  bool di6Seen = readDI(PIN_DI[6]);

  SensorEdge e1 = sens1.update(di1Seen, now, SENSOR_SEEN_STABLE_MS, SENSOR_CLEAR_STABLE_MS);
  SensorEdge e2 = sens2.update(di2Seen, now, SENSOR_SEEN_STABLE_MS, SENSOR_CLEAR_STABLE_MS);
  SensorEdge e3 = sens3.update(di3Seen, now, SENSOR_SEEN_STABLE_MS, SENSOR_CLEAR_STABLE_MS);
  SensorEdge e4 = sens4.update(di4Seen, now, SENSOR_SEEN_STABLE_MS, SENSOR_CLEAR_STABLE_MS);
  SensorEdge e6 = di6Cycle.update(di6Seen, now, SENSOR_SEEN_STABLE_MS, SENSOR_CLEAR_STABLE_MS);

  if (e1 != EDGE_NONE || e2 != EDGE_NONE || e3 != EDGE_NONE || e4 != EDGE_NONE) {
    g_lastSensorActivityAt = now;
  }

  if (ctrl.expectedSensor == 2) {
    if (e2 == EDGE_SEEN)  s2Cave.onSeen(now);
    if (e2 == EDGE_CLEAR) s2Cave.onClear(now);
  }
  if (ctrl.expectedSensor == 3) {
    if (e3 == EDGE_SEEN)  ledRail.onSensor3Seen(now);
    if (e3 == EDGE_CLEAR) ledRail.onSensor3Clear(now);
  }

  apply.service(now, ctrl.mode, ctrl.activeSeq, ctrl.pendingSeq);
  ctrl.onApplyFinished(now);

  if (SENSOR_STUCK_ENABLE && SENSOR_STUCK_TIMEOUT_MS > 0 && ctrl.mode != MODE_STOP) {
    if (sens1.stable && sens1.raw && (uint32_t)(now - sens1.lastStableFlip) > (uint32_t)SENSOR_STUCK_TIMEOUT_MS) {
      setFault(now, FAULT_SENSOR_STUCK_A, ctrl.mode, ctrl.activeSeq, ctrl.pendingSeq, apply.phase);
      ctrl.enterStop(now); auxApplyOutputs(now); g_lastSensorActivityAt = now; return;
    }
    if ((sens2.stable && sens2.raw && (uint32_t)(now - sens2.lastStableFlip) > (uint32_t)SENSOR_STUCK_TIMEOUT_MS) ||
        (sens3.stable && sens3.raw && (uint32_t)(now - sens3.lastStableFlip) > (uint32_t)SENSOR_STUCK_TIMEOUT_MS) ||
        (sens4.stable && sens4.raw && (uint32_t)(now - sens4.lastStableFlip) > (uint32_t)SENSOR_STUCK_TIMEOUT_MS)) {
      setFault(now, FAULT_SENSOR_STUCK_B, ctrl.mode, ctrl.activeSeq, ctrl.pendingSeq, apply.phase);
      ctrl.enterStop(now); auxApplyOutputs(now); g_lastSensorActivityAt = now; return;
    }
  }

  if (SENSOR_GAP_ENABLE && SENSOR_GAP_TIMEOUT_MS > 0 && ctrl.mode != MODE_STOP) {
    if (g_lastSensorActivityAt != 0 && (uint32_t)(now - g_lastSensorActivityAt) > (uint32_t)SENSOR_GAP_TIMEOUT_MS) {
      setFault(now, FAULT_SENSOR_GAP, ctrl.mode, ctrl.activeSeq, ctrl.pendingSeq, apply.phase);
      ctrl.enterStop(now); auxApplyOutputs(now); g_lastSensorActivityAt = now; return;
    }
  }

  if (!apply.isActive()) {
    if (e1 != EDGE_NONE) ctrl.applyFromSensor(now, 1, e1);
    if (e2 != EDGE_NONE) ctrl.applyFromSensor(now, 2, e2);
    if (e3 != EDGE_NONE) ctrl.applyFromSensor(now, 3, e3);
    if (e4 != EDGE_NONE) ctrl.applyFromSensor(now, 4, e4);
  }
  if (!apply.isActive() && e6 == EDGE_SEEN) {
    ctrl.applyFromSensor(now, ctrl.expectedSensor, EDGE_SEEN);
  }

  turnoutPulse.service(now);

  bool trig = sens1.stable || sens2.stable || sens3.stable || sens4.stable;

  // Apply operator overrides + AUTO aux logic (CH5..CH8)
  auxApplyOutputs(now);

  ctrl.render(now, trig);
  beaconService(now, ctrl.mode, g_lastFault);
}