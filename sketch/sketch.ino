/*
 * NORVI X CPU-ESPS3-X1  +  XY-MD02 (SHT20) Modbus RTU temperature/humidity sensor
 *
 * - Sensor : XY-MD02, RS-485 Modbus RTU, 9600 8N1, slave ID 1 (default)
 *            Function 0x04, input register 0x0001 = temperature, 0x0002 = humidity
 * - RS-485 : IO16 = RX, IO15 = TX (SN65HVD72, direction is automatic)
 * - Display: 2.0" ST7789 240x320 on SPI (MOSI 11, MISO 13, SCLK 12, CS 45, DC 46, RST 47)
 *
 * UI: arc-gauge dashboard, colour-coded values, min/max since boot, status dot.
 * Libraries: LovyanGFX, ModbusMaster
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <ModbusMaster.h>

// ---------------------------------------------------------------- Display
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 40000000;
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;

      cfg.pin_sclk = 12;
      cfg.pin_mosi = 11;
      cfg.pin_miso = 13;
      cfg.pin_dc   = 46;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = 45;
      cfg.pin_rst          = 47;
      cfg.pin_busy         = -1;
      cfg.panel_width      = 240;
      cfg.panel_height     = 320;
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.offset_rotation  = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable         = true;
      cfg.invert           = true;
      cfg.rgb_order        = false;
      cfg.dlen_16bit       = false;
      cfg.bus_shared       = true;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

LGFX tft;
ModbusMaster node;

// ---------------------------------------------------------------- Config
#define RS485_RX_PIN   16
#define RS485_TX_PIN   15

static const uint8_t  MODBUS_SLAVE_ID = 1;       // XY-MD02 default address
static const uint32_t MODBUS_BAUD     = 9600;    // XY-MD02 default baud rate
static const uint16_t REG_TEMPERATURE = 0x0001;  // input register: temperature
static const uint16_t REG_COUNT       = 2;       // temperature + humidity
static const uint32_t POLL_INTERVAL   = 1000;    // ms between reads
static const uint8_t  FAIL_LIMIT      = 3;       // consecutive failures before "error" state

// ---------------------------------------------------------------- Colours (RGB565)
static const uint16_t COL_BG     = 0x0000;
static const uint16_t COL_HDR    = 0x0210;
static const uint16_t COL_CARD   = 0x10A2;
static const uint16_t COL_TRACK  = 0x31A6;
static const uint16_t COL_WHITE  = 0xFFFF;
static const uint16_t COL_DIM    = 0x8410;
static const uint16_t COL_OK     = 0x07E0;
static const uint16_t COL_ERR    = 0xF800;
static const uint16_t COL_WARN   = 0xFD20;
static const uint16_t COL_COLD   = 0x04FF;
static const uint16_t COL_CYAN   = 0x07FF;
static const uint16_t COL_WET    = 0xA81F;

uint16_t tempColor(float t) {
  if (t < 18.0f)  return COL_COLD;
  if (t <= 28.0f) return COL_OK;
  if (t <= 35.0f) return COL_WARN;
  return COL_ERR;
}
uint16_t humColor(float h) {
  if (h < 30.0f)  return COL_WARN;
  if (h <= 70.0f) return COL_CYAN;
  return COL_WET;
}

// ---------------------------------------------------------------- Layout (portrait 240x320)
static const int CARD_X = 10, CARD_W = 220, CARD_H = 120;
static const int GAUGE_CX    = CARD_X + 56;   // gauge centre x
static const int R_OUT = 48, R_IN = 38;       // gauge ring radii
static const float ARC_START = 135.0f;        // 270 degree sweep, open at the bottom
static const float ARC_SWEEP = 270.0f;
static const int PANEL_X   = CARD_X + 118;    // right info panel left edge
static const int PANEL_R   = CARD_X + CARD_W - 8;

// Metric indices
#define M_TEMP 0
#define M_HUM  1

struct Metric {
  int         cardY;
  const char *label;
  float       lo, hi;           // gauge range
  float       mn, mx;           // min / max since boot
  bool        seen;
  String      shownVal;
  String      shownMin;
  String      shownMax;
};

Metric metrics[2] = {
  { 40,  "TEMPERATURE", 0.0f,  50.0f,  0, 0, false, "", "", "" },
  { 168, "HUMIDITY",    0.0f, 100.0f,  0, 0, false, "", "", "" }
};

// ---------------------------------------------------------------- State
uint16_t divisor     = 10;     // datasheet: value / 10 (auto-switches to /100 for firmware variants)
uint8_t  failCount   = 0;
String   shownStatus = "";
int      shownDot    = -1;

// ---------------------------------------------------------------- Drawing helpers
// Draw an arc segment between a0 and a1 degrees (0 = 3 o'clock, clockwise), a0 < a1 <= 450.
void arcSeg(int cx, int cy, int r0, int r1, float a0, float a1, uint16_t color) {
  if (a1 - a0 < 0.5f) return;
  if (a1 <= 360.0f) {
    tft.fillArc(cx, cy, r1, r0, a0, a1, color);
  } else if (a0 >= 360.0f) {
    tft.fillArc(cx, cy, r1, r0, a0 - 360.0f, a1 - 360.0f, color);
  } else {
    tft.fillArc(cx, cy, r1, r0, a0, 359.9f, color);
    if (a1 - 360.0f > 0.5f) tft.fillArc(cx, cy, r1, r0, 0.0f, a1 - 360.0f, color);
  }
}

void drawCard(int idx, const char *unit, bool degree) {
  Metric &m = metrics[idx];
  const int y  = m.cardY;
  const int cy = y + CARD_H / 2;

  tft.fillRoundRect(CARD_X, y, CARD_W, CARD_H, 10, COL_CARD);

  // Empty gauge track
  arcSeg(GAUGE_CX, cy, R_IN, R_OUT, ARC_START, ARC_START + ARC_SWEEP, COL_TRACK);

  // Unit inside the gauge opening
  tft.setFont(&fonts::Font2);
  tft.setTextColor(COL_DIM, COL_CARD);
  tft.setTextDatum(MC_DATUM);
  if (degree) {
    tft.drawCircle(GAUGE_CX - 8, cy + 22, 2, COL_DIM);
    tft.drawString(unit, GAUGE_CX + 3, cy + 27);
  } else {
    tft.drawString(unit, GAUGE_CX, cy + 27);
  }

  // Right panel: label, divider, MIN / MAX captions
  tft.setTextColor(COL_WHITE, COL_CARD);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(m.label, PANEL_X + (PANEL_R - PANEL_X) / 2, y + 18);
  tft.drawFastHLine(PANEL_X, y + 32, PANEL_R - PANEL_X, COL_TRACK);

  tft.setTextColor(COL_DIM, COL_CARD);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("MIN", PANEL_X, y + 56);
  tft.drawString("MAX", PANEL_X, y + 90);
}

void drawGauge(int idx, bool valid, float v, uint16_t color) {
  Metric &m = metrics[idx];
  const int cy = m.cardY + CARD_H / 2;

  char buf[12];
  if (valid) snprintf(buf, sizeof(buf), "%.1f", v);
  else       snprintf(buf, sizeof(buf), "--.-");
  String text(buf);
  if (text == m.shownVal) return;
  m.shownVal = text;

  // Gauge arc: coloured fill + empty remainder
  float frac = 0.0f;
  if (valid) {
    frac = (v - m.lo) / (m.hi - m.lo);
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
  }
  float aEnd = ARC_START + ARC_SWEEP * frac;
  arcSeg(GAUGE_CX, cy, R_IN, R_OUT, ARC_START, aEnd, color);
  arcSeg(GAUGE_CX, cy, R_IN, R_OUT, aEnd, ARC_START + ARC_SWEEP, COL_TRACK);

  // Centre value
  tft.fillRect(GAUGE_CX - 30, cy - 20, 60, 34, COL_CARD);
  tft.setFont(&fonts::Font4);
  tft.setTextSize(text.length() >= 5 ? 0.9f : 1.15f);
  tft.setTextColor(valid ? color : COL_DIM, COL_CARD);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(text, GAUGE_CX, cy - 3);
  tft.setTextSize(1.0f);
}

void drawMinMax(int idx) {
  Metric &m = metrics[idx];
  if (!m.seen) return;
  char b1[12], b2[12];
  snprintf(b1, sizeof(b1), "%.1f", m.mn);
  snprintf(b2, sizeof(b2), "%.1f", m.mx);
  tft.setFont(&fonts::Font2);
  tft.setTextDatum(MR_DATUM);
  if (String(b1) != m.shownMin) {
    m.shownMin = b1;
    tft.fillRect(PANEL_X + 30, m.cardY + 46, PANEL_R - PANEL_X - 30, 20, COL_CARD);
    tft.setTextColor(COL_CYAN, COL_CARD);
    tft.drawString(b1, PANEL_R, m.cardY + 56);
  }
  if (String(b2) != m.shownMax) {
    m.shownMax = b2;
    tft.fillRect(PANEL_X + 30, m.cardY + 80, PANEL_R - PANEL_X - 30, 20, COL_CARD);
    tft.setTextColor(COL_WARN, COL_CARD);
    tft.drawString(b2, PANEL_R, m.cardY + 90);
  }
}

void drawDot(uint16_t color) {
  if ((int)color == shownDot) return;
  shownDot = color;
  tft.fillRect(206, 4, 30, 24, COL_HDR);
  tft.fillCircle(221, 16, 6, color);
  tft.drawCircle(221, 16, 9, color);
}

void drawStatus(const String &text, uint16_t color) {
  if (text == shownStatus) return;
  shownStatus = text;
  tft.fillRect(0, 292, 240, 28, COL_BG);
  tft.setFont(&fonts::Font2);
  tft.setTextColor(color, COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(text, 120, 306);
}

void drawStaticUI() {
  tft.fillScreen(COL_BG);

  // Header
  tft.fillRect(0, 0, 240, 32, COL_HDR);
  tft.setFont(&fonts::Font4);
  tft.setTextColor(COL_WHITE, COL_HDR);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("NORVI X  MD02", 10, 16);

  drawCard(M_TEMP, "C",   true);
  drawCard(M_HUM,  "%RH", false);

  for (int i = 0; i < 2; i++) {
    metrics[i].shownVal = "";
    metrics[i].shownMin = "";
    metrics[i].shownMax = "";
  }
  shownStatus = "";
  shownDot = -1;
}

void showSplash() {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setFont(&fonts::Font4);
  tft.setTextSize(1.6f);
  tft.setTextColor(COL_WHITE, COL_BG);
  tft.drawString("NORVI X", 120, 130);
  tft.setTextSize(1.0f);
  tft.setFont(&fonts::Font2);
  tft.setTextColor(COL_CYAN, COL_BG);
  tft.drawString("Temperature & Humidity Monitor", 120, 170);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawString("XY-MD02  |  Modbus RTU", 120, 192);
  delay(1500);
}

const char *modbusErrorText(uint8_t code) {
  switch (code) {
    case 0xE0: return "Invalid slave ID";
    case 0xE1: return "Invalid function";
    case 0xE2: return "No response (timeout)";
    case 0xE3: return "CRC error";
    case 0x01: return "Illegal function";
    case 0x02: return "Illegal data address";
    case 0x03: return "Illegal data value";
    default:   return "Modbus error";
  }
}

void updateMinMax(int idx, float v) {
  Metric &m = metrics[idx];
  if (!m.seen) { m.mn = m.mx = v; m.seen = true; return; }
  if (v < m.mn) m.mn = v;
  if (v > m.mx) m.mx = v;
}

// ---------------------------------------------------------------- Arduino
void setup() {
  Serial.begin(115200);

  // RS-485 / Modbus RTU (transceiver direction is automatic)
  Serial1.begin(MODBUS_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  node.begin(MODBUS_SLAVE_ID, Serial1);

  // Display
  tft.init();
  tft.setRotation(0);   // portrait
  showSplash();
  drawStaticUI();
  drawGauge(M_TEMP, false, 0, COL_DIM);
  drawGauge(M_HUM,  false, 0, COL_DIM);
  drawDot(COL_DIM);
  drawStatus("Connecting to sensor...", COL_DIM);

  Serial.println("NORVI X + XY-MD02 temperature/humidity monitor");
}

void loop() {
  static uint32_t lastPoll = 0;
  if (millis() - lastPoll < POLL_INTERVAL) return;
  lastPoll = millis();

  // XY-MD02: function 0x04, start 0x0001, 2 registers (temperature, humidity)
  uint8_t result = node.readInputRegisters(REG_TEMPERATURE, REG_COUNT);

  if (result == node.ku8MBSuccess) {
    int16_t  rawT = (int16_t)node.getResponseBuffer(0);   // signed
    uint16_t rawH = node.getResponseBuffer(1);

    // Datasheet scaling is 0.1 units. Some units report 0.01 units; detect and latch.
    if (rawH > 1000 || abs((int)rawT) > 1250) divisor = 100;

    float tempC = rawT / (float)divisor;
    float hum   = rawH / (float)divisor;

    failCount = 0;
    updateMinMax(M_TEMP, tempC);
    updateMinMax(M_HUM,  hum);

    drawGauge(M_TEMP, true, tempC, tempColor(tempC));
    drawGauge(M_HUM,  true, hum,   humColor(hum));
    drawMinMax(M_TEMP);
    drawMinMax(M_HUM);
    drawDot(COL_OK);
    drawStatus("Modbus OK   ID:" + String(MODBUS_SLAVE_ID) + "   " + String(MODBUS_BAUD) + " 8N1", COL_OK);

    Serial.printf("Temperature: %.1f C, Humidity: %.1f %% (raw T=%d H=%u, /%u)\n",
                  tempC, hum, rawT, rawH, divisor);
  } else {
    if (failCount < 255) failCount++;
    Serial.printf("Modbus read failed: 0x%02X (%s)\n", result, modbusErrorText(result));

    if (failCount >= FAIL_LIMIT) {
      drawGauge(M_TEMP, false, 0, COL_DIM);
      drawGauge(M_HUM,  false, 0, COL_DIM);
      drawDot(COL_ERR);
      drawStatus(String("Sensor error: ") + modbusErrorText(result), COL_ERR);
    }
  }
}
