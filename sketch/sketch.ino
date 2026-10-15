#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <ModbusMaster.h>
#include "Untitled_Design.h"

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read  = 16000000;
      cfg.spi_3wire  = false;
      cfg.use_lock   = true;
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

#define RX_PIN 16
#define TX_PIN 15

void setup() {
  Serial.begin(115200);
  
  // Initialize Serial1 for RS-485 Modbus RTU communication to MD02
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Initialize Display
  tft.init();
  tft.setRotation(0); // 0 = Portrait
  tft.fillScreen(TFT_BLACK);
  
  // Draw the 240x160 PNG image from Untitled_Design.h at the top (x=0, y=0)
  tft.drawPng(Untitled_Design, sizeof(Untitled_Design), 0, 0);
  
  // Initialize Modbus communication on Serial1, slave address 1
  node.begin(1, Serial1);
}

void loop() {
  uint8_t result;
  
  // XY-MD02 uses function code 0x04 (Read Input Registers)
  result = node.readInputRegisters(1, 2);
  
  if (result == node.ku8MBSuccess) {
    int16_t tempRaw = node.getResponseBuffer(0);
    int16_t humRaw = node.getResponseBuffer(1);
    
    float tempC = tempRaw / 100.0;
    float hum = humRaw / 100.0;
    
    // Clear only the bottom half of the screen (y=160 to 320)
    tft.fillRect(0, 160, 240, 160, TFT_BLACK);
    
    // Draw Temperature Label (Warm Orange/Red color)
    tft.setTextColor(tft.color565(255, 100, 100)); 
    tft.setTextSize(2);
    tft.setCursor(20, 175);
    tft.print("Temperature:");
    
    // Draw Temperature Value
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(3);
    tft.setCursor(20, 200);
    tft.print(tempC, 2);
    tft.print(" C");
    
    // Draw Humidity Label (Cool Cyan/Blue color)
    tft.setTextColor(tft.color565(100, 200, 255)); 
    tft.setTextSize(2);
    tft.setCursor(20, 245);
    tft.print("Humidity:");
    
    // Draw Humidity Value
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(3);
    tft.setCursor(20, 270);
    tft.print(hum, 2);
    tft.print(" %");
    
    Serial.printf("Temperature: %.2f C, Humidity: %.2f %%\n", tempC, hum);
  } else {
    tft.fillRect(0, 160, 240, 160, TFT_BLACK);
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(20, 200);
    tft.println("Sensor Error!");
    tft.setCursor(20, 230);
    tft.print("Code: ");
    tft.println(result);
    
    Serial.printf("Failed to read from sensor, Modbus code: 0x%02X\n", result);
  }
  
  delay(2000);
}