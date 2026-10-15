#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <ModbusMaster.h>

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
  // MD02 default baud rate is 9600
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Initialize Display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("NORVI X1 - MD02 Sensor");
  
  // Initialize Modbus communication on Serial1, slave address 1
  node.begin(1, Serial1);
}

void loop() {
  uint8_t result;
  
  // XY-MD02 uses function code 0x04 (Read Input Registers)
  // Register 0x0001 is Temperature, 0x0002 is Humidity
  // We'll read 2 registers starting at 0x0001
  result = node.readInputRegisters(1, 2);
  
  if (result == node.ku8MBSuccess) {
    int16_t tempRaw = node.getResponseBuffer(0);
    int16_t humRaw = node.getResponseBuffer(1);
    
    float tempC = tempRaw / 10.0;
    float hum = humRaw / 10.0;
    
    tft.fillRect(10, 50, 300, 100, TFT_BLACK);
    tft.setCursor(10, 50);
    tft.print("Temp: ");
    tft.print(tempC, 1);
    tft.println(" C");
    
    tft.setCursor(10, 80);
    tft.print("Humidity: ");
    tft.print(hum, 1);
    tft.println(" %");
    
    Serial.printf("Temperature: %.1f C, Humidity: %.1f %%\n", tempC, hum);
  } else {
    tft.fillRect(10, 50, 300, 100, TFT_BLACK);
    tft.setCursor(10, 50);
    tft.println("Sensor Error!");
    tft.print("Code: ");
    tft.println(result);
    
    Serial.printf("Failed to read from sensor, Modbus code: 0x%02X\n", result);
  }
  
  delay(2000);
}