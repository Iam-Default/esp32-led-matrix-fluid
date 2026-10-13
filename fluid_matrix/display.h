/**
 * display.h - LovyanGFX configuration for the board's GC9A01 round LCD.
 *
 * Describes how the display is wired (SPI bus, control pins, backlight) so the
 * LovyanGFX library can drive it. Pins and options come from config.h.
 */
#pragma once
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "config.h"

// Display object used by the sketch. LovyanGFX is configured in the constructor.
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI      _bus;
  lgfx::Light_PWM    _light;
public:
  LGFX() {
    // SPI bus: pins, clock and DMA channel.
    { auto c = _bus.config();
      c.spi_host   = SPI2_HOST;
      c.spi_mode   = 0;
      c.freq_write = LCD_SPI_HZ;
      c.freq_read  = 20000000;
      c.spi_3wire  = false;
      c.use_lock   = true;
      c.dma_channel = SPI_DMA_CH_AUTO;
      c.pin_sclk = PIN_LCD_SCLK;
      c.pin_mosi = PIN_LCD_MOSI;
      c.pin_miso = PIN_LCD_MISO;
      c.pin_dc   = PIN_LCD_DC;
      _bus.config(c);
      _panel.setBus(&_bus); }
    // Panel: control pins, size, colour inversion and orientation.
    { auto c = _panel.config();
      c.pin_cs   = PIN_LCD_CS;
      c.pin_rst  = PIN_LCD_RST;
      c.pin_busy = -1;
      c.panel_width = SCREEN_PX;  c.panel_height = SCREEN_PX;
      c.memory_width = SCREEN_PX; c.memory_height = SCREEN_PX;
      c.offset_x = 0; c.offset_y = 0; c.offset_rotation = 0;
      c.readable = false;
      c.invert   = LCD_INVERT;
      c.rgb_order = false;
      c.dlen_16bit = false;
      c.bus_shared = false;
      _panel.config(c); }
    // Backlight: PWM on PIN_LCD_BL.
    { auto c = _light.config();
      c.pin_bl = PIN_LCD_BL;
      c.invert = false;
      c.freq   = 44100;
      c.pwm_channel = 7;
      _light.config(c);
      _panel.setLight(&_light); }
    setPanel(&_panel);
  }
};
