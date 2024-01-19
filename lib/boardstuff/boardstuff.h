#define TFT_ORIENTATION_PORTRAIT_INV
#define ESP32_2432S024R
#pragma once
#include <Arduino.h>
#include <lvgl.h>
#include <SPI.h>

#ifdef ESP32_3248S035C
#include <Wire.h>
#define TFT_WIDTH 320
#define TFT_HEIGHT 480
#define ST7796
#define ST7796_SPI_SCLK 14
#define ST7796_SPI_MOSI 13
#define ST7796_SPI_MISO 12
#define ST7796_PIN_CS 15
#define ST7796_PIN_DC 2
#define ST7796_SPI_FREQ 80000000
#define ST7796_PIN_BL 27
#define ST7796_PWM_CHANNEL_BL 12
#define ST7796_PWM_FREQ_BL 5000
#define ST7796_PWM_BITS_BL 8
#define ST7796_PWM_MAX_BL ((1 << ST7796_PWM_BITS_BL) - 1)
#define GT911
#define GT911_IIC_SDA 33
#define GT911_IIC_SCL 32
#define GT911_IIC_RST 25
#define GT911_I2C_SLAVE_ADDR 0x5D
#define GT911_MAX_CONTACTS 5
#define GT911_PRODUCT_ID1 0x8140
#define GT911_REG_COORD_ADDR 0x814E
#define GT911_TRACK_ID1 0x814F
#define GT911_PRODUCT_ID_LEN 4

#if !defined(TFT_ORIENTATION_PORTRAIT) && !defined(TFT_ORIENTATION_LANDSCAPE) && !defined(TFT_ORIENTATION_PORTRAIT_INV) && !defined(TFT_ORIENTATION_LANDSCAPE_INV)
#error Please define orientation: TFT_ORIENTATION_PORTRAIT, TFT_ORIENTATION_LANDSCAPE, TFT_ORIENTATION_PORTRAIT_INV or TFT_ORIENTATION_LANDSCAPE_INV
#endif


extern TwoWire i2c_gt911;
#endif

// ESP32_2432S024R
#ifdef ESP32_2432S024R
#define TFT_WIDTH 240
#define TFT_HEIGHT 320
#define ILI9431
#define ILI9431_SPI_SCLK 14
#define ILI9431_SPI_MOSI 13
#define ILI9431_SPI_MISO 12
#define ILI9341_PIN_CS 15
#define ILI9341_PIN_DC 2
#define ILI9341_SPI_FREQ 50000000
#define ILI9341_PIN_BL 27
#define ILI9341_PWM_CHANNEL_BL 12
#define ILI9341_PWM_FREQ_BL 5000
#define ILI9341_PWM_BITS_BL 8
#define ILI9341_PWM_MAX_BL ((1 << ILI9341_PWM_BITS_BL) - 1)
#define XPT2046
#define XPT2046_SPI_SCLK 25
#define XPT2046_SPI_MOSI 32
#define XPT2046_SPI_MISO 39
#define XPT2046_SPI_FREQ 2000000
#define XPT2046_PIN_INT 36
#define XPT2046_PIN_CS 33
// Calibration 240x320
#define XPT2046_MIN_X 349
#define XPT2046_MAX_X 3859
#define XPT2046_MIN_Y 247
#define XPT2046_MAX_Y 3871
#endif



// Build in RGB LED
#define LED_PIN_R 4
#define LED_PIN_G 16
#define LED_PIN_B 17
// PWM channels for RGB
#define LED_PWM_FREQ 5000
#define LED_PWM_CHANNEL_R 13
#define LED_PWM_CHANNEL_G 14
#define LED_PWM_CHANNEL_B 15
#define LED_PWM_BITS 8
#define LED_PWM_MAX 0xFF

// Photo resistor
#define CDS_PIN 34 // ANALOG_PIN_0

// Audio out
#define AUDIO_PIN 26

// TF Card
#define TF_PIN_CS 5
#define TF_PIN_MOSI 23
#define TF_PIN_SCLK 18
#define TF_PIN_MISC 19

extern void board_init();
extern void set_led(uint8_t red, uint8_t green, uint8_t blue);
extern void set_led(uint32_t hex_code);
extern void set_backlight(uint8_t brightness);
extern void (*touch_callback)();//if not null, pointed function will be called every time a touch is detected

