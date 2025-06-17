#include "gdeh0154d67.h"
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
// Partial Update Delay
#define GDEH0154D67_PU_DELAY 300


// Constructor
Gdeh0154d67::Gdeh0154d67(EpdSpi& dio): 
  Adafruit_GFX(GDEH0154D67_WIDTH, GDEH0154D67_HEIGHT),
  Epd(GDEH0154D67_WIDTH, GDEH0154D67_HEIGHT), IO(dio)
{
}

void Gdeh0154d67::initUpdate() {
  _wakeUp();
}

void Gdeh0154d67::_wakeUp() {
  if (_deeply_sleeping) {
    IO.reset(20);
    _deeply_sleeping = false;
  }

  IO.cmd(0x12);
  _waitBusy("epd_wakeup_power:ON", power_on_time);

  IO.cmd(0x01); // Driver output control
  IO.data(0xC7);
  IO.data(0x00);
  IO.data(0x00);

  IO.cmd(0x3C); // BorderWavefrom
  IO.data(0x05);
  IO.cmd(0x18); // Read built-in temperature sensor
  IO.data(0x80);
  
  // RAM data entry mode
  IO.cmd(0x11);
  IO.data(3);
}

void Gdeh0154d67::_PowerOn(void) {
}

void Gdeh0154d67::_sleep(){
}

//Initialize the display
void Gdeh0154d67::init(bool debug)
{
    IO.init(4, debug); // 4MHz frequency
    fillScreen(EPD_WHITE);
}

void Gdeh0154d67::fillScreen(uint16_t color)
{
  // 0xFF = 8 pixels black, 0x00 = 8 pix. white
  uint8_t data = (color == EPD_BLACK) ? GDEH0154D67_8PIX_BLACK : GDEH0154D67_8PIX_WHITE;
  for (uint16_t x = 0; x < sizeof(_buffer); x++)
  {
    _buffer[x] = data;
  }
}

void Gdeh0154d67::_SetRamArea(uint8_t Xstart, uint8_t Xend, uint8_t Ystart, uint8_t Ystart1, uint8_t Yend, uint8_t Yend1)
{
  IO.cmd(0x44);
  IO.data(Xstart);
  IO.data(Xend);
  IO.cmd(0x45);
  IO.data(Ystart);
  IO.data(Ystart1);
  IO.data(Yend);
  IO.data(Yend1);
}

void Gdeh0154d67::_SetRamPointer(uint8_t addrX, uint8_t addrY, uint8_t addrY1)
{
  IO.cmd(0x4e);
  IO.data(addrX);
  IO.cmd(0x4f);
  IO.data(addrY);
  IO.data(addrY1);
}

void Gdeh0154d67::_writeBuffer() {
  _SetRamArea(0x00, (GDEH0154D67_WIDTH - 1) / 8,
              0x00, 0x00, (GDEH0154D67_HEIGHT - 1) % 256, 0x00);  // X-source area,Y-gate area
  _SetRamPointer(0x00, 0x00, 0x00);
  _waitBusy("ram_pointer0", 100);
  IO.cmd(0x24);        // update current data
  for (uint16_t y = 0; y < GDEH0154D67_HEIGHT; y++)
  {
    for (uint16_t x = 0; x < GDEH0154D67_WIDTH / 8; x++)
    {
      uint16_t idx = y * (GDEH0154D67_WIDTH / 8) + x;
      uint8_t data = (idx < sizeof(_buffer)) ? _buffer[idx] : 0x00;
      IO.data(~data);
    }
  }
}

void Gdeh0154d67::_writeBuffer(int16_t x, int16_t y, int16_t w, int16_t h) {
  uint16_t xe = gx_uint16_min(GDEH0154D67_WIDTH, x + w) - 1;
  uint16_t ye = gx_uint16_min(GDEH0154D67_HEIGHT, y + h) - 1;
  uint16_t xs_d8 = x / 8;
  uint16_t xe_d8 = xe / 8;
  _SetRamArea(xs_d8, xe_d8, y % 256, y / 256, ye % 256, ye / 256); // X-source area,Y-gate area
  _SetRamPointer(xs_d8, y % 256, y / 256); // set ram
  _waitBusy("ram_pointer1", 100);
  IO.cmd(0x24);

  for (int16_t y1 = y; y1 <= ye; y1++)
  {
    for (int16_t x1 = xs_d8; x1 <= xe_d8; x1++)
    {
      uint16_t idx = y1 * (WIDTH / 8) + x1;
      uint8_t data = (idx < sizeof(_buffer)) ? _buffer[idx] : 0x00;
      IO.data(~data);
    }
  }
}

void Gdeh0154d67::update()
{
  _initial_refresh_pending = false;
  initUpdate();
  _writeBuffer();
  IO.cmd(0x22);
  IO.data(0xf7);
  IO.cmd(0x20);
  _waitBusy("_Update_Full", full_refresh_time);
  _writeBuffer();
}

void Gdeh0154d67::updateWindow(int16_t x, int16_t y, int16_t w, int16_t h, bool using_rotation)
{
  if (using_rotation) _rotate(x, y, w, h);
  if (x >= WIDTH) {
    ESP_LOGI(__FILE__, "x:%d exceeded boundary %d", x, WIDTH);
    return;
  }
  if (y >= HEIGHT) {
    ESP_LOGI(__FILE__, "y:%d exceeded boundary %d", y, HEIGHT);
    return;
  }

  if (_initial_refresh_pending) {
    ESP_LOGI(__FILE__, "updateWindow() doing initial refresh");
    update();
  }

  initUpdate();
  _writeBuffer(x, y, w, h);
  IO.cmd(0x22);
  IO.data(0xff);
  IO.cmd(0x20);
  _waitBusy("partial_update", 100);
  _writeBuffer(x, y, w, h);
}
  
void Gdeh0154d67::_waitBusy(const char* message, uint16_t busy_time){
  ESP_LOGD(TAG, "_waitBusy for %s", message);
  int64_t time_since_boot = esp_timer_get_time();
  // On high is busy
  if (gpio_get_level((gpio_num_t)CONFIG_EINK_BUSY) == 1) {
    while (1){
      if (gpio_get_level((gpio_num_t)CONFIG_EINK_BUSY) == 0)
        break;
      vTaskDelay(1);
      if (esp_timer_get_time() - time_since_boot > 7000000)
        break;
    }
  } else
    vTaskDelay(busy_time/portTICK_PERIOD_MS);
}

void Gdeh0154d67::_waitBusy(const char* message){
  ESP_LOGI(TAG, "_waitBusy for %s", message);
  int64_t time_since_boot = esp_timer_get_time();
  while (1) {
    // On low is not busy anymore
    if (gpio_get_level((gpio_num_t)CONFIG_EINK_BUSY) == 0)
      break;
    vTaskDelay(1);
    if (esp_timer_get_time()-time_since_boot>7000000)
      break;
  }
}

void Gdeh0154d67::deepSleep() {
  IO.cmd(0x10);
  IO.data(0x1);
  _deeply_sleeping = true;
}

void Gdeh0154d67::_rotate(int16_t& x, int16_t& y, int16_t& w, int16_t& h)
{
  switch (getRotation())
  {
    case 1:
      swap(x, y);
      swap(w, h);
      x = GDEH0154D67_WIDTH - x - w - 1;
      break;
    case 2:
      x = GDEH0154D67_WIDTH - x - w - 1;
      y = GDEH0154D67_HEIGHT - y - h - 1;
      break;
    case 3:
      swap(x, y);
      swap(w, h);
      y = GDEH0154D67_HEIGHT - y - h - 1;
      break;
  }
}


void Gdeh0154d67::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x < 0) || (x >= width()) || (y < 0) || (y >= height())) return;

  // check rotation, move pixel around if necessary
  switch (getRotation())
  {
    case 1:
      swap(x, y);
      x = GDEH0154D67_WIDTH - x - 1;
      break;
    case 2:
      x = GDEH0154D67_WIDTH - x - 1;
      y = GDEH0154D67_HEIGHT - y - 1;
      break;
    case 3:
      swap(x, y);
      y = GDEH0154D67_HEIGHT - y - 1;
      break;
  }
  uint16_t i = x / 8 + y * GDEH0154D67_WIDTH / 8;

  // This is the trick to draw colors right. Genious Jean-Marc
  if (color)
    _buffer[i] = (_buffer[i] & (0xFF ^ (1 << (7 - x % 8))));
  else
    _buffer[i] = (_buffer[i] | (1 << (7 - x % 8)));
}
