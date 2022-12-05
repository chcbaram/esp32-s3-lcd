#include "lcd/ili9481.h"
#include "cli.h"


#ifdef _USE_HW_ILI9481
#include "lcdc.h"






static void cliCmd(cli_args_t *args);
static bool writeParam(uint8_t cmd, const void *param, uint32_t length);
static bool writeBuffer(const void *buffer, uint32_t length, uint32_t timeout_ms);

static bool ili9481Reset(void);
static bool ili9481InitRegs(void);
static bool ili9481SendBuffer(void *p_data, uint32_t length, uint32_t timeout_ms);
static uint16_t ili9481GetWidth(void);
static uint16_t ili9481GetHeight(void);



static bool    is_init = false;
#if 1
static uint8_t data_bus_width = 8;  // 8bit
static uint8_t data_bus_freq  = 20; // 20Mhz
#else
static uint8_t data_bus_width = 16; // 16bit
static uint8_t data_bus_freq  = 10; // 10Mhz
#endif




bool ili9481Init(void)
{
  bool ret = false;


  logPrintf("[__] ili9481Init() {\n");


  ret = lcdcBegin(ILI9481_WIDTH, ILI9481_HEIGHT, data_bus_width, data_bus_freq); // 8/16bit, Freq Mhz
  logPrintf("[%s] lcdcBegin()\n", ret ? "OK":"NG");
  if (ret == false)
  {
    return false;
  }
  ili9481Reset();


  is_init = ret;

  logPrintf("[%s] }\n", ret ? "OK":"NG");
  cliAdd("ili9481", cliCmd);
  return true;
}

bool ili9481Reset(void)
{
  ili9481InitRegs();
  return true;
}

bool ili9481InitRegs(void)
{
  uint8_t buf[4];

  writeParam(ILI9481_C_SOFT_RESET,        NULL, 0);
  delay(50);
  writeParam(ILI9481_C_EXIT_SLEEP_MODE,   NULL, 0);
  delay(100);

  //writeParam(ILI9481_W_SET_ADDR_MODE,    (uint8_t[]){0x28}, 1);
  writeParam(ILI9481_W_SET_ADDR_MODE,    (uint8_t[]){0x08}, 1);
  writeParam(ILI9481_W_SET_PIXEL_FORMAT, (uint8_t[]){0x55}, 1);
  writeParam(ILI9481_C_ENTER_INVERT_MODE, NULL, 0);
  writeParam(ILI9481_WR_POWER_SET,       (uint8_t[]){0x07, 0x42, 0x15}, 3);



  writeParam(ILI9481_WR_FRAME_RATE_INV_CONTROL,(uint8_t[]){3}, 1); // 72Hz

  buf[0] = ((0) >> 8) & 0xFF;
  buf[1] = ((0) >> 0) & 0xFF;
  writeParam(ILI9481_W_SET_TEAR_SCANLINE, buf, 2);
  writeParam(ILI9481_W_SET_TEAR_ON,      (uint8_t[]){0}, 1);

  buf[0] = 0;
  buf[1] = 0;
  buf[2] = ((ILI9481_WIDTH-1) >> 8) & 0xFF;
  buf[3] = ((ILI9481_WIDTH-1) >> 0) & 0xFF;
  writeParam(ILI9481_W_SET_COLUMN_ADDR, buf, 4);

  buf[0] = 0;
  buf[1] = 0;
  buf[2] = ((ILI9481_HEIGHT-1) >> 8) & 0xFF;
  buf[3] = ((ILI9481_HEIGHT-1) >> 0) & 0xFF;
  writeParam(ILI9481_W_SET_PAGE_ADDR, buf, 4);

  writeParam(ILI9481_C_SET_DISPLAY_ON,    NULL, 0);

  return true;
}

bool ili9481SetCallBack(void (*p_func)(void))
{
  lcdcSetCallBack(p_func);
  return true;
}

void ili9481SetWindow(int32_t x, int32_t y, int32_t w, int32_t h)
{
  uint8_t buf[4];

  buf[0] = x >> 8;
  buf[1] = x >> 0;
  buf[2] = (((x+w)-1) >> 8) & 0xFF;
  buf[3] = (((x+w)-1) >> 0) & 0xFF;
  //writeParam(ILI9481_W_SET_COLUMN_ADDR, buf, 4);
  writeParam(ILI9481_W_SET_PAGE_ADDR, buf, 4);

  buf[0] = y >> 8;
  buf[1] = y >> 0;
  buf[2] = (((y+h)-1) >> 8) & 0xFF;
  buf[3] = (((y+h)-1) >> 0) & 0xFF;
  //writeParam(ILI9481_W_SET_PAGE_ADDR, buf, 4);
  writeParam(ILI9481_W_SET_COLUMN_ADDR, buf, 4);
}

uint16_t ili9481GetWidth(void)
{
  return ILI9481_WIDTH;
}

uint16_t ili9481GetHeight(void)
{
  return ILI9481_HEIGHT;
}

bool ili9481SendBuffer(void *p_data, uint32_t length, uint32_t timeout_ms)
{
  bool ret = true;

  ret &= writeParam(ILI9481_W_WRITE_MEMORY_START, NULL, 0);
  ret &= writeBuffer(p_data, length, timeout_ms);

  return ret;
}
  
lcd_driver_t *ili9481GetDriver(void)
{
  static lcd_driver_t lcd_driver = 
  {
    .init         = ili9481Init,
    .reset        = ili9481Reset,
    .setWindow    = ili9481SetWindow,
    .sendBuffer   = ili9481SendBuffer,
    .getWidth     = ili9481GetWidth,
    .getHeight    = ili9481GetHeight,
    .setCallBack  = ili9481SetCallBack, 
  };

  return &lcd_driver;
}

bool writeParam(uint8_t cmd, const void *param, uint32_t length)
{
  bool ret;
  lcdc_write_t write;

  write.cmd         = cmd;
  write.cmd_width   = 8;
  write.cmd_length  = 1;
  write.p_data      = param;
  write.data_width  = 8;
  write.data_length = length;

  ret = lcdcWritePoll(&write, 50);

  return ret;
}

bool writeBuffer(const void *buffer, uint32_t length, uint32_t timeout_ms)
{
  bool ret;
  lcdc_write_t write;

  write.cmd         = 0;
  write.cmd_width   = 8;
  write.cmd_length  = 0;
  write.p_data      = buffer;
  write.data_width  = data_bus_width;
  write.data_length = length;

  ret = lcdcWrite(&write, timeout_ms);

  return ret;
}

void cliCmd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "info"))
  {
    cliPrintf("is_init : %s\n", is_init ? "True":"False");
    ret = true;
  }



  if (ret == false)
  {
    cliPrintf("ili9481 info\n");
  }
}


#endif