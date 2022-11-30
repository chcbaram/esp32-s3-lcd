#include "lcd/ili9481.h"
#include "cli.h"


#ifdef _USE_HW_ILI9481
#include "lcdc.h"






static void cliCmd(cli_args_t *args);
static bool writeParam(uint8_t cmd, const void *param, uint32_t length);
static bool writeBuffer(const void *buffer, uint32_t length, uint32_t timeout_ms);
static bool writeBufferPoll(const void *buffer, uint32_t length, uint32_t timeout_ms);
static bool ili9481InitRegs(void);


static bool is_init = false;


uint16_t  *img_buf = NULL;




bool ili9481Init(void)
{
  bool ret = false;


  logPrintf("[__] ili9481Init() {\n");


  ret = lcdcBegin(ILI9481_WIDTH, ILI9481_HEIGHT, 16, 10); // 16bit, 10Mhz
  logPrintf("[%s] lcdcBegin()\n", ret ? "OK":"NG");
  if (ret == false)
  {
    return false;
  }

  ili9481InitRegs();


  img_buf = heap_caps_aligned_alloc(64, 
                                    ILI9481_WIDTH * ILI9481_HEIGHT * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  assert(img_buf);

  writeParam(ILI9481_W_WRITE_MEMORY_START, NULL, 0);

  #if 0
  for (int i=1; i<480*320/4; i+=1)
  {
    img_buf[0] = 0xF800;
    if (i%480 > 240)
      img_buf[0] = 0xFFFF;

    writeBufferPoll(&img_buf[0], 2, 100);
  }
  for (int i=0; i<480*320/4; i+=1)
  {
    img_buf[0] = 0x07E0;
    writeBufferPoll(&img_buf[0], 2, 100);
  }
  for (int i=0; i<480*320/4; i+=1)
  {
    img_buf[0] = 0x001F;
    writeBufferPoll(&img_buf[0], 2, 100);
  }
  for (int i=0; i<480*320/4; i+=1)
  {
    img_buf[0] = 0x0000;
    writeBufferPoll(&img_buf[0], 2, 100);
  }
  #else
  uint32_t pre_time;
  uint32_t exe_time;

  for (int cnt=0; cnt<10; cnt++)
  {
    uint16_t color[3] = {0xF800, 0x07E0, 0x001F};

    for (int i=0; i<480*320; i++)
    {
      img_buf[i] = color[cnt%3];
    }
    pre_time = micros();
    writeBuffer(img_buf, 480*320*sizeof(uint16_t), 500);
    exe_time = micros()-pre_time;
    logPrintf("draw time : %d us, %d fps\n", exe_time, 1000000/exe_time);

    delay(500);
  }

  #endif
  is_init = ret;

  logPrintf("[%s] }\n", ret ? "OK":"NG");
  cliAdd("ili9481", cliCmd);
  return true;
}

bool ili9481InitRegs(void)
{
  uint8_t buf[4];

  writeParam(ILI9481_C_SOFT_RESET,        NULL, 0);
  delay(50);
  writeParam(ILI9481_C_EXIT_SLEEP_MODE,   NULL, 0);
  delay(100);

  writeParam(ILI9481_W_SET_ADDR_MODE,    (uint8_t[]){0x28}, 1);
  writeParam(ILI9481_W_SET_PIXEL_FORMAT, (uint8_t[]){0x55}, 1);
  writeParam(ILI9481_C_ENTER_INVERT_MODE, NULL, 0);
  writeParam(ILI9481_WR_POWER_SET,       (uint8_t[]){0x07, 0x42, 0x15}, 3);


  writeParam(ILI9481_WR_FRAME_RATE_INV_CONTROL,(uint8_t[]){3}, 1); // 72Hz
  buf[0] = ((ILI9481_HEIGHT/2) >> 8) & 0xFF;
  buf[1] = ((ILI9481_HEIGHT/2) >> 0) & 0xFF;
  writeParam(ILI9481_W_SET_TEAR_SCANLINE, buf, 2);
  writeParam(ILI9481_W_SET_TEAR_ON,      (uint8_t[]){0}, 1);

  writeParam(ILI9481_C_SET_DISPLAY_ON,    NULL, 0);

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

  return true;
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
  write.data_width  = 16;
  write.data_length = length;

  ret = lcdcWrite(&write, timeout_ms);

  return ret;
}

bool writeBufferPoll(const void *buffer, uint32_t length, uint32_t timeout_ms)
{
  bool ret;
  lcdc_write_t write;

  write.cmd         = 0;
  write.cmd_width   = 8;
  write.cmd_length  = 0;
  write.p_data      = buffer;
  write.data_width  = 16;
  write.data_length = length;

  ret = lcdcWritePoll(&write, timeout_ms);

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