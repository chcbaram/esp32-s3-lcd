#include "lcd.h"
#include "cli.h"


#ifdef _USE_HW_LCD
#include "lcd/ili9481.h"
#include "pwm.h"


#define LCD_OPT_DEF   __attribute__((optimize("O2")))

#define LCD_FRAME_BUF_MAX     2

typedef struct
{
  uint16_t  *buffer[LCD_FRAME_BUF_MAX];
  uint32_t   index;
  uint16_t  *draw_buffer;
} lcd_frame_t;


static void lcdTransferDoneISR(void);
static void cliLcd(cli_args_t *args);
static bool lcdLoadCfg(void);
static bool lcdSaveCfg(void);


static lcd_driver_t *p_lcd_driver = NULL;

static bool is_init = false;
static bool is_request_draw = false;
static lcd_frame_t lcd_frame;
static uint8_t backlight_value = 100;





bool lcdInit(void)
{
  bool ret = true;


  p_lcd_driver = ili9481GetDriver();
  p_lcd_driver->init();
  p_lcd_driver->setCallBack(lcdTransferDoneISR);


  lcd_frame.index = 0;
  for (int i=0; i<LCD_FRAME_BUF_MAX; i++)
  {
    lcd_frame.buffer[i] = heap_caps_aligned_alloc(64, 
                                    LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t), 
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (lcd_frame.buffer[i] == NULL)
    {
      logPrintf("[NG] lcd frame buffer\n");
      break;
    }
  }
  is_init = ret;

  if (ret == true)
  {
    lcd_frame.draw_buffer = lcd_frame.buffer[lcd_frame.index];

    for (int i=0; i<LCD_WIDTH*LCD_HEIGHT; i++)
    {
      lcd_frame.draw_buffer[i] = black;
    }

    lcdUpdateDraw();
  }
  logPrintf("[%s] lcdInit()\n", is_init ? "OK":"NG");


  cliAdd("lcd", cliLcd);


  lcdLoadCfg();
  lcdSetBackLight(backlight_value);

  return ret;
}

bool lcdLoadCfg(void)
{
  bool ret = true;
  esp_err_t err;
  nvs_handle_t nvs_h;

  err = nvs_open("storage", NVS_READWRITE, &nvs_h);
  if (err == ESP_OK)
  {
    uint8_t pwm_value = backlight_value;

    if (nvs_get_u8(nvs_h, "lcd_cfg_pwm", &pwm_value) == ESP_ERR_NVS_NOT_FOUND)
    {
      pwm_value = backlight_value;
    }
    backlight_value = pwm_value;
    nvs_close(nvs_h);
  }

  return ret;
}

bool lcdSaveCfg(void)
{
  bool ret = true;
  esp_err_t err;
  nvs_handle_t nvs_h;

  err = nvs_open("storage", NVS_READWRITE, &nvs_h);
  if (err == ESP_OK)
  {
    nvs_set_u8(nvs_h, "lcd_cfg_pwm", backlight_value);
    nvs_commit(nvs_h);
    nvs_close(nvs_h);
  }

  return ret;
}

uint8_t lcdGetBackLight(void)
{
  return backlight_value;
}

void lcdSetBackLight(uint8_t value)
{
  value = constrain((int8_t)value, 0, 100);

  if (value != backlight_value)
  {
    backlight_value = value;
  }

  lcdSaveCfg();
  pwmWrite(0, cmap(value, 0, 100, 0, pwmGetMax(0)));
}

IRAM_ATTR void lcdTransferDoneISR(void)
{
  logPrintf("Done\n"); 
  is_request_draw = false; 
}

bool lcdRequestDraw(void)
{
  if (is_init != true) return false;
  if (is_request_draw == true) return false;


  p_lcd_driver->setWindow(0, 0, LCD_WIDTH, LCD_HEIGHT);

  is_request_draw = true;
  p_lcd_driver->sendBuffer(lcd_frame.draw_buffer, LCD_WIDTH * LCD_HEIGHT * 2, 0);
  return true;
}

void lcdUpdateDraw(void)
{
  uint32_t pre_time;

  if (is_init != true)
  {
    return;
  }

  lcdRequestDraw();

  pre_time = millis();
  while(lcdDrawAvailable() != true)
  {
    delay(1);
    if (millis()-pre_time >= 100)
    {
      break;
    }
  }
}

bool lcdDrawAvailable(void)
{
  bool ret = false;
 
  ret = !is_request_draw;

  return ret;
}

uint16_t *lcdGetFrameBuffer(void)
{
  return (uint16_t *)lcd_frame.draw_buffer;
}

LCD_OPT_DEF void lcdClearBuffer(uint32_t rgb_code)
{
  uint16_t *p_buf = lcdGetFrameBuffer();

  for (int i=0; i<LCD_WIDTH * LCD_HEIGHT; i++)
  {
    p_buf[i] = rgb_code;
  }
}

void cliLcd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "info") == true)
  {
    cliPrintf("Driver : ST7789\n");
    cliPrintf("Width  : %d\n", LCD_WIDTH);
    cliPrintf("Height : %d\n", LCD_HEIGHT);
    cliPrintf("BKL    : %d%%\n", lcdGetBackLight());
    cliPrintf("Free Heap  : %ld KB\n", esp_get_free_heap_size()/1024);
    cliPrintf("Free Heapi : %d KB\n", esp_get_free_internal_heap_size()/1024);
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "draw") == true)
  {
    
    uint32_t pre_time;
    uint32_t exe_time;

    for (int cnt=0; cnt<10; cnt++)
    {
      uint16_t color[3] = {0xF800, 0x07E0, 0x001F};

      lcdClearBuffer(color[cnt%3]);

      pre_time = micros();
      lcdUpdateDraw();
      exe_time = micros()-pre_time;
      logPrintf("draw time : %d us, %d fps\n", exe_time, 1000000/exe_time);

      delay(500);
    }

    ret = true;
  }
 
  if (args->argc == 2 && args->isStr(0, "bl") == true)
  {
    uint8_t bl_value;

    bl_value = args->getData(1);

    lcdSetBackLight(bl_value);

    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("lcd info\n");
    cliPrintf("lcd draw\n");
    cliPrintf("lcd bl 0~100\n");
  }
}
#endif
