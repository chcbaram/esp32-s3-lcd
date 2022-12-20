#include "audio_play.h"

#include "esp_dsp.h"
#include <math.h>


#define lock()      xSemaphoreTake(mutex_lock, portMAX_DELAY);
#define unLock()    xSemaphoreGive(mutex_lock);


#define N_SAMPLES       1024
#define BLOCK_X_CNT     18
#define BLOCK_Y_CNT     24



typedef struct
{
  uint8_t update_cnt;
  uint8_t block_target[BLOCK_X_CNT];
  uint8_t block_peak[BLOCK_X_CNT];
  uint8_t block_value[BLOCK_X_CNT];
} block_t;


static void cliCmd(cli_args_t *args);
static void drawBlock(int16_t bx, int16_t by, uint16_t color);
static void playCallBack(void *p_data, uint32_t frame_len);


static SemaphoreHandle_t mutex_lock;
static bool is_in_update = false; 
static uint32_t x_in_index = 0;

__attribute__((aligned(16))) float  x_in[N_SAMPLES];
__attribute__((aligned(16))) float  wind[N_SAMPLES];   // Window coefficients
__attribute__((aligned(16))) float  y_cf[N_SAMPLES*2]; // working complex array
__attribute__((aligned(16))) float* r_cf = &y_cf[0];   // Pointers to result arrays

block_t block_info;
LVGL_IMG_DEF(mouse_cursor_img);
static image_t mouse_cursor;
static int16_t mouse_x = LCD_WIDTH/2;
static int16_t mouse_y = LCD_HEIGHT/2;






bool audioPlayInit(void)
{
  esp_err_t esp_ret;
  mutex_lock = xSemaphoreCreateMutex();


  esp_ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
  if (esp_ret  != ESP_OK)
  {
    logPrintf("Not possible to initialize FFT. Error = %i", esp_ret);
    return false;
  }

  mouse_cursor = lcdCreateImage(&mouse_cursor_img, 0, 0, 0, 0);

  cliAdd("audio-play", cliCmd);
  return true;
}

bool audioPlayFile(const char *file_name)
{
  char file_path[128];
  uint32_t pre_time;
  uint32_t pre_time_fft;
  uint32_t pre_time_volume;
  uint32_t fft_pre_time;
  uint32_t fft_exe_time = 0;    
  audio_t audio;
  uint8_t mode = 0;
  touch_info_t touch_info;
  bt_hidh_mouse_info_t mouse_info;

  block_info.update_cnt = 0;
  memset(block_info.block_peak, 0, sizeof(block_info.block_peak));
  memset(block_info.block_value, 0, sizeof(block_info.block_value));
  memset(block_info.block_target, 0, sizeof(block_info.block_target));



  sprintf(file_path, "/sdcard/%s", file_name);
  audioOpen(&audio);
  audioSetWriteCallBack(&audio, playCallBack);

  cliPrintf("play file %s\n", file_name);
  audioPlayFile(&audio, file_path, false);

  pre_time = millis();
  pre_time_fft = millis();
  pre_time_volume = millis();
  while(audioIsPlaying(&audio))
  {
    if (cliAvailable() > 0)
    {
      uint8_t rx;

      rx = cliRead();

      if (rx == '0')
      {
        mode = 0;
      }
      else if (rx == '1')
      {
        mode = 1;
      }
      else
      {
        buzzerBeep(100);
        delay(200);
        break;
      }
    }
    
    touchGetInfo(&touch_info);
    if (touch_info.count >= 2)
    {
      break;
    }
    if (btHidhMouseAvailable() > 0)
    {
      btHidhMouseRead(&mouse_info);
      if (mouse_info.btn & 0x08 || mouse_info.btn & 0x02)
      {
        break;
      }
      mouse_x += mouse_info.x;
      mouse_y += mouse_info.y;
      mouse_x = constrain(mouse_x, 0, LCD_WIDTH-3);
      mouse_y = constrain(mouse_y, 0, LCD_HEIGHT-3);
    }
    if (mouse_info.btn & 0x01)
    {
      touch_info.count = 1;
      touch_info.point[0].id = 0;
      touch_info.point[0].x = mouse_x;
      touch_info.point[0].y = mouse_y;
    }
    delay(1);

    if (is_in_update == true)
    {
      if (millis()-pre_time_fft >= 50)
      {
        pre_time_fft = millis();

        // Generate hann window
        //
        //dsps_wind_hann_f32(wind, N_SAMPLES);
        //dsps_wind_blackman_harris_f32(wind, N_SAMPLES);
        dsps_wind_nuttall_f32(wind, N_SAMPLES);
        
        // Set Input
        //
        for (int i=0; i<N_SAMPLES; i++)
        {
          y_cf[i*2 + 0] = x_in[i] * wind[i];
          y_cf[i*2 + 1] = 0;
        } 


        fft_pre_time = micros();
        dsps_fft2r_fc32(y_cf, N_SAMPLES);     // FFT    
        dsps_bit_rev_fc32(y_cf, N_SAMPLES);   // Bit reverse
        dsps_cplx2reC_fc32(y_cf, N_SAMPLES);  // Convert one complex vector to two complex vectors

        for (int i = 0 ; i < N_SAMPLES/2 ; i++) 
        {
          r_cf[i] = (abs(y_cf[i*2+0]) + abs(y_cf[i*2+1]))/2;
        }          
        fft_exe_time = micros()-fft_pre_time;
      }
      is_in_update = false;
    }

    if (lcdDrawAvailable() == true && millis()-pre_time >= (1000/30))
    {
      pre_time = millis();

      lcdClearBuffer(black);
      lcdPrintf(5,16*0, green, "PLAY FILE : %s, fft %4d us, vol %d %%", file_name, fft_exe_time, audioGetVolume());
      lcdPrintf(5,16*1, green, "            %d fps, %d ms", lcdGetFps(), lcdGetFpsTime());

      if (btHidhIsConnect() == true)
      {
        lcdDrawImage(&mouse_cursor, mouse_x, mouse_y);
      }

      if (mode == 0)
      {
        int16_t xi;

        xi = 0;
        for (int i=0; i<BLOCK_X_CNT; i++)
        {
          int16_t h;
          int16_t max_h;

          max_h = 0;
          for (int j=0; j<N_SAMPLES/2/BLOCK_X_CNT; j++)
          {
            h = (int16_t)constrain(r_cf[xi]/1000, 0, 1000);
            h = cmap(h, 0, 1000, 0, 80);
            if (h > max_h)
            {
              max_h = h;
            }
            xi++;
          }
          h = cmap(max_h, 0, 80, 0, BLOCK_Y_CNT-1);

          block_info.block_target[i] = h;

          if (block_info.update_cnt%4 == 0)
          {
            if (block_info.block_peak[i] > 0)
            {
              block_info.block_peak[i]--;
            }
          }
          if (h >= block_info.block_peak[i])
          {
            block_info.block_peak[i] = h;
            block_info.block_value[i] = h;
          }
        }

        block_info.update_cnt++;

        for (int i=0; i<BLOCK_X_CNT; i++)
        {
          drawBlock(i, block_info.block_peak[i], red);

          if (block_info.block_value[i] > block_info.block_target[i])
          {
  
            block_info.block_value[i]--;
          }
          for (int j=0; j<block_info.block_value[i]; j++)
          {
            drawBlock(i, j, yellow);
          }
        }        
      }
      else
      {
        for (int i=0; i<480; i++)
        {
          int32_t value;

          value = (int32_t)constrain(r_cf[i]/100, 0, 3000);
          value = cmap(value, 0, 3000, 0, 300);

          lcdDrawLine(i, LCD_HEIGHT-1, i, LCD_HEIGHT-value-1, green);
        }
      }

      if (touch_info.count == 1)
      {
        uint32_t vol_len;
        uint32_t bar_len = 200;
        uint32_t volume;
        volume = audioGetVolume();

        vol_len = volume*bar_len/100;

        lcdDrawFillRect(10, 50, 50, bar_len, white);
        lcdDrawFillRect(10, 50+(bar_len-vol_len), 50, vol_len, red);

        int16_t x;
        int16_t y;

        x = touch_info.point[0].x;
        y = touch_info.point[0].y;

        if (x > (LCD_WIDTH - 100))
        {
          if (y < LCD_HEIGHT/2)
            lcdDrawFillRect(LCD_WIDTH-100, 0, 100, LCD_HEIGHT/2, green);
          else
            lcdDrawFillRect(LCD_WIDTH-100, LCD_HEIGHT/2, 100, LCD_HEIGHT/2, red);
        }

        if (millis()-pre_time_volume >= 50)
        {
          pre_time_volume = millis();

          if (x > (LCD_WIDTH - 100))
          {
            if (y < LCD_HEIGHT/2)
            {
              volume++;
            }
            else
            {
              if (volume > 0)
                volume--;
            }
            audioSetVolume(volume);
          }
        }
      }

      lcdRequestDraw();    
    }
  }

  audioClose(&audio);
  lcdClear(black);
  lcdClear(black);

  
  return true;
}


void IRAM_ATTR playCallBack(void *p_data, uint32_t frame_len)
{
  if (is_in_update == false)
  {
    int16_t *p_buf = (int16_t *)p_data;

    for (int i=0; i<frame_len; i++)
    {      
      x_in[x_in_index] = (float)p_buf[i];
      x_in_index++;
      if (x_in_index >= N_SAMPLES)
      {
        x_in_index = 0;
        is_in_update = true;
        break;
      }
    }
  }
  else
  {
    x_in_index = 0;
  }
}

void drawBlock(int16_t bx, int16_t by, uint16_t color)
{
  int16_t x;
  int16_t y;
  int16_t bw;
  int16_t bh;
  int16_t top_space = 32;
  int16_t sw;
  int16_t sh;

  sw = lcdGetWidth();
  sh = lcdGetHeight()-top_space;

  bw = (sw / BLOCK_X_CNT);
  bh = (sh / BLOCK_Y_CNT);

  x = bx*bw;
  y = sh - bh*by - bh;

  lcdDrawFillRect(x, y+top_space, bw-2, bh-2, color);
}

void cliCmd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 2 && args->isStr(0, "play"))
  {
    const char *file_name;
    char file_path[128];
    uint32_t pre_time;
    uint32_t pre_time_fft;
    uint32_t fft_pre_time;
    uint32_t fft_exe_time = 0;    
    audio_t audio;
    uint8_t mode = 0;


    file_name = args->getStr(1);

    block_info.update_cnt = 0;
    memset(block_info.block_peak, 0, sizeof(block_info.block_peak));
    memset(block_info.block_value, 0, sizeof(block_info.block_value));
    memset(block_info.block_target, 0, sizeof(block_info.block_target));


    sprintf(file_path, "/sdcard/%s", file_name);
    audioOpen(&audio);
    audioSetWriteCallBack(&audio, playCallBack);

    cliPrintf("play file %s\n", file_name);
    audioPlayFile(&audio, file_path, false);

    pre_time = millis();
    pre_time_fft = millis();
    while(audioIsPlaying(&audio))
    {
      if (cliAvailable() > 0)
      {
        uint8_t rx;

        rx = cliRead();

        if (rx == '0')
        {
          mode = 0;
        }
        else if (rx == '1')
        {
          mode = 1;
        }
        else
        {
          buzzerBeep(100);
          delay(200);
          break;
        }
      }
      delay(1);

      if (is_in_update == true)
      {
        if (millis()-pre_time_fft >= 50)
        {
          pre_time_fft = millis();

          // Generate hann window
          //
          //dsps_wind_hann_f32(wind, N_SAMPLES);
          //dsps_wind_blackman_harris_f32(wind, N_SAMPLES);
          dsps_wind_nuttall_f32(wind, N_SAMPLES);
          
          // Set Input
          //
          for (int i=0; i<N_SAMPLES; i++)
          {
            y_cf[i*2 + 0] = x_in[i] * wind[i];
            y_cf[i*2 + 1] = 0;
          } 


          fft_pre_time = micros();
          dsps_fft2r_fc32(y_cf, N_SAMPLES);     // FFT    
          dsps_bit_rev_fc32(y_cf, N_SAMPLES);   // Bit reverse
          dsps_cplx2reC_fc32(y_cf, N_SAMPLES);  // Convert one complex vector to two complex vectors

          for (int i = 0 ; i < N_SAMPLES/2 ; i++) 
          {
            r_cf[i] = (abs(y_cf[i*2+0]) + abs(y_cf[i*2+1]))/2;
          }          
          fft_exe_time = micros()-fft_pre_time;
        }
        is_in_update = false;
      }

      if (lcdDrawAvailable() == true && millis()-pre_time >= (1000/30))
      {
        pre_time = millis();

        lcdClearBuffer(black);
        lcdPrintf(5,16*0, green, "PLAY FILE : %s, fft %d us", file_name, fft_exe_time);


        if (mode == 0)
        {
          int16_t xi;

          xi = 0;
          for (int i=0; i<BLOCK_X_CNT; i++)
          {
            int16_t h;
            int16_t max_h;

            max_h = 0;
            for (int j=0; j<N_SAMPLES/2/BLOCK_X_CNT; j++)
            {
              h = (int16_t)constrain(r_cf[xi]/1000, 0, 1000);
              h = cmap(h, 0, 1000, 0, 80);
              if (h > max_h)
              {
                max_h = h;
              }
              xi++;
            }
            h = cmap(max_h, 0, 80, 0, BLOCK_Y_CNT-1);

            block_info.block_target[i] = h;

            if (block_info.update_cnt%4 == 0)
            {
              if (block_info.block_peak[i] > 0)
              {
                block_info.block_peak[i]--;
              }
            }
            if (h >= block_info.block_peak[i])
            {
              block_info.block_peak[i] = h;
              block_info.block_value[i] = h;
            }
          }

          block_info.update_cnt++;

          for (int i=0; i<BLOCK_X_CNT; i++)
          {
            drawBlock(i, block_info.block_peak[i], red);

            if (block_info.block_value[i] > block_info.block_target[i])
            {
              block_info.block_value[i]--;
            }
            for (int j=0; j<block_info.block_value[i]; j++)
            {
              drawBlock(i, j, yellow);
            }
          }        
        }
        else
        {
          for (int i=0; i<480; i++)
          {
            int32_t value;

            value = (int32_t)constrain(r_cf[i]/100, 0, 3000);
            value = cmap(value, 0, 3000, 0, 300);

            lcdDrawLine(i, LCD_HEIGHT-1, i, LCD_HEIGHT-value-1, green);
          }
        }
        lcdRequestDraw();    
      }
    }

    audioClose(&audio);
    lcdClear(black);
    lcdClear(black);

    ret = true;
  }

  if (ret == false)
  {
    cliPrintf("audio-play play file.wav\n");
  }
}