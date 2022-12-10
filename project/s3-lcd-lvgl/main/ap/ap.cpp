/*
 * ap.cpp
 *
 *  Created on: 2021. 1. 9.
 *      Author: baram
 */




#include "ap.h"
#include "lvgl/lvgl_main.h"


static void cliThread(void *args);
static uint8_t getDemoMode(void);



void apInit(void)
{
  cliOpen(_DEF_UART1, 115200);


  if (xTaskCreate(cliThread, "cliThread", _HW_DEF_RTOS_THREAD_MEM_CLI, NULL, _HW_DEF_RTOS_THREAD_PRI_CLI, NULL) != pdPASS)
  {
    logPrintf("[NG] cliThread()\n");   
  }  

  delay(500);
  logBoot(false);
}

void apMain(void)
{
  uint32_t pre_time;


  while(1)
  {
    touch_info_t info;

    if (touchGetInfo(&info))
    {
      if (info.count >= 2)
      {
        break;
      }
    }
    delay(1);
  }

  lcdClear(black);
  
  lvglInit();
  lvglMainInit();

  logPrintf("lvglMain() begin\n");
  pre_time = millis();
  while(1)
  {
    if (millis()-pre_time >= 500)
    {
      pre_time = millis();
    }
    delay(1);   

    lvglUpdate();
  }
}

uint8_t getDemoMode(void)
{
  uint8_t ret = 0;
  esp_err_t err;
  nvs_handle_t nvs_h;


  err = nvs_open("storage", NVS_READWRITE, &nvs_h);
  if (err == ESP_OK)
  {
    uint8_t mode = 0;

    if (nvs_get_u8(nvs_h, "demo_mode", &mode) != ESP_ERR_NVS_NOT_FOUND)
    {
      ret = mode;
    }

    mode = (mode + 1) % 4;

    nvs_set_u8(nvs_h, "demo_mode", mode);
    nvs_commit(nvs_h);

    nvs_close(nvs_h);
  }

  return ret;
}

void cliThread(void *args)
{
  while(1)
  {
    cliMain();
    delay(2);
  }
}
