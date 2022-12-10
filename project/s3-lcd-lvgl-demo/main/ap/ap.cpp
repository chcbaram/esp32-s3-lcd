/*
 * ap.cpp
 *
 *  Created on: 2021. 1. 9.
 *      Author: baram
 */




#include "ap.h"
#include "benchmark/lv_demo_benchmark.h"
#include "stress/lv_demo_stress.h"
#include "music/lv_demo_music.h"
#include "widgets/lv_demo_widgets.h"


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

  lcdClear(black);

  lvglInit();

  switch(getDemoMode())
  {
    case 0:
      lv_demo_benchmark(LV_DEMO_BENCHMARK_MODE_RENDER_AND_DRIVER);
      break;

    case 1:
      lv_demo_stress();
      break;

    case 2:
      lv_demo_music();
      break;

    case 3:
      lv_demo_widgets();
      break;

    default:
      lv_demo_benchmark(LV_DEMO_BENCHMARK_MODE_RENDER_AND_DRIVER);
      break;
  }

  logPrintf("lv_demo_benchmark() begin\n");
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
