/*
 * ap.cpp
 *
 *  Created on: 2021. 1. 9.
 *      Author: baram
 */




#include "ap.h"
#include "lvgl/lvgl_main.h"
#include "module/audio_play.h"


static void cliThread(void *args);
static void mainThread(void *args);




void apInit(void)
{
  cliOpen(_DEF_UART1, 115200);
  audioPlayInit();


  if (xTaskCreate(cliThread, "cliThread", _HW_DEF_RTOS_THREAD_MEM_CLI, NULL, _HW_DEF_RTOS_THREAD_PRI_CLI, NULL) != pdPASS)
  {
    logPrintf("[NG] cliThread()\n");   
  }  

  if (xTaskCreate(mainThread, "mainThread", 8*1024, NULL, 5, NULL) != pdPASS)
  {
    logPrintf("[NG] mainThread()\n");   
  }  
  
  delay(500);
  logBoot(false);
}

void apMain(void)
{
  uint32_t pre_time;

  pre_time = millis();
  while(1)
  {
    if (millis()-pre_time >= 500)
    {
      pre_time = millis();
    }
    delay(1);   
  }
}

void mainThread(void *args)
{
  uint32_t pre_time;


  // pre_time = millis();
  // while(1)
  // {
  //   touch_info_t info;

  //   if (touchGetInfo(&info))
  //   {
  //     if (info.count > 0)
  //     {
  //       if (millis()-pre_time > 200)
  //         break;
  //     }
  //     else
  //     {
  //       pre_time = millis();
  //     }
  //   }
  //   delay(10);
  // }
  delay(2000);

  // while(1)
  // {
  //   delay(1);
  // }
  touchSetEnable(false);

  lcdClear(black);

  lvglInit();
  lvglMainInit();
  
  btHidhBegin();
  btHidhConnect();
  btHidhReconnect(true);
  
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

void cliThread(void *args)
{
  while(1)
  {
    cliMain();
    delay(2);
  }
}
