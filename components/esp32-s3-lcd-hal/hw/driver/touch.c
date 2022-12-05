#include "touch.h"
#include "touch/ft6236.h"
#include "cli.h"
#include "cli_gui.h"


#ifdef _USE_HW_TOUCH


static void cliCmd(cli_args_t *args);


static bool is_init = false;
static uint16_t touch_width  = 480;
static uint16_t touch_height = 320;



bool touchInit(void)
{
  bool ret;


  ret = ft6236Init();
  if (ret == true)
  {
    touch_width  = ft6236GetWidth();
    touch_height = ft6236GetHeight();
    is_init = true;
  }


  logPrintf("[%s] touchInit()\n", ret ? "OK":"NG");

  cliAdd("touch", cliCmd);

  return ret;
}

bool touchGetInfo(touch_info_t *p_info)
{
  bool ret;
  ft6236_info_t ft6236_info;

  if (is_init == false) return false;

  ret = ft6236GetInfo(&ft6236_info);
  if (ret == true)
  {
    p_info->count = ft6236_info.count;
    for (int i=0; i<ft6236_info.count; i++)
    {
      p_info->point[i].event = ft6236_info.point[i].event;
      p_info->point[i].id    = ft6236_info.point[i].id;
      p_info->point[i].x     = ft6236_info.point[i].x;
      p_info->point[i].y     = ft6236_info.point[i].y;
      p_info->point[i].w     = ft6236_info.point[i].weight;
    }
  }

  return ret;
}

void cliCmd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "info"))
  {
    cliPrintf("is_init      : %s\n", is_init ? "True" : "False");
    cliPrintf("touch max ch : %d\n", TOUCH_MAX_CH);
    cliPrintf("touch width  : %d\n", touch_width);
    cliPrintf("touch height : %d\n", touch_height);
    ret = true;
  }

  if (args->argc == 2 && args->isStr(0, "get") && args->isStr(1, "info"))
  {
    touch_info_t info;
    uint32_t pre_time;
    uint32_t exe_time;

    while(cliKeepLoop())
    {
      pre_time = micros();
      if (touchGetInfo(&info) == true)
      {
        exe_time = micros()-pre_time;

        cliPrintf("cnt : %d %3dus, ", info.count, exe_time);

        for (int i=0; i<info.count; i++)
        {
          cliPrintf(" - ");
          cliPrintf("id=%d evt=%2d x=%3d y=%3d w=%3d ", 
            info.point[i].id,      
            info.point[i].event,      
            info.point[i].x, 
            info.point[i].y, 
            info.point[i].w 
            );
        }

        cliPrintf("\n");
      }
      else
      {
        cliPrintf("touchGetInfo() Fail\n");
        break;
      }
      delay(10);
    }
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "gui"))
  {
    touch_info_t info;
    touch_info_t info_pre;

    cliGui()->initScreen(80, 24);

    while(cliKeepLoop())
    {
      cliGui()->drawBox(0, 0, touch_width/10 + 1, touch_height/20 + 1, "");

      if (touchGetInfo(&info) == true)
      {
        uint16_t x;
        uint16_t y;

        for (int i=0; i<info_pre.count; i++)
        {
          if (info.point[i].x != info_pre.point[i].x || 
              info.point[i].y != info_pre.point[i].y ||
              info.count != info_pre.count)
          {
            x = info_pre.point[i].x/10;
            y = info_pre.point[i].y/20;          
            cliGui()->eraseBox(x, y, 6, 3);
            cliGui()->movePrintf(x+2, y+1, " ");
            cliGui()->movePrintf(x, y+3, "       ");
          }
        }
        for (int i=0; i<info.count; i++)
        {
          x = info.point[i].x/10;
          y = info.point[i].y/20;
          cliGui()->drawBox(x, y, 6, 3, "");
          cliGui()->movePrintf(x+2, y+1, "%d", info.point[i].id);
          cliGui()->movePrintf(x, y+3, "%3d:%3d", info.point[i].x, info.point[i].y);
        }
        info_pre = info;
      }
      delay(10);
    }

    cliGui()->closeScreen();    
    ret = true;
  }

  if (ret == false)
  {
    cliPrintf("touch info\n");
    cliPrintf("touch get info\n");
    cliPrintf("touch gui\n");
  }
}

#endif