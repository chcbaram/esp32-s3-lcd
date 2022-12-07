#include "lvgl.h"

#ifdef _USE_HW_LVGL
#include "lvgl/lv_port_disp.h"
#include "lvgl/lv_port_indev.h"
#include "lcd/ili9481.h"


static bool is_init = false;





bool lvglInit(void)
{

  if (is_init == true) return true;

  lv_init();
  lv_port_disp_init();
  lv_port_indev_init();

  ili9481SetRotate(true);

  is_init = true;

  return true;
}

bool lvglDeInit(void)
{
  ili9481SetRotate(false);
  return true;
}

bool lvglUpdate(void)
{
  if (is_init == false)
    return false;

  lv_task_handler();
  return true;
}

#endif