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

bool lvglSuspend(void)
{
  ili9481SetRotate(false);
  return true;
}

bool lvglResume(void)
{
  ili9481SetRotate(true);  
  return true;
}

bool lvglUpdate(void)
{
  if (is_init == false)
    return false;

  lv_task_handler();
  return true;
}

void *lvglMalloc(size_t size)
{
  void *ret;

  ret = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  return ret;
}

void *lvglRealloc(void * p, size_t new_size)
{
  void *ret;

  ret = heap_caps_realloc(p, new_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  return ret;
}

void lvglFree(void * p)
{
  heap_caps_free(p);
}

#endif