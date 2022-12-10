#include "lv_port_indev.h"
#include "touch.h"



static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);





lv_indev_t * indev_touchpad;





void lv_port_indev_init(void)
{
  /**
   * Here you will find example implementation of input devices supported by LittelvGL:
   *  - Touchpad
   *  - Mouse (with cursor support)
   *  - Keypad (supports GUI usage only with key)
   *  - Encoder (supports GUI usage only with: left, right, push)
   *  - Button (external buttons to press points on the screen)
   *
   *  The `..._read()` function are only examples.
   *  You should shape them according to your hardware
   */

  static lv_indev_drv_t indev_drv;

  /*------------------
    * Touchpad
    * -----------------*/

  /*Register a touchpad input device*/
  lv_indev_drv_init(&indev_drv);


  indev_drv.gesture_limit        = 20;
  indev_drv.gesture_min_velocity = 2;

  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touchpad_read;
  indev_touchpad = lv_indev_drv_register(&indev_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*------------------
 * Touchpad
 * -----------------*/


/*Will be called by the library to read the touchpad*/
static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
  static lv_coord_t last_x = 0;
  static lv_coord_t last_y = 0;
  static int8_t    last_id = -1;
  touch_info_t info;
  bool is_pressed = false;

  info.count = 0;
  touchGetInfo(&info);

  for (int i=0; i<info.count; i++)
  {
    if (last_id < 0)
    {
      last_id = info.point[i].id;
      last_x = info.point[i].x;
      last_y = info.point[i].y;
      is_pressed = true;
      break;
    }
    else
    {
      if (info.point[i].id == last_id)
      {
        last_x = info.point[i].x;
        last_y = info.point[i].y;
        is_pressed = true;
        break;
      }
    }
  }

  if(is_pressed == true) 
  {
    data->state = LV_INDEV_STATE_PR;
  }
  else 
  {
    last_id = -1;
    data->state = LV_INDEV_STATE_REL;
  }

  /*Set the last pressed coordinates*/
  data->point.x = last_x;
  data->point.y = last_y;
}
