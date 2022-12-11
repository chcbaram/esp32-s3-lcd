#include "lv_port_indev.h"
#include "touch.h"
#include "bt_hidh.h"


static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
static void mouse_init(void);
static void mouse_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);



lv_indev_t * indev_touchpad;
lv_indev_t * indev_mouse;
LV_IMG_DECLARE(mouse_cursor_icon); 
static bool is_mouse_pressed = false;


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

  static lv_indev_drv_t indev_drv_touch;
  static lv_indev_drv_t indev_drv_mouse;

  /*------------------
    * Touchpad
    * -----------------*/

  /*Register a touchpad input device*/
  lv_indev_drv_init(&indev_drv_touch);


  indev_drv_touch.gesture_limit        = 20;
  indev_drv_touch.gesture_min_velocity = 2;

  indev_drv_touch.type = LV_INDEV_TYPE_POINTER;
  indev_drv_touch.read_cb = touchpad_read;
  indev_touchpad = lv_indev_drv_register(&indev_drv_touch);


  /*------------------
    * Mouse
    * -----------------*/

  /*Initialize your mouse if you have*/
  mouse_init();

  /*Register a mouse input device*/
  lv_indev_drv_init(&indev_drv_mouse);
  indev_drv_mouse.type = LV_INDEV_TYPE_POINTER;
  indev_drv_mouse.read_cb = mouse_read;
  indev_mouse = lv_indev_drv_register(&indev_drv_mouse);

  /*Set cursor. For simplicity set a HOME symbol now.*/
  lv_obj_t * mouse_cursor = lv_img_create(lv_scr_act());
  lv_img_set_src(mouse_cursor, &mouse_cursor_icon);
  lv_indev_set_cursor(indev_mouse, mouse_cursor);  
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


  // if (is_mouse_used == true)
  // {
  //   return;
  // }

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


/*------------------
 * Mouse
 * -----------------*/

/*Initialize your mouse*/
static void mouse_init(void)
{
  btHidhMouseFlush();
}

/*Will be called by the library to read the mouse*/
static void mouse_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
  bt_hidh_mouse_info_t info;


  while (btHidhMouseAvailable() > 0)
  {
    btHidhMouseRead(&info);

    //if (info.btn & 0x02)
    {

      data->point.x += info.x;
      data->point.y += info.y;

      data->point.x = constrain(data->point.x, 0, HW_LCD_WIDTH-3);
      data->point.y = constrain(data->point.y, 0, HW_LCD_HEIGHT-3);

      if(info.btn & 0x01) 
      {
        is_mouse_pressed = true;
      }
      else 
      {
        is_mouse_pressed = false;
      }
    }
  }

  if (is_mouse_pressed == true)
  {
    data->state = LV_INDEV_STATE_PR;;
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}