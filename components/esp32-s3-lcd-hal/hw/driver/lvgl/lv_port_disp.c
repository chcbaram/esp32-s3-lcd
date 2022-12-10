#include "lv_port_disp.h"
#include "lcd.h"



static bool is_init = false;

static void disp_init(void);
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//        const lv_area_t * fill_area, lv_color_t color);



void lv_port_disp_init(void)
{
  /*-------------------------
    * Initialize your display
    * -----------------------*/
  disp_init();

  /*-----------------------------
    * Create a buffer for drawing
    *----------------------------*/

  static lv_disp_draw_buf_t draw_buf_dsc;
  static lv_color_t *p_buf_1;
  static lv_color_t *p_buf_2;

  p_buf_1 = (lv_color_t *)lcdGetFrameBuffer();
  p_buf_2 = (lv_color_t *)lcdGetCurrentFrameBuffer();

  lv_disp_draw_buf_init(&draw_buf_dsc, p_buf_1, p_buf_2, LCD_WIDTH * LCD_HEIGHT);  


  if (is_init) return;

  /*-----------------------------------
    * Register the display in LVGL
    *----------------------------------*/

  static lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
  lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

  /*Set up the functions to access to your display*/

  /*Set the resolution of the display*/
  disp_drv.hor_res = LCD_WIDTH;
  disp_drv.ver_res = LCD_HEIGHT;


  /*Used to copy the buffer's content to the display*/
  disp_drv.flush_cb = disp_flush;

  /*Set a display buffer*/
  disp_drv.draw_buf = &draw_buf_dsc;

  disp_drv.full_refresh = 1;

  /* Fill a memory array with a color if you have GPU.
    * Note that, in lv_conf.h you can enable GPUs that has built-in support in LVGL.
    * But if you have a different GPU you can use with this callback.*/
  //disp_drv.gpu_fill_cb = gpu_fill;

  /*Finally register the driver*/
  lv_disp_drv_register(&disp_drv);  

  is_init = true;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    /*You code here*/
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/*Flush the content of the internal buffer the specific area on the display
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_disp_flush_ready()' has to be called when finished.*/
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
  #if 0
  if(disp_flush_enabled) 
  {
      /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/

      int32_t x;
      int32_t y;
      for(y = area->y1; y <= area->y2; y++) {
          for(x = area->x1; x <= area->x2; x++) {
              /*Put a pixel to the display. For example:*/
              /*put_px(x, y, *color_p)*/
              color_p++;
          }
      }
  }
  logPrintf("disp_flush\n");
  #endif

  lcdRequestDraw();
  while(!lcdDrawAvailable());

  /*IMPORTANT!!!
    *Inform the graphics library that you are ready with the flushing*/
  lv_disp_flush_ready(disp_drv);
}

/*OPTIONAL: GPU INTERFACE*/

/*If your MCU has hardware accelerator (GPU) then you can use it to fill a memory with a color*/
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//                    const lv_area_t * fill_area, lv_color_t color)
//{
//    /*It's an example code which should be done by your GPU*/
//    int32_t x, y;
//    dest_buf += dest_width * fill_area->y1; /*Go to the first line*/
//
//    for(y = fill_area->y1; y <= fill_area->y2; y++) {
//        for(x = fill_area->x1; x <= fill_area->x2; x++) {
//            dest_buf[x] = color;
//        }
//        dest_buf+=dest_width;    /*Go to the next line*/
//    }
//}


