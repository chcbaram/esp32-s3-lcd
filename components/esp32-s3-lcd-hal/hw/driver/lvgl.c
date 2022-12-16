#include "lvgl.h"

#ifdef _USE_HW_LVGL
#include "lvgl/lv_port_disp.h"
#include "lvgl/lv_port_indev.h"
#include "lcd/ili9481.h"
#include "driver/hangul/han.h"


#define LVGL_FONT_MAX       2


typedef struct 
{
  lv_font_t   *p_lv_font;

  uint8_t    font_w;
  uint8_t    font_h;
  han_font_t font_buf;
} lvgl_font_t;


static void lvglFontInit(void);


static bool is_init = false;

const lv_font_t lv_han_font_16;
const lv_font_t lv_han_font_32;

static lvgl_font_t han_font[LVGL_FONT_MAX]; 


bool lvglInit(void)
{

  if (is_init == true) return true;

  lv_init();
  lv_port_disp_init();
  lv_port_indev_init();

  ili9481SetRotate(true);
  lvglFontInit();

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
  lv_port_disp_init(); 
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

bool hanFont_dsc_cb(const lv_font_t * font, lv_font_glyph_dsc_t * dsc_out, uint32_t unicode_letter, uint32_t unicode_letter_next)
{
  lvgl_font_t *p_han = (lvgl_font_t *)font->user_data;


  if (unicode_letter > 0xFF)
  {
    if (unicode_letter < 0xAC00) return false;
    if (unicode_letter > 0xD7AF) return false;
  }

  if (unicode_letter <= 0xFF)
  {
    dsc_out->adv_w = p_han->font_w/2;   /*Horizontal space required by the glyph in [px]*/
    dsc_out->box_w = p_han->font_w/2;   /*Width of the bitmap in [px]*/
  }
  else
  {
    dsc_out->adv_w = p_han->font_w/1;   /*Horizontal space required by the glyph in [px]*/
    dsc_out->box_w = p_han->font_w/1;   /*Width of the bitmap in [px]*/
  }
  dsc_out->box_h = p_han->font_h;       /*Height of the bitmap in [px]*/

  dsc_out->ofs_x = 0;                   /*X offset of the bitmap in [pf]*/
  dsc_out->ofs_y = 0;                   /*Y offset of the bitmap measured from the as line*/
  dsc_out->bpp   = 1;                   /*Bits per pixel: 1/2/4/8*/
  dsc_out->is_placeholder = false;

  return true;  
}

const uint8_t *hanFont_bitmap_cb(const lv_font_t * font, uint32_t unicode_letter)
{
  lvgl_font_t *p_han = (lvgl_font_t *)font->user_data;


  hanFontLoadUTF16(unicode_letter, &p_han->font_buf);

  return p_han->font_buf.FontBuffer;  
}

void lvglFontInit(void)
{
  lvgl_font_t *p_font;

  p_font = &han_font[0];
  p_font->font_w = 16;
  p_font->font_h = 16;
  p_font->p_lv_font = (lv_font_t *)&lv_han_font_16;
  p_font->p_lv_font->get_glyph_dsc = hanFont_dsc_cb;         /*Set a callback to get info about glyphs*/
  p_font->p_lv_font->get_glyph_bitmap = hanFont_bitmap_cb;   /*Set a callback to get bitmap of a glyph*/
  p_font->p_lv_font->line_height = 16;                       /*The real line height where any text fits*/
  p_font->p_lv_font->base_line = 0;                          /*Base line measured from the top of line_height*/
  p_font->p_lv_font->dsc = NULL;                             /*Store any implementation specific data here*/
  p_font->p_lv_font->user_data = p_font;                     /*Optionally some extra user data*/
  p_font->p_lv_font->subpx = LV_FONT_SUBPX_NONE;
  p_font->p_lv_font->fallback = &lv_font_montserrat_14;

  p_font = &han_font[1];
  p_font->font_w = 32;
  p_font->font_h = 32;
  p_font->p_lv_font = (lv_font_t *)&lv_han_font_32;
  p_font->p_lv_font->get_glyph_dsc = hanFont_dsc_cb;         /*Set a callback to get info about glyphs*/
  p_font->p_lv_font->get_glyph_bitmap = hanFont_bitmap_cb;   /*Set a callback to get bitmap of a glyph*/
  p_font->p_lv_font->line_height = 16;                       /*The real line height where any text fits*/
  p_font->p_lv_font->base_line = 0;                          /*Base line measured from the top of line_height*/
  p_font->p_lv_font->dsc = NULL;                             /*Store any implementation specific data here*/
  p_font->p_lv_font->user_data = p_font;                     /*Optionally some extra user data*/
  p_font->p_lv_font->subpx = LV_FONT_SUBPX_NONE;
  p_font->p_lv_font->fallback = &lv_font_montserrat_14;
}

lv_font_t *lvglGetFont(LvglFontType_t font_type)
{
  lv_font_t *p_font;

  if (font_type >= LVGL_FONT_MAX) return NULL;
  
  p_font = han_font[font_type].p_lv_font;

  return p_font;
}

#endif