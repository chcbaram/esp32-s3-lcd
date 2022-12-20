#include "lvgl.h"

#ifdef _USE_HW_LVGL
#include "lvgl/lv_port_disp.h"
#include "lvgl/lv_port_indev.h"
#include "lcd/ili9481.h"
#include "driver/hangul/han.h"
#include "resize.h"
#include "cli.h"


#define LVGL_FONT_MAX       5




typedef struct 
{
  lv_font_t   *p_lv_font;

  uint8_t    font_w;
  uint8_t    font_h;
  han_font_t font_buf;
  bool       resize_nearest;
} lvgl_font_t;


static void lvglFontInit(void);
static void cliCmd(cli_args_t *args);


static bool is_init = false;
static bool is_enable = true;

const lv_font_t lv_han_font_16;
const lv_font_t lv_han_font_20;
const lv_font_t lv_han_font_24;
const lv_font_t lv_han_font_28;
const lv_font_t lv_han_font_32;

static lvgl_font_t han_font[LVGL_FONT_MAX]; 
static uint8_t __attribute__((aligned(64))) font_src_buffer[16 * 16];
static uint8_t __attribute__((aligned(64))) font_dst_buffer[64 * 64];



bool lvglInit(void)
{

  if (is_init == true) return true;

  lv_init();
  lv_port_disp_init();
  lv_port_indev_init();

  ili9481SetRotate(true);
  lvglFontInit();

  is_init = true;

  cliAdd("lvgl", cliCmd);
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

  if (is_enable == true)
  {
    lv_task_handler();
  }
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

bool hanFontDraw(lvgl_font_t *p_han, uint32_t unicode_letter)
{
  resize_image_t r_src, r_dst;

  r_src.x = 0;
  r_src.y = 0;
  r_src.w = 16;
  r_src.h = 16;
  r_src.stride = 16;
  r_src.p_data = (uint16_t *)font_src_buffer;

  r_dst.x = 0;
  r_dst.y = 0;
  r_dst.w = p_han->font_w;
  r_dst.h = p_han->font_h;
  r_dst.stride = p_han->font_w;
  r_dst.p_data = (uint16_t *)font_dst_buffer;

  if (unicode_letter <= 0xFF)
  {
    r_src.w = 8;
    r_dst.w = p_han->font_w/2;
    r_dst.stride = p_han->font_w/2;
  }

  hanFontLoadUTF16(unicode_letter, &p_han->font_buf);


  uint16_t  FontSize = p_han->font_buf.Size_Char;
  uint16_t index_x;

  for (int i = 0 ; i < 16 ; i++)          // 16 Lines per Font/Char
  {
    index_x = 0;
    for (int j = 0 ; j < FontSize ; j++)  // 16 x 16 (2 Bytes)
    {
      uint8_t font_data;

      font_data = p_han->font_buf.FontBuffer[i*FontSize +j];

      for(int Loop=0; Loop<8; Loop++)
      {
        if(font_data & ((uint8_t)0x80>>Loop))
        {
          font_src_buffer[i*16 + index_x] = 0xFF;
        }
        else
        {
          font_src_buffer[i*16 + index_x] = 0x00;
        }
        index_x++;
      }
    }
  }

  if (p_han->resize_nearest == true)
    resizeImageNearest8(&r_src, &r_dst);
  else
    resizeImageFastGray8(&r_src, &r_dst);

  return true;
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

  if (p_han->font_w != 16)
  {
    dsc_out->bpp = 8;
  }
  return true;  
}

const uint8_t *hanFont_bitmap_cb(const lv_font_t * font, uint32_t unicode_letter)
{
  lvgl_font_t *p_han = (lvgl_font_t *)font->user_data;
  const uint8_t *p_ret;
  

  if (p_han->font_w == 16)
  {
    hanFontLoadUTF16(unicode_letter, &p_han->font_buf);
    p_ret = p_han->font_buf.FontBuffer;
  }
  else
  {
    hanFontDraw(p_han, unicode_letter);
    p_ret = font_dst_buffer;
  }

  return p_ret;  
}

void lvglFontInit(void)
{
  lvgl_font_t *p_font;


  han_font[LVGL_FONT_16].font_w = 16;
  han_font[LVGL_FONT_16].font_h = 16;
  han_font[LVGL_FONT_16].resize_nearest = false;
  han_font[LVGL_FONT_16].p_lv_font = (lv_font_t *)&lv_han_font_16;
  han_font[LVGL_FONT_16].p_lv_font->line_height = 16;                       
  han_font[LVGL_FONT_16].p_lv_font->fallback = &lv_font_montserrat_14;

  han_font[LVGL_FONT_20].font_w = 20;
  han_font[LVGL_FONT_20].font_h = 20;
  han_font[LVGL_FONT_20].resize_nearest = false;
  han_font[LVGL_FONT_20].p_lv_font = (lv_font_t *)&lv_han_font_20;
  han_font[LVGL_FONT_20].p_lv_font->line_height = 20;                       
  han_font[LVGL_FONT_20].p_lv_font->fallback = &lv_font_montserrat_14;

  han_font[LVGL_FONT_24].font_w = 24;
  han_font[LVGL_FONT_24].font_h = 24;
  han_font[LVGL_FONT_24].resize_nearest = false;
  han_font[LVGL_FONT_24].p_lv_font = (lv_font_t *)&lv_han_font_24;
  han_font[LVGL_FONT_24].p_lv_font->line_height = 24;                       
  han_font[LVGL_FONT_24].p_lv_font->fallback = &lv_font_montserrat_14;

  han_font[LVGL_FONT_28].font_w = 28;
  han_font[LVGL_FONT_28].font_h = 28;
  han_font[LVGL_FONT_28].resize_nearest = false;
  han_font[LVGL_FONT_28].p_lv_font = (lv_font_t *)&lv_han_font_28;
  han_font[LVGL_FONT_28].p_lv_font->line_height = 28;                       
  han_font[LVGL_FONT_28].p_lv_font->fallback = &lv_font_montserrat_14;

  han_font[LVGL_FONT_32].font_w = 32;
  han_font[LVGL_FONT_32].font_h = 32;
  han_font[LVGL_FONT_32].resize_nearest = true;
  han_font[LVGL_FONT_32].p_lv_font = (lv_font_t *)&lv_han_font_32;
  han_font[LVGL_FONT_32].p_lv_font->line_height = 32;                       
  han_font[LVGL_FONT_32].p_lv_font->fallback = &lv_font_montserrat_14;

  for (int i=0; i<LVGL_FONT_MAX; i++)
  {
    p_font = &han_font[i];
    p_font->p_lv_font->get_glyph_dsc = hanFont_dsc_cb;         /*Set a callback to get info about glyphs*/
    p_font->p_lv_font->get_glyph_bitmap = hanFont_bitmap_cb;   /*Set a callback to get bitmap of a glyph*/
    p_font->p_lv_font->base_line = 0;                          /*Base line measured from the top of line_height*/
    p_font->p_lv_font->dsc = NULL;                             /*Store any implementation specific data here*/
    p_font->p_lv_font->user_data = p_font;                     /*Optionally some extra user data*/
    p_font->p_lv_font->subpx = LV_FONT_SUBPX_NONE;
  }  
}

lv_font_t *lvglGetFont(LvglFontType_t font_type)
{
  lv_font_t *p_font;

  if (font_type >= LVGL_FONT_MAX) return NULL;
  
  p_font = han_font[font_type].p_lv_font;

  return p_font;
}

void cliCmd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "info"))
  {
    cliPrintf("is_init   : %d\n", is_init);
    cliPrintf("is_enable : %d\n", is_enable);
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "enable"))
  {
    is_enable = true;
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "disable"))
  {
    is_enable = false;
    ret = true;
  }

  if (ret == false)
  {
    cliPrintf("lvgl info\n");
    cliPrintf("lvgl enable\n");
    cliPrintf("lvgl disable\n");
  }
}
#endif