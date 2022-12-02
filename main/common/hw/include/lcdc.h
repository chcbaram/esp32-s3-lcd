#ifndef LCDC_H_
#define LCDC_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "hw_def.h"

#ifdef _USE_HW_LCDC


typedef struct
{
  uint16_t cmd;
  uint32_t cmd_width;
  uint32_t cmd_length;

  const void *p_data;
  uint32_t    data_width;
  uint32_t    data_length;
} lcdc_write_t;


bool lcdcInit(void);
bool lcdcBegin(uint16_t width, uint16_t height, uint8_t bus_width, uint32_t freq_mhz);
bool lcdcSetCallBack(void (*p_func)(void));
bool lcdcIsBusy(void);
bool lcdcWrite(lcdc_write_t *p_write, uint32_t timeout_ms);
bool lcdcWritePoll(lcdc_write_t *p_write, uint32_t timeout_ms);


#endif

#ifdef __cplusplus
}
#endif

#endif