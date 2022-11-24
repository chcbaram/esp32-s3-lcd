#ifndef FT6236_H_
#define FT6236_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "hw_def.h"

#ifdef _USE_HW_FT6236



typedef struct
{
  uint8_t  id;
  uint8_t  event;
  uint16_t x;
  uint16_t y;
  uint8_t  weight;
  uint8_t  area;
} ft6236_point_t;

typedef struct
{
  uint8_t gest_id;
  uint8_t count;
  ft6236_point_t point[2];
} ft6236_info_t;


bool ft6236Init(void);


#endif

#ifdef __cplusplus
 }
#endif

#endif