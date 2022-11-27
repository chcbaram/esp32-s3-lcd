#ifndef ILI9481_H_
#define ILI9481_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"


#ifdef _USE_HW_ILI9481
#include "ili9481_regs.h"



#define ILI9481_IF_SPI     1
#define ILI9481_IF_8BIT    2
#define ILI9481_IF_16BIT   3


#define ILI9481_IF_MODE   HW_ILI9481_IF_MODE
#define ILI9481_WIDTH     HW_ILI9481_WIDTH
#define ILI9481_HEIGHT    HW_ILI9481_HEIGHT



bool ili9481Init(void);



#endif

#ifdef __cplusplus
}
#endif

#endif