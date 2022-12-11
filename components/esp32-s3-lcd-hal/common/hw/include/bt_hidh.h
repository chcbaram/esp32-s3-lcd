#ifndef BT_HIDH_H_
#define BT_HIDH_H_


#ifdef __cplusplus
 extern "C" {
#endif


#include "hw_def.h"


#ifdef _USE_HW_BT_HIDH



typedef struct
{
  uint8_t btn;
  int16_t x;
  int16_t y;
} bt_hidh_mouse_info_t;


bool btHidhInit(void);
bool btHidhBegin(void);
bool btHidhIsBegin(void);
bool btHidhConnect(void);
bool btHidhIsConnect(void);
bool btHidhDisconnect(void);

void btHidhStopCmd(uint32_t timeout_ms);

#endif


#ifdef __cplusplus
 }
#endif


#endif 
