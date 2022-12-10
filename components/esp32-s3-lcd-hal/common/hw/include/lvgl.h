#ifndef LVGL_H_
#define LVGL_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "hw_def.h"

#ifdef _USE_HW_LVGL
#include "lvgl/lvgl.h"


bool lvglInit(void);
bool lvglUpdate(void);
bool lvglSuspend(void);
bool lvglResume(void);

void *lvglMalloc(size_t size);
void *lvglRealloc(void * p, size_t new_size);
void lvglFree(void * p);

#endif

#ifdef __cplusplus
}
#endif

#endif