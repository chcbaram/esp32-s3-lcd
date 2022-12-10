#include "lvgl_main.h"
#include "module/audio_play.h"


static void file_explorer_event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if(code == LV_EVENT_VALUE_CHANGED) {
        const char * cur_path =  lv_file_explorer_get_current_path(obj);
        const char * sel_fn = lv_file_explorer_get_selected_file_name(obj);
        uint16_t path_len = strlen(cur_path);
        uint16_t fn_len = strlen(sel_fn);

        if((path_len + fn_len) <= LV_FILE_EXPLORER_PATH_MAX_LEN) {
            char file_info[LV_FILE_EXPLORER_PATH_MAX_LEN];

            strcpy(file_info, cur_path);
            strcat(file_info, sel_fn);

            LV_LOG_USER("%s", file_info);
        }
        else    LV_LOG_USER("%s%s", cur_path, sel_fn);
    }
}


static void event_handler(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);

  if(code == LV_EVENT_CLICKED) 
  {
    LV_LOG_USER("Clicked");
    buzzerBeep(100);
    delay(200);

    lvglSuspend();
    if (e->user_data == (void *)0)
    {
      audioPlayFile("test2.wav");
    }
    else if (e->user_data == (void *)1)
    {
      audioPlayFile("test3.wav");
    }
    else
    {
      audioPlayFile("test.wav");
    }
    lvglResume();
  }
  else if(code == LV_EVENT_VALUE_CHANGED) 
  {
    LV_LOG_USER("Toggled");
    buzzerBeep(100);
  }
}

void lvglMainInit(void)
{
    // lv_obj_t * file_explorer = lv_file_explorer_create(lv_scr_act());
    // lv_file_explorer_set_sort(file_explorer, LV_EXPLORER_SORT_KIND);


    // lv_file_explorer_open_dir(file_explorer, "0:");
    // lv_obj_add_event_cb(file_explorer, file_explorer_event_handler, LV_EVENT_ALL, NULL);
    // return;



  lv_obj_t * label;

  lv_obj_t * btn1 = lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, (void *)0);
  lv_obj_align(btn1, LV_ALIGN_CENTER, -120, 0);
  lv_obj_set_width(btn1, 100);
  lv_obj_set_height(btn1, 100);

  label = lv_label_create(btn1);
  lv_label_set_text(label, "Play #1");
  lv_obj_center(label);

  lv_obj_t * btn2 = lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, (void *)1);
  lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_width(btn2, 100);
  lv_obj_set_height(btn2, 100);
  
  label = lv_label_create(btn2);
  lv_label_set_text(label, "Play #2");
  lv_obj_center(label);

  lv_obj_t * btn3 = lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn3, event_handler, LV_EVENT_ALL, (void *)2);
  lv_obj_align(btn3, LV_ALIGN_CENTER, 120, 0);
  lv_obj_set_width(btn3, 100);
  lv_obj_set_height(btn3, 100);

  label = lv_label_create(btn3);
  lv_label_set_text(label, "Play #3");
  lv_obj_center(label);
}



