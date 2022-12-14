#include "lvgl_main.h"
#include "module/audio_play.h"


static void lv_example_checkbox(lv_obj_t *scr);
static void lv_example_dropdown(lv_obj_t *scr);
static void lv_example_slider(lv_obj_t *scr);
static void slider_event_cb(lv_event_t * e);
static void slider_event_cb2(lv_event_t * e);
static void event_handler_file_explorer(lv_event_t * e);
static void event_handler(lv_event_t * e);
static void event_handler_dropdown(lv_event_t * e);
static void event_handler_touch(lv_event_t * e);
static bool isEndWith(const char * str1, const char * str2);


static lv_obj_t * slider_label;






void lvglMainInit(void)
{

  /*Create a Tab view object*/
  lv_obj_t * tabview;
  tabview = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, 5);

  /*Add 3 tabs (the tabs are page (lv_page) and can be scrolled*/
  lv_obj_t * tab1 = lv_tabview_add_tab(tabview, " ");
  lv_obj_t * tab2 = lv_tabview_add_tab(tabview, " ");
  lv_obj_t * tab3 = lv_tabview_add_tab(tabview, " ");

  lv_obj_t * file_explorer = lv_file_explorer_create(tab2);
  lv_file_explorer_set_sort(file_explorer, LV_EXPLORER_SORT_KIND);

  // explorer
  //
  lv_file_explorer_open_dir(file_explorer, "0:");
  lv_obj_add_event_cb(file_explorer, event_handler_file_explorer, LV_EVENT_ALL, NULL);


  lv_obj_t * cb;
  cb = lv_checkbox_create(tab1);
  lv_checkbox_set_text(cb, "Touch");
  lv_obj_add_event_cb(cb, event_handler_touch, LV_EVENT_ALL, NULL);
  lv_obj_align(cb, LV_ALIGN_TOP_LEFT, 0, 0);
  if (touchGetEnable())
    lv_obj_add_state(cb, LV_STATE_CHECKED);


  // buttons
  //
  lv_obj_t * label;


  label = lv_label_create(tab1);
  lv_label_set_text(label, "메뉴를 선택하세요.");
  lv_obj_set_style_text_font(label, &lv_han_font_28, _LV_STYLE_STATE_CMP_SAME);
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 30);
  

  lv_obj_t * btn1 = lv_btn_create(tab1);
  lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, (void *)0);
  lv_obj_align(btn1, LV_ALIGN_CENTER, -120, 0);
  lv_obj_set_width(btn1, 100);
  lv_obj_set_height(btn1, 100);

  label = lv_label_create(btn1);
  lv_label_set_text(label, "Play #1");
  lv_obj_center(label);



  lv_obj_t * btn2 = lv_btn_create(tab1);
  lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, (void *)1);
  lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_width(btn2, 100);
  lv_obj_set_height(btn2, 100);


  label = lv_label_create(btn2);
  lv_label_set_text(label, "Play #2");
  lv_obj_set_style_text_font(label, &lv_han_font_16, _LV_STYLE_STATE_CMP_SAME);
  lv_obj_center(label);

  

  lv_obj_t * btn3 = lv_btn_create(tab1);
  lv_obj_add_event_cb(btn3, event_handler, LV_EVENT_ALL, (void *)2);
  lv_obj_align(btn3, LV_ALIGN_CENTER, 120, 0);
  lv_obj_set_width(btn3, 100);
  lv_obj_set_height(btn3, 100);


  label = lv_label_create(btn3);
  lv_label_set_text(label, "Play #3");
  lv_obj_center(label);

  lv_example_checkbox(tab3);
  lv_example_dropdown(tab3);
  lv_example_slider(tab1);
}

static void event_handler_touch(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * obj = lv_event_get_target(e);
  if(code == LV_EVENT_VALUE_CHANGED) 
  {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED)
    {
      touchSetEnable(true);
    }
    else
    {
      touchSetEnable(false);
    }
  }
}

static void event_handler_checkbox(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        const char * txt = lv_checkbox_get_text(obj);
        const char * state = lv_obj_get_state(obj) & LV_STATE_CHECKED ? "Checked" : "Unchecked";
        LV_LOG_USER("%s: %s", txt, state);
    }
}

void lv_example_checkbox(lv_obj_t *scr)
{
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scr, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);

    lv_obj_t * cb;
    cb = lv_checkbox_create(scr);
    lv_checkbox_set_text(cb, "Apple");
    lv_obj_add_event_cb(cb, event_handler_checkbox, LV_EVENT_ALL, NULL);

    cb = lv_checkbox_create(scr);
    lv_checkbox_set_text(cb, "Banana");
    lv_obj_add_state(cb, LV_STATE_CHECKED);
    lv_obj_add_event_cb(cb, event_handler_checkbox, LV_EVENT_ALL, NULL);

    cb = lv_checkbox_create(scr);
    lv_checkbox_set_text(cb, "Lemon");
    lv_obj_add_state(cb, LV_STATE_DISABLED);
    lv_obj_add_event_cb(cb, event_handler_checkbox, LV_EVENT_ALL, NULL);

    cb = lv_checkbox_create(scr);
    lv_obj_add_state(cb, LV_STATE_CHECKED | LV_STATE_DISABLED);
    lv_checkbox_set_text(cb, "Melon\nand a new line");
    lv_obj_add_event_cb(cb, event_handler_checkbox, LV_EVENT_ALL, NULL);

    lv_obj_update_layout(cb);
}

void lv_example_dropdown(lv_obj_t *scr)
{

    /*Create a normal drop down list*/
    lv_obj_t * dd = lv_dropdown_create(scr);
    lv_dropdown_set_options(dd, "Apple\n"
                                "Banana\n"
                                "Orange\n"
                                "Cherry\n"
                                "Grape\n"
                                "Raspberry\n"
                                "Melon\n"
                                "Orange\n"
                                "Lemon\n"
                                "Nuts");

    lv_obj_align(dd, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_add_event_cb(dd, event_handler_dropdown, LV_EVENT_ALL, NULL);

}

void lv_example_slider(lv_obj_t *scr)
{
  /*Create a slider in the center of the display*/
  lv_obj_t * slider = lv_slider_create(scr);
  lv_obj_center(slider);
  lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_ALL, NULL);
  lv_slider_set_value(slider, audioGetVolume(), LV_ANIM_ON);

  /*Create a label below the slider*/
  slider_label = lv_label_create(scr);
  lv_label_set_text(slider_label, "VOL");

  lv_obj_align(slider, LV_ALIGN_BOTTOM_MID, 0, -20);
  lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_LEFT_MID, -15, 0);

  lv_obj_t * slider_label2;
  slider_label2 = lv_label_create(scr);
  lv_label_set_text(slider_label2, "LCD");

  lv_obj_t * slider2 = lv_slider_create(scr);
  lv_obj_center(slider2);
  lv_obj_add_event_cb(slider2, slider_event_cb2, LV_EVENT_ALL, NULL);
  lv_slider_set_value(slider2, lcdGetBackLight(), LV_ANIM_ON);
  lv_slider_set_range(slider2, 10, 100);
  lv_obj_align(slider2, LV_ALIGN_BOTTOM_MID, 0, -50);
  lv_obj_align_to(slider_label2, slider2, LV_ALIGN_OUT_LEFT_MID, -15, 0);

}

void event_handler_file_explorer(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * obj = lv_event_get_target(e);

  if(code == LV_EVENT_CLICKED) 
  {
    
  }

  if(code == LV_EVENT_VALUE_CHANGED) 
  {
    const char * cur_path =  lv_file_explorer_get_current_path(obj);
    const char * sel_fn = lv_file_explorer_get_selected_file_name(obj);
    uint16_t path_len = strlen(cur_path);
    uint16_t fn_len = strlen(sel_fn);

    if((path_len + fn_len) <= LV_FILE_EXPLORER_PATH_MAX_LEN) 
    {
      char file_info[LV_FILE_EXPLORER_PATH_MAX_LEN];

      strcpy(file_info, cur_path);
      strcat(file_info, sel_fn);

      if (isEndWith(sel_fn, ".wav"))
      {
        LV_LOG_USER("%s", file_info);
        lvglSuspend();
        audioPlayFile(&file_info[2]);
        lvglResume();      
      }
      if (isEndWith(sel_fn, ".jpg"))
      {
        LV_LOG_USER("%s", file_info);
        lvglSuspend();
        jpegdDrawFile(0, 0, 0, 0, &file_info[2]);
        lcdRequestDraw();
        delay(1000);
        lvglResume();      
      }
    }
    else    
    {
      LV_LOG_USER("%s%s", cur_path, sel_fn);
    }
  }
}

static void slider_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * slider = lv_event_get_target(e);

  if(code == LV_EVENT_VALUE_CHANGED)
  {
    // lv_snprintf(buf, sizeof(buf), "%d%%", (int)lv_slider_get_value(slider));
    // lv_label_set_text(slider_label, buf);
    // lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    audioSetVolume(lv_slider_get_value(slider));
  }
}

static void slider_event_cb2(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * slider = lv_event_get_target(e);

  if(code == LV_EVENT_VALUE_CHANGED)
  {
    //lv_snprintf(buf, sizeof(buf), "%d%%", (int)lv_slider_get_value(slider));
    //lv_label_set_text(slider_label, buf);
    //lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    int value = lv_slider_get_value(slider);

    value = constrain(value, 10, 100);
    lcdSetBackLight(value);
  }
}

static void event_handler_dropdown(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        char buf[32];
        lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
        LV_LOG_USER("Option: %s", buf);
    }
}
void event_handler(lv_event_t * e)
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


bool isEndWith(const char * str1, const char * str2)
{
    if(str1 == NULL || str2 == NULL)
        return false;

    uint16_t len1 = strlen(str1);
    uint16_t len2 = strlen(str2);
    if((len1 < len2) || (len1 == 0 || len2 == 0))
        return false;

    while(len2 >= 1) {
        if(str2[len2 - 1] != str1[len1 - 1])
            return false;

        len2--;
        len1--;
    }

    return true;
}