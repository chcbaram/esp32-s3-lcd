#include "bt_hidh.h"



#if defined(_USE_HW_BT_HIDH) && defined(CONFIG_BT_ENABLED)
#include "cli.h"
#include "qbuffer.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_hidh.h"
#include "bt_hid/esp_hid_gap.h"


#define lock()      xSemaphoreTake(mutex_lock, portMAX_DELAY);
#define unLock()    xSemaphoreGive(mutex_lock);

#define STORAGE_NAMESPACE         "storage"
#define BT_HIDH_DEV_LIST_MAX      32


typedef enum 
{
  BT_HIDH_SCAN,
  BT_HIDH_OPEN,
  BT_HIDH_OPEN_LIST,  
  BT_HIDH_CLOSE,
  BT_HIDH_MAX,
} BtHidhCmd_t;


typedef struct
{
  uint8_t             id;
  uint8_t             bd_addr[ESP_BD_ADDR_LEN];
  char                name[32];
  esp_ble_addr_type_t addr_type;
  esp_hid_transport_t trasport;
} ble_hid_dev_t;


typedef struct
{
  bool          is_open;
  qbuffer_t     msg_q;
  bt_hidh_mouse_info_t msg_buf[16];

  ble_hid_dev_t hid_dev;
} ble_mouse_info_t;

typedef struct
{
  uint32_t scan_time;
  size_t results_len; 
  esp_hid_scan_result_t *results;
} bt_hidh_scan_info_t;


typedef struct
{
  BtHidhCmd_t cmd;
  uint32_t    cmd_data;
  void       *args;
} bt_hidh_cmd_t;


static void cliCmd(cli_args_t *args);
static void btHidhThread(void* arg);
static void btHidhCallback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
static void btHidhProcessScan(bt_hidh_cmd_t *p_cmd);
static void btHidhProcessOpen(bt_hidh_cmd_t *p_cmd, bool use_list);
static bool btHidhSaveDevice(ble_hid_dev_t *p_dev);
static bool btHidhLoadDevice(ble_hid_dev_t *p_dev);

static bool is_init = false;
static bool is_begin = false;
static bool is_mouse_saved = false;
static bool is_thread_busy = false;
static bool is_thread_stop = false;

static SemaphoreHandle_t mutex_lock;
static ble_mouse_info_t ble_mouse_info;
static QueueHandle_t msg_cmd_q;
static bt_hidh_cmd_t msg_cmd;

static uint32_t      hid_scan_cnt = 0;
static ble_hid_dev_t hid_scan_list[BT_HIDH_DEV_LIST_MAX];



bool btHidhInit(void)
{
  if (is_init == true) return true;



  ble_mouse_info.is_open = false;
  qbufferCreateBySize(&ble_mouse_info.msg_q, 
                      (uint8_t *)&ble_mouse_info.msg_buf,
                      sizeof(bt_hidh_mouse_info_t),
                      16);

  mutex_lock = xSemaphoreCreateMutex();
  msg_cmd_q = xQueueCreate(3, sizeof(bt_hidh_cmd_t *));

  is_init = true;

  if (btHidhLoadDevice(&ble_mouse_info.hid_dev) == true)
  {
    is_mouse_saved = true;
    logPrintf("[__] load dev : %s\n", ble_mouse_info.hid_dev.name);  
  }
  else
  {
    logPrintf("[__] load dev : empty\n");  
  }

  logPrintf("[%s] btHidhInit()\n", is_init ? "OK":"NG");

  cliAdd("bt_hidh", cliCmd);
  return true;
}

bool btHidhBegin(void)
{
  esp_err_t ret;

  if (is_begin == true)
  {
    logPrintf("[__] btHidhBegin() already\n");
    return true;
  }


  ret = esp_hid_gap_init(HID_HOST_MODE);
  if (ret != ESP_OK)
  {
    logPrintf("[NG] esp_hid_gap_init() 0x%X\n", ret);
    return false;
  }

  ret = esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler);
  if (ret != ESP_OK)
  {
    logPrintf("[NG] esp_ble_gattc_register_callback() 0x%X\n", ret);
    return false;
  }
    
  esp_hidh_config_t config = 
  {
    .callback = btHidhCallback,
    .event_stack_size = 4096,
    .callback_arg = NULL,
  };

  ret = esp_hidh_init(&config);
  if (ret != ESP_OK)
  {
    logPrintf("[NG] esp_hidh_init() 0x%X\n", ret);
    return false;
  }  

  is_begin = true;
  xTaskCreate(&btHidhThread, "btHidhThread", _HW_DEF_RTOS_THREAD_MEM_BT_HIDH, NULL, _HW_DEF_RTOS_THREAD_PRI_BT_HIDH, NULL);  

  return true;
}

bool btHidhIsBegin(void)
{
  return is_begin;
}

bool btHidhSendCmd(bt_hidh_cmd_t *p_cmd, bool wait)
{
  if (is_init == false) return false;
  if (is_begin == false)
  {
    logPrintf("[NG] btHidh not begin\n");
    return false;
  } 
  if (is_thread_busy == true) return false;


  is_thread_busy = true;
  if (xQueueSend(msg_cmd_q, &p_cmd, 0) != pdTRUE) 
  {
    logPrintf("[NG] btHidhSendCmd()\n");
    return false;
  }

  if (wait == true)
  {
    while(1)
    {
      if (is_thread_busy != true)
      {
        break;
      }
      delay(5);
    }
  }

  return true;
}

void btHidhCallback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
  esp_hidh_event_t event = (esp_hidh_event_t)id;
  esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;
  uint8_t btn;
  int16_t x;
  int16_t y;

  switch (event) 
  {
    case ESP_HIDH_OPEN_EVENT: 
    {
      if (param->open.status == ESP_OK)
      { 
        const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
        logPrintf("[OK] ESP_HIDH_OPEN_EVENT \n");
        logPrintf("[__] " ESP_BD_ADDR_STR " OPEN: %s\n", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
        esp_hidh_dev_dump(param->open.dev, stdout);
        ble_mouse_info.is_open = true;
      } 
      else 
      {
        logPrintf("[NG] ESP_HIDH_OPEN_EVENT \n");
      }
      break;
    }

    case ESP_HIDH_BATTERY_EVENT: 
    {
      const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
      logPrintf("[__] " ESP_BD_ADDR_STR " BATTERY: %d%%\n", ESP_BD_ADDR_HEX(bda), param->battery.level);
      break;
    }

    case ESP_HIDH_INPUT_EVENT: 
    {        
      btn = param->input.data[0];
      x   = (param->input.data[3] & 0x0F) << 12;
      x  |= (param->input.data[2] & 0xFF) <<  4;
      x >>= 4;
      y   = (param->input.data[4] & 0xFF) <<  8;
      y  |= (param->input.data[3] & 0xF0) <<  0;
      y >>= 4;

      logPrintf("btn 0x%02X, x %d y %d\n", btn, x, y);
      break;
    }

    case ESP_HIDH_FEATURE_EVENT: 
    {
      break;
    }

    case ESP_HIDH_CLOSE_EVENT: 
    {
      const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
      logPrintf("[__] " ESP_BD_ADDR_STR " CLOSE: %s\n", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
      ble_mouse_info.is_open = false;
      break;
    }

    default:
        break;
  }
}


void btHidhThread(void* arg)
{
  bt_hidh_cmd_t *p_cmd;

  while(1)
  {
    if (xQueueReceive(msg_cmd_q, &p_cmd, portMAX_DELAY) == pdTRUE)
    {
      is_thread_busy = true;

      if (p_cmd != NULL)
      {
        switch(p_cmd->cmd)
        {
          case BT_HIDH_SCAN:
            btHidhProcessScan(p_cmd);
            break;

          case BT_HIDH_OPEN:
            btHidhProcessOpen(p_cmd, false);
            break;

          case BT_HIDH_OPEN_LIST:
            btHidhProcessOpen(p_cmd, true);
            break;

          case BT_HIDH_CLOSE:
            break;

          default:
            break;
        }
      }
      is_thread_stop = false;
      is_thread_busy = false;    
    }
  }
}

void btHidhProcessScan(bt_hidh_cmd_t *p_cmd)
{
  size_t results_len = 0;
  esp_hid_scan_result_t *results = NULL;

  if (is_begin == false)
  {
    logPrintf("[NG] btHidhBegin() not run\n");
    return;
  }

  hid_scan_cnt = 0;

  esp_hid_scan(p_cmd->cmd_data, &results_len, &results);
  logPrintf("[__] Scan Found %d\n", results_len);
  if (results_len)
  {
    esp_hid_scan_result_t *r = results;
    while (r) 
    {
      if (r->transport == ESP_HID_TRANSPORT_BLE && hid_scan_cnt < BT_HIDH_DEV_LIST_MAX) 
      {
        memcpy(hid_scan_list[hid_scan_cnt].bd_addr, r->bda, ESP_BD_ADDR_LEN);
        strncpy(hid_scan_list[hid_scan_cnt].name, r->name, 32);
        hid_scan_list[hid_scan_cnt].trasport = ESP_HID_TRANSPORT_BLE;
        hid_scan_list[hid_scan_cnt].id = hid_scan_cnt;
        hid_scan_list[hid_scan_cnt].addr_type = r->ble.addr_type;

        hid_scan_cnt++;
      }
      r = r->next;
    }
    esp_hid_scan_results_free(results);
  }
}

void btHidhProcessOpen(bt_hidh_cmd_t *p_cmd, bool use_list)
{
  bool is_run = true;
  bool ret;
  ble_hid_dev_t *p_dev;


  if (btHidhIsConnect() == true)
  {
    logPrintf("[__] Already Connected\n");
    return;
  }

  if (use_list == true)
  {
    if (p_cmd->cmd_data >= hid_scan_cnt)
    {
      return;
    }

    p_dev = &hid_scan_list[p_cmd->cmd_data];
  }
  else
  {
    p_dev = &ble_mouse_info.hid_dev;
  }
  while(is_run)
  {
    if (esp_hidh_dev_open(p_dev->bd_addr, p_dev->trasport, p_dev->addr_type) != NULL)
    {
      if (use_list == true)
      {
        ret = btHidhSaveDevice(p_dev);
        if (ret == true)
        {
          is_mouse_saved = true;
          memcpy(&ble_mouse_info.hid_dev, p_dev, sizeof(ble_hid_dev_t));
        }
        else
        {
          is_mouse_saved = false;
        }
        logPrintf("[%s] btHidhSaveDevice()\n", ret ? "OK":"NG");
      }
      logPrintf("[OK] btHidhCmdOpen()\n");
      break;
    }
    delay(1000);

    if (is_thread_stop == true)
    {
      logPrintf("[__] thread_stop\n");
      is_thread_stop = false;
      break;
    }
  }
}

void btHidhStopCmd(uint32_t timeout_ms)
{
  uint32_t pre_time;

  if (is_thread_busy == false) return;
  if (is_init == false) return;
  if (is_begin == false) return;


  is_thread_stop = true;

  pre_time = millis();
  while(is_thread_stop)
  {
    delay(10);
    if (millis()-pre_time > timeout_ms)
    {
      break;
    }
  }
}

bool btHidhConnect(void)
{
  bool ret = false;


  if (is_mouse_saved == true)
  {
    msg_cmd.cmd = BT_HIDH_OPEN;
    ret = btHidhSendCmd(&msg_cmd, false);
  }

  logPrintf("[%s] btHidhConnect()\n", ret?"OK":"NG");

  return ret;
}

bool btHidhIsConnect(void)
{
  return ble_mouse_info.is_open;
}

bool btHidhDisconnect(void)
{
  if (ble_mouse_info.is_open == false) return true;



  return true;
}

bool btHidhSaveDevice(ble_hid_dev_t *p_dev)
{
  nvs_handle_t f_handle;
  esp_err_t err;

  // Open
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &f_handle);
  if (err != ESP_OK) return false;


  err = nvs_set_blob(f_handle, "bt_hidh_dev", p_dev, sizeof(ble_hid_dev_t));
  if (err != ESP_OK) return false;

  // Commit
  err = nvs_commit(f_handle);
  if (err != ESP_OK) return false;

  // Close
  nvs_close(f_handle);

  return true;
}

bool btHidhLoadDevice(ble_hid_dev_t *p_dev)
{
  nvs_handle_t f_handle;
  esp_err_t err;

  // Open
  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &f_handle);
  if (err != ESP_OK) return false;


  size_t required_size = sizeof(ble_hid_dev_t);
  err = nvs_get_blob(f_handle, "bt_hidh_dev", p_dev, &required_size);
  if (err != ESP_OK) return false;

  // Close
  nvs_close(f_handle);

  return true;
}

void cliCmd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "info"))
  {
    cliPrintf("is_init     : %s\n", is_init ? "True" : "False");
    cliPrintf("is_begin    : %s\n", is_begin ? "True" : "False");
    cliPrintf("is_saved    : %s\n", is_mouse_saved ? "True" : "False");
    cliPrintf("is_open     : %s\n", ble_mouse_info.is_open ? "True" : "False");
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "begin"))
  {
    if (btHidhBegin() == true)
      cliPrintf("btHidhBegin() OK\n");
    else
      cliPrintf("btHidhBegin() Fail\n");

    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "scan"))
  {
    msg_cmd.cmd = BT_HIDH_SCAN;
    msg_cmd.cmd_data = 5;
    btHidhSendCmd(&msg_cmd, true);     
    cliPrintf("Scan Done\n"); 
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "scan_open"))
  {
    msg_cmd.cmd = BT_HIDH_SCAN;
    msg_cmd.cmd_data = 5;
    btHidhSendCmd(&msg_cmd, true);     
    cliPrintf("Scan Done\n"); 

    if (hid_scan_cnt > 0)
    {
      msg_cmd.cmd = BT_HIDH_OPEN_LIST;
      msg_cmd.cmd_data = 0;
      btHidhSendCmd(&msg_cmd, false);
      cliPrintf("Open Start\n"); 
    }
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "list"))
  {
    cliPrintf("\n");
    cliPrintf("Scan Cnt : %d\n", hid_scan_cnt);
    for (int i=0; i<hid_scan_cnt; i++)
    {
      cliPrintf("%d : " ESP_BD_ADDR_STR " NAME : %s\n", 
        i, 
        ESP_BD_ADDR_HEX(hid_scan_list[i].bd_addr),
        hid_scan_list[i].name);
    }
    ret = true;
  }

  if (args->argc == 2 && args->isStr(0, "open"))
  {
    uint32_t id;
    uint32_t pre_time;

    id = args->getData(1);

    if (id < hid_scan_cnt)
    {
      msg_cmd.cmd = BT_HIDH_OPEN_LIST;
      msg_cmd.cmd_data = id;
      btHidhSendCmd(&msg_cmd, false);

      pre_time = millis();
      while(cliKeepLoop())
      {
        if (millis()-pre_time >= 5000)
        {
          break;
        }
        if (ble_mouse_info.is_open == true)
        {
          cliPrintf("OPEN OK\n");
          break;
        }
      }
    }
    else
    {
      cliPrintf("id failed\n");
    }
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "connect"))
  {
    btHidhConnect();
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "disconnect"))
  {
    btHidhDisconnect();
    ret = true;
  }

  if (ret == false)
  {
    cliPrintf("bt_hidh info\n");
    cliPrintf("bt_hidh begin\n");
    cliPrintf("bt_hidh scan\n");
    cliPrintf("bt_hidh scan_open\n");
    cliPrintf("bt_hidh list\n");
    cliPrintf("bt_hidh open id[0~%d]\n", hid_scan_cnt > 0 ? hid_scan_cnt-1:0);
    cliPrintf("bt_hidh connect\n");
    cliPrintf("bt_hidh disconnect\n");    
    cliPrintf("bt_hidh stop\n");
  }
}
#else
bool btHidhInit(void)
{
  return false;
}
#endif