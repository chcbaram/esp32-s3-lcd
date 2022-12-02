#include "lcdc.h"


#ifdef _USE_HW_LCDC
#include "esp_private/periph_ctrl.h"
#include "esp_private/gdma.h"
#include "hal/lcd_ll.h"
#include "hal/lcd_hal.h"
#include "hal/dma_types.h"
#include "hal/gpio_hal.h"
#include "soc/lcd_periph.h"
#include "freertos/semphr.h"


typedef struct
{
  int32_t pin_cs;
  int32_t pin_dc;
  int32_t pin_wr;
  int32_t pin_data[16];
} lcd_gpio_dbi_t;

typedef struct
{

  // I80 BUS
  //
  lcd_hal_context_t hal;
  uint8_t *format_buffer;         // The driver allocates an internal buffer for DMA to do data format transformer
  uint32_t psram_trans_align;     // DMA transfer alignment for data allocated from PSRAM
  uint32_t sram_trans_align;      // DMA transfer alignment for data allocated from SRAM
  uint32_t max_transfer_bytes;
  uint32_t pclk_hz;               // Frequency of pixel clock
  uint32_t clock_prescale;
  uint32_t resolution_hz;         // LCD_CLK resolution, determined by selected clock source
  uint32_t num_dma_nodes;         // Number of DMA descriptors
  gdma_channel_handle_t dma_chan; // DMA channel handle
  dma_descriptor_t *dma_nodes;    // DMA descriptor pool, the descriptors are shared by all i80 devices
  intr_handle_t intr;             // LCD peripheral interrupt handle

  uint32_t       dc_sig;
  uint32_t       wr_sig;
  uint32_t       bus_width;
  lcd_gpio_dbi_t bus_gpio;

  uint16_t lcd_width;
  uint16_t lcd_height;
} lcd_bus_t;

typedef struct  
{
    lcd_bus_t *bus; 
    const void *data;         // Data buffer
    uint32_t    data_length;  // Data buffer size
    void *arg;   
    void (*done_cb)(lcd_bus_t *bus, void *arg); 
} lcd_bus_trans_descriptor_t;


static bool lcdcInitDbiBus(lcd_bus_t *bus);
static bool lcdcInitBusDma(lcd_bus_t *bus);
static bool lcdcInitDbiGpio(lcd_bus_t *bus);
static bool lcdcInitDbiPanel(lcd_bus_t *bus);
static void lcdcSetDmaDesc(dma_descriptor_t *desc_head, const void *buffer, size_t len);
static void lcdcISR(void *args);

// This function is located in ROM (also see esp_rom/${target}/ld/${target}.rom.ld)
extern int Cache_WriteBack_Addr(uint32_t addr, uint32_t size);


static lcd_bus_t lcd_bus;
static bool is_init = false;
static bool is_begin = false;
static bool is_busy = false;
static void (*p_call_func)(void) = NULL;




bool lcdcInit(void)
{
  is_init = true;
  return true;
}

bool lcdcBegin(uint16_t width, uint16_t height, uint8_t bus_width, uint32_t freq_mhz)
{
  bool ret = false;


  logPrintf("[__] lcdcBegin() {\n");

  lcd_bus.lcd_width  = width;
  lcd_bus.lcd_height = height;
  lcd_bus.bus_width  = bus_width;       
  lcd_bus.pclk_hz    = freq_mhz*1000*1000;

  ret = lcdcInitDbiBus(&lcd_bus);
  logPrintf("[%s] lcdcInitDbiBus()\n", ret ? "OK":"NG");

  ret = lcdcInitDbiPanel(&lcd_bus);
  logPrintf("[%s] lcdcInitDbiPanel()\n", ret ? "OK":"NG");

  is_begin = ret;

  logPrintf("[%s] }\n", ret ? "OK":"NG");
  return ret;
}

bool lcdcSetCallBack(void (*p_func)(void))
{
  p_call_func = p_func;
  return true;
}

bool lcdcInitDbiBus(lcd_bus_t *bus)
{
  bool ret;
  esp_err_t esp_ret;


  // LCD HAL 드라이버 초기화 
  lcd_hal_init(&bus->hal, 0);
  
  // enable APB to access LCD registers
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  
  // reset peripheral and FIFO
  lcd_ll_reset(bus->hal.dev);
  lcd_ll_fifo_reset(bus->hal.dev);
  lcd_ll_enable_clock(bus->hal.dev, true);

  lcd_ll_select_clk_src(bus->hal.dev, LCD_CLK_SRC_PLL160M);
  lcd_ll_set_group_clock_coeff(bus->hal.dev, 2, 0, 0);
  bus->resolution_hz = 160000000 / 2;

  // install interrupt service, (LCD peripheral shares the same interrupt source with Camera peripheral with different mask)
  // interrupt is disabled by default
  int isr_flags = ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED;
  esp_ret = esp_intr_alloc_intrstatus(lcd_periph_signals.buses[0].irq_id, isr_flags,
                                   (uint32_t)lcd_ll_get_interrupt_status_reg(bus->hal.dev),
                                   LCD_LL_EVENT_TRANS_DONE, lcdcISR, bus, &bus->intr);

  logPrintf("[%s] lcd_bus isr \n", esp_ret == ESP_OK ? "OK":"NG");

  lcd_ll_enable_interrupt(bus->hal.dev, LCD_LL_EVENT_TRANS_DONE, false); // disable all interrupts
  lcd_ll_clear_interrupt_status(bus->hal.dev, UINT32_MAX); // clear pending interrupt


  bus->format_buffer = heap_caps_calloc(1, CONFIG_LCD_PANEL_IO_FORMAT_BUF_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

  // DMA 체인 초기화 
  //
  bus->max_transfer_bytes = bus->lcd_width * bus->lcd_width * sizeof(uint16_t);
  bus->psram_trans_align  = 64;
  bus->sram_trans_align   = 4;
  bus->num_dma_nodes      = bus->max_transfer_bytes / DMA_DESCRIPTOR_BUFFER_MAX_SIZE + 1;
  bus->dma_nodes          = heap_caps_calloc(1, bus->num_dma_nodes * sizeof(dma_descriptor_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

  logPrintf("[%s] lcd_bus dma \n", bus->dma_nodes != NULL ? "OK":"NG");
  logPrintf("[__] lcd_bus dma_nodes : %d\n", bus->num_dma_nodes);

  ret = lcdcInitBusDma(bus);
  logPrintf("[%s] initBusDma() \n", ret == true ? "OK":"NG");
  if (ret == false) return false;


  // BUS 모드 초기화 
  //
  // enable 8080 mode and set bus width
  lcd_ll_enable_rgb_mode(bus->hal.dev, false);
  lcd_ll_set_data_width(bus->hal.dev, bus->bus_width);
  // number of data cycles is controlled by DMA buffer size
  lcd_ll_enable_output_always_on(bus->hal.dev, true);
  // enable trans done interrupt
  lcd_ll_enable_interrupt(bus->hal.dev, LCD_LL_EVENT_TRANS_DONE, true);

  // trigger a quick interrupt event by a dummy transaction, wait the LCD interrupt line goes active
  // next time when esp_intr_enable is invoked, we can go into interrupt handler immediately
  // where we dispatch transactions for i80 devices
  // lcd_ll_set_phase_cycles(bus->hal.dev, 0, 1, 0);
  // lcd_ll_start(bus->hal.dev);
  // while (!(lcd_ll_get_interrupt_status(bus->hal.dev) & LCD_LL_EVENT_TRANS_DONE)) {}

  // GPIO 초기화 
  //
  ret = lcdcInitDbiGpio(bus);
  logPrintf("[%s] initDbiGpio() \n", ret == true ? "OK":"NG");
  if (ret == false) return false;

  return ret;
}

bool lcdcInitBusDma(lcd_bus_t *bus)
{
  bool ret = false;
  esp_err_t esp_ret = ESP_OK;
  
  do
  {
    // chain DMA descriptors
    for (int i = 0; i < bus->num_dma_nodes; i++) 
    {
      bus->dma_nodes[i].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_CPU;
      bus->dma_nodes[i].next = &bus->dma_nodes[i + 1];
    }
    bus->dma_nodes[bus->num_dma_nodes - 1].next = NULL; // one-off DMA chain

    // alloc DMA channel and connect to LCD peripheral
    gdma_channel_alloc_config_t dma_chan_config = {
        .direction = GDMA_CHANNEL_DIRECTION_TX,
    };
    esp_ret = gdma_new_channel(&dma_chan_config, &bus->dma_chan);
    if (esp_ret != ESP_OK)
    {
      logPrintf("[NG] alloc DMA channel failed\n");
      break;
    }    
    gdma_connect(bus->dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));

    gdma_strategy_config_t strategy_config = {
        .auto_update_desc = true,
        .owner_check = true
    };
    gdma_apply_strategy(bus->dma_chan, &strategy_config);


    // set DMA transfer ability
    gdma_transfer_ability_t ability = {
        .psram_trans_align = bus->psram_trans_align,
        .sram_trans_align = bus->sram_trans_align,
    };
    gdma_set_transfer_ability(bus->dma_chan, &ability);
  } while(0);

  if (esp_ret == ESP_OK)
  {
    ret = true;
  }
  else
  {
    if (bus->dma_chan) 
    {
        gdma_del_channel(bus->dma_chan);
    }
  }
  return ret;
}

bool lcdcInitDbiGpio(lcd_bus_t *bus)
{
  bus->bus_gpio.pin_cs = 48;
  bus->bus_gpio.pin_wr = 21;
  bus->bus_gpio.pin_dc = 47;

  bus->bus_gpio.pin_data[0]  = 14;
  bus->bus_gpio.pin_data[1]  = 13;
  bus->bus_gpio.pin_data[2]  = 12;
  bus->bus_gpio.pin_data[3]  = 11;
  bus->bus_gpio.pin_data[4]  = 10;
  bus->bus_gpio.pin_data[5]  = 9;
  bus->bus_gpio.pin_data[6]  = 46;
  bus->bus_gpio.pin_data[7]  = 3;
  bus->bus_gpio.pin_data[8]  = 8;
  bus->bus_gpio.pin_data[9]  = 18;
  bus->bus_gpio.pin_data[10] = 17;
  bus->bus_gpio.pin_data[11] = 16;
  bus->bus_gpio.pin_data[12] = 15;
  bus->bus_gpio.pin_data[13] = 7;
  bus->bus_gpio.pin_data[14] = 39;
  bus->bus_gpio.pin_data[15] = 38;

  for (size_t i = 0; i < bus->bus_width; i++) 
  {
    gpio_set_direction(bus->bus_gpio.pin_data[i], GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(bus->bus_gpio.pin_data[i], lcd_periph_signals.buses[0].data_sigs[i], false, false);
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[bus->bus_gpio.pin_data[i]], PIN_FUNC_GPIO);
  }

  // DC
  gpio_set_direction(bus->bus_gpio.pin_dc, GPIO_MODE_OUTPUT);
  esp_rom_gpio_connect_out_signal(bus->bus_gpio.pin_dc, lcd_periph_signals.buses[0].dc_sig, false, false);
  gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[bus->bus_gpio.pin_dc], PIN_FUNC_GPIO);

  // WR
  gpio_set_direction(bus->bus_gpio.pin_wr, GPIO_MODE_OUTPUT);
  esp_rom_gpio_connect_out_signal(bus->bus_gpio.pin_wr, lcd_periph_signals.buses[0].wr_sig, false, false);
  gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[bus->bus_gpio.pin_wr], PIN_FUNC_GPIO);

  // CS
  gpio_set_direction(bus->bus_gpio.pin_cs, GPIO_MODE_OUTPUT);
  esp_rom_gpio_connect_out_signal(bus->bus_gpio.pin_cs, lcd_periph_signals.buses[0].cs_sig, false, false);
  gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[bus->bus_gpio.pin_cs], PIN_FUNC_GPIO);

  return true;
}

bool lcdcInitDbiPanel(lcd_bus_t *bus)
{
  bool ret = false;

  // check if pixel clock setting is valid
  uint32_t pclk_prescale = bus->resolution_hz / bus->pclk_hz;

  if (pclk_prescale > 0 && pclk_prescale <= LCD_LL_PCLK_DIV_MAX)
  {
    logPrintf("[__] PCLK %dMhz\n", bus->pclk_hz/1000000);
    ret = true;
  }
  else
  {
    logPrintf("[NG] PCLK %dMhz\n", bus->pclk_hz/1000000);
    return false;
  }
  bus->clock_prescale = pclk_prescale;

  // Configure PCLK
  lcd_ll_set_pixel_clock_prescale(bus->hal.dev, bus->clock_prescale);
  lcd_ll_set_clock_idle_level(bus->hal.dev, true);
  lcd_ll_set_pixel_clock_edge(bus->hal.dev, false); // pclk_active_neg
  // configure DC line level fr the new device
  lcd_ll_set_dc_level(bus->hal.dev, 0, 0, 0, 1);

  return ret;
}

void lcdcSetDmaDesc(dma_descriptor_t *desc_head, const void *buffer, size_t len)
{
  size_t prepared_length = 0;
  uint8_t *data = (uint8_t *)buffer;
  dma_descriptor_t *desc = desc_head;

  while (len > DMA_DESCRIPTOR_BUFFER_MAX_SIZE) 
  {
      desc->dw0.suc_eof = 0; // not the end of the transaction
      desc->dw0.size = DMA_DESCRIPTOR_BUFFER_MAX_SIZE;
      desc->dw0.length = DMA_DESCRIPTOR_BUFFER_MAX_SIZE;
      desc->dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
      desc->buffer = &data[prepared_length];
      desc = desc->next; // move to next descriptor
      prepared_length += DMA_DESCRIPTOR_BUFFER_MAX_SIZE;
      len -= DMA_DESCRIPTOR_BUFFER_MAX_SIZE;
  }
  if (len) 
  {
      desc->dw0.suc_eof = 1; // end of the transaction
      desc->dw0.size = len;
      desc->dw0.length = len;
      desc->dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
      desc->buffer = &data[prepared_length];
      desc = desc->next; // move to next descriptor
      prepared_length += len;
  }
}

bool lcdcIsBusy(void)
{
  return is_busy;
}

bool lcdcWritePoll(lcdc_write_t *p_write, uint32_t timeout_ms)
{
  bool ret = true;
  uint32_t intr_status;
  uint32_t pre_time;
  lcd_bus_t *bus = &lcd_bus;

  if (is_init == false) return false;


  pre_time = millis();
  while(is_busy)
  {
    if (millis()-pre_time >= timeout_ms)
    {
      return false;
    }
  }
  is_busy = true;


  lcd_ll_set_data_width(bus->hal.dev, p_write->data_width);

  intr_status = lcd_ll_get_interrupt_status(bus->hal.dev);
  lcd_ll_clear_interrupt_status(bus->hal.dev, intr_status);

  lcd_ll_reverse_bit_order(bus->hal.dev, false);
  lcd_ll_swap_byte_order(bus->hal.dev, bus->bus_width, false);

  lcdcSetDmaDesc(bus->dma_nodes, p_write->p_data, p_write->data_length);

  lcd_ll_set_phase_cycles(bus->hal.dev, p_write->cmd_length, 0, p_write->data_length);
  lcd_ll_set_command(bus->hal.dev, bus->bus_width, p_write->cmd);
  
  if (esp_ptr_external_ram(p_write->p_data)) 
  {
    // flush framebuffer from cache to the physical PSRAM
    Cache_WriteBack_Addr((uint32_t)p_write->p_data, p_write->data_length);
  }

  if (p_write->data_length)
  { 
    gdma_start(bus->dma_chan, (intptr_t)(bus->dma_nodes));
    // delay 1us is sufficient for DMA to pass data to LCD FIFO
    // in fact, this is only needed when LCD pixel clock is set too high
    esp_rom_delay_us(1);
  }
  lcd_ll_start(bus->hal.dev);

  // polling the trans done event, but don't clear the event status
  while (!(lcd_ll_get_interrupt_status(bus->hal.dev) & LCD_LL_EVENT_TRANS_DONE)) 
  {
    if (millis()-pre_time >= timeout_ms)
    {
      ret = false;
      break;
    }
  }
  is_busy = false;

  return ret;
}

bool lcdcWrite(lcdc_write_t *p_write, uint32_t timeout_ms)
{
  bool ret = true;
  uint32_t intr_status;
  uint32_t pre_time;
  lcd_bus_t *bus = &lcd_bus;

  if (is_init == false) return false;


  pre_time = millis();
  while(is_busy)
  {
    if (millis()-pre_time >= timeout_ms)
    {
      return false;
    }
  }
  is_busy = true;

  lcd_ll_set_data_width(bus->hal.dev, p_write->data_width);

  intr_status = lcd_ll_get_interrupt_status(bus->hal.dev);
  lcd_ll_clear_interrupt_status(bus->hal.dev, intr_status);

  lcd_ll_reverse_bit_order(bus->hal.dev, false);
  lcd_ll_swap_byte_order(bus->hal.dev, bus->bus_width, false);

  lcdcSetDmaDesc(bus->dma_nodes, p_write->p_data, p_write->data_length);

  lcd_ll_set_phase_cycles(bus->hal.dev, p_write->cmd_length, 0, p_write->data_length);
  lcd_ll_set_command(bus->hal.dev, bus->bus_width, p_write->cmd);
  

  if (esp_ptr_external_ram(p_write->p_data)) 
  {
    // flush framebuffer from cache to the physical PSRAM
    Cache_WriteBack_Addr((uint32_t)p_write->p_data, p_write->data_length);
  }

  if (p_write->data_length)
  { 
    gdma_start(bus->dma_chan, (intptr_t)(bus->dma_nodes));
    // delay 1us is sufficient for DMA to pass data to LCD FIFO
    // in fact, this is only needed when LCD pixel clock is set too high
    esp_rom_delay_us(1);
  }
  lcd_ll_start(bus->hal.dev);

  esp_intr_enable(bus->intr);

  // polling the trans done event, but don't clear the event status
  if (timeout_ms > 0)
  {
    while(is_busy)
    {
      if (millis()-pre_time >= timeout_ms)
      {
        ret = false;
        break;
      }
    }    
  }

  return ret;
}


IRAM_ATTR void lcdcISR(void *args)
{
  lcd_bus_t *bus = (lcd_bus_t *)args;
  uint32_t intr_status;
  
  intr_status = lcd_ll_get_interrupt_status(bus->hal.dev);
  if (intr_status & LCD_LL_EVENT_TRANS_DONE) 
  {
    lcd_ll_clear_interrupt_status(bus->hal.dev, intr_status);
    esp_intr_disable(bus->intr);
    if (p_call_func != NULL)
    {
      p_call_func();
    }
    is_busy = false;    
  }

}



#endif