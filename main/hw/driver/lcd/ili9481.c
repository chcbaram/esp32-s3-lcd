#include "lcd/ili9481.h"
#include "cli.h"


#ifdef _USE_HW_ILI9481
#include "esp_private/periph_ctrl.h"
#include "esp_private/gdma.h"
#include "hal/lcd_ll.h"
#include "hal/lcd_hal.h"
#include "hal/dma_types.h"
#include "hal/gpio_hal.h"
#include "soc/lcd_periph.h"


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

  uint32_t       dc_sig;
  uint32_t       wr_sig;
  uint32_t       bus_width;
  lcd_gpio_dbi_t bus_gpio;
} lcd_bus_t;

typedef struct  
{
    lcd_bus_t *bus; 
    const void *data;         // Data buffer
    uint32_t    data_length;  // Data buffer size
    void *arg;   
    void (*done_cb)(lcd_bus_t *bus, void *arg); 
} lcd_bus_trans_descriptor_t;


static void cliCmd(cli_args_t *args);
static bool initDbiBus(lcd_bus_t *bus);
static bool initBusDma(lcd_bus_t *bus);
static bool initDbiGpio(lcd_bus_t *bus);
static bool initDbiPanel(lcd_bus_t *bus);
static bool dbiWriteParam(lcd_bus_t *bus, uint16_t cmd, const void *param, uint32_t length);
static bool dbiWriteMemory(lcd_bus_t *bus, uint16_t cmd, const void *param, uint32_t length);
static bool dbiWriteData(lcd_bus_t *bus, const void *param, uint32_t length);

static lcd_bus_t lcd_bus;
static bool is_init = false;


uint16_t  img_buf[1024];


bool ili9481Init(void)
{
  bool ret = false;


  logPrintf("[__] ili9481Init() {\n");


  lcd_bus.bus_width = 16;           // 16bits
  lcd_bus.pclk_hz   = 10*1000*1000; // 10Mhz

  ret = initDbiBus(&lcd_bus);
  logPrintf("[%s] initDbiBus()\n", ret ? "OK":"NG");

  ret = initDbiPanel(&lcd_bus);
  logPrintf("[%s] initDbiPanel()\n", ret ? "OK":"NG");

  is_init = ret;


  uint8_t buf[4];

  dbiWriteParam(&lcd_bus, ILI9481_C_SOFT_RESET,        NULL, 0);
  delay(50);
  dbiWriteParam(&lcd_bus, ILI9481_C_EXIT_SLEEP_MODE,   NULL, 0);
  delay(100);

  dbiWriteParam(&lcd_bus, ILI9481_W_SET_ADDR_MODE,    (uint8_t[]){0x28}, 1);
  dbiWriteParam(&lcd_bus, ILI9481_W_SET_PIXEL_FORMAT, (uint8_t[]){0x55}, 1);
  dbiWriteParam(&lcd_bus, ILI9481_C_ENTER_INVERT_MODE, NULL, 0);
  dbiWriteParam(&lcd_bus, ILI9481_WR_POWER_SET,       (uint8_t[]){0x07, 0x42, 0x15}, 3);


  dbiWriteParam(&lcd_bus, ILI9481_WR_FRAME_RATE_INV_CONTROL,(uint8_t[]){3}, 1); // 72Hz
  buf[0] = ((ILI9481_HEIGHT/2) >> 8) & 0xFF;
  buf[1] = ((ILI9481_HEIGHT/2) >> 0) & 0xFF;
  dbiWriteParam(&lcd_bus, ILI9481_W_SET_TEAR_SCANLINE, buf, 2);
  dbiWriteParam(&lcd_bus, ILI9481_W_SET_TEAR_ON,      (uint8_t[]){0}, 1);

  dbiWriteParam(&lcd_bus, ILI9481_C_SET_DISPLAY_ON,    NULL, 0);

  buf[0] = 0;
  buf[1] = 0;
  buf[2] = ((ILI9481_WIDTH-1) >> 8) & 0xFF;
  buf[3] = ((ILI9481_WIDTH-1) >> 0) & 0xFF;
  dbiWriteParam(&lcd_bus, ILI9481_W_SET_COLUMN_ADDR, buf, 4);

  buf[0] = 0;
  buf[1] = 0;
  buf[2] = ((ILI9481_HEIGHT-1) >> 8) & 0xFF;
  buf[3] = ((ILI9481_HEIGHT-1) >> 0) & 0xFF;
  dbiWriteParam(&lcd_bus, ILI9481_W_SET_PAGE_ADDR, buf, 4);
  dbiWriteMemory(&lcd_bus, ILI9481_W_WRITE_MEMORY_START, NULL, 0);

  for (int i=0; i<480*320/4; i+=1)
  {
    img_buf[0] = 0xF800;
    if (i%480 > 240)
      img_buf[0] = 0xFFFF;

    dbiWriteData(&lcd_bus, &img_buf[0], 2);
  }
  for (int i=0; i<480*320/4; i+=1)
  {
    img_buf[0] = 0x07E0;
    dbiWriteData(&lcd_bus, &img_buf[0], 2);
  }
  for (int i=0; i<480*320/4; i+=1)
  {
    img_buf[0] = 0x001F;
    dbiWriteData(&lcd_bus, &img_buf[0], 2);
  }
  for (int i=0; i<480*320/4; i+=1)
  {
    img_buf[0] = 0x0000;
    dbiWriteData(&lcd_bus, &img_buf[0], 2);
  }
  

  logPrintf("[%s] }\n", ret ? "OK":"NG");
  cliAdd("ili9481", cliCmd);
  return true;
}

bool initDbiBus(lcd_bus_t *bus)
{
  bool ret;

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
  // int isr_flags = ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED;
  // ret = esp_intr_alloc_intrstatus(lcd_periph_signals.buses[bus_id].irq_id, isr_flags,
  //                                 (uint32_t)lcd_ll_get_interrupt_status_reg(bus->hal.dev),
  //                                 LCD_LL_EVENT_TRANS_DONE, lcd_default_isr_handler, bus, &bus->intr);
  // ESP_GOTO_ON_ERROR(ret, err, TAG, "install interrupt failed");
  lcd_ll_enable_interrupt(bus->hal.dev, LCD_LL_EVENT_TRANS_DONE, false); // disable all interrupts
  lcd_ll_clear_interrupt_status(bus->hal.dev, UINT32_MAX); // clear pending interrupt


  bus->format_buffer = heap_caps_calloc(1, CONFIG_LCD_PANEL_IO_FORMAT_BUF_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

  // DMA 체인 초기화 
  //
  bus->max_transfer_bytes = ILI9481_WIDTH * sizeof(uint16_t) * 100;
  bus->psram_trans_align  = 64;
  bus->sram_trans_align   = 4;
  bus->num_dma_nodes      = bus->max_transfer_bytes / DMA_DESCRIPTOR_BUFFER_MAX_SIZE + 1;
  bus->dma_nodes          = heap_caps_calloc(1, bus->num_dma_nodes * sizeof(dma_descriptor_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

  logPrintf("[%s] lcd_bus dma \n", bus->dma_nodes != NULL ? "OK":"NG");
  logPrintf("[__] lcd_bus dma_nodes : %d\n", bus->num_dma_nodes);

  ret = initBusDma(bus);
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
  lcd_ll_set_phase_cycles(bus->hal.dev, 0, 1, 0);
  lcd_ll_start(bus->hal.dev);
  while (!(lcd_ll_get_interrupt_status(bus->hal.dev) & LCD_LL_EVENT_TRANS_DONE)) {}


  // GPIO 초기화 
  //
  ret = initDbiGpio(bus);
  logPrintf("[%s] initDbiGpio() \n", ret == true ? "OK":"NG");
  if (ret == false) return false;

  return ret;
}

bool initBusDma(lcd_bus_t *bus)
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

bool initDbiGpio(lcd_bus_t *bus)
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

bool initDbiPanel(lcd_bus_t *bus)
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

void setDmaData(dma_descriptor_t *desc_head, const void *buffer, size_t len)
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

bool dbiWriteParam(lcd_bus_t *bus, uint16_t cmd, const void *param, uint32_t length)
{
  uint32_t intr_status;
  const uint8_t *p_param = param;
  
  for (int i=0; i<length; i++)
  {
    bus->format_buffer[i] = p_param[i];
  }

  lcd_ll_set_data_width(bus->hal.dev, 8);

  intr_status = lcd_ll_get_interrupt_status(bus->hal.dev);
  lcd_ll_clear_interrupt_status(bus->hal.dev, intr_status);

  lcd_ll_reverse_bit_order(bus->hal.dev, false);
  lcd_ll_swap_byte_order(bus->hal.dev, bus->bus_width, false);

  setDmaData(bus->dma_nodes, bus->format_buffer, length);

  lcd_ll_set_phase_cycles(bus->hal.dev, 1, 0, length);
  lcd_ll_set_command(bus->hal.dev, bus->bus_width, cmd);
  
  // some specific LCD commands can have no parameters
  if (length)
  { 
    gdma_start(bus->dma_chan, (intptr_t)(bus->dma_nodes));
    // delay 1us is sufficient for DMA to pass data to LCD FIFO
    // in fact, this is only needed when LCD pixel clock is set too high
    esp_rom_delay_us(1);
  }
  lcd_ll_start(bus->hal.dev);

  // polling the trans done event, but don't clear the event status
  while (!(lcd_ll_get_interrupt_status(bus->hal.dev) & LCD_LL_EVENT_TRANS_DONE)) {}

  return true;
}

bool dbiWriteMemory(lcd_bus_t *bus, uint16_t cmd, const void *param, uint32_t length)
{
  uint32_t intr_status;
  

  lcd_ll_set_data_width(bus->hal.dev, bus->bus_width);

  intr_status = lcd_ll_get_interrupt_status(bus->hal.dev);
  lcd_ll_clear_interrupt_status(bus->hal.dev, intr_status);

  lcd_ll_reverse_bit_order(bus->hal.dev, false);
  lcd_ll_swap_byte_order(bus->hal.dev, bus->bus_width, false);

  setDmaData(bus->dma_nodes, param, length);

  lcd_ll_set_phase_cycles(bus->hal.dev, 1, 0, length);
  lcd_ll_set_command(bus->hal.dev, bus->bus_width, cmd);
  
  // some specific LCD commands can have no parameters
  if (length)
  { 
    gdma_start(bus->dma_chan, (intptr_t)(bus->dma_nodes));
    // delay 1us is sufficient for DMA to pass data to LCD FIFO
    // in fact, this is only needed when LCD pixel clock is set too high
    esp_rom_delay_us(1);
  }
  lcd_ll_start(bus->hal.dev);

  // polling the trans done event, but don't clear the event status
  while (!(lcd_ll_get_interrupt_status(bus->hal.dev) & LCD_LL_EVENT_TRANS_DONE)) {}

  return true;
}

bool dbiWriteData(lcd_bus_t *bus, const void *param, uint32_t length)
{
  uint32_t intr_status;
  

  lcd_ll_set_data_width(bus->hal.dev, bus->bus_width);

  intr_status = lcd_ll_get_interrupt_status(bus->hal.dev);
  lcd_ll_clear_interrupt_status(bus->hal.dev, intr_status);

  lcd_ll_reverse_bit_order(bus->hal.dev, false);
  lcd_ll_swap_byte_order(bus->hal.dev, bus->bus_width, false);

  setDmaData(bus->dma_nodes, param, length);

  lcd_ll_set_phase_cycles(bus->hal.dev, 0, 0, length);
  //lcd_ll_set_command(bus->hal.dev, bus->bus_width, 0);
  
  // some specific LCD commands can have no parameters
  if (length)
  { 
    gdma_start(bus->dma_chan, (intptr_t)(bus->dma_nodes));
    // delay 1us is sufficient for DMA to pass data to LCD FIFO
    // in fact, this is only needed when LCD pixel clock is set too high
    esp_rom_delay_us(1);
  }
  lcd_ll_start(bus->hal.dev);

  // polling the trans done event, but don't clear the event status
  while (!(lcd_ll_get_interrupt_status(bus->hal.dev) & LCD_LL_EVENT_TRANS_DONE)) {}

  return true;
}


void cliCmd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "info"))
  {
    cliPrintf("is_init : %s\n", is_init ? "True":"False");
    ret = true;
  }



  if (ret == false)
  {
    cliPrintf("ili9481 info\n");
  }
}


#endif