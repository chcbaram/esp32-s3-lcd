#include "jpegd.h"



#if defined(_USE_HW_JPEGD) && defined(CONFIG_JD_SZBUF)
#include "cli.h"
#include "tjpgd.h"
#include "lcd.h"

#define BASE_PATH "/sdcard"

static void cliCmd(cli_args_t *args);

static uint8_t jpeg_work_buf[4096];  




bool jpegdInit(void)
{
  bool ret = true;


  cliAdd("jpegd", cliCmd);
  return ret;
}

size_t jpegdDataReader(JDEC *decoder, uint8_t *buffer, size_t size)
{
  FILE *fp = (FILE *)decoder->device;


  if (buffer) 
  {
    /* Read bytes from input stream. */
    return read(fileno(fp), buffer, size);  
  } 
  else 
  {
    /* Skip bytes from input stream. */
    if (lseek(fileno(fp), size, SEEK_CUR) > 0) 
    {
      return size;
    }
    /* Seek failed, causes TJPGD to abort. */
    return 0;
  }
}

int jpegdDataWriter(JDEC* decoder, void* bitmap, JRECT* rectangle)
{
  uint16_t *p_src_buf = (uint16_t *)bitmap;
  uint16_t *p_dst_buf = lcdGetFrameBuffer();
  uint16_t index;

  index = 0;
  for (int y=rectangle->top; y<=rectangle->bottom; y++)
  {
    for (int x=rectangle->left; x<=rectangle->right; x++)
    {
      p_dst_buf[LCD_GET_PIXEL_POS(x, y)] = p_src_buf[index];
      index++;
    }
  }
  return 1;
}

bool jpegdDrawFile(int16_t x, int16_t y, int16_t w, int16_t h, char *file_name)
{
  bool ret = false;
  FILE *fp;
  JDEC decoder;
  JRESULT result;
  char file_path[128];

  snprintf(file_path, 128, BASE_PATH "/%s", file_name);

  fp = fopen(file_path, "rb");
  if (fp != NULL)
  {
    result = jd_prepare(&decoder, jpegdDataReader, jpeg_work_buf, 4096, fp);
    if (JDR_OK == result) 
    {
      jd_decomp(&decoder, jpegdDataWriter, 0);
    };
    fclose(fp);
    ret = true;
  }

  return ret;
}
  
void cliCmd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 2 && args->isStr(0, "open"))
  {
    cliPrintf("file : %s\n", args->getStr(1));

    while(cliKeepLoop())
    {
      if (lcdDrawAvailable() > 0)
      {
        lcdClearBuffer(black);
        if (jpegdDrawFile(0, 0, 0, 0, args->getStr(1)) == false)
        {
          cliPrintf("fopen fail\n");
          break;
        }

        lcdPrintfResize(5, 32*0, red, 32, "%d fps", lcdGetFps());
        lcdPrintfResize(5, 32*1, red, 32, "%d ms ", lcdGetFpsTime());
        lcdRequestDraw();
      }
    }
    ret = true;
  }

  if (ret == false)
  {
    cliPrintf("jpegd open file_name\n");
  }
}
#else
bool jpegdInit(void)
{
  return false;  
}
#endif