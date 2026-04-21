#include "../lib/Config/DEV_Config.h"
#include "example.h"
#include "../lib/GUI/GUI_BMPfile.h"
#include "../lib/GUI/GUI_Paint.h"

#include <math.h>
#include <stdlib.h>     //exit()
#include <signal.h>     //signal()
#include <sys/stat.h>   //stat()
#include <unistd.h>     //usleep
#include <time.h>

UWORD VCOM = 1780;

//#define IMAGE_PATH "/tmp/eink_frame.bmp"
#define IMAGE_PATH "/tmp/eink_note.bin"

int file_exists(const char *filename)
{
    struct stat buffer;
    return (stat(filename, &buffer) == 0);
}

void Convert_8bpp_to_4bpp(UBYTE *src, UBYTE *dst, int pixels)
{
    for (int i = 0; i < pixels / 2; ++i)
    {
        dst[i] = (src[i * 2] & 0xF0) | ((src[i * 2 + 1] & 0xF0) >> 4);
    }
}

IT8951_Dev_Info Dev_Info = {0, 0};
UWORD Panel_Width;
UWORD Panel_Height;
UDOUBLE Init_Target_Memory_Addr;
int epd_mode = 0;	//0: no rotate, no mirror
					//1: no rotate, horizontal mirror, for 10.3inch
					//2: no totate, horizontal mirror, for 5.17inch
					//3: no rotate, no mirror, isColor, for 6inch color
					
void  Handler(int signo)
{
    Debug("\r\nHandler:exit\r\n");
    if(Refresh_Frame_Buf != NULL){
        free(Refresh_Frame_Buf);
        Debug("free Refresh_Frame_Buf\r\n");
        Refresh_Frame_Buf = NULL;
    }
    if(Panel_Frame_Buf != NULL){
        free(Panel_Frame_Buf);
        Debug("free Panel_Frame_Buf\r\n");
        Panel_Frame_Buf = NULL;
    }
    if(Panel_Area_Frame_Buf != NULL){
        free(Panel_Area_Frame_Buf);
        Debug("free Panel_Area_Frame_Buf\r\n");
        Panel_Area_Frame_Buf = NULL;
    }
    if(bmp_src_buf != NULL){
        free(bmp_src_buf);
        Debug("free bmp_src_buf\r\n");
        bmp_src_buf = NULL;
    }
    if(bmp_dst_buf != NULL){
        free(bmp_dst_buf);
        Debug("free bmp_dst_buf\r\n");
        bmp_dst_buf = NULL;
    }
    if (Dev_Info.Panel_W != 0)
    {
  //      Debug("Clearing the panel to white color\r\n");
       EPD_IT8951_Clear_Refresh(Dev_Info, Init_Target_Memory_Addr, INIT_Mode);

  //      Debug("Going to sleep\r\n");
  //      EPD_IT8951_Sleep();
    }

    Debug("Exiting the module\r\n");
    DEV_Delay_ms(5000);
    Debug("Exiting the module\r\n");
    DEV_Module_Exit();
    Debug("Exiting the program\r\n");
    exit(0);
}


int main(int argc, char *argv[])
{
    //Exception handling:ctrl + c
    signal(SIGINT, Handler);

    if (argc < 2){
        Debug("Please input VCOM value on FPC cable!\r\n");
        Debug("Example: sudo ./epd -2.51\r\n");
        exit(1);
    }
	if (argc != 3){
		Debug("Please input e-Paper display mode!\r\n");
		Debug("Example: sudo ./epd -2.51 0 or sudo ./epd -2.51 1\r\n");
		Debug("Now, 10.3 inch glass panle is mode1, else is mode0\r\n");
		Debug("If you don't know what to type in just type 0 \r\n");
		exit(1);
    }

    //Init the BCM2835 Device
    if(DEV_Module_Init()!=0){
        return -1;
    }

    double temp;
    sscanf(argv[1],"%lf",&temp);
    VCOM = (UWORD)(fabs(temp)*1000);
    Debug("VCOM value:%d\r\n", VCOM);
	sscanf(argv[2],"%d",&epd_mode);
    Debug("Display mode:%d\r\n", epd_mode);
    Dev_Info = EPD_IT8951_Init(VCOM);

    //get some important info from Dev_Info structure
    Panel_Width = Dev_Info.Panel_W;
    Panel_Height = Dev_Info.Panel_H;
    Init_Target_Memory_Addr = Dev_Info.Memory_Addr_L | (Dev_Info.Memory_Addr_H << 16);
    char* LUT_Version = (char*)Dev_Info.LUT_Version;
    if( strcmp(LUT_Version, "M641") == 0 ){
        //6inch e-Paper HAT(800,600), 6inch HD e-Paper HAT(1448,1072), 6inch HD touch e-Paper HAT(1448,1072)
        A2_Mode = 4;
        Four_Byte_Align = true;
    }else if( strcmp(LUT_Version, "M841_TFAB512") == 0 ){
        //Another firmware version for 6inch HD e-Paper HAT(1448,1072), 6inch HD touch e-Paper HAT(1448,1072)
        A2_Mode = 6;
        Four_Byte_Align = true;
    }else if( strcmp(LUT_Version, "M841") == 0 ){
        //9.7inch e-Paper HAT(1200,825)
        A2_Mode = 6;
    }else if( strcmp(LUT_Version, "M841_TFA2812") == 0 ){
        //7.8inch e-Paper HAT(1872,1404)
        A2_Mode = 6;
    }else if( strcmp(LUT_Version, "M841_TFA5210") == 0 ){
        //10.3inch e-Paper HAT(1872,1404)
        A2_Mode = 6;
    }else{
        //default set to 6 as A2 Mode
        A2_Mode = 6;
    }
    Debug("A2 Mode:%d\r\n", A2_Mode);

 //  Debug("doing clear refresh...\r\n");
//	EPD_IT8951_Clear_Refresh(Dev_Info, Init_Target_Memory_Addr, INIT_Mode);

    Debug("starting to display...\r\n");

    //  while(1)
    //  {
    //          Display_BMP_Example(Panel_Width, Panel_Height, Init_Target_Memory_Addr, BitsPerPixel_2);
    //          sleep(.5);  // Wait for 0.5 seconds before next display
    // }

    //*************************** */
    
const char *bmp_file = "/tmp/eink_note.bmp";
unsigned char header[54];
UBYTE *Refresh_Frame_Buf = NULL;
const char *gps_bmp_file = "/tmp/eink_gps.bmp";
time_t last_gps_render_time = 0;
time_t last_gc16_gps_refresh_time = 0;

time_t last_1bp_render_time = 0;
time_t last_4bp_render_time = 0;

while (1)
{
//############################################## render gps data

    // ---- GPS image render ----
    struct stat gps_st;

    if (file_exists(gps_bmp_file) && stat(gps_bmp_file, &gps_st) == 0)
    {
        time_t now = time(NULL);

        if (gps_st.st_mtime > last_gps_render_time) {
            Debug("Displaying GPS BMP at 1bpp...\r\n");

            FILE *fp = fopen(gps_bmp_file, "rb");
            if (!fp || fread(header, 1, 54, fp) != 54) {
                Debug("Failed to read GPS BMP header\r\n");
                if (fp) fclose(fp);
                continue;
            }

            UWORD GPS_WIDTH = *(unsigned int*)&header[18];
            UWORD GPS_HEIGHT = *(unsigned int*)&header[22];
            fclose(fp);

            UDOUBLE GPS_Imagesize = ((GPS_WIDTH * BitsPerPixel_1 + 7) / 8) * GPS_HEIGHT;
            UBYTE *Gps_Frame_Buf = (UBYTE *)malloc(GPS_Imagesize);
            if (!Gps_Frame_Buf) continue;

            Paint_NewImage(Gps_Frame_Buf, GPS_WIDTH, GPS_HEIGHT, 0, BLACK);
            Paint_SelectImage(Gps_Frame_Buf);
            Paint_SetBitsPerPixel(BitsPerPixel_1);
            GUI_ReadBmp(gps_bmp_file, 0, 0);

            EPD_IT8951_1bp_Refresh(
                Gps_Frame_Buf, 0, 0, GPS_WIDTH, GPS_HEIGHT,
                (now - last_gc16_gps_refresh_time > 30) ? GC16_Mode : A2_Mode,
                Init_Target_Memory_Addr, true
            );

            if (now - last_gc16_gps_refresh_time > 30)
                last_gc16_gps_refresh_time = now;

            last_gps_render_time = gps_st.st_mtime;

            free(Gps_Frame_Buf);
        }
    }


//###############################################



    struct stat st;
    if (file_exists(bmp_file) && stat(bmp_file, &st) == 0)
    {
        time_t age = time(NULL) - st.st_mtime;
        UBYTE bpp = (age < 5) ? BitsPerPixel_1 : BitsPerPixel_4;

        if ((bpp == BitsPerPixel_1 && st.st_mtime == last_1bp_render_time) ||
            (bpp == BitsPerPixel_4 && st.st_mtime == last_4bp_render_time)) {
            usleep(100000);
            continue;
        }

        Debug("Displaying BMP as %dbpp...\r\n", bpp == BitsPerPixel_1 ? 1 : 4);

        FILE *fp = fopen(bmp_file, "rb");
        if (!fp || fread(header, 1, 54, fp) != 54) {
            Debug("Failed to read BMP header\r\n");
            if (fp) fclose(fp);
            usleep(100000);
            continue;
        }

        UWORD WIDTH = *(unsigned int*)&header[18];
        UWORD HEIGHT = *(unsigned int*)&header[22];
        fclose(fp);

        UDOUBLE Imagesize = ((WIDTH * bpp + 7) / 8) * HEIGHT;
        if (Refresh_Frame_Buf) free(Refresh_Frame_Buf);
        Refresh_Frame_Buf = (UBYTE *)malloc(Imagesize);

        Paint_NewImage(Refresh_Frame_Buf, WIDTH, HEIGHT, 0, BLACK);
        Paint_SelectImage(Refresh_Frame_Buf);
        Paint_SetBitsPerPixel(bpp);
 //       Paint_Clear(WHITE);
        GUI_ReadBmp(bmp_file, 0, 0);

        if (bpp == BitsPerPixel_1) {
            EPD_IT8951_1bp_Refresh(Refresh_Frame_Buf, 224, 0, WIDTH, HEIGHT, GC16_Mode, Init_Target_Memory_Addr, true);
            last_1bp_render_time = st.st_mtime;
        } else {
            EPD_IT8951_4bp_Refresh(Refresh_Frame_Buf, 224, 0, WIDTH, HEIGHT, GC16_Mode, Init_Target_Memory_Addr, true);
            last_4bp_render_time = st.st_mtime;
        }
    }
    else
    {
        Debug("BMP not found, waiting...\r\n");
    }

    usleep(100000);
}
    //****************************** */


    //We recommended refresh the panel to white color before storing in the warehouse.
    //in handler function, we will refresh the panel to white color.

    return 0;
}