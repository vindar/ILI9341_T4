/********************************************************************
*
* ILI9341_T4 library example. Interfacing with the LVGL library
*
* This example demonstrates how to use this driver with the LVGL
* library. Here, allocate an internal framebuffer 320x240 for the 
* ILI9341_T4 driver but only a small 320x40 buffer for LVGL to draw 
* onto. Then we use the updateRegion() method from the library to 
* update the internal framebuffer and sync with the screen with
* differential updates. 
*
* The total memory consumption for the 'graphic part' is 191KB:
*
*   - 150KB for the internal framebuffer 
*   - 25KB for LVGL draw buffer
*   - 16KB for 2 diffs buffer with 8Kb each. 
*    
* -----------------------------------
* BUILDING THE EXAMPLE (FOR ARDUINO)
* -----------------------------------
* (1) Install the 'lvgl' and 'lv_demos' libraries in Arduino's library folder. 
*     (tested here with LVGL v8.0.1)
*
* (2) Copy and rename 'libraries/lvgl/lv_conf_template.h' to 'libraries/lv_conf.h' 
*     (i.e. put this file directly in Arduino's libraries root folder). 
*     
* (3) Edit 'lv_conf.h' such that:
*     - '#if 1' at the beginning of the file.
*     - #define LV_COLOR_DEPTH 16
*     - #define LV_DISP_DEF_REFR_PERIOD 15
*     - #define LV_USE_PERF_MONITOR 1
*     - #define LV_FONT_MONTSERRAT_12  1
*     - #define LV_FONT_MONTSERRAT_14  1
*     - #define LV_FONT_MONTSERRAT_16  1
*
* (4) Copy and rename 'libraries/lv_demos/lv_demo_conf_template.h' to 
*     'libraries/lv_demo_conf.h' (i.e. put it again in Arduino's libraries 
*     root folder).
*     
* (5) Edit 'libraries/lv_demo_conf.h' to enable the demo that must be run:    
*     - '#if 1' at the beginning of the file.
*     - #define LV_USE_DEMO_WIDGETS 1
*     - #define LV_USE_DEMO_BENCHMARK   1
*     - #define LV_USE_DEMO_STRESS      1
*     - #define LV_DEMO_MUSIC_LANDSCAPE 1
*     
*  NOTE: the music player demo is too big to be held in RAM. Large arrays 
*  must be put in flash for the demo to compile correctly:  edit the large 
*  files in 'libraries\lv_demos\src\lv_demo_music\assets\'  and add 'PROGMEM' 
*  to the arrays definitions. Do not forget to #include <Arduino.h>  at the 
*  beggining of the C files where PROGMEM is used (or simply #include <Arduino.h>
*  in lv_demo.conf). 
********************************************************************/


#include <ILI9341_T4.h>
#include <lvgl.h>
#include <lv_demo.h>


//
// DEFAULT WIRING USING SPI 0 ON TEENSY 4/4.1
//
#define PIN_SCK     13      // mandatory
#define PIN_MISO    12      // mandatory
#define PIN_MOSI    11      // mandatory
#define PIN_DC      10      // mandatory, can be any pin but using pin 10 (or 36 or 37 on T4.1) provides greater performance

#define PIN_CS      9       // optional (but recommended), can be any pin.  
#define PIN_RESET   6       // optional (but recommended), can be any pin. 
#define PIN_BACKLIGHT 255   // optional, set this only if the screen LED pin is connected directly to the Teensy.
#define PIN_TOUCH_IRQ 255   // optional, set this only if the touchscreen is connected on the same SPI bus
#define PIN_TOUCH_CS  255   // optional, set this only if the touchscreen is connected on the same spi bus


//
// ALTERNATE WIRING USING SPI 1 ON TEENSY 4/4.1 
//
//#define PIN_SCK     27      // mandatory 
//#define PIN_MISO    1       // mandatory
//#define PIN_MOSI    26      // mandatory
//#define PIN_DC      0       // mandatory, can be any pin but using pin 0 (or 38 on T4.1) provides greater performance

//#define PIN_CS      30      // optional (but recommended), can be any pin.  
//#define PIN_RESET   29      // optional (but recommended), can be any pin.  
//#define PIN_BACKLIGHT 255   // optional, set this only if the screen LED pin is connected directly to the Teensy. 
//#define PIN_TOUCH_IRQ 255   // optional, set this only if the touchscreen is connected on the same SPI bus
//#define PIN_TOUCH_CS  255   // optional, set this only if the touchscreen is connected on the same spi bus



// 40MHz SPI. Can do much better with short wires
#define SPI_SPEED       40000000

// screen size in landscape mode
#define LX  320
#define LY  240


// 2 diff buffers with about 8K memory each
ILI9341_T4::DiffBuffStatic<8000> diff1;
ILI9341_T4::DiffBuffStatic<8000> diff2;

// the internal framebuffer for the ILI9341_T4 driver (150KB) 
// in DMAMEM to save space in the lower (faster) partof RAM. 
DMAMEM uint16_t internal_fb[LX * LY];

// the screen driver object
ILI9341_T4::ILI9341Driver tft(PIN_CS, PIN_DC, PIN_SCK, PIN_MOSI, PIN_MISO, PIN_RESET, PIN_TOUCH_CS, PIN_TOUCH_IRQ);


// number of lines in lvgl's internal draw buffer 
#define BUF_LY 40

lv_color_t lvgl_buf[LX*BUF_LY]; // memory for lvgl draw buffer (25KB) 

lv_disp_draw_buf_t draw_buf;    // lvgl 'draw buffer' object
lv_disp_drv_t disp_drv;         // lvgl 'display driver'
lv_indev_drv_t indev_drv;       // lvgl 'input device driver'



/** Callback to draw on the screen */
void my_disp_flush(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p)
    {
    const bool redraw_now = lv_disp_flush_is_last(disp);  // check if when should update the screen (or just buffer the changes). 
    tft.updateRegion(redraw_now, (uint16_t*)color_p, area->x1, area->x2, area->y1, area->y2); // updat the interval framebuffer, redraw the screen if requested
    lv_disp_flush_ready(disp); // tell lvgl that we are done and that the lvgl draw buffer can be reused.  
    }


/** Call back to read the touchpad */
void my_touchpad_read(lv_indev_drv_t* indev_driver, lv_indev_data_t* data)
    {
    int touchX, touchY, touchZ;
    bool touched = tft.readTouch(touchX, touchY, touchZ); // read the touchpad
    if (!touched)   
        { // nothing
        data->state = LV_INDEV_STATE_REL;
        }
    else 
        { // pressed
        data->state = LV_INDEV_STATE_PR;
        data->point.x = touchX;
        data->point.y = touchY;
        } 
    }

 
IntervalTimer guiTimer;

// callback to update lvgl's tick every ms. 
void guiInc() 
    {
    lv_tick_inc(1);
    }



void setup() 
    {
    Serial.begin(9600);

    tft.output(&Serial);    // send debug info to serial port.

    while (!tft.begin(SPI_SPEED))
        {
        Serial.println("Initialization error...");
        delay(1000);
        }
    tft.setFramebuffers(internal_fb);
    tft.setDiffBuffers(&diff1, &diff2);
    tft.setRotation(1);                 // landscape
    tft.setDiffGap(4); 
    tft.setVSyncSpacing(1);             // lvgl is already controlling framerate. Set to 1 to minimize screen tearing. 
    tft.setRefreshRate(100); 
    tft.clear(0);

  
    int touch_calib[4] = { 3820, 335, 3890, 436 }; // <-- put calibration values returned by tft.calibrateTouch() here.

    tft.setTouchCalibration(touch_calib);    // set touch calibration

   //tft.calibrateTouch(touch_calib); // uncomment this line to run touch calibration (follow instruction sent to Serial). 


    // initialize lvgl
    lv_init();

    // initialize lvgl drawing buffer
    lv_disp_draw_buf_init(&draw_buf, lvgl_buf, nullptr, LX * BUF_LY);

    // Initialize lvgl display driver
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LX;
    disp_drv.ver_res = LY;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // Initialize lvgl input device driver
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    // set the interval timer that given lvgl ticks. 
    guiTimer.begin(guiInc, 1000);


    // Choose the demo to run. 

    //lv_demo_widgets(); 

    lv_demo_benchmark();

    //lv_demo_stress(); 

    //lv_demo_music();  // for this demo, some large array must be marked PROGMEM otherwise we run out of memory
    }




elapsedMillis em = 0; 

void loop() 
    {
    lv_task_handler(); // lvgl gui handler
    delay(5);
    
    // print driver infos every 5 seconds
    if (em > 5000)
        {
        em = 0;
        tft.printStats();
        diff1.printStats();
        diff2.printStats();
        }
    }


/** end of file */

