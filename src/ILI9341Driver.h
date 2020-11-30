/******************************************************************************
*  ILI9341_T4 library for driving an ILI9341 screen via SPI with a Teensy 4/4.1
*  Implements vsync and differential updates from a memory framebuffer.
*
*  Copyright (c) 2020 Arvind Singh.  All right reserved.
*
* This library is free software; you can redistribute it and/or
*  modify it under the terms of the GNU Lesser General Public
*  License as published by the Free Software Foundation; either
*  version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*  Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public
*  License along with this library; if not, write to the Free Software
*  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*******************************************************************************/


/**
*CREDITS. Parts of this code is based on: 
*
* (1) KurtE's highly optimized library for ILI9341: https://github.com/KurtE/ILI9341_t3n
*     -> for SPI / DMA and all the fancy low level hardware stuff... beautiful !
* 
* (2) PJRC's XPT2048 library https://github.com/PaulStoffregen/XPT2046_Touchscreen
*     -> for all the touchscreen related methods.  
**/

#pragma once

#include "StatsVar.h"
#include "DiffBuff.h"

#include <Arduino.h>
#include <DMAChannel.h>
#include <SPI.h>
#include <stdint.h>


// This libray uses specify hardware features of Teensy 4/4.1 and will not work with another MCU...
#if (!defined(__IMXRT1062__))  ||  (!defined(CORE_TEENSY))
#error "This library only supports Teensy 4/4.1"
#endif


namespace ILI9341_T4
{



/** Configuration */

#define ILI9341_T4_DEFAULT_SPICLOCK 30000000         // default SPI write speed, some display can work up to 80Mhz...
#define ILI9341_T4_DEFAULT_SPICLOCK_READ 4000000     // default SPI read speed (much slower then write speed)

#define ILI9341_T4_DEFAULT_REFRESH_RATE  90          // set an inital refresh rate around 90 fps
#define ILI9341_T4_DEFAULT_VSYNC_SPACING 2           // vsync on with framerate = refreshrate/2 = 45FPS. 
#define ILI9341_T4_DEFAULT_DIFF_GAP 10               // default gap for diffs (typ. between 5 and 50)
#define ILI9341_T4_DEFAULT_DIFF_SPLIT 6              // default number of split/subframes when creating diffs. 
#define ILI9341_T4_DEFAULT_LATE_START_RATIO 0.3      // default "proportion" of the frame admissible for late frame start when using vsync. 

#define ILI9341_T4_RETRY_INIT 3                     // number of times we try initialization in begin() before returning an error. 
#define ILI9341_T4_TFTWIDTH 240                     // screen dimension x (in default orientation 0)
#define ILI9341_T4_TFTHEIGHT 320                    // screen dimension y (in default orientation 0)
#define ILI9341_T4_NB_PIXELS (ILI9341_T4_TFTWIDTH * ILI9341_T4_TFTHEIGHT)   // total number of pixels

#define ILI9341_T4_NB_SCANLINE 162                  // scanlines are in the range [0,161]. 

#define ILI9341_T4_MAX_VSYNC_SPACING 10             // max number of screen refresh between frames 
#define ILI9341_T4_FAST_UNSAFE_DMA 1                // enable possibly unsafe optimizations. 
#define ILI9341_T4_IRQ_PRIORITY 128                 // priority at which we run the irqs (dma and pit timer).
#define ILI9341_T4_MAX_DELAY_MICROSECONDS 1000000   // maximum waiting time (1 second)

#define ILI9341_T4_TOUCH_Z_THRESHOLD     400        // for touch
#define ILI9341_T4_TOUCH_Z_THRESHOLD_INT 75         // same as https://github.com/PaulStoffregen/XPT2046_Touchscreen/blob/master/XPT2046_Touchscreen.cpp
#define ILI9341_T4_TOUCH_MSEC_THRESHOLD  3          //

#define ILI9341_T4_SELFDIAG_OK 0xC0                 // value returned by selfDiagStatus() if everything is OK.


/** ILI9341 command codes */

#define ILI9341_T4_NOP 0x00
#define ILI9341_T4_SWRESET 0x01
#define ILI9341_T4_RDDID 0x04
#define ILI9341_T4_RDDST 0x09

#define ILI9341_T4_SLPIN 0x10
#define ILI9341_T4_SLPOUT 0x11
#define ILI9341_T4_PTLON 0x12
#define ILI9341_T4_NORON 0x13

#define ILI9341_T4_RDMODE 0x0A
#define ILI9341_T4_RDMADCTL 0x0B
#define ILI9341_T4_RDPIXFMT 0x0C
#define ILI9341_T4_RDIMGFMT 0x0D
#define ILI9341_T4_RDSELFDIAG 0x0F

#define ILI9341_T4_INVOFF 0x20
#define ILI9341_T4_INVON 0x21
#define ILI9341_T4_GAMMASET 0x26
#define ILI9341_T4_DISPOFF 0x28
#define ILI9341_T4_DISPON 0x29

#define ILI9341_T4_CASET 0x2A
#define ILI9341_T4_PASET 0x2B
#define ILI9341_T4_RAMWR 0x2C
#define ILI9341_T4_RAMRD 0x2E

#define ILI9341_T4_PTLAR 0x30
#define ILI9341_T4_MADCTL 0x36
#define ILI9341_T4_VSCRSADD 0x37
#define ILI9341_T4_PIXFMT 0x3A

#define ILI9341_T4_FRMCTR1 0xB1
#define ILI9341_T4_FRMCTR2 0xB2
#define ILI9341_T4_FRMCTR3 0xB3
#define ILI9341_T4_INVCTR 0xB4
#define ILI9341_T4_DFUNCTR 0xB6

#define ILI9341_T4_PWCTR1 0xC0
#define ILI9341_T4_PWCTR2 0xC1
#define ILI9341_T4_PWCTR3 0xC2
#define ILI9341_T4_PWCTR4 0xC3
#define ILI9341_T4_PWCTR5 0xC4
#define ILI9341_T4_VMCTR1 0xC5
#define ILI9341_T4_VMCTR2 0xC7

#define ILI9341_T4_RDID1 0xDA
#define ILI9341_T4_RDID2 0xDB
#define ILI9341_T4_RDID3 0xDC
#define ILI9341_T4_RDID4 0xDD

#define ILI9341_T4_GMCTRP1 0xE0
#define ILI9341_T4_GMCTRN1 0xE1
#define ILI9341_T4_PWCTR6  0xFC








/*************************************************************************************************************
* ILI9341 screen driver for Teensy 4/4.1.
*
* The driver performs fast blitting of a memory framebuffer onto the screen using SPI transfer (synchronous
* or asynchronous with DMA) with the following additonal features:
* 
* (1) Implements partial redraw where only the pixels that changes between frames are updated. 
* 
* (2) Implements 'vsync' to prevent screen tearing (pixels are updated behind the refresh scanline).
* 
* (3) Adjustable screen refresh rate (30 - 120hz) and framerate (asap or locked with the refresh rate).
* 
* (4) Multiple buffering mode: no buffering / double buffering / triple buffering
* 
* (5) The object can also drive a XPT2048 touchscreen on the same SPI bus.
*
* 
* The memory framebuffers should have the usual layout:
*
* PIXEL(i,j) = framebuffer[i + (width * j)]
*
* for 0 <= i < width  and  0 <= j < height
* with the 'width' and 'height' value depending on the chosen screen orientation and where the pixels color are 
* given in uint16_t RGB565 format.
* 
****************************************************************************************************************/
class ILI9341Driver
{


public:



    /***************************************************************************************************
    ****************************************************************************************************
    *
    * Initialization and configuration.
    *
    ****************************************************************************************************
    ****************************************************************************************************/


    /**
    * Constructor. Set the hardware pins but do not initialize anything.
    *
    * - The reset pin is optional: set it to 255 if not present.
    * 
    * - The backlight pin is not managed by the driver. Don't forget to turn in on, I always do...
    *
    * --------------------------------------------------------------------------------------------
    * - THE DC PIN MUST BE ONE OF THE HARDWARE CHIP SELECT PINS OF THE CORRESPONDING SPI BUS.
    *   The CS pin on the other hand can be any other digital pin.
    *--------------------------------------------------------------------------------------------
    * 
    * - The touch_cs (and touch_irq) pins are optional. They should be set if and only  if the 
	*   XPT2048 touchscreen is present AND ON THE SAME SPI BUS THAT DRIVES THE DISPLAY.
    * 
    * --------------------------------------------------------------------------------------------
    * THE SPI BUS SHOULD BE DEDICATED TO THE SCREEN (EXCEPT FOR THE POSSIBLE XPT2048 TOUCHSCREEN)
    * BECAUSE ASYNC UPDATE MAY REQUIRE ACCESS TO THE SPI BUS AT ANY TIME AND CANNOT WAIT...
    * --------------------------------------------------------------------------------------------
    **/
    ILI9341Driver(uint8_t cs, uint8_t dc, uint8_t sclk, uint8_t mosi, uint8_t miso, uint8_t rst = 255, uint8_t touch_cs = 255, uint8_t touch_irq = 255);



    /**
    * Initialize the screen (and optionally set the speed for read/write spi transfers).
    *
    * Call this method only once. There is no associated end() method.
    * 
    * Return true if init OK, false if an error occured.
    **/
    bool begin(uint32_t spi_clock = ILI9341_T4_DEFAULT_SPICLOCK, uint32_t spi_clock_read = ILI9341_T4_DEFAULT_SPICLOCK_READ);


    /**
    * Query the value of the self-diagnostic register.
    * Should return ILI9341_T4_SELFDIAG_OK = 0xC0 if everything is fine.
    **/
    int selfDiagStatus();


    /**
    * Print some info about the screen status on a given output stream.
    * (for debug purpose).
    * 
    * Use printStats() instead to get statistics for optimization purposes. 
    **/
    void printStatus(Stream* outputStream = &Serial);


    /**
    * Set the speed for subsequent SPI writes.
    *
    * Remark: calling this method reset the statistics.
    **/
    inline void setSpiClock(int spi_clock = ILI9341_T4_DEFAULT_SPICLOCK) 
        { 
        waitUpdateAsyncComplete();
        _spi_clock = spi_clock; 
        statsReset();
        };


    /**
    * Query the spi speed for SPI writes.
    **/
    inline int  getSpiClock() const { return _spi_clock; }


    /**
    * Set the speed for subsequent SPI reads.
    *
    * Remark: calling this method reset the statistics.
    **/
    inline void setSpiClockRead(int spi_clock = ILI9341_T4_DEFAULT_SPICLOCK_READ) 
        { 
        waitUpdateAsyncComplete();
        _spi_clock_read = spi_clock; 
        statsReset();
        }
    


    /**
    * Query the spi speed for SPI reads.
    **/
    inline int  getSpiClockRead() const { return _spi_clock_read; }


    /**
    * Enter/exit sleep mode.
    **/
    void sleep(bool enable);


    /**
    * Set the screen Orientation (between 0 and 3)
    *
    * The default start up orientation is 0.
    * The framebuffer layout depend on whether the display is in portrait or landscape mode.
    * 
    * - orientation 0 and 2 in portrait mode : 240x320.
    * - orientation 1 and 3 in landscape mode : 320x240.
    *
    * NOTE: Orientation 0 the the only one for with pixels refresh order coincide with the framebuffer
    * ordering so it it is the orientation that will allow the fastest diff redraw with fewest screen 
    * tearing -> Use this orientation whenever possible.
    * 
    * Remark: calling this method reset the statistics (provided the orientation changes). 
    **/
    void setRotation(uint8_t r);


    /**
    * Return the current screen orientation (between 0 and 3)
    *
    * The default start up orientation is 0.
    * The framebuffer layout depend on whether the display is in portrait or landscape mode.
    **/
    inline int getRotation() const { return _rotation; }


    /**
    * Return the screen witdh (w.r.t the current orientation).
    **/
    inline int width() const { return _width; }


    /**
    * Return the screen height (w.r.t the current orientation).
    **/
    inline int height() const { return _height; }


    /**
    * Invert the display colors.
    **/
    void invertDisplay(bool i);


    /**
    * set the refresh mode between 0 and 31.
    * (corresponds to frame rate control, command  0xB1).
    *
    * - 0  : fastest refresh rate (around than 120/140hz). 
    * - 31 : slowest refresh rate (around 30/40hz).
    * 
    * NOTE: the refresh rate for a given mode varies from display to display. 
    * Once the mode set, use getRefreshRate() to find out the real refresh rate.
    *
    * Remark: calling this method reset the statistics.
    **/
    void setRefreshMode(int mode);


    /**
    * Return the current refresh mode. 
    *
    * - 0  : fastest refresh rate (around than 120/140hz). 
    * - 31 : slowest refresh rate (around 30/40hz). 
    *
    * NOTE: the refresh rate for a given mode varies from display to display.
    **/
    int getRefreshMode() const { return _refreshmode; }


    /**
    * Set the refresh mode for the display to match the requested refresh rate (in hz). 
    * as close as possible. 
    * 
    * After this method returns. Use getRefreshMode() and getRefreshRate() to find
    * out the actual mode and exact refresh rate set. Note that these values will varies
    * from display to display. 
    *
    * Remark: calling this method reset the statistics.
    **/
    void setRefreshRate(double refreshrate_hz)
        {
        const int m = _modeForRefreshRate(refreshrate_hz);
        setRefreshMode(m);
        }


    /**
    * Get the refresh rate in Hz of the display corresponding to the current
    * refresh mode set.
    **/
    double getRefreshRate() const { return (_period == 0) ? 0.0 : 1000000.0 / _period; }


    /**
    * Display all the screen refresh mode with corresponding refresh rates. 
    * 
    * This method is will take a few seconds as its cycles through all the modes
    * and must sample the exact refresh rate each time.
    **/
    void printRefreshMode(Stream* outputStream = &Serial);



    /**
    * This parameter defines the upload stategy and determines how the actual framerate relates 
    * to the display refresh rate. 
    *
    * This number must be between -1 and 10 = ILI9341_T4_MAX_VSYNC_SPACING.
    *
    * - vsync_spacing = -1. In this case, screen updates occur as soon as possible and some frames 
    *                       may even be dropped if they are pushed to update() faster than they can
    *                       be uploaded to the screen. This mode will provide the fastest 'apparent' 
    *                       framerate but at the expanse of screen tearing (no vsync) and it also
    *                       does not insure any framerate control/stability. 
    *
    * - vsync_spacing = 0. Again, screen updates are pushed to the screen as fast as possible but 
    *                      frames are not dropped so if all the internal framebuffers are filled
    *                      the  update() method will wait until it can buffer/push the next frame 
    *                      on the screen. Again, this mode provides no guaranty concerning screen 
    *                      tearing nor framerate stability. 
    *
    * - vsync_spacing > 0. The updates are synchronized with the screen refresh. This approach has 
	*                      two advantages:
    *                      1) It prevents screen tearing.
    *                      2) It insure a constant framerate.
    *
    *                      vsync_spacing is the number of screen refresh between two consecutives 
    *                      updates. Thus, the real framerate is given by:
    *
    *                      real_framerate = screen_refresh_rate / vsync_spacing
    *
    *                      vsync_spacing should be set to either 1 (framerate = refresh_rate) 
    *                      or 2 (framerate = half refresh rate). Larger values are not really 
    *                      useful except for corner cases. 
    *
    * NOTE 1: In order to insure that screen tearing cannot occur, the upload must be done fast 
    *         enough so that the refresh scanline does not catch up with the pixels being drawn. 
	*         This is true when the update must take at most:
    *                               (2 - 1/nb_split) * refresh_period 
    *         where nb_split in the number of splitting parts of the diff and refresh period is
    *         the inverse of the refresh rate obviously. Thus increasing the number of splitting 
	*         parts of the diffs with setDiffSplit() may help prevent screen tearing (but only
	*         up to a point). 
    *
    * NOTE 2: As stated above, screen tearing can be prevented if the frame upload can be done
    *         in roughly less than 2 refresh period. This is the reason why vsync_spacing = 2 
    *         should be the optimal choice in most case. However, if the diff is simple enough 
	*         so that the frame upload rate is faster than the refresh rate and also that frames
    *         can be generated fast enough,  then it may be viable to set vsync_spacing = 1 and 
	*         enjoy very high framerate without screen tearing! yeah :-) 
	*         This is the case for instance when the screen diplay a 'UI' where only text/widgets
	*         changes. 
    *
    * ADVICE : FOR MOST CASE: USE vsync_spacing = 2 AND ADJUST THE DISPLAY REFRESH RATE WITH 
    *          setRefreshRate() TO GET A CONSISTENT FRAMERATE. 
    * 
    * Remark: calling this method reset the statistics.
    **/
    void setVsyncSpacing(int vsync_spacing = ILI9341_T4_DEFAULT_VSYNC_SPACING)
        {
        waitUpdateAsyncComplete();
        _vsync_spacing = _clip(vsync_spacing, -1, ILI9341_T4_MAX_VSYNC_SPACING);
        statsReset();
        }


    /**
    * Return the current vsync_spacing parameter. 
    **/
    int getVsyncSpacing() const { return _vsync_spacing;  }


    /**
    * Set the gap used when creating diffs. 
    * 
    * See the DiffBuff class for more detail on the gap parameter.

    * This parameter correspond to the number of consecutive identical pixels needed to break a SPI transaction. 
    * A smaller value will give more accurate (but larger) diffs. The optimal value should be between 5 and 100.
    * 
    * Experimenting first with a gap somewhere between 10 and 20 is a safe bet.... Try gap = 5 if you can afford
    * diff buffers with up to 10K of memory. 
    *
    * Remark: calling this method reset the statistics.
    **/
    void setDiffGap(int gap = ILI9341_T4_DEFAULT_DIFF_GAP)
        {
        waitUpdateAsyncComplete();
        _diff_gap = _clip(gap,1,ILI9341_T4_NB_PIXELS);
        statsReset();
        }


    /**
    * Return the current gap used for creating diffs. 
    **/
    int getDiffGap() const { return _diff_gap; }


    /**
    * Set the number of subframes/splits used when creating a diff. If screen tearing occur, you can try increasing 
    * this value but setting it too large will have the opposite effect...  The default value should be good for
    * most cases. 
    *
    * Remark: calling this method reset the statistics.
    **/
    void setDiffSplit(int nb_split = ILI9341_T4_DEFAULT_DIFF_SPLIT)
        {
        waitUpdateAsyncComplete();
        _diff_nb_split = _clip(nb_split, 1, DiffBuffBase::MAX_NB_SUBFRAME);
        statsReset();
        }


    /**
    * Return the number of subframe/splits per diff. 
    **/
    int getDiffSplit() const { return _diff_nb_split; }


    /**
    * Set how late we can be behind the initial sync line and still start uploading a frame without
    * waiting for the next refresh for synchronization. Set a value in [0.0, 1.0]
    * 
    * - Choosing a small value will reduce screen tearing but may make the framerate oscillate more 
    *   when the timing is tight.
    * 
    * - Choosing a large value will stabilize the framerate but at the expense of more screen tearing 
    *   when the timing is tight.
    *
    * Remark: calling this method reset the statistics.
    **/
    void setLateStartRatio(double ratio = ILI9341_T4_DEFAULT_LATE_START_RATIO)
        {
        waitUpdateAsyncComplete();
        _late_start_ratio = _clip(ratio, 0.0, 1.0);
        statsReset();
        }


    /**
    * Return the value of the "late start ratio" parameter. 
    **/
    double getLateStartRatio() const { return _late_start_ratio; }



    /**
    * Set the mask used when creating a diff to check is a pixel is the same in both framebuffers. 
    * If the mask set is non-zero, then only the bits set in the mask are used for the comparison 
    * so pixels with differents values may be considered equal and may not redrawn.
    * 
    * Setting a mask may be useful when the framebuffer being uploaded to the screen comes from
    * a camera or another source that introduces random noise that would prevent the diff from
    * finding large gap hence making it pretty useless. 
    * Typically, you want to set the lower bits on each channel color to 0 so that color that
    * are 'close' are not always redrawn (see the other method version below). 
    * 
    * If called without argument, the compare mask is set to 0 hence disabled and strict equality
    * is enforced when creating diffs (default behaviour).
    **/
    void setCompareMask(uint16_t mask = 0)
        {
        if (mask == 65535) mask = 0;
        _compare_mask = mask;
        }


    /**
    * Set the compare mask by specifying for each color channel the number of lower bits
    * that should be ignored. 
    * 
    * Recall the there are 5 bits for the blue and  red channel and 6 bits for the green one. 
    **/
    void setCompareMask(int bitskip_red, int bitskip_green, int bitskip_blue)
        {
        _compare_mask = (((uint16_t)(((0xFF >> bitskip_red) << bitskip_red) & 31)) << 11)
                      | (((uint16_t)(((0xFF >> bitskip_green) << bitskip_green) & 63)) << 5)
                      | ((uint16_t)(((0xFF >> bitskip_blue) << bitskip_blue) & 31));
        if (_compare_mask == 65535) _compare_mask = 0; 
        }


    /**
    * Return the value of the current compare_mask. 
    * Return 0 if the mask is not set and strict comparison of pixel colors is enforced 
    * (default behaviour).
    **/
    uint16_t getCompareMask() const { return _compare_mask; }


    /**
    * Set/remove one (or two) internal framebuffers.
    * 
    * The mode of operation of the update() method depend on the number of internal diff buffer /framebuffers set:
    *
    * - 0 framebuffer: all updates operations are carried immediately (mode: NO_BUFFERING)
    * - 1 framebuffers + 1 diff buffer : double buffering using asynchronous transfer via DMA (mode: DOUBLE_BUFFERING_ONE_DIFF)
    * - 1 framebuffers + 2 diff buffer : faster double buffering using asynchronous transfer via DMA (mode: DOUBLE_BUFFERING_TWO_DIFF)
    * - 2 framebuffers + 2 diff buffer : triple buffering using asynchronous transfer via DMA  (mode: TRIPLE_BUFFERING)
    * 
    * ----------------------------------------------------------------------------------------------
    * THESE FRAMEBUFFERS ARE GIVEN FOR INTERNAL USE BY THE CLASS AND MUST NOT BE MODIFIED ONCE SET. 
    * YOU MUST USE ANOTHER FRAMEBUFFER FOR DRAWING AND THEN PASSING IT TO THE update() METHOD !
    * IF YOU NEED TO GET THE FRAMEBUFFERS BACK, CALL THE METHOD AGAIN BUT WITH EMPTY PARAMETERS TO 
    * DETACH THEM FROM THIS OBJECT. 
    * ----------------------------------------------------------------------------------------------
    * 
    * Remark: calling this method reset the statistics.
    **/
    void setFramebuffers(uint16_t* fb1 = nullptr, uint16_t * fb2 = nullptr);


    /**
    * Set/remove one (or two) internal diff buffers.
    *
    * The mode of operation of the update() method depend on the number of internal diff buffer /framebuffers set:
    *
    * - 0 framebuffer: all updates operations are carried immediately (mode: NO_BUFFERING)
    * - 1 framebuffers + 1 diff buffer : double buffering using asynchronous transfer via DMA (mode: DOUBLE_BUFFERING_ONE_DIFF)
    * - 1 framebuffers + 2 diff buffer : faster double buffering using asynchronous transfer via DMA (mode: DOUBLE_BUFFERING_TWO_DIFF)
    * - 2 framebuffers + 2 diff buffer : triple buffering using asynchronous transfer via DMA  (mode: TRIPLE_BUFFERING)
    *
    * ----------------------------------------------------------------------------------------------
    * ONCE SET, THE DIFFBUFFER BELONG TO THIS OBJECT AND MUST NOT BE USED FOR CREATING OR READING 
	* DIFFS OR THE WHOLE PROGRAM MAY CRASH !!!
    *
    * HOWEVER, IT IS STILL POSSIBLE TO CALL ALL THE STATSXXX() METHOD OF THE DIFFS TO CHECK MEMORY 
    * CONSUMPTION / CPU USAGE...
    * 
    * IF YOU WANT THE DIFF BUFFER BACK, JUST CALL THE METHOD AGAIN WITH EMPTY PARAMETERS. 
    * ----------------------------------------------------------------------------------------------
    * 
    * Remark: calling this method reset the stats if the buffering mode changes
    **/
    void setDiffBuffers(DiffBuffBase* diff1 = nullptr, DiffBuffBase* diff2 = nullptr);



    /** Buffering mode*/
    enum
        {
        NO_BUFFERING = 0,
        DOUBLE_BUFFERING_ONE_DIFF = 2,
        DOUBLE_BUFFERING_TWO_DIFF = 3,
        TRIPLE_BUFFERING = 4
        };


    /**
    * Return the current buffering mode:
    * 
    * - NO_BUFFERING              : all updates operations are carried immediately.
    * - DOUBLE_BUFFERING_ONE_DIFF : double buffering using asynchronous transfer via DMA.
    * - DOUBLE_BUFFERING_TWO_DIFF : faster double buffering using asynchronous transfer via DMA.
    * - TRIPLE_BUFFERING          : triple buffering using asynchronous transfer via DMA.
    **/
    int bufferingMode() const
        {
        if ((_fb1 == nullptr) || (_diff1 == nullptr)) return NO_BUFFERING;
        if (_diff2 == nullptr) return DOUBLE_BUFFERING_ONE_DIFF;
        if (_fb2 == nullptr) return DOUBLE_BUFFERING_TWO_DIFF;
        return TRIPLE_BUFFERING;
        }




    /***************************************************************************************************
    ****************************************************************************************************
    *
    * Screen updates
    *
    ****************************************************************************************************
    ****************************************************************************************************/




    /**
    *                              MAIN SCREEN UPDATE METHOD
    * 
    * Push a framebuffer to be displayed on the screen. The behaviour of the method depend on the 
    * current buffering mode and the vsync_spacing parameter. 
    * 
    * If force_full_redraw = true, then the whole screen updated even if a diff could have been used. 
    * Normally, this option should not be needed except in the case you know for sure that the diff
    * will be useless. 
    * 
    * When the method returns, the framebuffer may (or not) already be displayed on the screen yet
    * it can be reused immediately in any case. 
    * 
    * Buffering mode:
    * 
    * NO_BUFFERING: 
	*
	*   The framebuffer is displayed immediately, the method returns only when upload is complete.
    * 
    *   - if vsync_spacing <= 0. upload to the screen start immedialely (no vsync). 
    * 
    *   - if vsync_spacing >= 1. screen upload is synchronized with the screen refresh and the 
    *     method waits until vsync_spacing refreshes have occured since the previous update to insure
    *     a constant framerate equal to  (refresh_rate/vsync_spacing).
    *                      
    * 
    * DOUBLE_BUFFERING_ONE_DIFF / DOUBLE_BUFFERING_TWO_DIFF
    * 
    *   All updates are done async. via DMA and the method returns asap. 
    *
    *   - if vsync_spacing = -1. upload to the screen start immedialely unless there is already a
    *     transfer in progress in which case the frame is simply dropped. 
    * 
    *   - if vsync_spacing = 0. upload to the screen start immedialely unless there is already a
    *     transfer in progress in which case it waits until the transfer completes and then start
    *     immediatly another async transfer via DMA (no vsync). 
    * 
    *   - if vsync_spacing > 0. screen upload is synchronized with the screen refresh and the 
    *     method waits until vsync_spacing refreshes have occured since the previous update to 
    *     insure a constant framerate equal to  (refresh_rate/vsync_spacing). If a transfer is 
    *     already in progress, it waits for it to complete before scheduling the next transfer
    *     via DMA and returning.
    *   
    * 
    * TRIPLE_BUFFERING
    *
	*   All updates are done async. via DMA and the method returns asap. 
    *
    *   - if vsync_spacing = -1. Upload to the screen start immedialely unless there is already a
    *     transfer in progress in which case the frame is saved in the second internal framebuffer
    *     (possibly replacing a previous framebuffer waiting to be displayed). 
    *
    *   - if vsync_spacing = 0.  Upload to the screen start immedialely unless there is already a
    *     transfer in progress in which case the frame is saved in the second internal framebuffer
    *     if is is free otherwise it waits until the transfer completes and is then saved in the
    *     newly freed internal framebuffer and scheduled to be displayed asap. 
    *
    *   - if vsync_spacing > 1. screen upload is synchronized with the screen refresh and the 
    *     method waits until vsync_spacing refreshes have occured since the previous update to 
    *     insure a constant framerate equal to  (refresh_rate/vsync_spacing). If a transfer is 
    *     already in progress, it will save the framebuffer in the second internal framebuffer
    *     if available then schedule the redraw and return immediately. If the second framebuffer
    *     is already full, the method wait for the current transfer to complete before saving 
    *     the framebuffer and scheduling the redraw and then returning. 
    * 
    * 
    * NOTE: (1) double buffering give a HUGE improvement over the no buffering method at the 
    *           expense of a second framebuffer and at least one diff buffer.
    *       (2) A second diff buffer cost only a few additonal kilobytes but will improve the 
    *           framerate in cases where the upload takes most of a frame period. 
    *       (3) Triple buffering requires another framebuffer dedicated for storing intermediate 
    *           frames. Some testing suggests that triple buffering provides only modest improvement 
    *           over double buffering with two diff buffers hence it should be used only if you 
    *           really have nothing better to do will all that memory !
    *     
    * ADVICE: Use double buffering with two diff buffers (with size ranging from 5K to 10K). 
    **/
    void update(const uint16_t* fb, bool force_full_redraw = false);


    /**
    * Wait until any currently ongoing async update completes.
    * 
    * Returns immediatly if no update is ongoing.
    * 
    * NOTE: This method should not be called in normal use cases as it will create a 
            "busy wait" and wastes precious CPU time...
    **/
    inline void waitUpdateAsyncComplete() { while ((_dma_state != ILI9341_T4_DMA_IDLE)); }


    /**
    * Return true if an Async update is currently ongoing and false otherwise.
    **/
    inline bool asyncUpdateActive() const { return (_dma_state != ILI9341_T4_DMA_IDLE); }



    /***************************************************************************************************
    ****************************************************************************************************
    *
    * Statistics
    *
    ****************************************************************************************************
    ****************************************************************************************************/


    /**
    * Reset all statistics
    **/
    void statsReset(); 


    /**
    * Return the number of frames drawn since the last call to statReset().
    **/
    uint32_t statsNbFrames() const { return _stats_nb_frame; }


    /**
    * Return the number of milliseconds since the last call to statReset().
    **/
    uint32_t statsTotalTime() const { return _stats_elapsed_total; }


    /**
    * Return the average framerate in Hz which is simply the number of 
    * frame drawn divided the total time. 
    **/
    double statsFramerate() const { return  (_stats_nb_frame == 0) ? 0.0 : ((_stats_nb_frame * 1000.0) / _stats_elapsed_total); }


    /**
    * Return an object containing statistics about the CPU time
    * used spend preparing and updating the screen (dma interrupt time). 
    * !!! This does NOT count the time needed to create the diffs. !!!
    **/
    StatsVar statsCPUtimePerFrame() const { return _statsvar_cputime; }


    /**
    * Return an object containing statistics about the number of pixels
    * uploaded per frame. 
    **/
    StatsVar statsPixelsPerFrame() const { return _statsvar_uploaded_pixels; }


    /**
    * Return an object containing statistics about the number of transactions
    * per frame.
    **/
    StatsVar statsTransactionsPerFrame() const { return _statsvar_transactions; }


    /**
    * Return an object containing statistics about the "margin" during upload a vsynced frame.
    * 
    * The margin of a vsynced uploaded frame is minimum difference during the upload between 
    * the position of the pixels being uploaded and the screen scanline currently beign refreshed.
    *
    * When this value becomes negative, it means tearing occurs. A large positive value means that 
    * there is plenty of time for redraw without tearing so the framerate may be increased.
    **/
    StatsVar statsMarginPerFrame() const { return _statsvar_margin; }


    /**
    * Return an object containing the effective statistics about the vsync_spacing beween screen refresh. 
    **/
    StatsVar statsRealVSyncSpacing() const { return _statsvar_vsyncspacing; }


    /**
    * Return the number of frame with vsync active for which screen tearing may
    * have occured.
    **/
    uint32_t statsNbTeared() const { return _nbteared; }


    /**
    * Return the ratio of frame with vsync active for which screen tearing
    * may have occured.
    * (returns 1.0 when vsync is OFF).  
    **/
    double statsRatioTeared() const { return (_vsync_spacing <= 0) ? 1.0 : ((_statsvar_vsyncspacing.count() == 0) ? 0.0 : (((double)_nbteared) / _statsvar_margin.count())); }


    /**
    * Output statistics about the object into a stream.
    * 
    * Useful for fine-tuning the parameters.
    **/
    void printStats(Stream* outputStream = &Serial) const;

   



    /***************************************************************************************************
    ****************************************************************************************************
    *
    * Touch screen.
    *
    * These methods are available only if the XPT2048 touchscreen is on the same SPI bus as the screen
    * and the touch_cs (and optionally _touch_irq) pin have been assigned in the constructor.
    *
    ****************************************************************************************************
    ****************************************************************************************************/


    /**
    * Return the number of milliseconds since the touch interrupt was last triggered or -1 if no touch
    * interrupt occured since the last call to lastTouched().
    *
    * If the touch_irq pin is not assigned, this method always return -1;
    *
    * The benefit of lastTouched() over readTouch() is that it never uses the SPI bus (and thus always
    * returns immediately).
    **/
    int32_t lastTouched();


    /**
    * Read the touchscreen. Return the position (x,y) and pressure level z.
    *
    * If the touch_irq pin is assigned, it will avoid using the spi bus whenever possible.
    *
    * If the spi bus must be used. The method will wait until the current ongoinc async
    * transfer completes (if any).
    **/
    void readTouch(int& x, int& y, int& z);


    /**
    * Set a mapping from touch coordinates to screen coordinates.
    *
    * - Until this method is called (or after being called with default parameters),
    *   readTouch() returns the "raw" values for the (x,y) coordinate.
    *
    * - Once the range has been set, readTouch() will subsequently return the (x,y)
    *    coordinates mapped to [0, width-1] * [0, height-1] where width and height
    *    are the screen dimensions. 
    *
    * NOTE: The range should be reset when the screen orientation changes.
    **/
    void setTouchRange(int minx = 0, int maxx = 0, int miny = 0, int maxy = 0);







private:


    /**********************************************************************************************************
    * 
    * You shall go no further.
    * Hey ! This is private ! don't look !
    * 
    ***********************************************************************************************************/



    /**********************************************************************************************************
    * General settings.
    ***********************************************************************************************************/

    typedef void (*callback_t)(void*);                  // function callback signature 
    using methodCB_t = void (ILI9341Driver::*)(void);   // typedef to method callback. 


    int16_t _width, _height;                    // Display w/h as modified by current rotation    
    int     _rotation;                          // current screen orientation
    int     _refreshmode;                       // refresh mode (between 0 = fastest refresh rate and 15 = slowest refresh rate). 



    /**********************************************************************************************************
    * About buffering / update mode. 
    ***********************************************************************************************************/


    volatile int _diff_nb_split;                // number of subframes per diff
    volatile int _diff_gap;                     // gap when creating diffs.
    volatile int _vsync_spacing;                // update stategy / framerate divider. 
    volatile double _late_start_ratio;          // late start parameter (by how much we can miss the first sync line and still start the frame without waiting for the next refresh).
    volatile uint16_t _compare_mask;             // the compare mask used to compare pixels when doing a diff

    DiffBuffBase * volatile  _diff1;             // first diff buffer
    DiffBuffBase * volatile  _diff2;             // second diff buffer (if non null, then _diff1 is also non zero). 
    DiffBuffDummy * volatile _dummydiff1;        // fake diff buffer used for complete refresh and when buffering is disabled. 
    DiffBuffDummy * volatile _dummydiff2;        // fake diff buffer used for complete refresh and when buffering is disabled. 

    DiffBuffDummy _dd1, _dd2;                    // the dummy diff themselves. 

    uint16_t* volatile _fb1;                    // first internal framebuffer
    uint16_t* volatile _fb2;                    // second internal framebuffer (if non null, then _fb1 is also non zero). 
    uint16_t* volatile _mirrorfb;               // framebuffer that currently mirrors the screen (or will mirror it when upload completes).
    volatile bool _fb2full;                     // true if the second framebuffer is currently full and waiting to be uploaded. 



    /** called when fb2 is full and must be drawn on the screen */
    void _buffer2fullCB();


    /**
    * Update part of the screen using a diff buffer object representing the changes between
    * the old framebuffer and the new one 'fb'.
    * - return only when update completed.
    * - uses the _vsync_spacing parameter to choose the vsync stategy.
    **/
    void _updateNow(const uint16_t* fb, DiffBuffBase* diff);


    /**
    * Update part of the screen using a diff buffer object representing the changes between
    * the old framebuffer and the new one 'fb'.
    * - return asap and update is async via DMA.
    * - uses the _vsync_spacing parameter to choose the vsync stategy.
    **/
    void _updateAsync(const uint16_t* fb, DiffBuffBase* diff);



    /** clip val to [min,max] */
    template<typename T> static T _clip(T val, T min, T max)
        {
        if (val < min) val = min;
        if (val > max) val = max;
        return val;
        }


    /** copy between framebuffers*/
    void _copyfb(uint16_t * fb_dst, const uint16_t * fb_src)
        {
        memcpy(fb_dst, fb_src, ILI9341_T4_NB_PIXELS*2); 
        return;
        if (((((size_t)fb_dst) & 3) == 0) && ((((size_t)fb_src) & 3) == 0))
            { // use fast aligned copy
            uint32_t* f1 = (uint32_t*)fb_dst;
            uint32_t* f2 = (uint32_t*)fb_src;
            int n = ILI9341_T4_NB_PIXELS / 32;  // we know it is a multiple of 32
            while (n-- > 0)
                {
                *(f1++) = *(f2++); *(f1++) = *(f2++); *(f1++) = *(f2++); *(f1++) = *(f2++); *(f1++) = *(f2++); *(f1++) = *(f2++); *(f1++) = *(f2++); *(f1++) = *(f2++);
                *(f1++) = *(f2++); *(f1++) = *(f2++); *(f1++) = *(f2++); *(f1++) = *(f2++); *(f1++) = *(f2++); *(f1++) = *(f2++); *(f1++) = *(f2++); *(f1++) = *(f2++);
                }
            }
        else
            { // falls back to slow copy. 
            for (int i = 0; i < ILI9341_T4_NB_PIXELS; i++) fb_dst[i] = fb_src[i];                
            }
       
        }


    /** swap _diff1 and _diff2 */
    void _swapdiff() { auto t = _diff1;  _diff1 = _diff2;  _diff2 = t;  }


    /** swap the dummydiff */
    void _swapdummydiff() { auto t = _dummydiff1; _dummydiff1 = _dummydiff2; _dummydiff2 = t; }


    /** swap _fb1 and _fb2 */
    void _swapfb() { auto t = _fb1; _fb1 = _fb2; _fb2 = t; }




    /**********************************************************************************************************
    * About timing and vsync. 
    ***********************************************************************************************************/

    uint32_t _period_mode0;                     // number of microsceonds between screen refresh for the fastest mode. 

    uint32_t _period;                           // number of microsceonds between screen refresh. 
    elapsedMicros _synced_em;                   // number of microseconds sinces the last scanline synchronization
    uint32_t _synced_scanline;                  // scanline at the time of the last synchronization
    uint32_t _timeframestart;                   // time when drawing the last uploaded frame will start. 



    /** 
    * wait a given number of microseconds provided the delay is smaller
    * than ILI9341_T4_MAX_DELAY_MICROSECONDS 
    **/
    void delayMicro(uint32_t t)
        {
        if (t < ILI9341_T4_MAX_DELAY_MICROSECONDS) delayMicroseconds(t); // only allow delay of at most 1 second. 
        }


    /**
    * Number of scanline drawn during t microseconds. 
    **/
    int _nbScanlineDuring(uint32_t t) const
        {
        return ((t * ILI9341_T4_NB_SCANLINE) / _period);
        }


    /**
    * Number of microsecond to perform a given number of scanlines
    **/
    uint32_t timeForScanline(uint32_t nbscanline) const
        {
        return (nbscanline * _period) / ILI9341_T4_NB_SCANLINE;
        }


    /**
    * Convert a ratio in [0.0, 1.0] to a scanline in [|0, ILI9341_T4_NB_SCANLINE - 1|]
    **/
    int _ratioToScanline(double r) const __attribute__((always_inline))
        {
        int l = (int)(r * ILI9341_T4_NB_SCANLINE);
        if (l < 0) l = 0;
        if (l >= ILI9341_T4_NB_SCANLINE) l = ILI9341_T4_NB_SCANLINE - 1;
        return l;
        }


    /**
    * Return true is the current scanline is in the given range [|start, end|] 
    **/
    bool _isScanlineInRange(int start, int end)  __attribute__((always_inline))
        {
        int v = _getScanLine(false);
        return ((start <= v) && (v <= end));
        }


    /** 
    * Return the number of microsecond before we exit the given range. 
    **/
    uint32_t _microToExitRange(int start, int end) 
        {
        const int delta = end - start;
        if ((delta < 0) || ((5*delta) >= (4*ILI9341_T4_NB_SCANLINE))) return 0; // invalid range (negative or too big). 
        int v = _getScanLine(false);
        if ((v < start) || (v > end)) return 0; // no wait needed. 
        return 1 + _microToReachScanLine(((end + 1) % ILI9341_T4_NB_SCANLINE), false); // make sur it is not 0. 
        }


    /**     
    * Return the number of microseconds remaining until we reach a given scanline.
    * WARNING: if sync = true, SPI must NOT be in use !
    **/
    uint32_t _microToReachScanLine(int scanline, bool sync) __attribute__((always_inline))
        {
        int now = _getScanLine(sync);
        const uint32_t diff = (now <= scanline) ? (uint32_t)(scanline - now) : (uint32_t)(scanline - now + ILI9341_T4_NB_SCANLINE);
        return (diff* _period) / ILI9341_T4_NB_SCANLINE;
        }


    /** 
    * Return the current scanline 
    * WARNING: if sync = true, SPI must NOT be in use !
    **/
    int _getScanLine(bool sync);


    /** 
    * estimate the current refresh rate by banging the get_scanline 0x45 command 
    **/
    void _sampleRefreshRate();


    /** 
    * estimate the refreshrate (in Hz) for a given mode based on the refresh rate for mode 0 
    **/
    double _refreshRateForMode(int mode) const;


    /**
    * Find the mode with closest refresh rate. 
    **/
    int _modeForRefreshRate(double hz) const;



    /**********************************************************************************************************
    * About DMA
    ***********************************************************************************************************/

    volatile methodCB_t _pcb;                   // function callback (nullptr if none) 

    const uint16_t* volatile _fb;               // the framebuffer to push

    DiffBuffBase* volatile _diff;               // and corresponding diff buffer

    volatile int _stride;                       // stride for the diff buffer (should be equal to _width).

    elapsedMicros _em_async;                    // timer for async drawing
    volatile uint32_t _nbdrawnstart;            // remember begining of (previous) subframe. 
    volatile uint32_t _slinitpos;               // initial scanline position
    volatile int32_t _margin;                   // margin between scanline and redraw line. 
    volatile uint32_t _last_delta;              // number of refresh that occured beetween the previous two frames. 

    static ILI9341Driver * volatile _dmaObject[3];  // points back to this-> (for the corresponding spi bus)

    enum
        {
        ILI9341_T4_DMA_IDLE = 0,
        ILI9341_T4_DMA_ON = 2,
        };

    volatile uint8_t _dma_state;                // DMA current status

    DMAChannel _dmatx;                          // the dma channel object. 

    DMASetting          _dmasettingsDiff[6];    // dma settings chain

    uint8_t             _comCommand0;           // the commands for DMASettings 0
    uint8_t             _comCommand2;           // the commands for DMASettings 2
    uint8_t             _comCommand4;           // the commands for DMASettings 4

    uint16_t            _comData1;              // the data for DMASetting 1
    uint16_t            _comData3;              // the data for DMASetting 3

    int                 _prev_caset_x;          // previous position set with the caset command
    int                 _prev_paset_y;          // previous position set with the paset command

    volatile int        _partdma = 0;           // which dmasettingsDiff is currently loaded in _dmatx

 
    static void _dmaInterruptSPI0Diff() { if (_dmaObject[0]) { _dmaObject[0]->_dmaInterruptDiff(); } } // called when using spi 0
    static void _dmaInterruptSPI1Diff() { if (_dmaObject[1]) { _dmaObject[1]->_dmaInterruptDiff(); } } // called when using spi 1
    static void _dmaInterruptSPI2Diff() { if (_dmaObject[2]) { _dmaObject[2]->_dmaInterruptDiff(); } } // called when using spi 2

    void _dmaInterruptDiff(); // called when doing partial diff redraw


    /** set/remove  the callback at end of transfer */
    void _setCB(methodCB_t pcb = nullptr) { _pcb = pcb; }


    /** 
     * flush the cache if the array is located in DMAMEM.
     * This can take a while (100us) so don't abuse it !
     **/
    void _flush_cache(const void* ptr, size_t len) __attribute__((always_inline))
        {
        if ((uint32_t)ptr >= 0x20200000u) arm_dcache_flush((void*)ptr, len);
        asm("dsb");
        }


    void _subFrameTimerStartcb();    // called at start of subframe

    void _subFrameTimerStartcb2();   // called at start of subframe

    void _subFrameInterruptDiff();   // called by _dmaInterruptDiff() and by timer when changing subframe.

    void _subFrameInterruptDiff2();  // called by after a pause for synchronization


 

    /**********************************************************************************************************
    * IntervalTimer
    ***********************************************************************************************************/


    IntervalTimer _it;      // the PIT timer object 

    static ILI9341Driver* volatile _pitObj[4];   // point back to this->
    volatile int _pitindex; // index for which pitObj this-> refers to. 
    volatile bool _istimer; // true if a timer is currently waiting to ring. 

    static void _pitcb0() { if (_pitObj[0]) { _pitObj[0]->_it.end(); _pitObj[0]->_istimer = false; ((_pitObj[0])->*(_pitObj[0]->_pitcb))(); } }  // forward to the timer method cb
    static void _pitcb1() { if (_pitObj[1]) { _pitObj[1]->_it.end(); _pitObj[1]->_istimer = false; ((_pitObj[1])->*(_pitObj[1]->_pitcb))(); } }  // forward to the timer method cb
    static void _pitcb2() { if (_pitObj[2]) { _pitObj[2]->_it.end(); _pitObj[2]->_istimer = false; ((_pitObj[2])->*(_pitObj[2]->_pitcb))(); } }  // forward to the timer method cb
    static void _pitcb3() { if (_pitObj[3]) { _pitObj[3]->_it.end(); _pitObj[3]->_istimer = false; ((_pitObj[3])->*(_pitObj[3]->_pitcb))(); } }  // forward to the timer method cb

    volatile methodCB_t _pitcb;        // timer callback method. 


    /** call at startup to initialize the timer */
    void _timerinit(); 


    /** Set the timer to ring in us microseconds. */
    void _setTimerIn(uint32_t us, methodCB_t timercb) __attribute__((always_inline))
        {
        _it.end(); // stop ongoing timer before changing callback method. 
        _pitcb = timercb;
        if ((us <= 1) ||(us > ILI9341_T4_MAX_DELAY_MICROSECONDS)) { us = 1; } // asap
        NVIC_SET_PRIORITY(IRQ_PIT, ILI9341_T4_IRQ_PRIORITY); // set priority directly for all pit timer (on teensy 4: priority is shared by all pit timer). 
        _istimer = true;
        switch (_pitindex)
            {
            case 0: _it.begin(_pitcb0, us); break;
            case 1: _it.begin(_pitcb1, us); break;
            case 2: _it.begin(_pitcb2, us); break;
            case 3: _it.begin(_pitcb3, us); break;
            }                
        NVIC_SET_PRIORITY(IRQ_PIT, ILI9341_T4_IRQ_PRIORITY); // just in case...
        }


    /** Set the timer to ring when micros() reaches ustime */
    void _setTimerAt(uint32_t ustime, methodCB_t timercb) __attribute__((always_inline))
        {
        const uint32_t m = micros(); 
        const int32_t d = (int32_t)(ustime - m); 
        _setTimerIn( (d < 1) ? 1 : (uint32_t)d, timercb); // asap if d <= 0 or more than 1 seconds. 
        }


    /** Cancel the timer (if ticking). */
    void _cancelTimer() __attribute__((always_inline))
        {
        _it.end();
        _istimer = false;
        }


    /** Query if the timer is currently ticking. */
    bool _isTimer() const __attribute__((always_inline))
        {
        return _istimer;
        }



   /**********************************************************************************************************
   * About Stats
   ***********************************************************************************************************/


        uint32_t        _stats_nb_frame;            // number of frame drawn since last reset. 
        elapsedMillis   _stats_elapsed_total;       // total time since the last reset. 

        elapsedMicros   _stats_elapsed_cputime;     // timer for the cpu time use during a frame
        uint32_t        _stats_cputime;             // cpu time spend in a frame
        StatsVar        _statsvar_cputime;          // statistics about the cpu time usage. 

        elapsedMicros   _stats_elapsed_uploadtime;  // timer for the dma time single frame
        uint32_t        _stats_uploadtime;          // cpu time spend in a frame
        StatsVar        _statsvar_uploadtime;       // statistics about the cpu time usage. 

        uint32_t        _stats_nb_uploaded_pixels;  // number of pixel upload during a frame. 
        StatsVar        _statsvar_uploaded_pixels;  // statistics about the number of pixels uploaded per frame.

        uint32_t        _stats_nb_transactions;     // number of transactions for the frame.
        StatsVar        _statsvar_transactions;     // statistics about the number of transactions per frame.

        StatsVar        _statsvar_margin;           // statistics about the 'margin' per frame

        StatsVar        _statsvar_vsyncspacing;     // statistics about the effective vsync_spacing

        uint32_t        _nbteared;                  // number of frame for which screen tearing may have occured. 

    
        void _startframe(bool vsynonc)
            {
            _stats_nb_uploaded_pixels = 0; 
            _stats_nb_transactions = 0;
            _stats_cputime = 0;;           
            _stats_elapsed_cputime = 0; 
            _stats_uploadtime = 0; 
            _stats_elapsed_uploadtime = 0;
            }


        void _restartCpuTime() __attribute__((always_inline))
            {
            _stats_elapsed_cputime = 0;
            }


        void _pauseCpuTime() __attribute__((always_inline))
            {
            _stats_cputime += _stats_elapsed_cputime;
            }


        void _restartUploadTime() __attribute__((always_inline))
            {
            _stats_elapsed_uploadtime = 0;
            }


        void _pauseUploadTime() __attribute__((always_inline))
            {
            _stats_uploadtime += _stats_elapsed_uploadtime;
            }


        void _endframe();






   /**********************************************************************************************************
   * About SPI
   ***********************************************************************************************************/

    uint32_t _spi_clock;                        // spi write speed
    uint32_t _spi_clock_read;                   // spi read speed

    uint8_t _cs, _dc;                           // hardware pins
    uint8_t _sclk, _mosi, _miso;                // for the screen
    uint8_t _rst;                               // 

    uint8_t _touch_cs, _touch_irq;              // pins for XPT2048 if present

    uint8_t  _spi_num;                          //  spi busd number: 0 1 or 2
    SPIClass* _pspi = nullptr;                  // the main spi object
    IMXRT_LPSPI_t* _pimxrt_spi;                 // Raw access to the spi registers (same as _pspi->port() private method). 
    SPIClass::SPI_Hardware_t* _spi_hardware;    // Raw internal spi hardware (same as _pspi->hardware() private method). Hacked from _pspi and used for DMA transfer.

    uint32_t _cspinmask;                        // mask for the CS pin when writing directly to register port (used by directWriteHigh/Low methods)
    volatile uint32_t* _csport;                 // port to which the CS pin belongs (used by directWriteHigh/Low methods)

    uint32_t _spi_tcr_current;                  // current value of the transfer command register (TCR) (see chap 48 of the IMXRT1060 manual). 
    uint32_t _tcr_dc_assert;                    // mask for the TCR register when DC is asserted (low)
    uint32_t _tcr_dc_not_assert;                // mask for the TCR register when DC is not asserted (high)

    uint8_t _pending_rx_count;                  // hack ...


    void _beginSPITransaction(uint32_t clock) __attribute__((always_inline))
        {
        _pspi->beginTransaction(SPISettings(clock, MSBFIRST, SPI_MODE0));
        _spi_tcr_current = _pimxrt_spi->TCR; //  DC is on hardware CS
        if (_csport) _directWriteLow(_csport, _cspinmask); // drive CS low
        }


    void _endSPITransaction() __attribute__((always_inline))
        {
        if (_csport) _directWriteHigh(_csport, _cspinmask); // drive CS high
        _pspi->endTransaction();
        }


    uint8_t _readcommand8(uint8_t reg, uint8_t index = 0);


    void _writecommand_cont(uint8_t c) __attribute__((always_inline))
        {
        _maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CONT); // 
        _pimxrt_spi->TDR = c;
        _pending_rx_count++; //
        _waitFifoNotFull();
        }


    void _writedata8_cont(uint8_t c) __attribute__((always_inline))
        {
        _maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CONT);
        _pimxrt_spi->TDR = c;
        _pending_rx_count++; //
        _waitFifoNotFull();
        }


    void _writedata16_cont(uint16_t d) __attribute__((always_inline))
        {
        _maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_CONT);
        _pimxrt_spi->TDR = d;
        _pending_rx_count++; //
        _waitFifoNotFull();
        }


    void _writecommand_last(uint8_t c) __attribute__((always_inline))
        {
        _maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7));
        _pimxrt_spi->TDR = c;
        // _pimxrt_spi->SR = LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF;
        _pending_rx_count++; //
        _waitTransmitComplete();
        }


    void _writedata8_last(uint8_t c) __attribute__((always_inline))
        {
        _maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));
        _pimxrt_spi->TDR = c;
        // _pimxrt_spi->SR = LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF;
        _pending_rx_count++; //
        _waitTransmitComplete();
        }


    void _writedata16_last(uint16_t d) __attribute__((always_inline))
        {
        _maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15));
        _pimxrt_spi->TDR = d;
        // _pimxrt_spi->SR = LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF;
        _pending_rx_count++; //
        _waitTransmitComplete();
        }


    void _maybeUpdateTCR(uint32_t requested_tcr_state) __attribute__((always_inline))
        {
        #define ILI9341_T4_TCR_MASK (LPSPI_TCR_PCS(3) | LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_CONT | LPSPI_TCR_RXMSK)
        if ((_spi_tcr_current & ILI9341_T4_TCR_MASK) != requested_tcr_state) // we must update the TRANSMIT COMMAND REGISTER (TCR). 
            {
            _spi_tcr_current = (_spi_tcr_current & ~ILI9341_T4_TCR_MASK) | requested_tcr_state;
            // only output when Transfer queue is empty.       
            while ((_pimxrt_spi->FSR & 0x1f));
            _pimxrt_spi->TCR = _spi_tcr_current; // update the TCR    
            }
        }


    void _waitFifoNotFull();


    void _waitTransmitComplete();


    //. From Onewire utility files
    void _directWriteLow(volatile uint32_t* base, uint32_t mask) __attribute__((always_inline)) { *(base + 34) = mask; }

    void _directWriteHigh(volatile uint32_t* base, uint32_t mask) __attribute__((always_inline)) { *(base + 33) = mask; }




    /**********************************************************************************************************
    * About Touch
    ***********************************************************************************************************/


    volatile int _touch_request_read;           // flag set to true when reading is requested. 

    elapsedMillis _em_touched_read;             // number of ms since the touch position was last read. 
    elapsedMillis _em_touched_irq;              // number of ms since the last touch irq occured
    volatile bool _touched;                     // true if touch irq has occured 
    volatile bool _touched_read;                // true if touch irq has occured 
    volatile int _touch_x, _touch_y, _touch_z;  // last touch position 

    volatile int _touch_minx;                   //
    volatile int _touch_maxx;                   // mapping from touch coord
    volatile int _touch_miny;                   // to screen coord. 
    volatile int _touch_maxy;                   // 

    static ILI9341Driver* volatile _touchObjects[4];   // point back to this->

    static void _touch_int0() { if (_touchObjects[0]) { _touchObjects[0]->_touch_int(); } }  // forward to the touch interrupt method cb
    static void _touch_int1() { if (_touchObjects[1]) { _touchObjects[1]->_touch_int(); } }  // forward to the touch interrupt method cb
    static void _touch_int2() { if (_touchObjects[2]) { _touchObjects[2]->_touch_int(); } }  // forward to the touch interrupt method cb
    static void _touch_int3() { if (_touchObjects[3]) { _touchObjects[3]->_touch_int(); } }  // forward to the touch interrupt method cb

    /** the touch interrupt */
    void _touch_int()
        {
        _touched = true;
        _touched_read = true;
        _em_touched_irq = 0;
        }


    /** set the touch interrupt routine */
    void _setTouchInterrupt();


    /** update the touch position via spi read (if needed) */
    void _updateTouch();

    /** update the touch position via spi read (if needed), may be called at dma completion */
    void _updateTouch2();

    /** poor man's noise filtering */
    static int16_t _besttwoavg(int16_t x, int16_t y, int16_t z);





};





}
/** end of file */


