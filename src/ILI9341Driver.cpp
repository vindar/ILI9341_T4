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


#include "ILI9341Driver.h"
#include <SPI.h>



namespace ILI9341_T4
{




    ILI9341Driver::ILI9341Driver(uint8_t cs, uint8_t dc, uint8_t sclk, uint8_t mosi, uint8_t miso, uint8_t rst, uint8_t touch_cs, uint8_t touch_irq)
        {
        // general
        _width = ILI9341_T4_TFTWIDTH;
        _height = ILI9341_T4_TFTHEIGHT;
        _rotation = 0;
        _refreshmode = 0; 

        // buffering
        _diff_nb_split = ILI9341_T4_DEFAULT_DIFF_SPLIT;
        _diff_gap = ILI9341_T4_DEFAULT_DIFF_GAP;
        _vsync_spacing = ILI9341_T4_DEFAULT_VSYNC_SPACING;
        _diff1 = nullptr;
        _diff2 = nullptr;
        _fb1 = nullptr;
        _fb2 = nullptr;
        _dummydiff1 = &_dd1;
        _dummydiff2 = &_dd2;
        _mirrorfb = nullptr;
        _fb2full = false;

        // vsync
        _period = 0;        
        _synced_em = 0;
        _synced_scanline = 0;
        _timeframestart = 0;

        // dma
        _pcb = nullptr;        
        _fb = nullptr;
        _diff = nullptr;
        _dma_state = ILI9341_T4_DMA_IDLE;

        _last_delta = 0; 
        _vsyncok = true;

        // spi
        _cs = cs;
        _dc = dc;
        _sclk = sclk;
        _mosi = mosi;
        _miso = miso;
        _rst = rst;
        _touch_cs = touch_cs;
        _touch_irq = touch_irq;
        _cspinmask = 0;
        _csport = NULL;

        _setTouchInterrupt();
        _timerinit();

        statsReset();
        }



    FLASHMEM bool ILI9341Driver::begin(uint32_t spi_clock, uint32_t spi_clock_read)
        {
        static const uint8_t init_commands[] = { 4, 0xEF, 0x03, 0x80, 0x02,                 // undocumented commands
                                                 4, 0xCF, 0x00, 0XC1, 0X30,                 //
                                                 5, 0xED, 0x64, 0x03, 0X12, 0X81,           //
                                                 4, 0xE8, 0x85, 0x00, 0x78,                 //
                                                 6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,     //
                                                 2, 0xF7, 0x20,                             //
                                                 3, 0xEA, 0x00, 0x00,                       //
                                                 2, ILI9341_T4_PWCTR1, 0x20, // Power control 0x23
                                                 2, ILI9341_T4_PWCTR2, 0x10, // Power control
                                                 3, ILI9341_T4_VMCTR1, 0x3e, 0x28, // VCM control
                                                 2, ILI9341_T4_VMCTR2, 0x86, // VCM control2
                                                 2, ILI9341_T4_MADCTL, 0x48, // Memory Access Control
                                                 2, ILI9341_T4_PIXFMT, 0x55, 3, ILI9341_T4_FRMCTR1, 0x00, 0x18, 4, ILI9341_T4_DFUNCTR, 0x08, 0x82, 0x27, // Display Function Control
                                                 2, 0xF2, 0x00, // Gamma Function Disable
                                                 2, ILI9341_T4_GAMMASET, 0x01, // Gamma curve selected
                                                16, ILI9341_T4_GMCTRP1, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
                                                16, ILI9341_T4_GMCTRN1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
                                             //  3, 0xb1, 0x00, 0x10 + ILI9341_T4_DEFAULT_REFRESH_MODE, // FrameRate Control 
                                                 0 };

        if (_touch_cs != 255)
            { // set touch CS high to prevent interference.
            digitalWrite(_touch_cs, HIGH);
            pinMode(_touch_cs, OUTPUT);
            digitalWrite(_touch_cs, HIGH);
            }
        if (_cs != 255)
            { // set screen CS high also. 
            digitalWrite(_cs, HIGH);
            pinMode(_cs, OUTPUT);
            digitalWrite(_cs, HIGH);
            }
        _rotation = 0; // default rotation
        // verify SPI pins are valid; allow user to say use current ones...
        _spi_clock = spi_clock;
        _spi_clock_read = spi_clock_read;
        if (SPI.pinIsMOSI(_mosi) && ((_miso == 0xff) || SPI.pinIsMISO(_miso)) && SPI.pinIsSCK(_sclk))
            {
            _pspi = &SPI;
            _spi_num = 0; // Which buss is this spi on?
            _pimxrt_spi = &IMXRT_LPSPI4_S; // Could hack our way to grab this from SPI object, but...        
            }
        else if (SPI1.pinIsMOSI(_mosi) && ((_miso == 0xff) || SPI1.pinIsMISO(_miso)) && SPI1.pinIsSCK(_sclk))
            {
            _pspi = &SPI1;
            _spi_num = 1; // Which buss is this spi on?
            _pimxrt_spi = &IMXRT_LPSPI3_S; // Could hack our way to grab this from SPI object, but...
            }
        else if (SPI2.pinIsMOSI(_mosi) && ((_miso == 0xff) || SPI2.pinIsMISO(_miso)) && SPI2.pinIsSCK(_sclk))
            {
            _pspi = &SPI2;
            _spi_num = 2; // Which buss is this spi on?
            _pimxrt_spi = &IMXRT_LPSPI1_S; // Could hack our way to grab this from SPI object, but...
            }
        else
            {
            return false; // INVALID SPI PINS !
            }
        // Make sure we have all of the proper SPI pins selected.
        _pspi->setMOSI(_mosi);
        _pspi->setSCK(_sclk);
        if (_miso != 0xff) _pspi->setMISO(_miso);

        // Hack to get hold of the SPI Hardware information...
        uint32_t* pa = (uint32_t*)((void*)_pspi);
        _spi_hardware = (SPIClass::SPI_Hardware_t*)(void*)pa[1];
        _pspi->begin();

        _pending_rx_count = 0; // Make sure it is zero if we we do a second begin...

        // CS pin direct access via port.
        _csport = portOutputRegister(_cs);
        _cspinmask = digitalPinToBitMask(_cs);
        pinMode(_cs, OUTPUT);
        _directWriteHigh(_csport, _cspinmask);

        _spi_tcr_current = _pimxrt_spi->TCR; // get the current TCR value

        if (!_pspi->pinIsChipSelect(_dc))
            {
            return false; // ERROR, DC is not a hardware CS pin for the SPI bus. 
            }
        // Ok, DC is on a hardware CS pin 
        uint8_t dc_cs_index = _pspi->setCS(_dc);
        dc_cs_index--; // convert to 0 based
        _tcr_dc_assert = LPSPI_TCR_PCS(dc_cs_index);
        _tcr_dc_not_assert = LPSPI_TCR_PCS(3);
        _maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7)); // drive DC high now. 

        if (_rst < 255)
            { // Reset the screen
            pinMode(_rst, OUTPUT);
            digitalWrite(_rst, HIGH);
            delay(10);
            digitalWrite(_rst, LOW);
            delay(20);
            digitalWrite(_rst, HIGH);
            delay(150);
            }

        _beginSPITransaction(_spi_clock);
        const uint8_t* addr = init_commands;
        while (1)
            {
            uint8_t count = *addr++;
            if (count-- == 0) break;
            _writecommand_cont(*addr++);
            while (count-- > 0) { _writedata8_cont(*addr++); }
            }
        _writecommand_last(ILI9341_T4_SLPOUT); // Exit Sleep
        _endSPITransaction();
        delay(200); // must wait for the screen to exit sleep mode. 
        _beginSPITransaction(_spi_clock);
        _writecommand_last(ILI9341_T4_DISPON); // Display on
        _endSPITransaction();

        setRefreshMode(0);
        _period_mode0 = _period; // save the period for fastest mode. 
        setRefreshRate(ILI9341_T4_DEFAULT_REFRESH_RATE); // change mode if needed. 
        statsReset();
        _mirrorfb = nullptr; // force full redraw.
        return (selfDiagStatus() == ILI9341_T4_SELFDIAG_OK);
        }


    int ILI9341Driver::selfDiagStatus()
        {
        waitUpdateAsyncComplete();
        return _readcommand8(ILI9341_T4_RDSELFDIAG);
        }


    void ILI9341Driver::printStatus(Stream* outputStream)
        {
        waitUpdateAsyncComplete();
        outputStream->printf("--- ILI9341Driver Status---\n");
        uint8_t x = _readcommand8(ILI9341_T4_RDMODE);
        outputStream->print("- Display Power Mode: 0x"); outputStream->println(x, HEX);
        x = _readcommand8(ILI9341_T4_RDMADCTL);
        outputStream->print("- MADCTL Mode: 0x"); outputStream->println(x, HEX);
        x = _readcommand8(ILI9341_T4_RDPIXFMT);
        outputStream->print("- Pixel Format: 0x"); outputStream->println(x, HEX);
        x = _readcommand8(ILI9341_T4_RDIMGFMT);
        outputStream->print("- Image Format: 0x"); outputStream->println(x, HEX);
        x = _readcommand8(ILI9341_T4_RDSELFDIAG);
        outputStream->print("- Self Diagnostic: 0x"); outputStream->print(x, HEX);
        if (x == ILI9341_T4_SELFDIAG_OK)
            outputStream->println(" [OK].\n");
        else
            outputStream->println(" [ERROR].\n");
    }


    void ILI9341Driver::sleep(bool enable)
        {
        waitUpdateAsyncComplete();
        _mirrorfb = nullptr; // force full redraw.
        _beginSPITransaction(_spi_clock);
        if (enable)
            {
            _writecommand_cont(ILI9341_T4_DISPOFF);
            _writecommand_last(ILI9341_T4_SLPIN);
            _endSPITransaction();
            delay(200);
            }
        else
            {
            _writecommand_cont(ILI9341_T4_DISPON);
            _writecommand_last(ILI9341_T4_SLPOUT);
            _endSPITransaction();
            delay(5);
            }
        }


    void ILI9341Driver::setRotation(uint8_t m)
        {
        m = _clip(m, (uint8_t)0, (uint8_t)3);
        if (m == _rotation) return;
        waitUpdateAsyncComplete();
        _mirrorfb = nullptr; // force full redraw.
        statsReset();
        _rotation = m;
        static const uint8_t MADCTL_MY = 0x80;
        static const uint8_t MADCTL_MX = 0x40;
        static const uint8_t MADCTL_MV = 0x20;
        //static const uint8_t MADCTL_ML = 0x10;
        //static const uint8_t MADCTL_RGB = 0x00;
        static const uint8_t MADCTL_BGR = 0x08;
        //static const uint8_t MADCTL_MH = 0x04;
        _beginSPITransaction(_spi_clock);
        _writecommand_cont(ILI9341_T4_MADCTL);
        switch (m)
            {
            case 0: // portrait 240x320
                _writedata8_last(MADCTL_MX | MADCTL_BGR);
                _width = ILI9341_T4_TFTWIDTH;
                _height = ILI9341_T4_TFTHEIGHT;
                break;
            case 1: // landscape 320x240
                _writedata8_last(MADCTL_MV | MADCTL_BGR);
                _width = ILI9341_T4_TFTHEIGHT;
                _height = ILI9341_T4_TFTWIDTH;
                break;
            case 2: // portrait 240x320
                _writedata8_last(MADCTL_MY | MADCTL_BGR);
                _width = ILI9341_T4_TFTWIDTH;
                _height = ILI9341_T4_TFTHEIGHT;
                break;
            case 3: // landscape 320x240
                _writedata8_last(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
                _width = ILI9341_T4_TFTHEIGHT;
                _height = ILI9341_T4_TFTWIDTH;
                break;
            }
        _endSPITransaction();
        }


    void ILI9341Driver::invertDisplay(bool i)
        {
        waitUpdateAsyncComplete();
        _beginSPITransaction(_spi_clock);
        _writecommand_last(i ? ILI9341_T4_INVON : ILI9341_T4_INVOFF);
        _endSPITransaction();
        }



    void ILI9341Driver::setFramebuffers(uint16_t* fb1, uint16_t* fb2)
        {
        waitUpdateAsyncComplete();
        _mirrorfb = nullptr; // complete redraw needed.
        _fb2full = false; 
        if (fb1)
            {
            _fb1 = fb1;
            _fb2 = fb2;
            }
        else
            {
            _fb1 = fb2;
            _fb2 = fb1;
            }
        }


    void ILI9341Driver::setDiffBuffers(DiffBuffBase* diff1, DiffBuffBase* diff2)
        {
        waitUpdateAsyncComplete();       
        if (diff1)
            {
            _diff1 = diff1;
            _diff2 = diff2;
            }
        else
            {
            _diff1 = diff2;
            _diff2 = diff1;
            }
        }





    /**********************************************************************************************************
    * About timing and vsync.
    ***********************************************************************************************************/


    void ILI9341Driver::setRefreshMode(int mode) 
        {
        if ((mode < 0) || (mode > 31)) return; // invalid mode, do nothing. 
        _refreshmode = mode;
        uint8_t diva = 0;
        if (mode >= 16) 
            {
            mode -= 16; 
            diva = 1;
            }
        waitUpdateAsyncComplete();
        _beginSPITransaction(_spi_clock);
        _writecommand_cont(ILI9341_T4_FRMCTR1); // Column addr set
        _writedata8_cont(diva);
        _writedata8_last(0x10 + mode);
        _endSPITransaction();
        delayMicroseconds(50); 
        _sampleRefreshRate(); // estimate the real refreshrate
        statsReset();
        }


    void ILI9341Driver::printRefreshMode(Stream* outputStream)
        {
        const int om = getRefreshMode();
        outputStream->printf("--- ILI9341Driver Refresh Modes ---\n");
        for(int m = 0; m <= 31; m++)
            {
            setRefreshMode(m);
            double r = getRefreshRate();
            outputStream->printf("- mode %u : %fHz (%u FPS with vsync_spacing = 2).\n", m, r, (uint32_t)round(r/2));
            }
        outputStream->println("");
        setRefreshMode(om);
        }


    int ILI9341Driver::_getScanLine(bool sync)
        {
        if (!sync)
            {
            return (((((uint64_t)_synced_em) * ILI9341_T4_NB_SCANLINE) / _period) + _synced_scanline) % ILI9341_T4_NB_SCANLINE;
            }
        int res[3] = { 255 }; // invalid value.
        _beginSPITransaction(_spi_clock_read);
        _maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CONT);
        _pimxrt_spi->TDR = 0x45; // send command
        delayMicroseconds(5); // wait as requested by manual. 
        _maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));
        _pimxrt_spi->TDR = 0; // send nothing
        _maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));
        _pimxrt_spi->TDR = 0; // send nothing        
        uint8_t rx_count = 3;
        while (rx_count)
            { // receive answer. 
            if ((_pimxrt_spi->RSR & LPSPI_RSR_RXEMPTY) == 0)
                {
                res[--rx_count] = _pimxrt_spi->RDR;
                }
            }
        _synced_em = 0;
        _synced_scanline = res[0];
        _endSPITransaction();
        return res[0];
        }


    void ILI9341Driver::_sampleRefreshRate()
        {
        static const uint32_t NB_SAMPLE_FRAMES = 10;
        elapsedMicros em;
        uint32_t sum = 0;
        for (uint32_t i = 0; i < NB_SAMPLE_FRAMES; i++)
            {
            delayMicroseconds(5000); // must be less than 200 FPS so wait at least 5ms
            while (_getScanLine(true) != 0); // wait to reach scanline 0
            while (_getScanLine(true) != 1);  // wait to begin scanline 1. 
            em = 0; // start counter at begining of scanline 1
            delayMicroseconds(5000); // must be less than 200 FPS so wait at least 5ms
            while (_getScanLine(true) != 0); // wait to reach scanline 0
            while (_getScanLine(true) != 1);  // wait to begin scanline 1. 
            sum += em; // stop counter and add to current sum. 
            }
        _period = (uint32_t)round(((double)sum) / NB_SAMPLE_FRAMES);
        }


    double ILI9341Driver::_refreshRateForMode(int mode) const
        { 
        double freq = 1000000.0 / _period_mode0;
        if (mode >= 16)
            {
            freq /= 2.0; 
            mode -= 16;
            }
        return (freq*16.0)/(16.0+mode);
        }


    int ILI9341Driver::_modeForRefreshRate(double hz) const
    {
        if (hz <= _refreshRateForMode(31)) return 31;
        if (hz >= _refreshRateForMode(0)) return 0;
        int a = 0;
        int b = 31;
        while (b - a > 1)
        { // dichotomy. 
            int c = (a + b) / 2;
            ((hz < _refreshRateForMode(c)) ? a : b) = c;
        }
        double da = _refreshRateForMode(a) - hz;
        double db = hz - _refreshRateForMode(b);
        return (da < db ? a : b);
    }






    /**********************************************************************************************************
    * Update
    ***********************************************************************************************************/



    void ILI9341Driver::update(const uint16_t* fb)
        {
        const int bmode = bufferingMode();
        switch (bmode)
            {
            case NO_BUFFERING:
                {
                waitUpdateAsyncComplete(); // wait until update is done (normally useless but still). 
                _mirrorfb = nullptr;
                if (_vsync_spacing <= 0)
                    {                    
                    _updateNow(fb); // direct upload without sync.                   
                    return;
                    }
                _dummydiff1->computeDummyDiff(width(), height(), getRotation(), _diff_nb_split); // create a dummy diff. 
                _updateNow(fb, _dummydiff1);
                return;
                }

            case DOUBLE_BUFFERING_ONE_DIFF:
                {                
                if ((_vsync_spacing == -1) && (asyncUpdateActive())) { return; } // just drop the frame. 
                waitUpdateAsyncComplete(); // wait until update is done. 
                if (_mirrorfb == nullptr)
                    { // complete redraw needed. 
                    _dummydiff1->computeDiff(_fb1, fb, width(), height(), getRotation(), _diff_nb_split, _diff_gap, true); // create a dummy diff and copy to fb1. 
                    _updateAsync(_fb1, _dummydiff1); // launch update
                    }
                else
                    { // diff redraw 
                    _diff1->computeDiff(_fb1, fb, width(), height(), getRotation(), _diff_nb_split, _diff_gap, true); // create a diff and copy to fb1. 
                    _updateAsync(_fb1, _diff1); // launch update
                    }
                _mirrorfb = _fb1; // set as mirror
                return;
                }

            case DOUBLE_BUFFERING_TWO_DIFF:
                {
                if ((_vsync_spacing == -1)&&(asyncUpdateActive())) return; // just drop the frame. 
                if (_mirrorfb == nullptr)
                    { // complete redraw needed. 
                    waitUpdateAsyncComplete(); // wait until update is done. 
                    _dummydiff1->computeDiff(_fb1, fb, width(), height(), getRotation(), _diff_nb_split, _diff_gap, true); // create a dummy diff and copy to fb1. 
                    _updateAsync(_fb1, _dummydiff1); // launch update
                    _mirrorfb = _fb1; // set as mirror
                    return;
                    }
                if (asyncUpdateActive())
                    { // _diff2 is available so we use it to create the diff while update is in progress. 
                    _diff2->computeDiff(_fb1, fb, width(), height(), getRotation(), _diff_nb_split, _diff_gap, false); // create a diff without copying
                    waitUpdateAsyncComplete(); // wait until update is done. 
                    _copyfb(_fb1, fb); // save the framebuffer in fb1
                    _swapdiff();  // swap the diffs so that diff1 contain the new diff. 
                    _updateAsync(_fb1, _diff1); // launch update
                    }
                else
                    {
                    _diff1->computeDiff(_fb1, fb, width(), height(), getRotation(), _diff_nb_split, _diff_gap, true); // create a diff and copy
                    _updateAsync(_fb1, _diff1); // launch update
                    }
                _mirrorfb = _fb1; // set as mirror
                return;
                }

            case TRIPLE_BUFFERING:
                {
                if (!asyncUpdateActive())
                    { // we can launch immediately
                    if (_mirrorfb == nullptr)
                        { // complete redraw needed. 
                        _dummydiff1->computeDiff(_fb1, fb, width(), height(), getRotation(), _diff_nb_split, _diff_gap, true); // create a dummy diff and copy to fb1. 
                        _updateAsync(_fb1, _dummydiff1); // launch update
                        }
                    else
                        {
                        _diff1->computeDiff(_fb1, fb, width(), height(), getRotation(), _diff_nb_split, _diff_gap, true); // create a diff and copy
                        _updateAsync(_fb1, _diff1); // launch update
                        }
                    _mirrorfb = _fb1; // set as mirror
                    return;
                    }

                // there is an update in progress
                if (_vsync_spacing != -1)
                    {
                    while (_fb2full); // we wait until the _fb2 is free. 
                    }

                // try again 
                noInterrupts();
                if (asyncUpdateActive())
                    { // update still in progress so we replace_fb2.
                    _setCB(); // remove callback to prevent upload of fb2
                    interrupts();
                    if (_mirrorfb)
                        {
                        _diff2->computeDiff(_fb1, fb, width(), height(), getRotation(), _diff_nb_split, _diff_gap, false); // create a diff without copying
                        _copyfb(_fb2, fb); // save in fb2
                        noInterrupts();
                        if (asyncUpdateActive())
                            { // update still in progress...
                            _setCB(&ILI9341Driver::_buffer2fullCB); // set a callback
                            _fb2full = true;  // and mark buffer as full. 
                            _mirrorfb = _fb2; // this inform that we have a real diff in diff2.
                            interrupts();
                            return; // done
                            }
                        else
                            { // update done
                            interrupts();
                            _swapdiff();
                            _swapfb();
                            _mirrorfb = _fb1;
                            _updateAsync(_fb1, _diff1); // launch update
                            return;
                            }
                        }
                    else
                        {
                        _dummydiff2->computeDiff(_fb1, fb, width(), height(), getRotation(), _diff_nb_split, _diff_gap, false); // create a dummy diff without copy
                        _copyfb(_fb2, fb); // save in fb2
                        noInterrupts();
                        if (asyncUpdateActive())
                            { // update still in progress...
                            _setCB(&ILI9341Driver::_buffer2fullCB); // set a callback
                            _fb2full = true; // and mark buffer as full. 
                            _mirrorfb = nullptr; // this infoprm that we have a dummy diff in _dummdiff2
                            interrupts();
                            return; // done
                            }
                        else
                            {
                            interrupts();
                            _swapdummydiff();
                            _swapfb();
                            _mirrorfb = _fb1;
                            _updateAsync(_fb1, _dummydiff1); // launch update
                            return;
                            }
                        }
                    }
                else
                    { // we can launch immediately
                    if (_mirrorfb == nullptr)
                        { // complete redraw needed. 
                        _dummydiff1->computeDiff(_fb1, fb, width(), height(), getRotation(), _diff_nb_split, _diff_gap, true); // create a dummy diff and copy to fb1. 
                        _updateAsync(_fb1, _dummydiff1); // launch update
                        }
                    else
                        {
                        _diff1->computeDiff(_fb1, fb, width(), height(), getRotation(), _diff_nb_split, _diff_gap, true); // create a diff and copy
                        _updateAsync(_fb1, _diff1); // launch update
                        }
                    _mirrorfb = _fb1; // set as mirror
                    return;
                    }
                }
            }
        }



    void ILI9341Driver::_buffer2fullCB()
        {        
        if (_mirrorfb)
            {
            _swapdiff();
            _swapfb(); 
            _mirrorfb = _fb1;
            _fb2full = false;
            _updateAsync(_fb1, _diff1); // launch update
            }
        else
            {
            _swapdummydiff();
            _swapfb();
            _mirrorfb = _fb1;
            _fb2full = false;
            _updateAsync(_fb1, _dummydiff1); // launch update
            }
        _setCB(); // disable itself, just in case. 
        }




    bool ILI9341Driver::_updateNow(const uint16_t* fb)
        {
        if (fb == nullptr) return false;
        waitUpdateAsyncComplete();
        _startframe(false);
        _vsyncok = false;   // no sync so tearing may occur.
        _beginSPITransaction(_spi_clock);
        _setAddr(0, 0, _width, _height);
        _writecommand_cont(ILI9341_T4_RAMWR);
        int n = ILI9341_T4_TFTWIDTH * ILI9341_T4_TFTHEIGHT;
        while (--n > 0) { _writedata16_cont(*fb++); }
        _writedata16_last(*fb);
        _endSPITransaction();
        _endframe();
        return true; 
        }


    bool ILI9341Driver::_updateNow(const uint16_t* fb, DiffBuffBase* diff)
        {
        if (fb == nullptr) return false;
        waitUpdateAsyncComplete();
        if (diff == nullptr)
            { // fallback 
            _updateNow(fb);
            return false; 
            }
        _startframe(_vsync_spacing > 0);
        _vsyncok = (_vsync_spacing <= 0) ? false : true;
        _margin = ILI9341_T4_NB_SCANLINE; 
        diff->initRead();
        const int stride = diff->width();
        int res, x, y, len;
        while ((res = diff->readDiff(x, y, len)) == diff->NEW_SUBFRAME); // skip empty subframes 
        if (res == diff->DIFF_END)
            { // Diff is empty. 
            if (_vsync_spacing > 0)
                { // note the next time. 
                const uint32_t t1 = micros() + _microToReachScanLine(0, true);
                const uint32_t t2 = _timeframestart + (_vsync_spacing * _period);
                const uint32_t tfs = (t1 > t2) ? t1 : t2;
                _last_delta = (int)round(((double)(tfs - _timeframestart)) / (_period));  // number of refresh between this frame and the previous one. 
                _timeframestart = tfs;
                }
            _endframe();           
            return true;
            }
        // ok we have at least one instruction
        if (_vsync_spacing > 0)
            {
            const uint32_t dd = (_timeframestart + ((_vsync_spacing - 1)*_period)) - micros();
            delayMicro(dd); // wait until start of screen refresh. 
            double start, end;
            diff->subFrameSyncTimes(start, end);
            int starti = _ratioToScanline(start);
            int endi = _ratioToScanline(end);
            uint32_t t = _microToReachScanLine(endi, true);                         // we want scanline at endi when we start drawing the frame. resync.
            delayMicroseconds(t);                                                   // wait...
            while ((t = _microToExitRange(0, endi))) { delayMicroseconds(t); }     // make sure we are good (in case delayMicroseconds() in not precise enough).
            // ok, scanline is just after endi.  
            _nbdrawnstart = starti;             // number of line at beggining of subframe
            _slinitpos = _getScanLine(false);   // save initial scanline position
            _margin = ILI9341_T4_NB_SCANLINE - _slinitpos + starti; // initial margin
            _em_async = 0;                      // start the counter
            const uint32_t tfs = micros() + _microToReachScanLine(0, false);        // time when this frame will start being displayed on the screen. 
            _last_delta = (int)round(((double)(tfs - _timeframestart)) / (_period));
            _timeframestart = tfs;
            }
        _beginSPITransaction(_spi_clock);
       while (1)
            {
            switch(res)
                {
                case diff->NEW_SUBFRAME:
                    {
                    if (_vsync_spacing > 0)
                        {
                        double start, end;
                        diff->subFrameSyncTimes(start, end);
                        int starti = _ratioToScanline(start);
                        int endi = _ratioToScanline(end);

                        const int mm = (int)(_nbdrawnstart + ILI9341_T4_NB_SCANLINE) - (int)(_slinitpos + _nbScanlineDuring(_em_async));
                        if (mm < _margin) _margin = mm;
                        if (mm <= 0) _vsyncok = false; // we have been catched-up :( Tearing may have occured

                        _nbdrawnstart = starti;
                        uint32_t t = timeForScanline(endi - _slinitpos); // time for scanline to go over endi. 
                        if (t > (uint32_t)(_em_async))
                            { // we need to wait.
                            delayMicro(t - (uint32_t)(_em_async));
                            }
                        }
                    break;
                    }
                case diff->DIFF_END:
                    { 
                    if (_vsync_spacing > 0)
                        {
                        const int mm = (int)(_nbdrawnstart + ILI9341_T4_NB_SCANLINE) - (int)(_slinitpos + _nbScanlineDuring(_em_async));
                        if (mm < _margin) _margin = mm;
                        if (mm <= 0) _vsyncok = false; // we have been catched-up :( Tearing may have occured
                        }
                    _writecommand_last(ILI9341_T4_NOP);     // never hurts to add a little nop if the diff was empty 
                    _endSPITransaction();
                    _endframe();
                    return _vsyncok;
                    }
                case diff->INSTRUCTION:
                    { // upload a bunch of pixels
                    _setAddr(x, y, _width + 1, _height + 1);
                    _writecommand_cont(ILI9341_T4_RAMWR);
                    const uint16_t* p = fb + x + (y * stride);
                    while (len-- > 0) { _writedata16_cont(*p++); }
                    break;
                    }
                }
            res = diff->readDiff(x, y, len);
            }
        }


    void ILI9341Driver::_updateAsync(const uint16_t* fb)
        {
        if (fb == nullptr) return;  // do not call callback if buffer is invalid
        waitUpdateAsyncComplete();
        _startframe(false);
        _dma_state = ILI9341_T4_DMA_ON;
        _dmaObject[_spi_num] = this; // set up object callback.        
        _flush_cache(fb, 2 * ILI9341_T4_NB_PIXELS);

        const int nbw = (DiffBuffBase::MAX_WRITE < ILI9341_T4_NB_PIXELS) ? DiffBuffBase::MAX_WRITE : ILI9341_T4_NB_PIXELS;
        
        _vsyncok = false;   // no sync so tearing may occur.
        _fb = fb;  // save buffer. 
        _boff = nbw;  // and position

        _dmasettingFull.sourceBuffer(fb, nbw * 2);
        _dmasettingFull.destination(_pimxrt_spi->TDR);
        _dmasettingFull.TCD->ATTR_DST = 1;
        _dmasettingFull.interruptAtCompletion();
        _dmasettingFull.disableOnCompletion();

        _beginSPITransaction(_spi_clock);
        _setAddr(0, 0, _width-1, _height-1);
        _writecommand_last(ILI9341_T4_RAMWR);
        _pimxrt_spi->FCR = 0;
        _maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_RXMSK);
        _pimxrt_spi->DER = LPSPI_DER_TDDE;
        _pimxrt_spi->SR = 0x3f00;

        _dmatx = _dmasettingFull;
        _dmatx.triggerAtHardwareEvent(_spi_hardware->tx_dma_channel);
        if (_spi_num == 0) _dmatx.attachInterrupt(_dmaInterruptSPI0Full);
        else if (_spi_num == 1) _dmatx.attachInterrupt(_dmaInterruptSPI1Full);
        else _dmatx.attachInterrupt(_dmaInterruptSPI2Full);
        NVIC_SET_PRIORITY(IRQ_DMA_CH0 + _dmatx.channel, ILI9341_T4_IRQ_PRIORITY);
        _dmatx.begin(false);
        _dmatx.enable();
        NVIC_SET_PRIORITY(IRQ_DMA_CH0 + _dmatx.channel, ILI9341_T4_IRQ_PRIORITY);
        _pauseframe();
        return;
        }



    void ILI9341Driver::_updateAsync(const uint16_t* fb, DiffBuffBase* diff)
        {
        if (fb == nullptr) return; // do not call callback if buffer is invalid
        if (diff == nullptr)
            { // fallback 
            _updateAsync(fb);
            return;
            }
        waitUpdateAsyncComplete();
        _startframe(_vsync_spacing > 0);
        _dma_state = ILI9341_T4_DMA_ON;
        _dmaObject[_spi_num] = this; // set up object callback.         
        _flush_cache(fb, 2 * ILI9341_T4_NB_PIXELS);
        _fb = fb;
        _diff = diff;                
        _vsyncok = (_vsync_spacing <= 0) ? false : true;
        _margin = ILI9341_T4_NB_SCANLINE;
        diff->initRead();
        int x = 0, y = 0, len = 0;        
        int res; 
        while ((res = _diff->readDiff(x, y, len)) == _diff->NEW_SUBFRAME); // skip empty subframes 
        if (res == diff->DIFF_END)
            { // Diff is empty. 
            _dmaObject[_spi_num] = nullptr;
            if (_vsync_spacing > 0)
                { // note the next time. 
                const uint32_t t1  = micros() + _microToReachScanLine(0, true); 
                const uint32_t t2  = _timeframestart + (_vsync_spacing * _period);
                const uint32_t tfs = (t1 > t2) ? t1 : t2;
                _last_delta = (int)round(((double)(tfs - _timeframestart)) / (_period));  // number of refresh between this frame and the previous one. 
                _timeframestart = tfs;
                }
            _endframe();
            _dma_state = ILI9341_T4_DMA_IDLE;
            if (_pcb) { (this->*_pcb)(); }
            _pcb = nullptr; // remove it afterward. 
            return;
            }
        // ok, we have the first instruction in (x,y,len).
        _stride = _diff->width();

        // prepare dma
        _comAsync[0] = ILI9341_T4_CASET; // first command, set x;
        _comAsync[1] = ILI9341_T4_PASET; // second command, set y
        _comAsync[2] = ILI9341_T4_RAMWR; // third command, draw pixels
        _comAsyncX[0] = x;
        _comAsyncX[1] = _width + 1;
        _comAsyncY[0] = y;
        _comAsyncY[1] = _height + 1;

        _dmasettingsDiff[0].sourceBuffer(&_comAsync[0], 1);
        _dmasettingsDiff[0].destination(_pimxrt_spi->TDR);
        _dmasettingsDiff[0].TCD->ATTR_DST = 0;
        _dmasettingsDiff[0].replaceSettingsOnCompletion(_dmasettingsDiff[1]);
        _dmasettingsDiff[0].interruptAtCompletion();
        _dmasettingsDiff[0].disableOnCompletion();

        _dmasettingsDiff[1].sourceBuffer(_comAsyncX, 4);
        _dmasettingsDiff[1].destination(_pimxrt_spi->TDR);
        _dmasettingsDiff[1].TCD->ATTR_DST = 1;
        _dmasettingsDiff[1].replaceSettingsOnCompletion(_dmasettingsDiff[2]);
        _dmasettingsDiff[1].interruptAtCompletion();
        _dmasettingsDiff[1].disableOnCompletion();

        _dmasettingsDiff[2].sourceBuffer(&_comAsync[1], 1);
        _dmasettingsDiff[2].destination(_pimxrt_spi->TDR);
        _dmasettingsDiff[2].TCD->ATTR_DST = 0;
        _dmasettingsDiff[2].replaceSettingsOnCompletion(_dmasettingsDiff[3]);
        _dmasettingsDiff[2].interruptAtCompletion();
        _dmasettingsDiff[2].disableOnCompletion();

        _dmasettingsDiff[3].sourceBuffer(_comAsyncY, 4);
        _dmasettingsDiff[3].destination(_pimxrt_spi->TDR);
        _dmasettingsDiff[3].TCD->ATTR_DST = 1;
        _dmasettingsDiff[3].replaceSettingsOnCompletion(_dmasettingsDiff[4]);
        _dmasettingsDiff[3].interruptAtCompletion();
        _dmasettingsDiff[3].disableOnCompletion();

        _dmasettingsDiff[4].sourceBuffer(&_comAsync[2], 1);
        _dmasettingsDiff[4].destination(_pimxrt_spi->TDR);
        _dmasettingsDiff[4].TCD->ATTR_DST = 0;
        _dmasettingsDiff[4].replaceSettingsOnCompletion(_dmasettingsDiff[5]);
        _dmasettingsDiff[4].interruptAtCompletion();
        _dmasettingsDiff[4].disableOnCompletion();

        _dmasettingsDiff[5].sourceBuffer(_fb + x + (y*_stride), 2*len);
        _dmasettingsDiff[5].destination(_pimxrt_spi->TDR);
        _dmasettingsDiff[5].TCD->ATTR_DST = 1;
        _dmasettingsDiff[5].replaceSettingsOnCompletion(_dmasettingsDiff[0]);
        _dmasettingsDiff[5].interruptAtCompletion();
        _dmasettingsDiff[5].disableOnCompletion();

        _partdma = 0;

        _dmatx = _dmasettingsDiff[0];
        _dmatx.triggerAtHardwareEvent(_spi_hardware->tx_dma_channel);
        if (_spi_num == 0) _dmatx.attachInterrupt(_dmaInterruptSPI0Diff);
        else if (_spi_num == 1) _dmatx.attachInterrupt(_dmaInterruptSPI1Diff);
        else _dmatx.attachInterrupt(_dmaInterruptSPI2Diff);

        // ready, set up the timer 
        if (_vsync_spacing <= 0)
            { // no vsync so we start now 
            _beginSPITransaction(_spi_clock);
            _pimxrt_spi->FCR = 0;
            _maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK); // bug with the continu flag | LPSPI_TCR_CONT , why ?
            _pimxrt_spi->DER = LPSPI_DER_TDDE;
            _pimxrt_spi->SR = 0x3f00;
            _dmatx.enable();
            }
        else
            { // start at begining of frame redraw.             
            //_getScanLine(true); // force resync while spi in not active.   
            _setTimerAt(_timeframestart + (_vsync_spacing - 1)*_period, &ILI9341Driver::_subFrameTimerStartcb); // call at start of screen refresh. 
            }        
        _pauseframe();
        return;
        }


    void ILI9341Driver::_subFrameTimerStartcb()
        {
        // we should be around scanline 0 (unless we are late). 
        _restartframe();
        _getScanLine(true); // force resync while spi in not active.                     
        double start, end;
        _diff->subFrameSyncTimes(start, end);
        int endi = _ratioToScanline(end);        
        uint32_t t = _microToReachScanLine(endi, false);        // we want scanline at endi when we start drawing the frame.
        _setTimerIn(t, &ILI9341Driver::_subFrameTimerStartcb2); // call when ready to start transfer.        
        _pauseframe();
        return;
        }


    void ILI9341Driver::_subFrameTimerStartcb2()
        {
        _restartframe();
        double start, end;
        _diff->subFrameSyncTimes(start, end);
        int starti = _ratioToScanline(start);
        int endi = _ratioToScanline(end);
        uint32_t t;
        while ((t = _microToExitRange(0, endi))) { delayMicroseconds(t); }         // make sure we are good         
         // ok, scanline is just after endi. 
        _nbdrawnstart = starti;             // number of line at beggining of subframe
        _slinitpos = _getScanLine(false);   // save initial scanline position
        _em_async = 0;                      // start the counter
        _margin = ILI9341_T4_NB_SCANLINE - _slinitpos + starti; // initial margin
        //Serial.printf("\n\ninitstart  pos=%u, scanline=%u,  [start==%u, end=%u]\n", starti, _slinitpos, starti, endi);

        const uint32_t tfs = micros() + _microToReachScanLine(0, false);         // time when this frame will start being displayed on the screen. 
        _last_delta = (int)round(((double)(tfs - _timeframestart)) / (_period)); // number of refresh between this frame and the previous one. 
        _timeframestart = tfs;                                                   // save the time this frame will start being displayed. 

        // start spi transaction
        _beginSPITransaction(_spi_clock);
        _pimxrt_spi->FCR = 0;
        _maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK); // bug with the continu flag | LPSPI_TCR_CONT , why ?
        _pimxrt_spi->DER = LPSPI_DER_TDDE;
        _pimxrt_spi->SR = 0x3f00;        
        NVIC_SET_PRIORITY(IRQ_DMA_CH0 + _dmatx.channel, ILI9341_T4_IRQ_PRIORITY);
        _dmatx.begin(false);
        _dmatx.enable(); // go !
        NVIC_SET_PRIORITY(IRQ_DMA_CH0 + _dmatx.channel, ILI9341_T4_IRQ_PRIORITY);
        _pauseframe();
        }


    void ILI9341Driver::_subFrameInterruptDiff()
        {
        _restartframe();
    _dmaInterruptDiff_reread:            
        int x = 0, y = 0, len = 0;
        int res = _diff->readDiff(x, y, len);
        if (res == _diff->DIFF_END)
            { // we are done !                    
               //{
               // double start, end;
               // _diff->subFrameSyncTimes(start, end);
               // int starti = _ratioToScanline(start);
               // int endi = _ratioToScanline(end);
               // Serial.printf("done  pos=%u, scanline=%u,  [start==%u, end=%u]\n", _nbdrawn, _slinitpos + _nbScanlineDuring(_em_async), starti, endi);
               // }
            if (_vsync_spacing > 0)
                {
                const int mm = (int)(_nbdrawnstart + ILI9341_T4_NB_SCANLINE) - (int)(_slinitpos + _nbScanlineDuring(_em_async));
                if (mm < _margin) _margin = mm;
                if (mm <= 0) _vsyncok = false; // we have been catched-up :( Tearing may have occured
                }
            while (_pimxrt_spi->FSR & 0x1f);        // wait for transmit fifo to be empty
            while (_pimxrt_spi->SR & LPSPI_SR_MBF); // wait while spi bus is busy. 
            _pimxrt_spi->FCR = LPSPI_FCR_TXWATER(15); // Transmit Data Flag (TDF) should now be set when there if less or equal than 15 words in the transmit fifo
            _pimxrt_spi->DER = 0; // DMA no longer doing TX (nor RX)
            _pimxrt_spi->CR = LPSPI_CR_MEN | LPSPI_CR_RRF | LPSPI_CR_RTF; // enable module (MEM), reset RX fifo (RRF), reset TX fifo (RTF)
            _pimxrt_spi->SR = 0x3f00; // clear out all of the other status...
            _maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7)); // output Command with 8 bits            
            _writecommand_last(ILI9341_T4_NOP);
            _endSPITransaction();
            _endframe();
            if (_touch_request_read)
                {
                _updateTouch2(); // touch update requested. do it now.
                _touch_request_read = false;
                }
            _flush_cache(_fb, 2 * ILI9341_T4_NB_PIXELS);
            _dmaObject[_spi_num] = nullptr;
            _dma_state = ILI9341_T4_DMA_IDLE;
            if (_pcb) { (this->*_pcb)(); }
            _pcb = nullptr; // remove it afterward. 
            return;
            }
        else if (res == _diff->NEW_SUBFRAME)
            { // new subframe. 
            if (_vsync_spacing > 0)
                {
                   // {
                   // double start, end;
                   // _diff->subFrameSyncTimes(start, end);
                   // int starti = _ratioToScanline(start);
                   // int endi = _ratioToScanline(end);
                   // Serial.printf("done  pos=%u, scanline=%u,  [start==%u, end=%u]\n", _nbdrawn, _slinitpos + _nbScanlineDuring(_em_async), starti, endi);
                   // }

                const int mm = (int)(_nbdrawnstart + ILI9341_T4_NB_SCANLINE) - (int)(_slinitpos + _nbScanlineDuring(_em_async));
                if (mm < _margin) _margin = mm;
                if (mm <= 0) _vsyncok = false; // we have been catched-up :( Tearing may have occured

                double start, end;
                _diff->subFrameSyncTimes(start, end);
                int starti = _ratioToScanline(start);
                int endi = _ratioToScanline(end);
                _nbdrawnstart = starti;
                uint32_t t = timeForScanline(endi - _slinitpos); // time for scanline to go over endi. 
                if (t >  (uint32_t)(_em_async))
                    { // we need to wait.
                    //Serial.printf("Waiting %t\n", t);
                    while (_pimxrt_spi->FSR & 0x1f);        // wait for transmit fifo to be empty
                    while (_pimxrt_spi->SR & LPSPI_SR_MBF); // wait while spi bus is busy. 
                    _setTimerIn((t - (uint32_t)(_em_async)), &ILI9341Driver::_subFrameInterruptDiff);
                    _pauseframe();
                    return;
                    }
                }
            goto _dmaInterruptDiff_reread; // continue without delay.
            }
        // new instruction
        _comAsyncX[0] = x;
        _comAsyncY[0] = y;
        _dmasettingsDiff[5].sourceBuffer(_fb + x + (y*_stride), len * 2);
        _dmasettingsDiff[5].destination(_pimxrt_spi->TDR);
        _dmasettingsDiff[5].TCD->ATTR_DST = 1;
        _dmasettingsDiff[5].replaceSettingsOnCompletion(_dmasettingsDiff[0]);
        _partdma = 0;

        //this corresponds to _maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK);
        const uint32_t req = (_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK);
        _spi_tcr_current = (_spi_tcr_current & (~(LPSPI_TCR_PCS(3) | LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_CONT | LPSPI_TCR_RXMSK))) | req;
        while ((_pimxrt_spi->FSR & 0x1f));

    #if ILI9341_T4_FAST_UNSAFE_DMA
        // unsafe order. faster.
        _dmatx.enable();
        _pimxrt_spi->TCR = _spi_tcr_current; // update the TCR    
    #else
        // safe order
        _pimxrt_spi->TCR = _spi_tcr_current; // update the TCR    
        _dmatx.enable();
    #endif
        _pauseframe();
        return;
        }



    /**********************************************************************************************************
    * DMA Interrupts
    ***********************************************************************************************************/

    ILI9341Driver* volatile ILI9341Driver::_dmaObject[3] = { nullptr, nullptr, nullptr };  // definition 


    void ILI9341Driver::_dmaInterruptDiff()
        {
        _dmatx.clearInterrupt();
        _dmatx.clearComplete();
        if (_partdma == 5)
            {
            _subFrameInterruptDiff();
            return;
            }
        _restartframe();

        //this corresponds to _maybeUpdateTCR((_partdma++ & 1) ? (_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK) : (_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_RXMSK));
        const uint32_t req = (_partdma++ & 1) ? (_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK) : (_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_RXMSK);
        _spi_tcr_current = (_spi_tcr_current & (~(LPSPI_TCR_PCS(3) | LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_CONT | LPSPI_TCR_RXMSK))) | req;
        while ((_pimxrt_spi->FSR & 0x1f));
    #if ILI9341_T4_FAST_UNSAFE_DMA
        // unsafe order. faster.
        _dmatx.enable();
        _pimxrt_spi->TCR = _spi_tcr_current; // update the TCR    
    #else
        // safe order
        _pimxrt_spi->TCR = _spi_tcr_current; // update the TCR    
        _dmatx.enable();
    #endif

        _pauseframe();
        return;
        }



    void ILI9341Driver::_dmaInterruptFull()
        {
        _dmatx.clearInterrupt();
        _dmatx.clearComplete(); // done with DMA
        _restartframe();
        if (_boff == ILI9341_T4_NB_PIXELS)
            { // done !            
            while (_pimxrt_spi->FSR & 0x1f);        // wait for transmit fifo to be empty
            while (_pimxrt_spi->SR & LPSPI_SR_MBF); // wait while spi bus is busy. 
            _pimxrt_spi->FCR = LPSPI_FCR_TXWATER(15); // Transmit Data Flag (TDF) should now be set when there if less or equal than 15 words in the transmit fifo
            _pimxrt_spi->DER = 0; // DMA no longer doing TX (nor RX)
            _pimxrt_spi->CR = LPSPI_CR_MEN | LPSPI_CR_RRF | LPSPI_CR_RTF; // enable module (MEM), reset RX fifo (RRF), reset TX fifo (RTF)
            _pimxrt_spi->SR = 0x3f00; // clear out all of the other status...
            _maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7)); // output Command with 8 bits            
            _writecommand_last(ILI9341_T4_NOP);
            _endSPITransaction();
            _endframe();
            if (_touch_request_read)
                {
                _updateTouch2(); // touch update requested. do it now.
                _touch_request_read = false;
                }
            _flush_cache(_fb, 2 * ILI9341_T4_NB_PIXELS);            
            _dmaObject[_spi_num] = nullptr;
            _dma_state = ILI9341_T4_DMA_IDLE;
            if (_pcb) { (this->*_pcb)(); }
            _pcb = nullptr; // remove it afterward. 
            return;
            }
        const int nbw = (_boff + DiffBuffBase::MAX_WRITE <= ILI9341_T4_NB_PIXELS) ? DiffBuffBase::MAX_WRITE : (ILI9341_T4_NB_PIXELS - _boff);
        _dmasettingFull.sourceBuffer(_fb + _boff, 2*nbw);   // 2*nbw
        _dmasettingFull.destination(_pimxrt_spi->TDR);
        _dmasettingFull.TCD->ATTR_DST = 1;
        _boff += nbw;
        _dmatx = _dmasettingFull;
        _dmatx.enable();
        _pauseframe();
        }








    /**********************************************************************************************************
    * IntervalTimer
    ***********************************************************************************************************/

    ILI9341Driver* volatile ILI9341Driver::_pitObj[4] = {nullptr, nullptr, nullptr, nullptr};   // definition 

    void ILI9341Driver::_timerinit()
        {
        _istimer = false; 
        for (int i = 0; i < 4; i++)
            {
            if (_pitObj[i] == nullptr)
                {
                _pitObj[i] = this;
                _pitindex = i;
                return;
                }
            }
        // OUCH !Boom boom boom booom...
        Serial.print("\n *** TOO MANY INSTANCES OF ILI9341Driver CREATED ***\n\n");
        }








    /**********************************************************************************************************
    * SPI
    ***********************************************************************************************************/


    uint8_t ILI9341Driver::_readcommand8(uint8_t c, uint8_t index)
        {
        // Bail if not valid miso
        if (_miso == 0xff) return 0;
        uint16_t wTimeout = 0xffff;
        uint8_t r = 0;
        _beginSPITransaction(_spi_clock_read);
        // Lets assume that queues are empty as we just started transaction.
        _pimxrt_spi->CR = LPSPI_CR_MEN | LPSPI_CR_RRF /* | LPSPI_CR_RTF */; // actually clear both...        
        _maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CONT);
        _pimxrt_spi->TDR = 0xD9; // writecommand(0xD9); // sekret command        
        _maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CONT);
        _pimxrt_spi->TDR = 0x10 + index; // writedata(0x10 + index);        
        _maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CONT);
        _pimxrt_spi->TDR = c; // writecommand(c);        
        _maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));
        _pimxrt_spi->TDR = 0; // readdata
        // Now wait until completed.
        wTimeout = 0xffff;
        uint8_t rx_count = 4;
        while (rx_count && wTimeout)
            {
            if ((_pimxrt_spi->RSR & LPSPI_RSR_RXEMPTY) == 0)
                {
                r = _pimxrt_spi->RDR; // Read any pending RX bytes in
                rx_count--;           // decrement count of bytes still levt
                }
            }
        _endSPITransaction();
        return r; // get the received byte... should check for it first...
        }



    void ILI9341Driver::_waitFifoNotFull()
        {
        uint32_t tmp __attribute__((unused));
        do
            {
            if ((_pimxrt_spi->RSR & LPSPI_RSR_RXEMPTY) == 0)
                {
                tmp = _pimxrt_spi->RDR; // Read any pending RX bytes in
                if (_pending_rx_count) _pending_rx_count--; // decrement count of bytes still levt
                }
            } 
        while ((_pimxrt_spi->SR & LPSPI_SR_TDF) == 0);
        }


    void ILI9341Driver::_waitTransmitComplete()
        {
        uint32_t tmp __attribute__((unused));
        while (_pending_rx_count)
            {
            if ((_pimxrt_spi->RSR & LPSPI_RSR_RXEMPTY) == 0)
                {
                tmp = _pimxrt_spi->RDR; // Read any pending RX bytes in
                _pending_rx_count--;     // decrement count of bytes still levt
                }
            }
        _pimxrt_spi->CR = LPSPI_CR_MEN | LPSPI_CR_RRF; // Clear RX FIFO
        }







    /**********************************************************************************************************
    * Statistics
    ***********************************************************************************************************/


    void ILI9341Driver::statsReset()
        {
        _nbframe = 0;
        _tottime = 0;
        _cputime = 0;
        _min_cputime = -1;
        _max_cputime = 0;
        _sum_cputime = 0;
        _sumsqr_cputime = 0;
        _nbteared = 0;
        _vsync_spacing_nb = 0;
        _min_vsync_spacing = -1;
        _max_vsync_spacing = 0;
        _vsync_spacing_sum = 0;
        _vsync_spacing_sqr = 0;
        _min_margin = 100000000;
        _max_margin = -100000000;
        _sum_margin = 0;
        _sumsqr_margin = 0;
        }


    void ILI9341Driver::printStats(Stream* outputStream) const
        {
        //waitUpdateAsyncComplete();
        outputStream->printf("--- ILI9341Driver ---\n\n");
        outputStream->printf("Configuration.\n");
        outputStream->printf("- SPI speed          : write=%u  read=%u\n", _spi_clock, _spi_clock_read);
        outputStream->printf("- screen orientation : %u  (%u x %u ", getRotation(), width(), height());
        outputStream->printf((getRotation() & 1) ? " landscape)\n" : " portrait)\n");
        outputStream->printf("- refresh rate       : %.2fHz  (mode %u)\n\n", getRefreshRate(), getRefreshMode());
        outputStream->printf("Buffering settings.\n");
        int m = bufferingMode();
        outputStream->printf("- mode               : %u", m);
        switch (m)
            {
        case NO_BUFFERING: 
            outputStream->printf(" (NO BUFFERING)\n", getRefreshRate(), getRefreshMode()); break;
        case DOUBLE_BUFFERING_ONE_DIFF:
            outputStream->printf(" (DOUBLE BUFFERING, 1 DIFF)\n", getRefreshRate(), getRefreshMode()); break;
        case DOUBLE_BUFFERING_TWO_DIFF:
            outputStream->printf(" (DOUBLE BUFFERING, 2 DIFFS)\n", getRefreshRate(), getRefreshMode()); break;
        case TRIPLE_BUFFERING:
            outputStream->printf(" (TRIPLE BUFFERING)\n", getRefreshRate(), getRefreshMode()); break;
            }
        outputStream->printf("- vsync_spacing      : %i\n", _vsync_spacing);
        outputStream->printf("- requested FPS      : ");
        if (_vsync_spacing == -1)
            outputStream->printf("max fps (drop frames if busy)\n");
        else if (_vsync_spacing == 0)
            outputStream->printf("max fps (do not drop frames)\n");
        else
            outputStream->printf("%.2fHz\n", getRefreshRate()/_vsync_spacing);
        outputStream->printf("- diff [nb_split]    : %u\n", _diff_nb_split);
        outputStream->printf("- diff [gap]         : %u\n\n", _diff_gap);
        outputStream->printf("Statistics.\n");
        outputStream->printf("- average framerate  : %.2fHz  (%u frames in %ums)\n", statsFramerate(), statsNbFrame(), statsTime());
        outputStream->printf("- cpu time per frame : avg=%uus  [min=%uus , max=%uus], std=%uus\n", statsAvgCpuTime(), statsMinCpuTime(), statsMaxCpuTime(), statsStdCpuTime());
        outputStream->printf("- nb unsynced frames : %u\n", statsNbFrame() - statsNbFrameWithVSync());
        outputStream->printf("- nb synced frames   : %u \n", statsNbFrameWithVSync());
        outputStream->printf("- nb teared frames   : %u (%.2f%%)\n", statsNbTeared(), statsRatioTeared() * 100);
        outputStream->printf("- real vsync spacing : avg=%.2f  [min=%u , max=%u], std=%.2f\n", statsAvgVSyncSpacing(), statsMinVSyncSpacing(), statsMaxVSyncSpacing(), statsStdVSyncSpacing());
        outputStream->printf("- margin(vs scanline): avg=%i  [min=%i , max=%i], std=%i\n\n", statsAvgMargin(), statsMinMargin(), statsMaxMargin(), statsStdMargin());
        if ((statsRatioTeared() > 0.05) && (statsNbTeared() >= 10) && (statsNbFrameWithVSync() > 1000))
            outputStream->printf("*** SCREEN TEARING ***\ntry higher refreshMode() / increasing the nb_split parameter.\n\n\n", statsAvgVSyncSpacing(), statsMinVSyncSpacing(), statsMaxVSyncSpacing(), statsStdVSyncSpacing());
        if ((statsStdVSyncSpacing() > 0.25)&&(statsNbFrameWithVSync() > 1000))
            outputStream->printf("*** FLUCTUATING FRAMERATE ***\ntry increasing the vsync_spacing parameter.\n\n\n", statsAvgVSyncSpacing(), statsMinVSyncSpacing(), statsMaxVSyncSpacing(), statsStdVSyncSpacing());
        }


    void ILI9341Driver::_endframe()
        {
        _nbframe++;
        _cputime += _emframe;
        if (_min_cputime > _cputime) _min_cputime = _cputime;
        if (_max_cputime < _cputime) _max_cputime = _cputime;
        _sum_cputime += _cputime;
        _sumsqr_cputime += _cputime * _cputime;
        _cputime = 0;
        if (_vsyncon)
            {
            if (!_vsyncok) _nbteared++;
            _vsync_spacing_nb++;
            if (_vsync_spacing_nb > 1)
                {
                if (_min_vsync_spacing > _last_delta) _min_vsync_spacing = _last_delta;
                if (_max_vsync_spacing < _last_delta) _max_vsync_spacing = _last_delta;
                _vsync_spacing_sum += _last_delta;
                _vsync_spacing_sqr += _last_delta * _last_delta;    // sum of the square of the spacings.                
                }

            if (_margin < _min_margin) _min_margin = _margin;
            if (_margin > _max_margin) _max_margin = _margin;
            _sum_margin += _margin;
            _sumsqr_margin += _margin * _margin;
            }
        }







    /**********************************************************************************************************
    * Touch
    ***********************************************************************************************************/


    ILI9341Driver* volatile ILI9341Driver::_touchObjects[4] = { nullptr, nullptr, nullptr, nullptr };


    /** set the touch interrupt routine */
    void ILI9341Driver::_setTouchInterrupt()
        {
        _touch_request_read = false;
        _touched = true;;
        _touched_read = true;
        _touch_x = _touch_y = _touch_z = 0;
        setTouchRange();

        bool slotfound = false;
        if ((_touch_irq >= 0) && (_touch_irq < 42)) // valid digital pin
            {
            pinMode(_touch_irq, INPUT);
            if ((!slotfound) && (_touchObjects[0] == nullptr)) { _touchObjects[0] = this; attachInterrupt(_touch_irq, _touch_int0, FALLING); slotfound = true; }
            if ((!slotfound) && (_touchObjects[1] == nullptr)) { _touchObjects[1] = this; attachInterrupt(_touch_irq, _touch_int1, FALLING); slotfound = true; }
            if ((!slotfound) && (_touchObjects[2] == nullptr)) { _touchObjects[2] = this; attachInterrupt(_touch_irq, _touch_int2, FALLING); slotfound = true; }
            if ((!slotfound) && (_touchObjects[3] == nullptr)) { _touchObjects[3] = this; attachInterrupt(_touch_irq, _touch_int3, FALLING); slotfound = true; }
            }
        if (!slotfound) { _touch_irq = 255; } // disable touch irq
        }


    int32_t ILI9341Driver::lastTouched()
        {
        const bool b = _touched;
        _touched = false;
        return (b && (_touch_irq != 255)) ? ((int32_t)_em_touched_irq) : -1;
        }


    void ILI9341Driver::_updateTouch2()
        {
        int16_t data[6];
        int z;
        _pspi->beginTransaction(SPISettings(_spi_clock_read, MSBFIRST, SPI_MODE0));
        digitalWrite(_touch_cs, LOW);
        _pspi->transfer(0xB1);
        int16_t z1 = _pspi->transfer16(0xC1 /* Z2 */) >> 3;
        z = z1 + 4095;
        int16_t z2 = _pspi->transfer16(0x91 /* X */) >> 3;
        z -= z2;
        if (z >= ILI9341_T4_TOUCH_Z_THRESHOLD)
            {
            _pspi->transfer16(0x91 /* X */);  // dummy X measure, 1st is always noisy
            data[0] = _pspi->transfer16(0xD1 /* Y */) >> 3;
            data[1] = _pspi->transfer16(0x91 /* X */) >> 3; // make 3 x-y measurements
            data[2] = _pspi->transfer16(0xD1 /* Y */) >> 3;
            data[3] = _pspi->transfer16(0x91 /* X */) >> 3;
            }
        else
            {
            data[0] = data[1] = data[2] = data[3] = 0;	// Compiler warns these values may be used unset on early exit.
            }
        data[4] = _pspi->transfer16(0xD0 /* Y */) >> 3;	// Last Y touch power down
        data[5] = _pspi->transfer16(0) >> 3;
        digitalWrite(_touch_cs, HIGH);
        _pspi->endTransaction();

        if (z < 0) z = 0;
        if (z < ILI9341_T4_TOUCH_Z_THRESHOLD)
            {
            _touch_z = 0;
            if (z < ILI9341_T4_TOUCH_Z_THRESHOLD_INT)
                {
                if (_touch_irq != 255) _touched_read = false;
                }
            return;
            }
        _touch_z = z;

        int16_t x = _besttwoavg(data[0], data[2], data[4]);
        int16_t y = _besttwoavg(data[1], data[3], data[5]);

        if (z >= ILI9341_T4_TOUCH_Z_THRESHOLD)
            {
            _em_touched_read = 0; // good read completed, set wait
            switch (_rotation)
                {
            case 0: // portrait 240x320
                _touch_x = 4095 - y;
                _touch_y = 4095 - x;
                break;
            case 1: // landscape 320x240
                _touch_x = 4095 - x;
                _touch_y = y;
                break;
            case 2: // portrait 240x320
                _touch_x = y;
                _touch_y = x;
                break;
            default: // 3  - landscape 320x240
                _touch_x = x;
                _touch_y = 4095 - y;
                }
            }
        }


    void ILI9341Driver::_updateTouch()
        {
        if (_em_touched_read < ILI9341_T4_TOUCH_MSEC_THRESHOLD) return; // read not so long ago
        if ((_touch_irq != 255) && (_touched_read == false)) return; // nothing to do. 
        if (asyncUpdateActive())
            {
            _touch_request_read = true; // request read at end of transfer
            while ((_touch_request_read) && (asyncUpdateActive())); // wait until transfer complete or reading done. 
            if (!_touch_request_read) return; // reading was done, nothing to do. 
            _touch_request_read = false; // remove request. 
            }
        // we can do the reading now
        _updateTouch2();
        return;
        }



    void ILI9341Driver::readTouch(int& x, int& y, int& z)
        {
        _updateTouch();
        z = _touch_z;
        if ((_touch_minx < _touch_maxx) && (_touch_minx < _touch_maxx))
            { // valid mapping
            x = map(_touch_x, _touch_minx, _touch_maxx, 0, _width - 1);
            y = map(_touch_y, _touch_miny, _touch_maxy, 0, _height - 1);
            }
        else
            { // raw values
            x = _touch_x;
            y = _touch_y;
            }
        }



    int16_t ILI9341Driver::_besttwoavg(int16_t x, int16_t y, int16_t z)
        {
        int16_t da, db, dc;
        int16_t reta = 0;
        if (x > y) da = x - y; else da = y - x;
        if (x > z) db = x - z; else db = z - x;
        if (z > y) dc = z - y; else dc = y - z;
        if (da <= db && da <= dc) reta = (x + y) >> 1;
        else if (db <= da && db <= dc) reta = (x + z) >> 1;
        else reta = (y + z) >> 1;   //    else if ( dc <= da && dc <= db ) reta = (x + y) >> 1;
        return (reta);
        }



    void ILI9341Driver::setTouchRange(int minx, int maxx, int miny, int maxy)
        {
        _touch_minx = minx;
        _touch_maxx = maxx;
        _touch_miny = miny;
        _touch_maxy = maxy;
        }






}


/** end of file */

