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

#pragma once


#include "StatsVar.h"

#include <Arduino.h>
#include <math.h>




namespace ILI9341_T4
{


    /******************************************************************************************
    * Abstract base class describing the public interface of a "diff" object.
    *
    * A diff is an object that keeps track of the pixels that differ between two framebuffers
    * and it can be used to redraw only part of the screen during an update. 
    * 
    * (1) the diff between two framebuffers is created with the 'computeDiff' method. 
    * 
    * (2) Once a diff has been computed, it can be read back using the 'readDiff' method. 
    * 
    * A diff object can be used multiple times: each new call to computeDiff() overwrites
    * the previous one.
    * 
    * Derived class that can be instantiated:
    *
    * - DiffBuff      : diff using user-supplied memory.
    * - DiffBuffStatic: diff using static memory allocation.
    * - DiffBuffDummy : diff without memory alloc holding only trivial diffs.
    * 
    *******************************************************************************************/
    class DiffBuffBase
    {
    public:

        static const int LX = 240;                  // framebuffer width in orientation 0
        static const int LY = 320;                  // framebuffer height in orientation 0
        static const int MAX_WRITE_LINE = 120;      // max number of lines to be written in a single operation.
        static const int MIN_SCANLINE_SPACE = 8;    // min number of lines between the current write line and the current scanline

        static_assert((LX & 3) == 0, "LX must be divisible by 4");


        /** Framebuffer orientation**/
        enum
            {
            PORTRAIT_240x320 = 0,
            LANDSCAPE_320x240 = 1,
            PORTRAIT_240x320_FLIPPED = 2,
            LANDSCAPE_320x240_FLIPPED = 3,
            };


        /**
        * Compute the diff between two framebuffers. Any previous diff is overwritten.
        *
        * - fb_old       : the old framebuffer 
        * 
        * - fb_new       : the new framebuffer 
        * 
        * - fb_new_orientation  : orientation for the new frame_buffer 
        *                         The old framebuffer must always be in orientation 0. 
        * 
        * - gap          : number of consecutives identical pixels between the two framebuffers needed to break 
        *                  the diff in two instructions. This value should be between 4 and 20. Lower values
        *                  will create more accurate diffs (fewer identical pixels will be redrawn) but it will 
        *                  require more memory to store the diff. 
        * 
        * copy_over_old  : If true, the old buffer is overwritten at the same time as the diff is computed so 
		*                  that when the method returns, the old buffer mirrors the new one. This is faster 
        *                  than doing a diff followed by a copyfb()... 
        *
        * compare_mask   : The default behaviour when creating a diff is to redraw every pixels that differ between 
        *                  framebuffers however it might be useful in some case to keep pixels if they have 'close'
        *                  colors. This is particularly useful when the framebuffer contain camera image with random
        *                  noise which is not relevant to the image but will prevent the diff from finding large
        *                  gap of similar pixels, making the diff basically useless and reupload the whole frame each
        *                  time.
        *                  If compare_mask is different from 0 and 65535 then only the color bits set to 1 in the mask
        *                  are checked and therefore pixels that only differ in the unset bits of compare_mask will be 
        *                  considered equal and may not be redrawn. 
        * 
        * NOTE : this method always returns a valid diff even if it runs out of memory to store the diff. However, 
		*        when this happens, the diff returned is (partly) trivial and this will have a negative impact on 
		*        the upload speed. For optimal speed, the diff buffer size/gap parameter should be chosen such that
		*        a typical diff do not overflow... The printStats() method can be useful to find how much memory a 
		*        diff typically use and thus dimension the buffer size and gap accordingly. The size of a typical 
        *        diff will depend on how much changes occurs between frames but in most case, choosing  gap=10 and a 
        *        buffer size around 5K is a good starting point. 
        **/
        virtual void computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation, int gap, bool copy_new_over_old, uint16_t compare_mask) = 0;


        /**
        * Copy the new framebuffer over the old one (and rotate it to put it in orientation 0). 
        **/
        static void copyfb(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation);
            

        /**
        * Call this method to reinitialize the diff prior to the first call
        * to readDiff().          
        **/
        virtual void initRead() = 0;


        /**
        * Read the next instruction in the diff.
        * - 'x','y' and 'len' are used to store the next instruction. 
        * - 'scanline' must contain the current position of the scanline
        * 
        * returns 0 :  in this case  x, y to contain the start position and
        *              len contains the number of pixels to write. 
        * 
        * returns a>0 : must wait until scanline reaches 'a' then call the 
        *               method again to get the instructions.
        *               (x,y) are set to the same value that the next read
        *               will return when timing is right but len is set to 0
        * 
        * - returns a<0 : finished reading the diff. 
        **/
        virtual int readDiff(int& x, int& y, int& len, int scanline) = 0;




    private:
        
        static void _copy_rotate_0(uint16_t* fb_dest, const uint16_t* fb_src);           

        static void _copy_rotate_90(uint16_t* fb_dest, const uint16_t* fb_src);        

        static void _copy_rotate_180(uint16_t* fb_dest, const uint16_t* fb_src);
           
        static void _copy_rotate_270(uint16_t* fb_dest, const uint16_t* fb_src);
           

    };








    /******************************************************************************************
    * Class used to compute the "diff" between 2 framebuffers.
    *
    * The memory for holding the diff is allocated by the user and is passed to the object a
    * construction time.
    *
    * PERFORMANCE: On teensy 4.1, for framebuffers of size 320x240. It takes around 1ms to
    * compute a diff. This means that computing the diff consumes around 5-10% of a frame period 
    * at 60FPS (but still leaves around 15ms to generate each frame).
    *******************************************************************************************/
    class DiffBuff : public DiffBuffBase
    {

    public:

        /**
        * Constructor. Set the buffer (and its size).
        * sizebuf should not be too small (at least MIN_BUFFER_SIZE but say 1K to be useful).
        **/
        DiffBuff(uint8_t* buffer, size_t sizebuf) : DiffBuffBase(), _tab(buffer), _sizebuf(sizebuf - PADDING), _posw(0), _posr(0)
            {
            _write_encoded(TAG_END);
            initRead();
            }


        virtual void computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation, int gap, bool copy_new_over_old, uint16_t compare_mask) override;


        virtual void initRead() override
            {
            _r_cont = false;
            _posr = 0; 
            _off = 0; 
            }


        virtual int readDiff(int& x, int& y, int& len, int scanline) override;


        /**
        * Return the current size of the diff.
        * (return the total size of the buffer in case of overflow).
        **/
        int size() const { return ((_posw >= _sizebuf) ? (_sizebuf + PADDING) : _posw); }


        /************************************************************************
        * STATISTICS.
        *
        * Methods used to monitor resource use and optimize the diff buffer size. 
        ************************************************************************/


        /**
        * Reset all statistics.
        **/
        void statsReset();


        /**
        * Return the number of diff computed (since the last call to statsReset()).
        **/
        uint32_t statsNbComputed() const { return _stats_size.count(); }


        /**
        * Return the number of diff for which the buffer overflowed.
        **/
        uint32_t statsNbOverflow() const { return _stat_overflow; }


        /**
        * Return the percentage of diff that overflowed (between 0 and 1.0).
        **/
        double statsOverflowRatio() const { return ((statsNbComputed() > 0) ? (((double)_stat_overflow) / statsNbComputed()) : 0.0); }


        /**
        * Return a StatsVar object containing statisitcs about the time
        * it took to compute the diffs.
        **/
        ILI9341_T4::StatsVar statsTime() const { return _stats_time; }


        /**
        * Return a StatVar  object containing statisitcs about the size
        * of the computed buffers.
        **/
        ILI9341_T4::StatsVar statsSize() const { return _stats_size; }


        /**
        * Print all the statistics into a Stream object.
        **/
        void printStats(Stream* outputStream = &Serial) const;




    private:

        static const int        MIN_BUFFER_SIZE = 16;             // minimum buffer size
        static const int        PADDING = 8;                      // reserved at end of buffer (in case of overflow)
        static const uint32_t   TAG_END = (0x400000 - 1);         // tag at end of diff
        static const uint32_t   TAG_WRITE_ALL = (0x400000 - 2);   // tag to write everything remaining

        uint8_t* const _tab;                // the buffer itself
        const int _sizebuf;                 // and its size (with PADDING already substracted). 

        int _posw;                          // current position in the array (for writing)
        int _posr;                          // current position in the array (for reading)

        int _r_x, _r_y, _r_len;             // current instruction (for reading)
        bool _r_cont;                       // true is (_r_x, _r_y_, _r_len) contain a valid instruction (for reading). 
        int _off;                           // current offset

        volatile uint32_t _stat_overflow;   // number of times a diff buffer overflowed
        ILI9341_T4::StatsVar _stats_size;   // statistics on buffer size
        ILI9341_T4::StatsVar _stats_time;   // statistics on compute times. 


        /** Read a value */
        uint32_t _read_encoded() __attribute__((always_inline))
            {
            const uint8_t b = _tab[_posr++];
            switch (b & 3)
                {
                case 1: // 2 bytes:encoding
                    {
                    uint32_t r = (uint32_t)(b >> 2);
                    r += (((uint32_t)_tab[_posr++]) << 6);
                    return r;
                    }
                case 3: // 3 bytes encoding
                    {
                    uint32_t r = (uint32_t)(b >> 2);
                    r += (((uint32_t)_tab[_posr++]) << 6);
                    r += (((uint32_t)_tab[_posr++]) << 14);
                    return r;
                    }
                default: // single byte encoding
                    {
                    return (uint32_t)(b >> 1);
                    }
                }
            }


        /** Write a value. WARNING : val MUST BE STRICTLY SMALLER THAN 2^22 */
        void _write_encoded(uint32_t val)  __attribute__((always_inline))
            {
            if (val <= 127)
                { // val is encoded with a single byte
                _tab[_posw++] = (uint8_t)(val << 1); // bit0=0 to indicate single byte encoding
                }
            else
                {
                if (val <= 16383)
                    { // val is encoded with 2 bytes
                    _tab[_posw++] = ((uint8_t)((val & 63) << 2) | 1); // bit0=1 and bit1=0 to indicate 2 bytes encoding
                    _tab[_posw++] = (uint8_t)((val >> 6) & 255);
                    }
                else
                    { // val is encoded on 3 bytes. 
                    _tab[_posw++] = ((uint8_t)((val & 63) << 2) | 3); // bit0=1 and bit1=1 to indicate 3 bytes encoding
                    _tab[_posw++] = (uint8_t)((val >> 6) & 255);
                    _tab[_posw++] = (uint8_t)((val >> 14) & 255);
                    }
                }
            }


        /** Write a [write,skip] sequence in the  buffer. */
        bool _write_chunk(uint32_t nbwrite, uint32_t nbskip)
            {            
            if (_posw >= _sizebuf)
                { // running out of memory buffer
                _write_encoded(TAG_WRITE_ALL);
                return false;
                }                
            _write_encoded(nbwrite); // write remaining        
            _write_encoded(nbskip); // skip remaining
            return true;
            }


        /** templated version of computeDiff */
        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        void _computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation, int gap, uint16_t compare_mask);


        /** called when the src framebuffer is in orientation 0 */
        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        void _computeDiff0(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask);


        /** called when the src framebuffer is in orientation 1 */
        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        void _computeDiff1(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask);


        /** called when the src framebuffer is in orientation 2 */
        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        void _computeDiff2(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask);


        /** called when the src framebuffer is in orientation 3 */
        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        void _computeDiff3(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask);


    };







    /******************************************************************************************
    * Class used to compute the "diff" between 2 framebuffers.
    *
    * Memory is statically allocated. The buffer size is given as template parameter SIZEBUF.
    *******************************************************************************************/
    template<int SIZEBUF>
    class DiffBuffStatic : public DiffBuff
    {

        static_assert(SIZEBUF >= MIN_BUFFER_SIZE, "template parameter SIZEBUF too small !");

    public:

        /**
        * Constructor. Assign the static array as buffer.
        **/
        DiffBuffStatic() : DiffBuff(_statictab, SIZEBUF)
            {
            }


    private:

        uint8_t _statictab[SIZEBUF];

    };





    /******************************************************************************************
    * Class used to compute a "dummy" diff between 2 framebuffers.
    *
    * No memory is allocated. The diff constructed with this object is trivial in the sense that
    * it will tell that every pixels should be rewritten...
    *******************************************************************************************/
    class DiffBuffDummy : public DiffBuffBase
    {

    public:


        /** ctor */
        DiffBuffDummy() : DiffBuffBase(), _current_line(0)
            {
            }


        /** dummy diff, but copy if needed*/
        virtual void computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation, int gap, bool copy_new_over_old, uint16_t compare_mask) override
            {
            if (copy_new_over_old)
                { // still copy if requested. 
                copyfb(fb_old, fb_new, fb_new_orientation);
                }
            initRead();
            }


        /** for compatibility withg previous verisons */
        void computeDummyDiff()
            {
            initRead();
            }


        virtual void initRead() override
            {
            _current_line = 0;
            }


        virtual int readDiff(int& x, int& y, int& len, int scanline) override;
        


    public:


        int _current_line; // index of the next line to be drawn. 

    };




}



/** end of file */

