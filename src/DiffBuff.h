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

#include <Arduino.h>
#include <inttypes.h>
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
    * - DiffBuffDummy : diff without memory alloc holding only trivial diffs (used for vsync).
    * 
    *******************************************************************************************/
    class DiffBuffBase
    {
    public:

        static const uint32_t MAX_WRITE = 32000;    // maximum number of bytes that can be written in a single instruction (to prevent DMA error with too large transfers). 

        static const int MAX_NB_SUBFRAME = 10;      // maximum number of subframes allowed in a diff.


        /** Possible Diff orientations. Matching screen orientation values **/
        enum
            {
            TOP_TO_BOTTOM = 0,
            LEFT_TO_RIGHT = 1,
            BOTTOM_TO_TOP = 2,
            RIGHT_TO_LEFT = 3
            };


        /**
        * Compute the diff between two framebuffers. Any previous diff is overwritten.
        *
        * - fb_old       : the old framebuffer of size (lx,ly) with layout fb_old(i,j) = fb_old[i + j*lx].
        * 
        * - fb_new       : the new framebuffer of size (lx,ly) with layout fb_new(i,j) = fb_new[i + j*lx].
        * 
        * - lx, ly       : framebuffers size (width and height). Must mirror the screen size. 
        * 
        * - orientation  : the diff can be split in multiple subframes in order to be uploaded to the screen 
        *                  trialing the refresh scanline to prevent screen tearing. This parameter sets the 
		*                  splitting to use for the subframes. This value should be equal to the rotation 
        *                  parameter of the ILI9341 screen. 
        * 
        * - nb_split     : number of subframes to split the diff into (between 1 and MAX_NB_SUBFRAME). Higher 
        *                  value will create slightly larger diffs but can also possibly help with vsync.
        *
        * - gap          : number of consecutives identical pixels between the two framebuffers needed to break 
        *                  the diff in two instructions. This value should be between 5 and 100. Lower values
        *                  will create more accurate diffs (fewer identical pixels will be redrawn) but it will 
        *                  require more memory to store the diff. 
        * 
        * copy_over_old  : If true, the old buffer is overwritten at the same time as the diff is computed so 
		*                  that when the method returns, the old buffer mirrors the new one. This is faster 
        *                  than doing a diff followed by a memcpy()... 
        *
        * NOTE : this method always returns a valid diff even if it runs out of memory to store the diff. However, 
		*        when this happens, the diff returned is (partly) trivial and this will have a negative impact on 
		*        the upload speed. For optimal speed, the diff buffer size/gap parameter should be chosen such that
		*        a typical diff do not overflow... The printStats() method can be useful to find how much memory a 
		*        diff typically use and thus dimension the buffer size and gap accordingly. Of course, the size of 
		*        a typical diff will depend on how much changes occurs between frames but in most case, choosing 
		*        gap=10 and a buffer size around 5K is a good starting point. 
        **/
        virtual void computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int lx, int ly, int orientation, int nb_split, int gap, bool copy_new_over_old) = 0;



        /**
        * Return the number of subframes that this diff contains.
        **/
        virtual int nbSubFrame() const = 0;


        /**
        * Return the width of the framebuffer associated with this diff.
        **/
        virtual int width() const = 0;


        /**
        * Return the height of the framebuffer associated with this diff.
        **/
        virtual int height() const = 0;


        /**
        * Call this method to reinitialize the diff prior to the first call
        * to readDiff().          
        **/
        virtual void initRead() = 0;


        /** readDiff() return values enumeration. */
        enum
            {
            DIFF_END,           // we have reached the end of the diff. 
            INSTRUCTION,        // an instruction has been put in (x,y, len)
            NEW_SUBFRAME        // starting a new subframe
            };


        /**
        * Read the next instruction in the diff.
        * 
        * - returns INSTRUCTION: in this case  x, y, len are updated to contain the 
        *   instruction meaning: 
        *    'copy the [len] next pixels starting from offset [x] + width()*[y]'
        * 
        * - returns NEW_SUBFRAME. the variables x,y,len are NOT changed but 
        *   subFrameSyncTimes() can now be called to find the subframe relative
        *   position. 
        * 
        * - returns DIFF_END. This means that the end of the diff has been reached 
		*   and no more instruction are available. readDiff() should not be called 
		*   anymore until calling initRead() or computeDiff() again.
        **/
        virtual int readDiff(int& x, int& y, int& len) = 0;


        /**
        * Return the normalized starting and ending position of the current subframe
        * (during diff reading). 
        **/
        virtual void subFrameSyncTimes(double& start, double& end) = 0;

    };




    /******************************************************************************************
    * Class used to compute the "diff" between 2 framebuffers.
    *
    * The memory for holding the diff is allocated by the user and is passed to the object a 
    * construction time.
    *
    * PERFORMANCE: On teensy 4.1, for framebuffers of size 320x240. It takes around 1ms to 
    * compute a diff when both framebuffers are located in DMAMEM and around 700us when they 
    * are in DTCM. This means that computing the diff consumes about 5% of a frame period at 60FPS. 
    * So there is still plenty of CPU compute time available to generate the frame...
    *******************************************************************************************/
    class DiffBuff : public DiffBuffBase
    {

    public:


        /**
        * Constructor. Set the buffer and its buffer size.
        * sizebuf should not be too small (at least PADDING but say 1K to be sure).
        **/
        DiffBuff(uint8_t* buffer, size_t sizebuf) : _tab(buffer), _sizebuf(sizebuf - PADDING), _posw(0), _posr(0), _nbsubframe(0), _stride(0), _height(0), _scale(1),  _r_cont(false)
            {
            if (_sizebuf > 0) _write_encoded(TAG_END);
            _posw = 0;
            initRead();
            statsReset();
            }


        virtual void computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int lx, int ly, int orientation, int nb_split, int gap, bool copy_new_over_old) override;
          

        virtual int nbSubFrame() const override { return _nbsubframe; }


        virtual int width() const override { return _stride*_scale; }


        virtual int height() const  override { return _height; }


        virtual void initRead() override
            {
            _posr = 0;
            _r_cont = false;
            }


        virtual int readDiff(int& x, int& y, int& len) override;


        virtual void subFrameSyncTimes(double& start, double& end) override;


        /**
        * Return the current size of the diff.
        * Return the total size of the allocated buffer if full (or nearly full).
        **/
        uint32_t size() const { return (uint32_t)((_posw >= _sizebuf) ? (_sizebuf + PADDING) : _posw); }



        /************************************************************************
        * STATISTICS. 
        * 
        * The methods can be useful to monitor resource use and optimize the diff
        * parameters (memory size / gap / number of splits...). 
        ************************************************************************/


        /**
        * Reset all statistics.
        **/
        void statsReset();


        /**
        * Return the number of diff created (since the last call to statsReset()). 
        **/
        uint32_t statsNb() const { return _stat_nb; }


        /**
        * Return the number of diff for which the buffer overflowed.
        **/
        uint32_t statsOverflow() const { return _stat_overflow; }


        /**
        * Return the percentage of diff that overflowed (between 0 and 100).
        **/
        double statsOverflowRatio() const { return ((_stat_nb > 0) ? ((_stat_overflow * 100.0) / _stat_nb) : 0.0); }


        /**
        * Return the size of the smallest diff created (in bytes).
        **/
        uint32_t statsMinSize() const { return _stat_min; }


        /**
        * Return the size of the largest diff created (in bytes).
        **/
        uint32_t statsMaxSize() const { return _stat_max; }


        /**
        * Return the average size the diffs created.
        **/
        uint32_t statsAvgSize() const { return ((_stat_nb > 0) ? round(((double)_stat_sum) / _stat_nb) : 0); }


        /**
        * Return the std on the size of the diffs created.
        **/
        uint32_t statsStdSize() const
            {
            if (_stat_nb == 0) return 0;
            const double a = ((double)_stat_sum);
            const double b = ((double)_stat_sumsqr);
            const double c = sqrt((b / _stat_nb) - ((a * a) / (((uint64_t)_stat_nb) * _stat_nb)));
            return (uint32_t)round(c);
            }


        /**
        * Return the minimum time used to compute a diff (in us).
        **/
        uint32_t statsMinTime() const { return _stat_mintime; }


        /**
        * Return the maximum time used to compute a diff (in us).
        **/
        uint32_t statsMaxTime() const { return _stat_maxtime; }


        /**
        * Return the average time used to compute a diff (in us).
        **/
        uint32_t statsAvgTime() const { return ((_stat_nb > 0) ? round(((double)_stat_sumtime) / _stat_nb) : 0); }


        /**
        * Return the std on the time used to compute a diff (in us).
        **/
        uint32_t statsStdTime() const
            {
            if (_stat_nb == 0) return 0;
            const double a = ((double)_stat_sumtime);
            const double b = ((double)_stat_sumsqrtime);
            const double c = sqrt((b / _stat_nb) - ((a * a) / (((uint64_t)_stat_nb) * _stat_nb)));
            return (uint32_t)round(c);
            }


        /**
        * Print all the statistics into a Stream object.
        **/
        void printStats(Stream* outputStream = &Serial) const;
       


    private:


        /************************************************************************
        * This is private ! Move along !
        *************************************************************************/

        static const int PADDING = (17 * MAX_NB_SUBFRAME) + 12; // minimum size needed to accomodate all subframes.

        static const int TRY_EXPAND_LOOP = 4;                   // number of times the inner loop in computeDiff() is expanded for speedup. 4 seems optimal... 
        static_assert((TRY_EXPAND_LOOP > 0) && (TRY_EXPAND_LOOP <= 8), "TRY_EXPAND_LOOP must be between 1 and 8 !");

        static const uint32_t TAG_END = (0x400000 - 1);         // tag at end of diff
        static const uint32_t TAG_HEADER = (0x400000 - 2);      // tag that annonces a subframe header
        static const uint32_t TAG_WRITE_ALL = (0x400000 - 3);   // tag meaning 'write everything from this point on for this subframe' (used when running out of buffer space).

        uint8_t* const _tab;                // the buffer    
        const int _sizebuf;                 // and its size (where we already substracted PADDING). 

        int _posw;                          // current position in the array (for writing)
        int _posr;                          // current position in the array (for reading)
        int _nbsubframe;                    // number of subframe stored in the diffbuffer. 
        int _stride;                        // the framebuffer stride ( = width)
        int _height;                        // the framebuffer height
        int _orientation;                   // the framebuffer orientation. 
        int _scale;                         // either 1 or 2. Value (ie positions) in the diff buffer are mutliplied by this value when reading. 

        /* member variables used when reading the diff */

        int _frame_x, _frame_y, _frame_lx, _frame_ly; // info about the current frame (for reading).    
        int _read_off = 0;                            // current offset in the subframe (for reading).

        int _r_x, _r_y, _r_len;                       // current instruction (for reading)
        bool _r_cont;                                 // true is (_r_x, _r_y_, _r_len) contain a valid instruction (for reading). 


        /* member variables used for statistics */

        volatile uint32_t _stat_nb;         // number of time a diff buffer was used
        volatile uint32_t _stat_overflow;   // number of times a diff buffer overflowed
        volatile uint32_t _stat_min;        // min diff buffer size used
        volatile uint32_t _stat_max;        // max diff buffer size used
        volatile uint64_t _stat_sum;        // sum of all the diff buffer size
        volatile uint64_t _stat_sumsqr;     // sum of the square of the buffer size for computing the std
        volatile uint32_t _stat_mintime;    // min time it tok to compute a diff
        volatile uint32_t _stat_maxtime;    // max time it tok to compute a diff
        volatile uint64_t _stat_sumtime;    // sum of times used to compute diffs
        volatile uint64_t _stat_sumsqrtime; // sum of the squares of the times for computing the std


        /**
        * Read a value in the diff buffer
        **/
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


        /**
        * Read a 'raw' instruction from the diff buffer.
        * return false if we reached the end of the diff.
        **/
        int _readDiffRaw(int& x, int& y, int& len);


        /**
         * Write val in the diff buffer.
         * WARNING : val MUST BE STRICTLY SMALLER THAN 2^22.
        **/
        void _write_encoded(uint32_t val) __attribute__((always_inline))
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


        /**
        * Write a [write,skip] sequence in the diff buffer.
        **/
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


        /**
        * Compute size of subframe (trying to keep it multiple of TRY_EXPAND_LOOP).
        **/
        static int _getsplit(int l, int nb_split)
            {
            int u = l / (TRY_EXPAND_LOOP * nb_split);
            return (TRY_EXPAND_LOOP * u);
            }


        /**
        * templated version of computeDiff.
        **/
        template<bool COPY_NEW_OVER_OLD, typename T>
        void _computeDiff(T * fb_old, const T * fb_new, int lx, int ly, int orientation, int nb_split, int gap);


        /**
        * write the diff of two framebuffers inside a sub-frame.
        * templated on COPY_OVER_OLD parameter. 
        **/
        template<bool COPY_NEW_OVER_OLD, typename T>
        void _computeSubFrame(T * fb_old, const T * fb_new, const int x, const int y, const int lx, const int ly, const int stride, const int gap);


        /**
        * as above, but templated also on EXPAND_LOOP value.
        **/
        template<bool COPY_NEW_OVER_OLD, int EXPAND_LOOP, typename T>
        void _computeSubFrame2(T * fb_old, const T * fb_new, const int x, const int y, const int lx, const int ly, const int stride, const int gap);


    };




    /******************************************************************************************
    * Class used to compute the "diff" between 2 framebuffers.
    *
    * Memory is statically alloacated. The buffer size is given as template parameter SIZEBUF.
    *
    * PERFORMANCE: On teensy 4.1, for framebuffers of size 320x240. It takes around 1ms to
    * compute a diff when both framebuffers are located in DMAMEM and around 700us when they
    * are in DTCM. This means that computing the diff consumes about 5% of a frame period at 60FPS.
    * So there is still plenty of CPU compute time available to generate the frame...
    *******************************************************************************************/
    template<int SIZEBUF>
    class DiffBuffStatic : public DiffBuff
    {

        static_assert(SIZEBUF > PADDING, "template parameter SIZEBUF too small !");


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
    * it will tell that every pixels should be rewritten... Yet, it is still useful because it 
    * generates subframes splitting so it can be used to redraw a whole screen with vsync. 
    *******************************************************************************************/
    class DiffBuffDummy : public DiffBuffBase
    {

    public:

 
        DiffBuffDummy() : _nbsubframe(0), _stride(0), _height(0), _orientation(0), _curframe(0), _curline(-1)
            {
            }


        virtual void computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int lx, int ly, int orientation, int nb_split, int gap, bool copy_new_over_old) override;


        void computeDummyDiff(int lx, int ly, int orientation, int nb_split)
            {
            computeDiff(nullptr, nullptr, lx, ly, orientation, nb_split, 0, false);
            }


        virtual int nbSubFrame() const override { return _nbsubframe; }


        virtual int width() const override { return _stride; }


        virtual int height() const  override { return _height; }


        virtual void initRead() override
            {
            _curframe = 0;
            _curline = -1; 
            }


        virtual int readDiff(int& x, int& y, int& len) override;


        virtual void subFrameSyncTimes(double& start, double& end) override;


    private:


        int _nbsubframe;                    // number of subframe stored in the diffbuffer. 
        int _stride;                        // the framebuffer stride ( = width)
        int _height;                        // the framebuffer height
        int _orientation;                   // the framebuffer orientation. 

        int _curframe;                      // current frame
        int _curline;                       // current line. 
        int _frame_x, _frame_y, _frame_lx, _frame_ly; // info about the current frame

    };




}


/** end of file */

