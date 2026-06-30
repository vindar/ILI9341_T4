/******************************************************************************
* Generic differential framebuffer engine used by T4 SPI display drivers.
*
* This file is header-only because the diff engine is templated on display geometry.
*******************************************************************************/

#ifndef _T4_DIFFBUFF_TEMPLATE_H_
#define _T4_DIFFBUFF_TEMPLATE_H_

#ifdef __cplusplus

#include "StatsVar.h"

#include <Arduino.h>
#include <math.h>

#ifndef T4DIFF_ALWAYS_INLINE
#define T4DIFF_ALWAYS_INLINE __attribute__((always_inline))
#endif

#ifndef T4DIFF_INLINE_COMPUTE_KERNEL
#define T4DIFF_INLINE_COMPUTE_KERNEL inline __attribute__((always_inline))
#endif

namespace T4Diff
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
    template<class Traits>
    class DiffBuffBaseT
    {
    public:


        /** Framebuffer orientation**/
        enum
            {
            ROT0 = 0,
            ROT90 = 1,
            ROT180 = 2,
            ROT270 = 3,

            // Legacy ILI9341_T4 orientation names kept for source compatibility.
            PORTRAIT_240x320 = ROT0,
            LANDSCAPE_320x240 = ROT90,
            PORTRAIT_240x320_FLIPPED = ROT180,
            LANDSCAPE_320x240_FLIPPED = ROT270,
            };


        static constexpr int LX = Traits::LX;                            // framebuffer width in orientation 0
        static constexpr int LY = Traits::LY;                            // framebuffer height in orientation 0
        static constexpr int MAX_WRITE_LINE = Traits::MAX_WRITE_LINE;    // max number of lines to be written in a single operation.
        static constexpr int MIN_SCANLINE_SPACE = Traits::MIN_SCANLINE_SPACE; // min number of lines between current write line and scanline

        static_assert(LX > 0, "LX must be positive");
        static_assert(LY > 0, "LY must be positive");
        static_assert((LX & 3) == 0, "LX must be divisible by 4");
        static_assert(((LX * LY) & 1) == 0, "LX*LY must be even for 32-bit diff fast paths");


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
        * Compute a diff between a (old) framebuffer and a region of a new framebuffer while merging the
        * result with a previous diff (if provided).
        * 
        * old_diff must be another diff buffer (not this object) or nullptr if nothing was previously changed.  
        *
        * the position of the region in the old diff buffer is described by sub_fb_new is (xmin,xmax,ymin,ymax) 
        * when fb_old is rotated from orientation 0 to orientation 'fb_new_orientation'
        *
        * The layout is Pixel(xmin+x,ymin+y) = sub_fb_new[x + stride*y]
        *
        **/
        virtual void computeDiff(uint16_t* fb_old, DiffBuffBaseT<Traits>* diff_old, const uint16_t* sub_fb_new, int xmin, int xmax, int ymin, int ymax, int stride, 
                                 int fb_new_orientation, int gap, bool copy_new_over_old = true, uint16_t compare_mask = 0) = 0;


        /**
        * Copy the new framebuffer over the old one (and rotate it to put it in orientation 0 in fb_old). 
        **/
        static void copyfb(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation);
            

        /**
        * Copy a sub framebuffer over an old one (and rotate it to put it in orientation 0 in fb_old).
        *
        * the position of the region in the old diff buffer is described by sub_fb_new is (xmin,xmax,ymin,ymax) 
        * when fb_old is rotated from orientation 0 to orientation 'fb_new_orientation'
        *
        * The layout is Pixel(xmin+x,ymin+y) = sub_fb_new[x + stride*y]
        *
        **/
        static void copyfb(uint16_t* fb_old, const uint16_t* fb_new, int xmin, int xmax, int ymin, int ymax, int src_stride, int fb_new_orientation);

 
        /**
        * Call this method to reinitialize the diff prior to the first call
        * to readDiff().          
        **/
        virtual void initRead() = 0;


        /**
        * Return the best possible scanline position at the begining of the diff
        * - scanline: the current position of the scanline
        **/
        virtual int scanlineStartInit() = 0; 

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




        /**
        * Call this method to reinitialize the diff prior to the first call
        * to readRaw().
        **/
        virtual void initRaw() = 0;


        /**
        * Read a 'raw' instruction (nb_write, nb_read) from the diff buffer.
        * return (LX*LY + 1, 0) for TAG_WRITE_ALL and (0, LX*LY + 1) for TAG_END.  
        **/
        virtual void readRaw(int & nbwrite, int & nbskip) = 0;



        /**
        * Transform a box according from a given orientation to orientation 0.
        * (xmin,xmax,ymin,ymax) describe the box w.r.t. orientation 'orientation'
        * 
        * the method fills (x1,x2,y1,y2) with the box coord. according to orientation 0. 
        **/
        static void rotationBox(int orientation, int xmin, int xmax, int ymin, int ymax, int & x1, int & x2, int & y1, int & y2);


        /**
        * Print all the statistics into a Stream object.
        **/
        virtual void printStats(Stream* outputStream = &Serial) const = 0;


    private:
        
        // copy and rotate a framebuffer
        
        static void _copy_rotate_0(uint16_t* fb_dest, const uint16_t* fb_src);           

        static void _copy_rotate_90(uint16_t* fb_dest, const uint16_t* fb_src);        

        static void _copy_rotate_180(uint16_t* fb_dest, const uint16_t* fb_src);
           
        static void _copy_rotate_270(uint16_t* fb_dest, const uint16_t* fb_src);
           
        // copy and rotate a sub-framebuffer into a framebuffer.
        
        static void _copy_rotate_0(uint16_t* fb_dest, const uint16_t* fb_src, int x1, int x2, int y1, int y2, int w, int h, int src_stride);

        static void _copy_rotate_90(uint16_t* fb_dest, const uint16_t* fb_src, int x1, int x2, int y1, int y2, int w, int h, int src_stride);

        static void _copy_rotate_180(uint16_t* fb_dest, const uint16_t* fb_src, int x1, int x2, int y1, int y2, int w, int h, int src_stride);

        static void _copy_rotate_270(uint16_t* fb_dest, const uint16_t* fb_src, int x1, int x2, int y1, int y2, int w, int h, int src_stride);
                     

    };








    /******************************************************************************************
    * Class used to compute the "diff" between 2 framebuffers.
    *
    * The memory for holding the diff is allocated by the user and is passed to the object a
    * construction time.
    *
    * PERFORMANCE: On Teensy 4.x, a full QVGA-size framebuffer diff typically takes around 1ms.
    * This means that computing the diff consumes around 5-10% of a frame period at 60FPS
    * (but still leaves around 15ms to generate each frame).
    *******************************************************************************************/
    template<class Traits>
    class DiffBuffT : public DiffBuffBaseT<Traits>
    {

    public:

        /**
        * Constructor. Set the buffer (and its size).
        * sizebuf should not be too small (at least MIN_BUFFER_SIZE but say 1K to be useful).
        **/
        DiffBuffT(uint8_t* buffer, size_t sizebuf) : DiffBuffBaseT<Traits>(), _tab(buffer), _sizebuf(sizebuf - PADDING), _posw(0), _posr(0), _posraw(0)
            {
            statsReset();
            _write_encoded(TAG_END);
            initRead();
            initRaw();
            }



        virtual void computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation, int gap, bool copy_new_over_old, uint16_t compare_mask) override;


        virtual void computeDiff(uint16_t* fb_old, DiffBuffBaseT<Traits>* diff_old, const uint16_t* sub_fb_new, int xmin, int xmax, int ymin, int ymax, int stride,
                                 int fb_new_orientation, int gap, bool copy_new_over_old, uint16_t compare_mask) override;


        virtual void initRead() override
            {
            _r_cont = false;
            _posr = 0; 
            _off = 0; 
            }


        virtual int scanlineStartInit() override; 


        virtual int readDiff(int& x, int& y, int& len, int scanline) override;


        virtual void initRaw()
            {
            _posraw = 0;
            }


        virtual void readRaw(int& nbwrite, int& nbskip) override
            {
            nbwrite = _read_encoded(_posraw);
            if (nbwrite == TAG_END) 
                { 
                nbwrite = 0;  
                nbskip = DiffBuffBaseT<Traits>::LX * DiffBuffBaseT<Traits>::LY + 1;
                }
            else if (nbwrite == TAG_WRITE_ALL) 
                { 
                nbwrite = DiffBuffBaseT<Traits>::LX * DiffBuffBaseT<Traits>::LY + 1;
                nbskip = 0; 
                }
            else 
                { 
                nbskip = _read_encoded(_posraw);
                }
            }


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
        * Print all the statistics into a Stream object.
        **/
        virtual void printStats(Stream* outputStream = &Serial) const override;


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
        * Return the percentage of diff that overflowed (between 0 and 1).
        **/
        float statsOverflowRatio() const { return ((statsNbComputed() > 0) ? (((float)_stat_overflow) / statsNbComputed()) : 0.0f); }


        /**
        * Return a StatsVar object containing statisitcs about the time
        * it took to compute the diffs.
        **/
        StatsVar statsTime() const { return _stats_time; }


        /**
        * Return a StatVar  object containing statisitcs about the size
        * of the computed buffers.
        **/
        StatsVar statsSize() const { return _stats_size; }






        static const int        MIN_BUFFER_SIZE = 16;             // minimum buffer size

    private:

        static const int        PADDING = 16;                     // reserved at end of buffer (in case of overflow)
        static const uint32_t   TAG_END = (0x400000 - 1);         // tag at end of diff
        static const uint32_t   TAG_WRITE_ALL = (0x400000 - 2);   // tag to write everything remaining


        uint8_t* const _tab;                // the buffer itself
        const int _sizebuf;                 // and its size (with PADDING already substracted). 

        int _posw;                          // current position in the array (for writing)
        int _posr;                          // current position in the array (for reading)
        int _posraw;                        // current position in the array for raw reading

        int _r_x, _r_y, _r_len;             // current instruction (for reading)
        bool _r_cont;                       // true is (_r_x, _r_y_, _r_len) contain a valid instruction (for reading). 
        int _off;                           // current offset

        volatile uint32_t _stat_overflow;   // number of times a diff buffer overflowed
        StatsVar _stats_size;   // statistics on buffer size
        StatsVar _stats_time;   // statistics on compute times. 


        /** Read a value */
        uint32_t _read_encoded(int & pos) T4DIFF_ALWAYS_INLINE
            {
            const uint8_t b = _tab[pos++];
            switch (b & 3)
                {
                case 1: // 2 bytes:encoding
                    {
                    uint32_t r = (uint32_t)(b >> 2);
                    r += (((uint32_t)_tab[pos++]) << 6);
                    return r;
                    }
                case 3: // 3 bytes encoding
                    {
                    uint32_t r = (uint32_t)(b >> 2);
                    r += (((uint32_t)_tab[pos++]) << 6);
                    r += (((uint32_t)_tab[pos++]) << 14);
                    return r;
                    }
                default: // single byte encoding
                    {
                    return (uint32_t)(b >> 1);
                    }
                }
            }


        /** Write a value. WARNING : val MUST BE STRICTLY SMALLER THAN 2^22 */
        void _write_encoded(uint32_t val) T4DIFF_ALWAYS_INLINE
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
        int _computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation, int gap, uint16_t compare_mask);


        /** called when the src framebuffer is in orientation 0 */
        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        int _computeDiff0(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask);


        /** called when the src framebuffer is in orientation 1 */
        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        int _computeDiff1(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask);


        /** called when the src framebuffer is in orientation 2 */
        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        int _computeDiff2(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask);


        /** called when the src framebuffer is in orientation 3 */
        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        int _computeDiff3(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask);


        /** main method when computing partial diff */
        int _computeDiff(uint16_t* fb_old, DiffBuffBaseT<Traits>* diff_old, const uint16_t* sub_fb_new, int xmin, int xmax, int ymin, int ymax, int stride,
                          int fb_new_orientation, int gap, bool copy_new_over_old, uint16_t compare_mask);

    };







    /******************************************************************************************
    * Class used to compute the "diff" between 2 framebuffers.
    *
    * Memory is statically allocated. The buffer size is given as template parameter SIZEBUF.
    *******************************************************************************************/
    template<class Traits, int SIZEBUF>
    class DiffBuffStaticT : public DiffBuffT<Traits>
    {

        static_assert(SIZEBUF >= DiffBuffT<Traits>::MIN_BUFFER_SIZE, "template parameter SIZEBUF too small !");

    public:

        /**
        * Constructor. Assign the static array as buffer.
        **/
        DiffBuffStaticT() : DiffBuffT<Traits>(_statictab, SIZEBUF)
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
    template<class Traits>
    class DiffBuffDummyT : public DiffBuffBaseT<Traits>
    {

    public:


        /** ctor */
        DiffBuffDummyT() : DiffBuffBaseT<Traits>(), _current_line(0), _begin(0), _end(DiffBuffBaseT<Traits>::LY), _rawnb(0)
            {
            initRead();
            initRaw();
            }


        /** dummy diff, but copy if needed*/
        virtual void computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation, int gap, bool copy_new_over_old, uint16_t compare_mask) override
            {
            if (copy_new_over_old)
                { // still copy if requested. 
                DiffBuffBaseT<Traits>::copyfb(fb_old, fb_new, fb_new_orientation);
                }
            _begin = 0;
            _end = DiffBuffBaseT<Traits>::LY;
            initRead();
            }


        virtual void computeDiff(uint16_t* fb_old, DiffBuffBaseT<Traits>* diff_old, const uint16_t* sub_fb_new, int xmin, int xmax, int ymin, int ymax, int stride,
                                 int fb_new_orientation, int gap, bool copy_new_over_old, uint16_t compare_mask) override
            {
            if (copy_new_over_old)
                { // still copy if requested. 
                DiffBuffBaseT<Traits>::copyfb(fb_old, sub_fb_new, xmin, xmax, ymin, ymax, stride, fb_new_orientation);
                }
            _begin = 0;
            _end = DiffBuffBaseT<Traits>::LY;
            initRead();
            }


        /** create a diff that redraws every line in [begin, end[ */
        void computeDummyDiff(int begin = 0, int end = DiffBuffBaseT<Traits>::LY)
            {
            _begin = (begin < 0) ? 0 : begin;
            _end = (end > DiffBuffBaseT<Traits>::LY) ? DiffBuffBaseT<Traits>::LY : end;
            initRead();
            }


        virtual void initRead() override
            {
            _current_line = _begin;
            }


        virtual int scanlineStartInit() override; 

        virtual int readDiff(int& x, int& y, int& len, int scanline) override;
        

        virtual void initRaw()
            {
            _rawnb = 0;
            }

        virtual void readRaw(int& nbwrite, int& nbskip) override
            {
            if (_rawnb == 0)
                {
                _rawnb = 1;
                nbwrite = 0;
                nbskip = DiffBuffBaseT<Traits>::LX * _begin;
                if (nbskip > 0) return;
                }
            if (_rawnb == 1)
                {
                _rawnb = 2;
                nbwrite = DiffBuffBaseT<Traits>::LX * (_end - _begin);
                nbskip = DiffBuffBaseT<Traits>::LX * (DiffBuffBaseT<Traits>::LY - _end);
                return;
                }
            nbwrite = 0;
            nbskip = DiffBuffBaseT<Traits>::LX * DiffBuffBaseT<Traits>::LY + 1;
            }


        /** set the diff buffer as empty(no change) when using the readRaw command */ 
        void setRawEmpty()
            {
            _rawnb = 2;    
            }


        /**
        * Print all the statistics into a Stream object.
        **/
        virtual void printStats(Stream* outputStream = &Serial) const override
            {
            if (outputStream)
                {
                outputStream->print("--- DiffBuffDummyT ---\n");
                }
            }


    private:


        int _current_line; // index of the next line to be drawn. 
        int _begin;
        int _end;
        int _rawnb;

    };








        template<class Traits>
        static FLASHMEM void copyfbFromOffset(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation, int offset)
            {
            const int total = DiffBuffBaseT<Traits>::LX * DiffBuffBaseT<Traits>::LY;
            if (offset >= total) return;
            switch (fb_new_orientation)
                {
            case DiffBuffBaseT<Traits>::ROT0:
                memcpy(fb_old + offset, fb_new + offset, sizeof(uint16_t) * (total - offset));
                return;
            case DiffBuffBaseT<Traits>::ROT90:
                for (int y = offset / DiffBuffBaseT<Traits>::LX; y < DiffBuffBaseT<Traits>::LY; y++)
                    {
                    const int x0 = offset - (DiffBuffBaseT<Traits>::LX * y);
                    uint16_t* pold = fb_old + x0 + (DiffBuffBaseT<Traits>::LX * y);
                    const uint16_t* pnew = fb_new + y + DiffBuffBaseT<Traits>::LY * (DiffBuffBaseT<Traits>::LX - 1 - x0);
                    for (int x = x0; x < DiffBuffBaseT<Traits>::LX; x++)
                        {
                        *(pold++) = *pnew;
                        pnew -= DiffBuffBaseT<Traits>::LY;
                        }
                    offset = DiffBuffBaseT<Traits>::LX * (y + 1);
                    }
                return;
            case DiffBuffBaseT<Traits>::ROT180:
                {
                uint16_t* pold = fb_old + offset;
                const uint16_t* pnew = fb_new + total - 1 - offset;
                while (offset++ < total) { *(pold++) = *(pnew--); }
                }
                return;
            case DiffBuffBaseT<Traits>::ROT270:
                for (int y = offset / DiffBuffBaseT<Traits>::LX; y < DiffBuffBaseT<Traits>::LY; y++)
                    {
                    const int x0 = offset - (DiffBuffBaseT<Traits>::LX * y);
                    uint16_t* pold = fb_old + x0 + (DiffBuffBaseT<Traits>::LX * y);
                    const uint16_t* pnew = fb_new + (DiffBuffBaseT<Traits>::LY - 1 - y) + DiffBuffBaseT<Traits>::LY * x0;
                    for (int x = x0; x < DiffBuffBaseT<Traits>::LX; x++)
                        {
                        *(pold++) = *pnew;
                        pnew += DiffBuffBaseT<Traits>::LY;
                        }
                    offset = DiffBuffBaseT<Traits>::LX * (y + 1);
                    }
                return;
                }
            }


        template<class Traits>
        static FLASHMEM void copySubFbFromOffset(uint16_t* fb_old, const uint16_t* sub_fb_new, int xmin, int xmax, int ymin, int ymax, int stride,
            int fb_new_orientation, int offset)
            {
            for (int yc = ymin; yc <= ymax; yc++)
                {
                int m = 0, mdelta = 0;
                switch (fb_new_orientation)
                    {
                case DiffBuffBaseT<Traits>::ROT0:
                    m = stride * (yc - ymin);
                    mdelta = 1;
                    break;
                case DiffBuffBaseT<Traits>::ROT90:
                    m = (yc - ymin) + stride * (xmax - xmin);
                    mdelta = -stride;
                    break;
                case DiffBuffBaseT<Traits>::ROT180:
                    m = stride * (ymax - yc) + (xmax - xmin);
                    mdelta = -1;
                    break;
                case DiffBuffBaseT<Traits>::ROT270:
                    m = ymax - yc;
                    mdelta = stride;
                    break;
                    }

                int n = xmin + (DiffBuffBaseT<Traits>::LX * yc);
                const int nend = xmax + (DiffBuffBaseT<Traits>::LX * yc);
                if (offset > nend) continue;
                if (offset > n)
                    {
                    const int d = offset - n;
                    n += d;
                    m += d * mdelta;
                    }
                while (n <= nend)
                    {
                    fb_old[n++] = sub_fb_new[m];
                    m += mdelta;
                    }
                }
            }



        template<class Traits>
        FLASHMEM void DiffBuffBaseT<Traits>::rotationBox(int orientation, int xmin, int xmax, int ymin, int ymax, int& x1, int& x2, int& y1, int& y2)
            {
            switch (orientation)
                {
            case DiffBuffBaseT<Traits>::ROT90:
                x1 = DiffBuffBaseT<Traits>::LX - 1 - ymax;
                x2 = DiffBuffBaseT<Traits>::LX - 1 - ymin;
                y1 = xmin;
                y2 = xmax;
                break;
            case DiffBuffBaseT<Traits>::ROT180:
                x1 = DiffBuffBaseT<Traits>::LX - 1 - xmax;
                x2 = DiffBuffBaseT<Traits>::LX - 1 - xmin;
                y1 = DiffBuffBaseT<Traits>::LY - 1 - ymax;
                y2 = DiffBuffBaseT<Traits>::LY - 1 - ymin;
                break;
            case DiffBuffBaseT<Traits>::ROT270:
                x1 = ymin;
                x2 = ymax;
                y1 = DiffBuffBaseT<Traits>::LY - 1 - xmax;
                y2 = DiffBuffBaseT<Traits>::LY - 1 - xmin;
                break;
            default: // case DiffBuffBaseT<Traits>::ROT0:
                x1 = xmin;
                x2 = xmax;
                y1 = ymin;
                y2 = ymax;
                break;
                }
            }


        template<class Traits>
        FLASHMEM void DiffBuffBaseT<Traits>::copyfb(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation)
            {
            switch (fb_new_orientation)
                {
                case DiffBuffBaseT<Traits>::ROT0:
                    _copy_rotate_0(fb_old, fb_new);
                    return;
                case DiffBuffBaseT<Traits>::ROT90:
                    _copy_rotate_90(fb_old, fb_new);
                    return;
                case DiffBuffBaseT<Traits>::ROT180:
                    _copy_rotate_180(fb_old, fb_new);
                    return;
                case DiffBuffBaseT<Traits>::ROT270:
                    _copy_rotate_270(fb_old, fb_new);
                    return;
                }
            // hum...
            return;
            }


        template<class Traits>
        FLASHMEM void DiffBuffBaseT<Traits>::copyfb(uint16_t* fb_old, const uint16_t* fb_new, int xmin, int xmax, int ymin, int ymax, int src_stride, int fb_new_orientation)
            {
            int x1, x2, y1, y2;
            rotationBox(fb_new_orientation, xmin, xmax, ymin, ymax, x1, x2, y1, y2);
            const int w = x2 - x1 + 1;
            const int h = y2 - y1 + 1;
            switch (fb_new_orientation)
                {
                case DiffBuffBaseT<Traits>::ROT0:
                    _copy_rotate_0(fb_old, fb_new, x1, x2, y1, y2, w, h, src_stride);
                    return;
                case DiffBuffBaseT<Traits>::ROT90:
                    _copy_rotate_90(fb_old, fb_new, x1, x2, y1, y2, w, h, src_stride);
                    return;
                case DiffBuffBaseT<Traits>::ROT180:
                    _copy_rotate_180(fb_old, fb_new, x1, x2, y1, y2, w, h, src_stride);
                    return;
                case DiffBuffBaseT<Traits>::ROT270:
                    _copy_rotate_270(fb_old, fb_new, x1, x2, y1, y2, w, h, src_stride);
                    return;
                }
            // hum...
            return;
            }


    
        template<class Traits>
        FLASHMEM void DiffBuffBaseT<Traits>::_copy_rotate_0(uint16_t* fb_dest, const uint16_t* fb_src)
            {
            memcpy(fb_dest, fb_src, sizeof(uint16_t) * DiffBuffBaseT<Traits>::LX * DiffBuffBaseT<Traits>::LY);
            }


        template<class Traits>
        FLASHMEM void DiffBuffBaseT<Traits>::_copy_rotate_90(uint16_t* fb_dest, const uint16_t* fb_src)
            {
            uint16_t* p = fb_dest;
            for (int i = 0; i < DiffBuffBaseT<Traits>::LY; i++)
                {
                int j = DiffBuffBaseT<Traits>::LX - 1;
                while (j >= 0)
                    {
                    *(p++) = fb_src[i + DiffBuffBaseT<Traits>::LY * (j--)];
                    *(p++) = fb_src[i + DiffBuffBaseT<Traits>::LY * (j--)];
                    *(p++) = fb_src[i + DiffBuffBaseT<Traits>::LY * (j--)];
                    *(p++) = fb_src[i + DiffBuffBaseT<Traits>::LY * (j--)];
                    }
                }
            }


        template<class Traits>
        FLASHMEM void DiffBuffBaseT<Traits>::_copy_rotate_180(uint16_t* fb_dest, const uint16_t* fb_src)
            {
            uint16_t* p = fb_dest;
            for (int j = DiffBuffBaseT<Traits>::LY - 1; j >= 0; j--)
                {
                int i = DiffBuffBaseT<Traits>::LX - 1;
                const int oo = DiffBuffBaseT<Traits>::LX * j;
                while (i >= 0)
                    {
                    *(p++) = fb_src[(i--) + oo];
                    *(p++) = fb_src[(i--) + oo];
                    *(p++) = fb_src[(i--) + oo];
                    *(p++) = fb_src[(i--) + oo];
                    }
                }
            }


        template<class Traits>
        FLASHMEM void DiffBuffBaseT<Traits>::_copy_rotate_270(uint16_t* fb_dest, const uint16_t* fb_src)
            {
            uint16_t* p = fb_dest;
            for (int i = DiffBuffBaseT<Traits>::LY - 1; i >= 0; i--)
                {
                int j = 0;
                while (j < DiffBuffBaseT<Traits>::LX)
                    {
                    *(p++) = fb_src[i + DiffBuffBaseT<Traits>::LY * (j++)];
                    *(p++) = fb_src[i + DiffBuffBaseT<Traits>::LY * (j++)];
                    *(p++) = fb_src[i + DiffBuffBaseT<Traits>::LY * (j++)];
                    *(p++) = fb_src[i + DiffBuffBaseT<Traits>::LY * (j++)];
                    }
                }
            }


        template<class Traits>
        FLASHMEM void DiffBuffBaseT<Traits>::_copy_rotate_0(uint16_t* fb_dest, const uint16_t* fb_src, int x1, int x2, int y1, int y2, int w, int h, int src_stride)
            {
            uint16_t* p = fb_dest + x1 + (DiffBuffBaseT<Traits>::LX*y1);
            for (int j = 0; j < h; j++)
                {
                for (int i = 0; i < w; i++)
                    {
                    *(p++)= fb_src[i + (src_stride * j)];
                    }
                p += (DiffBuffBaseT<Traits>::LX - w);
                }
            }


        template<class Traits>
        FLASHMEM void DiffBuffBaseT<Traits>::_copy_rotate_90(uint16_t* fb_dest, const uint16_t* fb_src, int x1, int x2, int y1, int y2, int w, int h, int src_stride)
            {
            uint16_t* p = fb_dest + x1 + (DiffBuffBaseT<Traits>::LX*y1);
            for (int j = 0; j < h; j++)
                {   
                for (int i = w-1; i >= 0 ; i--)                    
                    {
                    *(p++) = fb_src[j + (src_stride * i)];
                    }
                p += (DiffBuffBaseT<Traits>::LX - w);
                }
            }
        

        template<class Traits>
        FLASHMEM void DiffBuffBaseT<Traits>::_copy_rotate_180(uint16_t* fb_dest, const uint16_t* fb_src, int x1, int x2, int y1, int y2, int w, int h, int src_stride)
            {
            uint16_t* p = fb_dest + x1 + (DiffBuffBaseT<Traits>::LX*y1);
            for (int j = h - 1; j >= 0; j--)
                {
                for (int i = w - 1; i >= 0; i--)
                    {
                    *(p++)= fb_src[i + (src_stride * j)];
                    }
                p += (DiffBuffBaseT<Traits>::LX - w);
                }
            }


        template<class Traits>
        FLASHMEM void DiffBuffBaseT<Traits>::_copy_rotate_270(uint16_t* fb_dest, const uint16_t* fb_src, int x1, int x2, int y1, int y2, int w, int h, int src_stride)
            {
            uint16_t* p = fb_dest + x1 + (DiffBuffBaseT<Traits>::LX*y1);
            for (int j = h - 1; j >= 0; j--)
                {   
                for (int i = 0; i < w ; i++)   
                    {
                    *(p++) = fb_src[j + (src_stride * i)];
                    }
                p += (DiffBuffBaseT<Traits>::LX - w);
                }
            }


        template<class Traits>
        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        T4DIFF_INLINE_COMPUTE_KERNEL int DiffBuffT<Traits>::_computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation, int gap, uint16_t compare_mask)
            {
            switch (fb_new_orientation)
                {
                case DiffBuffBaseT<Traits>::ROT0:
                    return this->template _computeDiff0<COPY_NEW_OVER_OLD, USE_MASK>(fb_old, fb_new, gap, compare_mask);
                case DiffBuffBaseT<Traits>::ROT90:
                    return this->template _computeDiff1<COPY_NEW_OVER_OLD, USE_MASK>(fb_old, fb_new, gap, compare_mask);
                case DiffBuffBaseT<Traits>::ROT180:
                    return this->template _computeDiff2<COPY_NEW_OVER_OLD, USE_MASK>(fb_old, fb_new, gap, compare_mask);
                case DiffBuffBaseT<Traits>::ROT270:
                    return this->template _computeDiff3<COPY_NEW_OVER_OLD, USE_MASK>(fb_old, fb_new, gap, compare_mask);
                }
            // hum...
            return 0;
            }


       
        
#define COMPUTE_DIFF_LOOP_SUB            {                                                       \
                                         if (COPY_NEW_OVER_OLD) { fb_old[n] = fb_new[ind]; }     \
                                         if (cgap >= gap)                                        \
                                             {                                                   \
                                             if (!this->_write_chunk(n - pos - cgap, cgap)) return n + 1;\
                                             pos = n;                                            \
                                             }                                                   \
                                         cgap = 0;                                               \
                                         }

#define COMPUTE_DIFF_LOOP_SUB_VAL(NEWP)  {                                                       \
                                         if (COPY_NEW_OVER_OLD) { fb_old[n] = (NEWP); }          \
                                         if (cgap >= gap)                                        \
                                             {                                                   \
                                             if (!this->_write_chunk(n - pos - cgap, cgap)) return n + 1;\
                                             pos = n;                                            \
                                             }                                                   \
                                         cgap = 0;                                               \
                                         }


#define COMPUTE_DIFF_LOOP_MASK(INDEX)    {                                                       \
                                         const int ind = (INDEX);                                \
                                         if (((fb_old[n]) ^ (fb_new[ind])) & compare_mask)       \
                                             COMPUTE_DIFF_LOOP_SUB                               \
                                         else { cgap++; }                                        \
                                         n++;                                                    \
                                         }

#define COMPUTE_DIFF_LOOP_NOMASK(INDEX)  {                                                       \
                                         const int ind = (INDEX);                                \
                                         if (fb_old[n] != fb_new[ind])                           \
                                             COMPUTE_DIFF_LOOP_SUB                               \
                                         else { cgap++; }                                        \
                                         n++;                                                    \
                                         }

#define COMPUTE_DIFF_LOOP_NOMASK_VAL(OLDP, NEWP) {                                               \
                                         if ((OLDP) != (NEWP))                                   \
                                             COMPUTE_DIFF_LOOP_SUB_VAL(NEWP)                     \
                                         else { cgap++; }                                        \
                                         n++;                                                    \
                                         }

#define COMPUTE_DIFF_LOOP_MASK_VAL(OLDP, NEWP) {                                                 \
                                         if (((OLDP) ^ (NEWP)) & compare_mask)                   \
                                             COMPUTE_DIFF_LOOP_SUB_VAL(NEWP)                     \
                                         else { cgap++; }                                        \
                                         n++;                                                    \
                                         }

#define COMPUTE_DIFF_LOOP_MASK_PTR(PTR, STEP) {                                                  \
                                         const uint16_t newp = *(PTR);                           \
                                         (PTR) += (STEP);                                        \
                                         if (((fb_old[n]) ^ newp) & compare_mask)                \
                                             COMPUTE_DIFF_LOOP_SUB_VAL(newp)                     \
                                         else { cgap++; }                                        \
                                         n++;                                                    \
                                         }

#define COMPUTE_DIFF_LOOP_NOMASK_PTR(PTR, STEP) {                                                \
                                         const uint16_t newp = *(PTR);                           \
                                         (PTR) += (STEP);                                        \
                                         if (fb_old[n] != newp)                                  \
                                             COMPUTE_DIFF_LOOP_SUB_VAL(newp)                     \
                                         else { cgap++; }                                        \
                                         n++;                                                    \
                                         }


#define COMPUTE_DIFF_LOOP(INDEX)    {                                       \
                                    if (USE_MASK)                           \
                                        {                                   \
                                        COMPUTE_DIFF_LOOP_MASK(INDEX)       \
                                        COMPUTE_DIFF_LOOP_MASK(INDEX)       \
                                        }                                   \
                                    else                                    \
                                        {                                   \
                                        COMPUTE_DIFF_LOOP_NOMASK(INDEX)     \
                                        COMPUTE_DIFF_LOOP_NOMASK(INDEX)     \
                                        }                                   \
                                    }

#define COMPUTE_DIFF_LOOP_PTR(PTR, STEP) {                                  \
                                    if (USE_MASK)                           \
                                        {                                   \
                                        COMPUTE_DIFF_LOOP_MASK_PTR(PTR, STEP)\
                                        COMPUTE_DIFF_LOOP_MASK_PTR(PTR, STEP)\
                                        }                                   \
                                    else                                    \
                                        {                                   \
                                        COMPUTE_DIFF_LOOP_NOMASK_PTR(PTR, STEP)\
                                        COMPUTE_DIFF_LOOP_NOMASK_PTR(PTR, STEP)\
                                        }                                   \
                                    }


#define COMPUTE_DIFF_END    { const int cpos = DiffBuffBaseT<Traits>::LX * DiffBuffBaseT<Traits>::LY;                   \
                              if (cpos - pos - cgap != 0)                 \
                                  {                                       \
                                  if (!this->_write_chunk(cpos - pos - cgap, cgap)) return cpos; \
                                  }                                       \
                              return cpos;                                \
                            }

                            


        template<class Traits>
        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        T4DIFF_INLINE_COMPUTE_KERNEL int DiffBuffT<Traits>::_computeDiff0(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask)
            {
            int cgap = 0;   // current gap size;
            int pos = 0;    // number of pixel written in diffbuf
            int n = 0;      // current offset  
            if (((((uintptr_t)fb_old) | ((uintptr_t)fb_new)) & 3) == 0)
                {
                const uint32_t* pold32 = (const uint32_t*)fb_old;
                const uint32_t* pnew32 = (const uint32_t*)fb_new;
                const uint32_t mask32 = ((uint32_t)compare_mask) | (((uint32_t)compare_mask) << 16);
                const int nb32 = (DiffBuffBaseT<Traits>::LX * DiffBuffBaseT<Traits>::LY) / 2;
                int m = 0;
                while (m < nb32)
                    {
                    const uint32_t old2 = pold32[m];
                    const uint32_t new2 = pnew32[m++];
                    if (USE_MASK ? (((old2 ^ new2) & mask32) == 0) : (old2 == new2))
                        {
                        cgap += 2;
                        n += 2;
                        }
                    else
                        {
                        const uint16_t oldp0 = (uint16_t)old2;
                        const uint16_t newp0 = (uint16_t)new2;
                        if (USE_MASK)
                            {
                            COMPUTE_DIFF_LOOP_MASK_VAL(oldp0, newp0)
                            }
                        else
                            {
                            COMPUTE_DIFF_LOOP_NOMASK_VAL(oldp0, newp0)
                            }
                        const uint16_t oldp1 = (uint16_t)(old2 >> 16);
                        const uint16_t newp1 = (uint16_t)(new2 >> 16);
                        if (USE_MASK)
                            {
                            COMPUTE_DIFF_LOOP_MASK_VAL(oldp1, newp1)
                            }
                        else
                            {
                            COMPUTE_DIFF_LOOP_NOMASK_VAL(oldp1, newp1)
                            }
                        }
                    }
                COMPUTE_DIFF_END
                }
            int m = 0; 
            while(m < DiffBuffBaseT<Traits>::LX*DiffBuffBaseT<Traits>::LY)
                {
                COMPUTE_DIFF_LOOP((m++)) 
                }
            COMPUTE_DIFF_END
            }

        template<class Traits>
        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        T4DIFF_INLINE_COMPUTE_KERNEL int DiffBuffT<Traits>::_computeDiff1(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask)
            {
            int cgap = 0;   // current gap size;
            int pos = 0;    // number of pixel written in diffbuf
            int n = 0;      // current offset  
            if (COPY_NEW_OVER_OLD)
                {
                for (int i = 0; i < DiffBuffBaseT<Traits>::LY; i++)
                    {
                    int j = DiffBuffBaseT<Traits>::LX;
                    const uint16_t* pnew = fb_new + i + (DiffBuffBaseT<Traits>::LY * (DiffBuffBaseT<Traits>::LX - 1));
                    while (j > 0)
                        {
                        COMPUTE_DIFF_LOOP_PTR(pnew, -DiffBuffBaseT<Traits>::LY)
                        j -= 2;
                        }
                    }
                }
            else
                {
                for (int i = 0; i < DiffBuffBaseT<Traits>::LY; i++)
                    {
                    int j = DiffBuffBaseT<Traits>::LX - 1;
                    while (j >= 0)
                        {
                        COMPUTE_DIFF_LOOP((i + DiffBuffBaseT<Traits>::LY * (j--)))
                        }
                    }
                }
            COMPUTE_DIFF_END
            }


        template<class Traits>
        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        T4DIFF_INLINE_COMPUTE_KERNEL int DiffBuffT<Traits>::_computeDiff2(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask)
            {
            int cgap = 0;   // current gap size;
            int pos = 0;    // number of pixel written in diffbuf
            int n = 0;      // current offset  
            if (((((uintptr_t)fb_old) | ((uintptr_t)fb_new)) & 3) == 0)
                {
                const uint32_t* pold32 = (const uint32_t*)fb_old;
                const uint32_t* pnew32 = (const uint32_t*)fb_new;
                const uint32_t mask32 = ((uint32_t)compare_mask) | (((uint32_t)compare_mask) << 16);
                const int nb32 = (DiffBuffBaseT<Traits>::LX * DiffBuffBaseT<Traits>::LY) / 2;
                int m = 0;
                while (m < nb32)
                    {
                    const uint32_t old2 = pold32[m];
                    const uint32_t new2raw = pnew32[nb32 - 1 - m];
                    const uint32_t new2 = (new2raw << 16) | (new2raw >> 16);
                    m++;
                    if (USE_MASK ? (((old2 ^ new2) & mask32) == 0) : (old2 == new2))
                        {
                        cgap += 2;
                        n += 2;
                        }
                    else
                        {
                        const uint16_t oldp0 = (uint16_t)old2;
                        const uint16_t newp0 = (uint16_t)new2;
                        if (USE_MASK)
                            {
                            COMPUTE_DIFF_LOOP_MASK_VAL(oldp0, newp0)
                            }
                        else
                            {
                            COMPUTE_DIFF_LOOP_NOMASK_VAL(oldp0, newp0)
                            }
                        const uint16_t oldp1 = (uint16_t)(old2 >> 16);
                        const uint16_t newp1 = (uint16_t)(new2 >> 16);
                        if (USE_MASK)
                            {
                            COMPUTE_DIFF_LOOP_MASK_VAL(oldp1, newp1)
                            }
                        else
                            {
                            COMPUTE_DIFF_LOOP_NOMASK_VAL(oldp1, newp1)
                            }
                        }
                    }
                COMPUTE_DIFF_END
                }
            for (int j = DiffBuffBaseT<Traits>::LY - 1; j >= 0; j--)
                {
                const int oo = DiffBuffBaseT<Traits>::LX * j;
                int i = DiffBuffBaseT<Traits>::LX - 1;
                while (i >= 0)
                    {
                    COMPUTE_DIFF_LOOP(((i--) + oo))
                    }
                }
            COMPUTE_DIFF_END
            }


        template<class Traits>
        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        T4DIFF_INLINE_COMPUTE_KERNEL int DiffBuffT<Traits>::_computeDiff3(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask)
            {
            int cgap = 0;   // current gap size;
            int pos = 0;    // number of pixel written in diffbuf
            int n = 0;      // current offset  
            if (COPY_NEW_OVER_OLD)
                {
                for (int i = DiffBuffBaseT<Traits>::LY - 1; i >= 0; i--)
                    {
                    int j = DiffBuffBaseT<Traits>::LX;
                    const uint16_t* pnew = fb_new + i;
                    while (j > 0)
                        {
                        COMPUTE_DIFF_LOOP_PTR(pnew, DiffBuffBaseT<Traits>::LY)
                        j -= 2;
                        }
                    }
                }
            else
                {
                for (int i = DiffBuffBaseT<Traits>::LY - 1; i >= 0; i--)
                    {
                    int j = 0;
                    while (j < DiffBuffBaseT<Traits>::LX)
                        {
                        COMPUTE_DIFF_LOOP((i + DiffBuffBaseT<Traits>::LY * (j++)))
                        }
                    }
                }
            COMPUTE_DIFF_END
            }


#undef COMPUTE_DIFF_LOOP_SUB
#undef COMPUTE_DIFF_LOOP_SUB_VAL
#undef COMPUTE_DIFF_LOOP_MASK
#undef COMPUTE_DIFF_LOOP_NOMASK
#undef COMPUTE_DIFF_LOOP_NOMASK_VAL
#undef COMPUTE_DIFF_LOOP_MASK_VAL
#undef COMPUTE_DIFF_LOOP_MASK_PTR
#undef COMPUTE_DIFF_LOOP_NOMASK_PTR
#undef COMPUTE_DIFF_LOOP
#undef COMPUTE_DIFF_LOOP_PTR
#undef COMPUTE_DIFF_END


        template<class Traits>
        FLASHMEM void DiffBuffT<Traits>::computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation, int gap, bool copy_new_over_old, uint16_t compare_mask)
            {
            elapsedMicros em; // for stats. 
            if (gap < 1) gap = 1;
            if ((fb_new_orientation < 0) || (fb_new_orientation > 3)) fb_new_orientation = 0;
            this->_posw = 0; // reset buffer
            if ((this->_sizebuf <= 0) || (fb_old == nullptr) || (fb_new == nullptr))
                {
                this->_write_encoded(TAG_END);
                this->_posw = 0;
//                initRead();
                return;
                }
            const bool use_mask = ((compare_mask != 0) && (compare_mask != 0xffff));
            int copy_from = DiffBuffBaseT<Traits>::LX * DiffBuffBaseT<Traits>::LY;
            if (use_mask)
                {
                if (copy_new_over_old) 
                    copy_from = this->template _computeDiff<true, true>(fb_old, fb_new, fb_new_orientation, gap, compare_mask);
                else
                    copy_from = this->template _computeDiff<false, true>(fb_old, fb_new, fb_new_orientation, gap, compare_mask);
                }
            else
                {
                if (copy_new_over_old)
                    copy_from = this->template _computeDiff<true, false>(fb_old, fb_new, fb_new_orientation, gap, compare_mask);
                else
                    copy_from = this->template _computeDiff<false, false>(fb_old, fb_new, fb_new_orientation, gap, compare_mask);
                }

            this->_write_encoded(TAG_END);
            if ((unsigned int)this->size() >= (unsigned int)this->_sizebuf)
                { // diff is full so copy from new to old may not have been completed...
                if (copy_new_over_old)
                    {
                    if (use_mask) DiffBuffBaseT<Traits>::copyfb(fb_old, fb_new, fb_new_orientation); // copy again.
                    else copyfbFromOffset<Traits>(fb_old, fb_new, fb_new_orientation, copy_from);
                    }
                }
//            initRead();
            // done. record stats
            this->_stats_size.push(this->size());
            if ((unsigned int)this->size() >= (unsigned int)this->_sizebuf) this->_stat_overflow++;
            this->_stats_time.push(em);
            }


        template<class Traits>
        FLASHMEM int DiffBuffT<Traits>::readDiff(int& x, int& y, int& len, int scanline)
            {
            if (!this->_r_cont)
                { // we must load a new instruction. 
                int nb_write, nb_skip;
                while(1)
                    {
                    nb_write = this->_read_encoded(this->_posr);         // number of pixel to write
                    if (nb_write == TAG_END) return -1; // done !
                    if (nb_write == TAG_WRITE_ALL)
                        { // must write everything
                        nb_write = DiffBuffBaseT<Traits>::LX * DiffBuffBaseT<Traits>::LY - this->_off;
                        nb_skip = 0;
                        if (nb_write <= 0) return -1;
                        }
                    else
                        {
                        nb_skip = this->_read_encoded(this->_posr);      // number of pixels to skip
                        }
                    if (nb_write > 0) break;
                    this->_off += nb_skip;
                    }
                this->_r_y = this->_off / DiffBuffBaseT<Traits>::LX;
                this->_r_x = this->_off - (DiffBuffBaseT<Traits>::LX * this->_r_y);
                this->_off += nb_skip + nb_write;
                this->_r_len = nb_write;
                this->_r_cont = true;
                }            
            // we have a valid instruction in this->_r_x, this->_r_y, this->_r_len and this->_r_cont=true            
            x = this->_r_x;
            y = this->_r_y;
            const int l = this->_r_y + DiffBuffBaseT<Traits>::MIN_SCANLINE_SPACE;
            if ((scanline < DiffBuffBaseT<Traits>::LY) && (scanline < l))
                { // we must wait a bit.
                len = 0;
                return ((l < DiffBuffBaseT<Traits>::LY) ? l : DiffBuffBaseT<Traits>::LY);
                }
            if (this->_r_x > 0)
                { // not at the beginning of a line. 
                if (this->_r_x + this->_r_len <= DiffBuffBaseT<Traits>::LX)
                    { // everything fits on the line
                    len = this->_r_len;
                    this->_r_cont = false;
                    return 0;
                    }
                len = DiffBuffBaseT<Traits>::LX - this->_r_x;
                this->_r_len -= len; 
                this->_r_x = 0;
                this->_r_y++;
                return 0;
                }
            // at the beginning of a line 
            int maxl = scanline - this->_r_y; // max number of lines available now
            if (maxl > DiffBuffBaseT<Traits>::MAX_WRITE_LINE) maxl = DiffBuffBaseT<Traits>::MAX_WRITE_LINE; // clamp at max value. 
            const int nbw = maxl * DiffBuffBaseT<Traits>::LX; // max number of pixels that we can write 
            if (this->_r_len <= nbw)
                { // ok, we can write everything now
                len = this->_r_len;
                this->_r_cont = false;
                return 0;
                }
            // cannot write everything yet. 
            len = nbw;
            this->_r_len -= nbw;
            this->_r_x = 0;
            this->_r_y += maxl;
            return 0;
            }


        template<class Traits>
        FLASHMEM int DiffBuffT<Traits>::scanlineStartInit()
            {                       
            this->initRead();
            while(1)
                {
                int nb_skip;
                int nb_write = this->_read_encoded(this->_posr); // number of pixel to write
                if (nb_write == TAG_END) return -1; // done !
                if (nb_write == TAG_WRITE_ALL)
                    { // must write everything
                    nb_write = DiffBuffBaseT<Traits>::LX * DiffBuffBaseT<Traits>::LY - this->_off;
                    nb_skip = 0;
                    if (nb_write <= 0) return -1;
                    }
                else
                    {
                    nb_skip = this->_read_encoded(this->_posr);      // number of pixels to skip
                    }
                if (nb_write > 0) break;
                this->_off += nb_skip;
                }
            const int yy = this->_off / DiffBuffBaseT<Traits>::LX;
            this->initRead();
            const int l = yy + DiffBuffBaseT<Traits>::MIN_SCANLINE_SPACE;
            return ((l < DiffBuffBaseT<Traits>::LY) ? l : DiffBuffBaseT<Traits>::LY);
            }







        template<class Traits>
        FLASHMEM void DiffBuffT<Traits>::computeDiff(uint16_t* fb_old, DiffBuffBaseT<Traits>* diff_old, const uint16_t* sub_fb_new, int xmin, int xmax, int ymin, int ymax, int stride,
                int fb_new_orientation, int gap, bool copy_new_over_old, uint16_t compare_mask)
            {
            elapsedMicros em; // for stats. 
            if (gap < 1) gap = 1;
            if ((fb_new_orientation < 0) || (fb_new_orientation > 3)) fb_new_orientation = 0;
            this->_posw = 0; // reset buffer
            if ((this->_sizebuf <= 0) || (fb_old == nullptr) || (sub_fb_new == nullptr))
                {
                this->_write_encoded(TAG_END);
                this->_posw = 0;
//                initRead();
                return;
                }

            int x1, x2, y1, y2;
            DiffBuffBaseT<Traits>::rotationBox(fb_new_orientation, xmin, xmax, ymin, ymax, x1, x2, y1, y2);

            const bool use_mask = ((compare_mask != 0) && (compare_mask != 0xffff));
            const int copy_from = this->_computeDiff(fb_old, diff_old, sub_fb_new, x1, x2, y1, y2, stride, fb_new_orientation, gap, copy_new_over_old, compare_mask);

            this->_write_encoded(TAG_END);
            if ((unsigned int)this->size() >= (unsigned int)this->_sizebuf)
                { // diff is full so copy from new to old may not have been completed...
                if (copy_new_over_old)
                    {
                    if (use_mask) DiffBuffBaseT<Traits>::copyfb(fb_old, sub_fb_new, xmin, xmax, ymin, ymax, stride, fb_new_orientation); // copy again.
                    else copySubFbFromOffset<Traits>(fb_old, sub_fb_new, x1, x2, y1, y2, stride, fb_new_orientation, copy_from);
                    }
                }
            // done. record stats
//            initRead();
            this->_stats_size.push(this->size());
            if ((unsigned int)this->size() >= (unsigned int)this->_sizebuf) this->_stat_overflow++;
            this->_stats_time.push(em);
            }


        template<class Traits>
        FLASHMEM int DiffBuffT<Traits>::_computeDiff(uint16_t* fb_old, DiffBuffBaseT<Traits>* diff_old, const uint16_t* sub_fb_new, int xmin, int xmax, int ymin, int ymax, int stride,
            int fb_new_orientation, int gap, bool copy_new_over_old, uint16_t compare_mask)
            {
            DiffBuffDummyT<Traits> dd; 
            if (diff_old)
                {
                diff_old->initRaw();
                }
            else
                {
                dd.setRawEmpty();
                diff_old = &dd; // set a dummy diff if none provided.                    
                }
 
            if (compare_mask == 0) compare_mask = 0xFFFF;
            int nb_write = 0, nb_skip = 0; // number of pixel to write / skip in the old diff
            diff_old->readRaw(nb_write, nb_skip);

            int prv = 0;   // last position written in the diff log
            int cgap = 0; // current gap

                {
                int cur = 0;   // current position
                int targetpos = DiffBuffBaseT<Traits>::LX * ymin; // target to reach
                while (cur < targetpos)
                    {
                    if (nb_write > 0)
                        {
                        if (cgap >= gap)
                            {
                            if (!this->_write_chunk(cur - prv - cgap, cgap)) return xmin + (DiffBuffBaseT<Traits>::LX * ymin);
                            prv = cur;
                            }
                        cgap = 0;
                        const int r = targetpos - cur;
                        if (nb_write < r)
                            {
                            cur += nb_write;
                            nb_write = 0;
                            }
                        else
                            {
                            nb_write -= r;
                            cur = targetpos;
                            }
                        }
                    else if (nb_skip > 0)
                        {
                        const int r = targetpos - cur;
                        if (nb_skip < r)
                            {
                            cur += nb_skip;
                            cgap += nb_skip;
                            nb_skip = 0;
                            }
                        else
                            {
                            nb_skip -= r;
                            cgap += r;
                            cur = targetpos;
                            }
                        }
                    else diff_old->readRaw(nb_write, nb_skip);
                    }
                }

            // We are at position (0, ymin)
            for (int yc = ymin; yc <= ymax; yc++)
                { 
                int xc = 0;
                while (xc < xmin)
                    {
                    if (nb_write > 0)
                        {
                        if (cgap >= gap)
                            {
                            const int cur = xc + (DiffBuffBaseT<Traits>::LX * yc);
                            if (!this->_write_chunk(cur - prv - cgap, cgap)) return xmin + (DiffBuffBaseT<Traits>::LX * yc);
                            prv = cur;
                            }
                        cgap = 0;
                        const int r = xmin - xc;
                        if (nb_write < r)
                            {
                            xc += nb_write;
                            nb_write = 0;
                            }
                        else
                            {
                            nb_write -= r;
                            xc = xmin;
                            }
                        }
                    else if (nb_skip > 0)
                        {
                        const int r = xmin - xc;
                        if (nb_skip < r)
                            {
                            xc += nb_skip;
                            cgap += nb_skip;
                            nb_skip = 0;
                            }
                        else
                            {
                            nb_skip -= r;
                            cgap += r;
                            xc = xmin;
                            }
                        }
                    else diff_old->readRaw(nb_write, nb_skip);                    
                    }

                int m = 0, mdelta = 0;
                switch (fb_new_orientation)
                    {
                case DiffBuffBaseT<Traits>::ROT0:
                    m = stride * (yc - ymin);
                    mdelta = 1;
                    break;
                case DiffBuffBaseT<Traits>::ROT90:
                    m = (yc - ymin) + stride*(xmax - xmin);
                    mdelta =  -stride;
                    break;
                case DiffBuffBaseT<Traits>::ROT180:
                    m = stride * (ymax - yc) + (xmax - xmin);
                    mdelta = -1;
                    break;
                case DiffBuffBaseT<Traits>::ROT270:
                    m = ymax - yc;
                    mdelta = stride;
                    break;
                    }

                if (nb_write + nb_skip == 0) diff_old->readRaw(nb_write, nb_skip); // reload if needed

                const int nend = xmax + (DiffBuffBaseT<Traits>::LX * yc);
                for (int n = xmin + (DiffBuffBaseT<Traits>::LX * yc); n <= nend; n++, m += mdelta)
                    {
                    if (((fb_old[n]) ^ (sub_fb_new[m])) & compare_mask)
                        {
                        if (copy_new_over_old) 
                            { 
                            fb_old[n] = sub_fb_new[m]; 
                            }
                        if (cgap >= gap)
                            {
                            if (!this->_write_chunk(n - prv - cgap, cgap)) return n + 1;
                            prv = n;
                            }
                        cgap = 0;
                        if (nb_write > 0) nb_write--;
                        else
                            {
                            nb_skip--;
                            if (nb_skip == 0) diff_old->readRaw(nb_write, nb_skip); // reload if needed
                            }
                        }
                    else if (nb_write > 0)
                        { // same pixel, nb_write > 0
                        if (cgap >= gap)
                            {
                            if (!this->_write_chunk(n - prv - cgap, cgap)) return n + 1;
                            prv = n;
                            }
                        cgap = 0;
                        nb_write--;
                        }
                    else
                        { // same pixel, nb_write=0 and nb_skip>0
                        cgap++;
                        nb_skip--;
                        if (nb_skip == 0) diff_old->readRaw(nb_write, nb_skip); // reload if needed
                        }
                    }

                xc = xmax + 1;  
                while (xc < DiffBuffBaseT<Traits>::LX)
                    { 
                    if (nb_write > 0)
                        {
                        if (cgap >= gap)
                            {
                            const int cur = xc + (DiffBuffBaseT<Traits>::LX * yc);
                            if (!this->_write_chunk(cur - prv - cgap, cgap)) return xmin + (DiffBuffBaseT<Traits>::LX * (yc + 1));
                            prv = cur;
                            }
                        cgap = 0;
                        const int r = DiffBuffBaseT<Traits>::LX - xc;
                        if (nb_write < r)
                            {
                            xc += nb_write;
                            nb_write = 0;
                            }
                        else
                            {
                            nb_write -= r;
                            xc = DiffBuffBaseT<Traits>::LX;
                            }
                        }
                    else if (nb_skip > 0)
                        {
                        const int r = DiffBuffBaseT<Traits>::LX - xc;
                        if (nb_skip < r)
                            {
                            xc += nb_skip;
                            cgap += nb_skip;
                            nb_skip = 0;
                            }
                        else
                            {
                            nb_skip -= r;
                            cgap += r;
                            xc = DiffBuffBaseT<Traits>::LX;
                            }
                        }
                    else diff_old->readRaw(nb_write, nb_skip);
                    }
                }

                {
                int cur = DiffBuffBaseT<Traits>::LX * (ymax + 1);
                int targetpos = DiffBuffBaseT<Traits>::LX * DiffBuffBaseT<Traits>::LY; // target to reach
                while (cur < targetpos)
                    {
                    if (nb_write > 0)
                        {
                        if (cgap >= gap)
                            {
                            if (!this->_write_chunk(cur - prv - cgap, cgap)) return DiffBuffBaseT<Traits>::LX * DiffBuffBaseT<Traits>::LY;
                            prv = cur;
                            }
                        cgap = 0;
                        const int r = targetpos - cur;
                        if (nb_write < r)
                            {
                            cur += nb_write;
                            nb_write = 0;
                            }
                        else
                            {
                            nb_write -= r;
                            cur = targetpos;
                            }
                        }
                    else if (nb_skip > 0)
                        {
                        const int r = targetpos - cur;
                        if (nb_skip < r)
                            {
                            cur += nb_skip;
                            cgap += nb_skip;
                            nb_skip = 0;
                            }
                        else
                            {
                            nb_skip -= r;
                            cgap += r;
                            cur = targetpos;
                            }
                        }
                    else diff_old->readRaw(nb_write, nb_skip);
                    }
                }

            // complete the diff with the remaining part. 
            const int cpos = DiffBuffBaseT<Traits>::LX * DiffBuffBaseT<Traits>::LY;
            if (cpos - prv - cgap != 0)
                {                                       
                if (!this->_write_chunk(cpos - prv - cgap, cgap)) return cpos;
                }
            return cpos;

            }


        template<class Traits>
        FLASHMEM void DiffBuffT<Traits>::statsReset()
            {
            this->_stat_overflow = 0;
            this->_stats_size.reset();
            this->_stats_time.reset();
            }


        template<class Traits>
        FLASHMEM void DiffBuffT<Traits>::printStats(Stream* outputStream) const
            {
            if (outputStream)
                {
                outputStream->print("------------------- DiffBuff Stats -------------------\n");
                outputStream->print("- max. buffer size   : ");
                outputStream->print(this->_sizebuf + DiffBuffT<Traits>::PADDING);
                outputStream->print('\n');
                outputStream->print("- overflow ratio     : ");
                outputStream->print(100 * this->statsOverflowRatio(), 1);
                outputStream->print("%  (");
                outputStream->print(this->statsNbOverflow());
                outputStream->print(" out of ");
                outputStream->print(this->statsNbComputed());
                outputStream->print(" computed)\n");
                outputStream->print("- buffer size used   : "); this->_stats_size.print("", "\n", outputStream);
                outputStream->print("- computation time   : "); this->_stats_time.print("us", "\n\n", outputStream);
                }
            }




        template<class Traits>
        FLASHMEM int DiffBuffDummyT<Traits>::readDiff(int& x, int& y, int& len, int scanline)
            {
            if (_current_line >= _end) return -1; // we are done. 
            if (scanline >= _end)
                { // scanline after end of drawing, go as fast as possible. 
                x = 0;
                y = _current_line;
                if (_current_line + DiffBuffBaseT<Traits>::MAX_WRITE_LINE <= _end)
                    {
                    len = DiffBuffBaseT<Traits>::MAX_WRITE_LINE * DiffBuffBaseT<Traits>::LX;
                    _current_line += DiffBuffBaseT<Traits>::MAX_WRITE_LINE;
                    }
                else
                    {
                    len = (_end - _current_line) * DiffBuffBaseT<Traits>::LX;
                    _current_line = _end;
                    }
                return 0;
                }
            int maxl = scanline - _current_line; // number of line available for drawing. 
            x = 0;
            y = _current_line;
            if (maxl < DiffBuffBaseT<Traits>::MIN_SCANLINE_SPACE)
                { // we must wait a bit. 
                const int l = _current_line + DiffBuffBaseT<Traits>::MIN_SCANLINE_SPACE;
                return ((l < _end) ? l : _end);
                }
            if (maxl > DiffBuffBaseT<Traits>::MAX_WRITE_LINE) maxl = DiffBuffBaseT<Traits>::MAX_WRITE_LINE; // not too much lines at once
            len = maxl * DiffBuffBaseT<Traits>::LX;
            _current_line += maxl;
            return 0;
            }


        template<class Traits>
        FLASHMEM int DiffBuffDummyT<Traits>::scanlineStartInit()
            {
            this->initRead();
            if (_current_line >= _end) return -1; // we are done. 
            //if (scanline >= _end)
            //    { // scanline after end of drawing, go as fast as possible. 
            //    return scanline; // we can run now
            //    }
            const int l = _current_line + DiffBuffBaseT<Traits>::MIN_SCANLINE_SPACE;            
            return ((l < _end) ? l : _end);
            }




}

#endif

#endif

/** end of file */

