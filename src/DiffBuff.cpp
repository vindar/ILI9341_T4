#include "DiffBuff.h"



namespace ILI9341_T4
{




        void DiffBuffBase::copyfb(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation)
            {
            switch (fb_new_orientation)
                {
                case PORTRAIT_240x320:
                    {
                    _copy_rotate_0(fb_old, fb_new);
                    return;
                    }
                case LANDSCAPE_320x240:
                    {
                    _copy_rotate_90(fb_old, fb_new);
                    return;
                    }
                case PORTRAIT_240x320_FLIPPED:
                    {
                    _copy_rotate_180(fb_old, fb_new);
                    return;
                    }
                case LANDSCAPE_320x240_FLIPPED:
                    {
                    _copy_rotate_270(fb_old, fb_new);
                    return;
                    }
                }
            // hum...
            return;
            }

    
        void DiffBuffBase::_copy_rotate_0(uint16_t* fb_dest, const uint16_t* fb_src)
            {
            memcpy(fb_dest, fb_src, sizeof(uint16_t) * LX * LY);
            }


        void DiffBuffBase::_copy_rotate_90(uint16_t* fb_dest, const uint16_t* fb_src)
            {
            uint16_t* p = fb_dest;
            for (int i = 0; i < LY; i++)
                {
                int j = LX - 1;
                while (j >= 0)
                    {
                    *(p++) = fb_src[i + LY * (j--)];
                    *(p++) = fb_src[i + LY * (j--)];
                    *(p++) = fb_src[i + LY * (j--)];
                    *(p++) = fb_src[i + LY * (j--)];
                    }
                }
            }


        void DiffBuffBase::_copy_rotate_180(uint16_t* fb_dest, const uint16_t* fb_src)
            {
            uint16_t* p = fb_dest;
            for (int j = LY - 1; j >= 0; j--)
                {
                int i = LX - 1;
                while (i >= 0)
                    {
                    const int oo = LX * j;
                    *(p++) = fb_src[(i--) + oo];
                    *(p++) = fb_src[(i--) + oo];
                    *(p++) = fb_src[(i--) + oo];
                    *(p++) = fb_src[(i--) + oo];
                    }
                }
            }


        void DiffBuffBase::_copy_rotate_270(uint16_t* fb_dest, const uint16_t* fb_src)
            {
            uint16_t* p = fb_dest;
            for (int i = LY - 1; i >= 0; i--)
                {
                int j = 0;
                while (j < LX)
                    {
                    *(p++) = fb_src[i + LY * (j++)];
                    *(p++) = fb_src[i + LY * (j++)];
                    *(p++) = fb_src[i + LY * (j++)];
                    *(p++) = fb_src[i + LY * (j++)];
                    }
                }
            }






        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        void DiffBuff::_computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation, int gap, uint16_t compare_mask)
            {
            switch (fb_new_orientation)
                {
                case 0:  
                    _computeDiff0<COPY_NEW_OVER_OLD, USE_MASK>(fb_old, fb_new, gap, compare_mask);
                    return;
                case 1:
                    _computeDiff1<COPY_NEW_OVER_OLD, USE_MASK>(fb_old, fb_new, gap, compare_mask);
                    return;
                case 2:
                    _computeDiff2<COPY_NEW_OVER_OLD, USE_MASK>(fb_old, fb_new, gap, compare_mask);
                    return;
                case 3:
                    _computeDiff3<COPY_NEW_OVER_OLD, USE_MASK>(fb_old, fb_new, gap, compare_mask);
                    return;
                }
            // hum...
            return;
            }


       
        
#define COMPUTE_DIFF_LOOP_SUB            {                                                       \
                                         if (COPY_NEW_OVER_OLD) { fb_old[n] = fb_new[ind]; }     \
                                         if (cgap >= gap)                                        \
                                             {                                                   \
                                             if (!_write_chunk(n - pos - cgap, cgap)) return;    \
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


#define COMPUTE_DIFF_END    { const int cpos = LX * LY;                   \
                              if (cpos - pos - cgap != 0)                 \
                                  {                                       \
                                  _write_chunk(cpos - pos - cgap, cgap);  \
                                  }                                       \
                            }

                            


        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        void DiffBuff::_computeDiff0(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask)
            {
            int cgap = 0;   // current gap size;
            int pos = 0;    // number of pixel written in diffbuf
            int n = 0;      // current offset  
            int m = 0; 
            while(m < LX*LY)
                {
                COMPUTE_DIFF_LOOP((m++)) 
                }
            COMPUTE_DIFF_END
            }

        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        void DiffBuff::_computeDiff1(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask)
            {
            int cgap = 0;   // current gap size;
            int pos = 0;    // number of pixel written in diffbuf
            int n = 0;      // current offset  
            for (int i = 0; i < LY; i++)
                {
                int j = LX - 1;
                while (j >= 0)
                    {
                    COMPUTE_DIFF_LOOP((i + LY * (j--)))
                    }
                }
            COMPUTE_DIFF_END
            }


        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        void DiffBuff::_computeDiff2(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask)
            {
            int cgap = 0;   // current gap size;
            int pos = 0;    // number of pixel written in diffbuf
            int n = 0;      // current offset  
            for (int j = LY - 1; j >= 0; j--)
                {
                int i = LX - 1;
                while (i >= 0)
                    {
                    const int oo = LX * j;
                    COMPUTE_DIFF_LOOP(((i--) + oo))
                    }
                }
            COMPUTE_DIFF_END
            }


        template<bool COPY_NEW_OVER_OLD, bool USE_MASK>
        void DiffBuff::_computeDiff3(uint16_t* fb_old, const uint16_t* fb_new, int gap, uint16_t compare_mask)
            {
            int cgap = 0;   // current gap size;
            int pos = 0;    // number of pixel written in diffbuf
            int n = 0;      // current offset  
            for (int i = LY - 1; i >= 0; i--)
                {
                int j = 0;
                while (j < LX)
                    {
                    COMPUTE_DIFF_LOOP((i + LY * (j++)))
                    }
                }
            COMPUTE_DIFF_END
            }


#undef COMPUTE_DIFF_LOOP_SUB
#undef COMPUTE_DIFF_LOOP_MASK
#undef COMPUTE_DIFF_LOOP_NOMASK
#undef COMPUTE_DIFF_LOOP
#undef COMPUTE_DIFF_END


        void DiffBuff::computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int fb_new_orientation, int gap, bool copy_new_over_old, uint16_t compare_mask)
            {
            elapsedMicros em; // for stats. 
            if (gap < 1) gap = 1;
            if ((fb_new_orientation < 0) || (fb_new_orientation > 3)) fb_new_orientation = 0;
            _posw = 0; // reset buffer
            if ((_sizebuf <= 0) || (fb_old == nullptr) || (fb_new == nullptr))
                {
                _write_encoded(TAG_END);
                _posw = 0;
                initRead();
                return;
                }
            if ((compare_mask != 0) && (compare_mask != 0xffff))
                {
                if (copy_new_over_old) 
                    _computeDiff<true, true>(fb_old, fb_new, fb_new_orientation, gap, compare_mask);
                else
                    _computeDiff<false, true>(fb_old, fb_new, fb_new_orientation, gap, compare_mask);
                }
            else
                {
                if (copy_new_over_old)
                    _computeDiff<true, false>(fb_old, fb_new, fb_new_orientation, gap, compare_mask);
                else
                    _computeDiff<false, false>(fb_old, fb_new, fb_new_orientation, gap, compare_mask);
                }

            _write_encoded(TAG_END);
            if (size() >= _sizebuf)
                { // diff is full so copy from new to old may not have been completed...
                if (copy_new_over_old) copyfb(fb_old, fb_new, fb_new_orientation); // copy again. 
                }
            initRead();
            // done. record stats
            _stats_size.push(size());
            if (size() >= _sizebuf) _stat_overflow++;
            _stats_time.push(em);
            }


        int DiffBuff::readDiff(int& x, int& y, int& len, int scanline)
            {
            if (!_r_cont)
                { // we must load a new instruction. 
                int nb_write, nb_skip;
                while(1)
                    {
                    nb_write = _read_encoded();         // number of pixel to write
                    if (nb_write == TAG_END) return -1; // done !
                    if (nb_write == TAG_WRITE_ALL)
                        { // must write everything
                        nb_write = LX * LY - _off;
                        nb_skip = 0;
                        if (nb_write <= 0) return -1;
                        }
                    else
                        {
                        nb_skip = _read_encoded();      // number of pixels to skip
                        }
                    if (nb_write > 0) break;
                    _off += nb_skip;
                    }
                _r_y = _off / LX;
                _r_x = _off - (LX * _r_y);
                _off += nb_skip + nb_write;
                _r_len = nb_write;
                _r_cont = true;
                }            
            // we have a valid instruction in _r_x, _r_y, _r_len and _r_cont=true            
            x = _r_x;
            y = _r_y;
            if ((scanline < LY) && (_r_y + MIN_SCANLINE_SPACE > scanline))
                { // we must wait a bit.
                len = 0;
                const int l = _r_y + MIN_SCANLINE_SPACE;
                return ((l < LY) ? l : LY);
                }
            if (_r_x > 0)
                { // not at the beginning of a line. 
                if (_r_x + _r_len <= LX)
                    { // everything fits on the line
                    len = _r_len;
                    _r_cont = false;
                    return 0;
                    }
                len = LX - _r_x;
                _r_len -= len; 
                _r_x = 0;
                _r_y++;
                return 0;
                }
            // at the beginning of a line 
            int maxl = scanline - _r_y; // max number of lines available now
            if (maxl > MAX_WRITE_LINE) maxl = MAX_WRITE_LINE; // clamp at max value. 
            const int nbw = maxl * LX; // max number of pixels that we can write 
            if (_r_len <= nbw)
                { // ok, we can write everything now
                len = _r_len;
                _r_cont = false;
                return 0;
                }
            // cannot write everything yet. 
            len = nbw;
            _r_len -= nbw;
            _r_x = 0;
            _r_y += maxl;
            return 0;
            }




        void DiffBuff::statsReset()
            {
            _stat_overflow = 0;
            _stats_size.reset();
            _stats_time.reset();
            }


        void DiffBuff::printStats(Stream* outputStream) const
            {
            outputStream->printf("------------------- DiffBuff Stats -------------------\n");
            outputStream->printf("- max. buffer size   : %u\n", _sizebuf + PADDING);
            outputStream->printf("- overflow ratio     : %.1f%%  (%u out of %u computed)\n", 100 * statsOverflowRatio(), statsNbOverflow(), statsNbComputed());
            outputStream->printf("- buffer size used   : "); _stats_size.print("", "\n", outputStream);
            outputStream->printf("- computation time   : "); _stats_time.print("us", "\n\n", outputStream);
            }







        int DiffBuffDummy::readDiff(int& x, int& y, int& len, int scanline)
            {
            if (_current_line >= LY) return -1; // we are done. 
            if (scanline >= LY)
                { // scanline started drawing the new frame, go as fast as possible. 
                x = 0;
                y = _current_line;
                if (_current_line + MAX_WRITE_LINE <= LY)
                    {
                    len = MAX_WRITE_LINE * LX;
                    _current_line += MAX_WRITE_LINE;
                    }
                else
                    {
                    len = (LY - _current_line) * LX;
                    _current_line = LY;
                    }
                return 0;
                }
            int maxl = scanline - _current_line; // number of line available for drawing. 
            if (maxl < MIN_SCANLINE_SPACE)
                { // we must wait a bit. 
                const int l = _current_line + MIN_SCANLINE_SPACE;
                return ((l < LY) ? l : LY);
                }
            x = 0;
            y = _current_line;
            if (maxl > MAX_WRITE_LINE) maxl = MAX_WRITE_LINE; // not too much lines at once
            len = maxl * LX;
            _current_line += maxl;
            return 0;
            }




}



/** end of file */

