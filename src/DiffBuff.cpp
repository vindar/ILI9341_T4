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

#include "DiffBuff.h"



namespace ILI9341_T4
{



    /***************************************************************************************
    * Implementation of DiffBuff
    ****************************************************************************************/


    template<bool COPY_NEW_OVER_OLD>
    void DiffBuff::_computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int lx, int ly, int orientation, int nb_split, int gap)
        {
        elapsedMicros em; // for stats. 
        if (gap < 1) gap = 1;
        if (nb_split < 1) nb_split = 1;
        if (nb_split > MAX_NB_SUBFRAME) nb_split = MAX_NB_SUBFRAME;
        if ((_orientation < 0) || (orientation > 3)) orientation = 0;
        _orientation = orientation;
        _nbsubframe = 0;
        _stride = lx;   // save dimensions
        _height = ly;   //
        _posw = 0; // reset buffer
        if ((_sizebuf <= 0) || (fb_old == nullptr) || (fb_new == nullptr) || (lx <= TRY_EXPAND_LOOP * nb_split) || (ly <= TRY_EXPAND_LOOP * nb_split))
            {
            _write_encoded(TAG_END);
            _posw = 0;
            initRead();
            return;
            }
        switch (orientation)
            {
            case TOP_TO_BOTTOM:
                {
                int sy = _getsplit(ly, nb_split);
                for (int k = 0; k < (nb_split - 1); k++) _computeSubFrame<COPY_NEW_OVER_OLD>(fb_old, fb_new, 0, (sy * k), lx, sy, _stride, gap);
                _computeSubFrame<COPY_NEW_OVER_OLD>(fb_old, fb_new, 0, sy * (nb_split - 1), lx, ly - (sy * (nb_split - 1)), _stride, gap);
                break;
                }

            case LEFT_TO_RIGHT:
                {
                int sx = _getsplit(lx, nb_split);
                for (int k = 0; k < (nb_split - 1); k++) _computeSubFrame<COPY_NEW_OVER_OLD>(fb_old, fb_new, (sx * k), 0, sx, ly, _stride, gap);
                _computeSubFrame<COPY_NEW_OVER_OLD>(fb_old, fb_new, sx * (nb_split - 1), 0, lx - (sx * (nb_split - 1)), ly, _stride, gap);
                break;
                }

            case BOTTOM_TO_TOP:
                {
                int sy = _getsplit(ly, nb_split);
                for (int k = 0; k < (nb_split - 1); k++) _computeSubFrame<COPY_NEW_OVER_OLD>(fb_old, fb_new, 0, ly - (sy * (k + 1)), lx, sy, _stride, gap);
                _computeSubFrame<COPY_NEW_OVER_OLD>(fb_old, fb_new, 0, 0, lx, ly - (sy * (nb_split - 1)), _stride, gap);
                break;
                }

            case RIGHT_TO_LEFT:
                {
                int sx = _getsplit(lx, nb_split);
                for (int k = 0; k < (nb_split - 1); k++) _computeSubFrame<COPY_NEW_OVER_OLD>(fb_old, fb_new, lx - (sx * (k + 1)), 0, sx, ly, _stride, gap);
                _computeSubFrame<COPY_NEW_OVER_OLD>(fb_old, fb_new, 0, 0, lx - (sx * (nb_split - 1)), ly, _stride, gap);
                break;
                }
            }
        _write_encoded(TAG_END);
        // done. record stats
        _stat_nb++;
        const uint32_t s = size();
        if (((int)s) >= _sizebuf) _stat_overflow++;
        if (s < _stat_min) _stat_min = s;
        if (s > _stat_max) _stat_max = s;
        _stat_sum += s;
        _stat_sumsqr += s * s;

        const uint32_t t = em;
        if (t < _stat_mintime) _stat_mintime = t;
        if (t > _stat_maxtime) _stat_maxtime = t;
        _stat_sumtime += t;
        _stat_sumsqrtime += t * t;
        initRead();
        // return buffer size
        return;
        }



    template<bool COPY_NEW_OVER_OLD>
    void DiffBuff::_computeSubFrame(uint16_t* fb_old, const uint16_t* fb_new, const int x, const int y, const int lx, const int ly, const int stride, const int gap)
        {
        if (_nbsubframe >= MAX_NB_SUBFRAME) return; // should not happen, but safety precaution anyway. 
        _nbsubframe++;
        // sub frame format [TAG_HEADER, x, y, lx, ly, stride] [write0 skip0] [write1 skip1] ... [writeN, skipN] [TAG_WRITE_ALL / TAG_SKIP_ALL]
        _write_encoded(TAG_HEADER);
        _write_encoded(x);
        _write_encoded(y);
        _write_encoded(lx);
        _write_encoded(ly);
        if (lx % TRY_EXPAND_LOOP == 0)
            _computeSubFrame2<COPY_NEW_OVER_OLD, TRY_EXPAND_LOOP>(fb_old, fb_new, x, y, lx, ly, stride, gap);  // ok expand the loop
        else
            _computeSubFrame2<COPY_NEW_OVER_OLD, 1>(fb_old, fb_new, x, y, lx, ly, stride, gap); // do not expand lop
        }


    // innner loop is expanded EXPAND_LOOP times. 
#define ILI9341_T4_DIFFBUF_INNER_LOOP(INDEX)  { if (fb_old[v + INDEX] != fb_new[v + INDEX])                     \
                                                  {                                                             \
                                                  if (COPY_NEW_OVER_OLD) fb_old[v + INDEX] = fb_new[v + INDEX]; \
                                                  if (cgap >= gap)                                              \
                                                      {                                                         \
                                                      const int cpos = i + (lx * j) +  + INDEX;                 \
                                                      if (!_write_chunk(cpos - pos - cgap, cgap)) return;       \
                                                      pos = cpos;                                               \
                                                      }                                                         \
                                                  cgap = 0;                                                     \
                                                  }                                                             \
                                              else cgap++; }                                                  


    template<bool COPY_NEW_OVER_OLD, int EXPAND_LOOP>
    void DiffBuff::_computeSubFrame2(uint16_t* fb_old, const uint16_t* fb_new, const int x, const int y, const int lx, const int ly, const int stride, const int gap)
        {
        int cgap = 0; // current gap size;
        int pos = 0; // number of pixel written in diffbuf
        // it seems two 'for' loops give the fastest way of accessing memory when the stride 
        // may be different from the width (faster than a while loop and increasing a pointer).
        for (int j = 0; j < ly; j++)
            {
            const int linestart = stride * (j + y) + x;
            for (int i = 0; i < lx; i += EXPAND_LOOP)
                {
                const int v = linestart + i;
                ILI9341_T4_DIFFBUF_INNER_LOOP(0);
                // all conditionals below are removed at compile time
                if (EXPAND_LOOP >= 2) ILI9341_T4_DIFFBUF_INNER_LOOP(1);
                if (EXPAND_LOOP >= 3) ILI9341_T4_DIFFBUF_INNER_LOOP(2);
                if (EXPAND_LOOP >= 4) ILI9341_T4_DIFFBUF_INNER_LOOP(3);
                if (EXPAND_LOOP >= 5) ILI9341_T4_DIFFBUF_INNER_LOOP(4);
                if (EXPAND_LOOP >= 6) ILI9341_T4_DIFFBUF_INNER_LOOP(5);
                if (EXPAND_LOOP >= 7) ILI9341_T4_DIFFBUF_INNER_LOOP(6);
                if (EXPAND_LOOP >= 8) ILI9341_T4_DIFFBUF_INNER_LOOP(7);
                }
            }
        // no more pixels. write the last batch
        const int cpos = lx * ly;
        if (cpos - pos - cgap != 0)
            { // something to write
            if (!_write_chunk(cpos - pos - cgap, cgap)) return;
            }
        //_write_encoded(TAG_SKIP_ALL);
        }

#undef ILI9341_T4_DIFFBUF_INNER_LOOP


    
    void DiffBuff::subFrameSyncTimes(double& start, double& end)
        {
        start = 0.0; // default value for incorrect
        end = 1.0;   // orientation or when _nbsubframe = 1
        if (_nbsubframe > 1)
            {
            switch (_orientation)
                {
                case TOP_TO_BOTTOM:
                    {
                    start = ((double)_frame_y) / _height;
                    end = ((double)(_frame_y + _frame_ly)) / _height;
                    break;
                    }
                case LEFT_TO_RIGHT:
                    {
                    start = ((double)_frame_x) / _stride;
                    end = ((double)(_frame_x + _frame_lx)) / _stride;
                    break;
                    }
                case BOTTOM_TO_TOP:
                    {
                    start = 1.0 - (((double)(_frame_y + _frame_ly)) / _height);
                    end = 1.0 - (((double)_frame_y) / _height);
                    break;
                    }
                case RIGHT_TO_LEFT:
                    {
                    start = 1.0 - (((double)(_frame_x + _frame_lx)) / _stride);
                    end = 1.0 - (((double)_frame_x) / _stride);
                    break;
                    }
                }
            }
        return;
        }


    
    int DiffBuff::readDiff(int& x, int& y, int& len)
        {
        if (!_r_cont)
            {
            int r = _readDiffRaw(_r_x, _r_y, _r_len);
            if (r != INSTRUCTION) return r;
            }
        // we have a valid instruction in _r_x, _r_y, _r_len
        if (_r_x + _r_len <= _frame_x + _frame_lx)
            { // everything fits on the line. 
            x = _r_x;
            y = _r_y;
            len = _r_len;
            _r_cont = false;
            return INSTRUCTION;
            }
        // strictly more than one line
        if ((_r_x == 0) && (_frame_lx == _stride))
            { // optimization: here we can go faster and write more than a single line.       
            x = 0;
            y = _r_y;
            if (_r_len <= (int)MAX_WRITE)
                { // write everything. 
                len = _r_len;
                _r_cont = false;
                }
            else
                {
                const int nbl = (int)MAX_WRITE / _stride;
                len = (nbl * _stride);
                _r_len -= len;
                _r_y += nbl;
                _r_cont = (_r_len > 0); // must be true but just in case.... 
                }
            return INSTRUCTION;
            }
        // write a single line
        x = _r_x;
        y = _r_y;
        len = _frame_lx + _frame_x - _r_x;
        _r_len -= len;
        _r_x = _frame_x;
        _r_y++;
        _r_cont = true;
        return INSTRUCTION;
        }


    int DiffBuff::_readDiffRaw(int& x, int& y, int& len)
        {
        while (1)
            {
            uint32_t w = _read_encoded();
            switch (w)
                {
                case TAG_END: return DIFF_END; // reached the end of the diff buffer
                case TAG_HEADER:
                    { // frame the subframe info
                    _frame_x = _read_encoded();
                    _frame_y = _read_encoded();
                    _frame_lx = _read_encoded();
                    _frame_ly = _read_encoded();
                    _read_off = 0;
                    return NEW_SUBFRAME;
                    }
                case TAG_WRITE_ALL:
                    {
                    int l = (_frame_lx * _frame_ly) - _read_off;  // number of pixels remaining
                    if (l == 0) break;
                    len = l;                                      // number of pixels to write
                    const int v = (_read_off / _frame_lx);        // y coord in the sub frame
                    const int u = _read_off - (v * _frame_lx);    // x coord in the sub frame
                    x = u + _frame_x;                             // x coord in the full framebuffer
                    y = v + _frame_y;                             // y coord in the full framebuffer                    
                    return INSTRUCTION;
                    }
                default:
                    {
                    uint32_t sk = _read_encoded(); // number of pixels to skip. 
                    if (w == 0)
                        {
                        _read_off += sk;
                        break;
                        }
                    len = w;                                    // number of pixels to write
                    const int v = (_read_off / _frame_lx);       // y coord in the sub frame
                    const int u = _read_off - (v * _frame_lx);   // x coord in the sub frame
                    x = u + _frame_x;                            // x coord in the full framebuffer
                    y = v + _frame_y;                            // y coord in the full framebuffer                   
                    _read_off += (w + sk);
                    return INSTRUCTION;
                    }
                }
            }
        }


    void DiffBuff::statsReset()
        {
        _stat_nb = 0;
        _stat_overflow = 0;
        _stat_min = -1;
        _stat_max = 0;
        _stat_sum = 0;
        _stat_sumsqr = 0;
        _stat_mintime = -1;
        _stat_maxtime = 0;
        _stat_sumtime = 0;
        _stat_sumsqrtime = 0;
        }


    void DiffBuff::printStats(Stream* outputStream) const
        {
        outputStream->printf("--- DiffBuff Stats ---\n");
        outputStream->printf("- nb of diff computed: %u overflowed %u (%u%%)\n", statsNb(), statsOverflow(), statsOverflowRatio());
        outputStream->printf("- buffer size        : %u\n", _sizebuf + PADDING);
        outputStream->printf("- diff buffer use    : avg=%u [min=%u , max=%u], std=%u\n", statsAvgSize(), statsMinSize(), statsMaxSize(), statsStdSize());
        outputStream->printf("- computation time   : avg=%uus [min=%uus , max=%uus], std=%uus\n\n", statsAvgTime(), statsMinTime(), statsMaxTime(), statsStdTime());
        }






    /***************************************************************************************
    * Implementation of DiffBuffDummy
    ****************************************************************************************/


    void DiffBuffDummy::computeDiff(uint16_t* fb_old, const uint16_t* fb_new, int lx, int ly, int orientation, int nb_split, int gap, bool copy_new_over_old)
        {
        if (gap < 1) gap = 1;
        if (nb_split < 1) nb_split = 1;
        if (nb_split > MAX_NB_SUBFRAME) nb_split = MAX_NB_SUBFRAME;
        if ((_orientation < 0) || (orientation > 3)) orientation = 0;
        _orientation = orientation;
        _nbsubframe = nb_split;
        _stride = lx;   // save dimensions
        _height = ly;   //
        if ((lx < 1) | (ly < 1))
            {
            _nbsubframe = 0;
            return;
            }
        if ((fb_old) && (fb_new) && (copy_new_over_old))
            {
            for (int i = 0; i < (lx * ly); i++) fb_old[i] = fb_new[i];
            }
        initRead();
        return;
        }



    int DiffBuffDummy::readDiff(int& x, int& y, int& len)
        {
        if (_curframe >= _nbsubframe) return DIFF_END;
        if (_curline < 0)
            { // compute sub-frame number _curframe coordinates. 
            const int lx = _stride / _nbsubframe;
            const int rx = _stride - (lx * (_nbsubframe - 1));
            const int fx = (_curframe * lx);
            const int ly = _height / _nbsubframe;
            const int ry = _height - (ly * (_nbsubframe - 1));
            const int fy = (_curframe * ly);
            switch (_orientation)
                {
                case TOP_TO_BOTTOM:
                    {
                    _frame_x = 0;
                    _frame_y = fy;
                    _frame_lx = _stride;
                    _frame_ly = ((_curframe == _nbsubframe - 1) ? ry : ly);
                    break;
                    }
                case LEFT_TO_RIGHT:
                    {
                    _frame_x = fx;
                    _frame_y = 0;
                    _frame_lx = ((_curframe == _nbsubframe - 1) ? rx : lx);
                    _frame_ly = _height;
                    break;
                    }
                case BOTTOM_TO_TOP:
                    {
                    _frame_x = 0;
                    _frame_y = _height - fy - ry;
                    _frame_lx = _stride;
                    _frame_ly = ((_curframe == 0) ? ry : ly);
                    break;
                    }
                case RIGHT_TO_LEFT:
                    {
                    _frame_x = _stride - fx - rx;
                    _frame_y = 0;
                    _frame_lx = ((_curframe == 0) ? rx : lx);
                    _frame_ly = _height;
                    break;
                    }
                }                
            _curline = 0;
            return NEW_SUBFRAME;
            }
        // we have an instruction. 
        if (_stride == _frame_lx)
            { // may write multiple line at once. 
            const int maxl = MAX_WRITE / _frame_lx;
            const int nbl = (_frame_ly - _curline > maxl) ? maxl : (_frame_ly - _curline);
            x = _frame_x;
            y = _frame_y + _curline;
            len = _frame_lx * nbl;
            _curline += nbl;
            }
        else
            { // write only a single line
            x = _frame_x;
            y = _frame_y + _curline;
            len = _frame_lx;
            _curline++;
            }   
        if (_curline >= _frame_ly)
            { // end of subframe
            _curline = -1;
            _curframe++;
            }
        return INSTRUCTION;
        }


    void DiffBuffDummy::subFrameSyncTimes(double& start, double& end)
        {
        start = 0.0; // default value for incorrect
        end = 1.0;   // orientation or when _nbsubframe = 1
        if (_nbsubframe > 1)
            {
            switch (_orientation)
                {
                case TOP_TO_BOTTOM:
                    {
                    start = ((double)_frame_y) / _height;
                    end = ((double)(_frame_y + _frame_ly)) / _height;
                    break;
                    }
                case LEFT_TO_RIGHT:
                    {
                    start = ((double)_frame_x) / _stride;
                    end = ((double)(_frame_x + _frame_lx)) / _stride;
                    break;
                    }
                case BOTTOM_TO_TOP:
                    {
                    start = 1.0 - (((double)(_frame_y + _frame_ly)) / _height);
                    end = 1.0 - (((double)_frame_y) / _height);
                    break;
                    }
                case RIGHT_TO_LEFT:
                    {
                    start = 1.0 - (((double)(_frame_x + _frame_lx)) / _stride);
                    end = 1.0 - (((double)_frame_x) / _stride);
                    break;
                    }
                }   
            }
        return;
        }


}


/** end of file */
