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

#ifndef _II9341_T4_DIFFBUFF_H_
#define _II9341_T4_DIFFBUFF_H_

#ifdef __cplusplus

#include "DiffBuffTemplate.h"

namespace ILI9341_T4
{

    struct DiffBuffTraits
    {
        static constexpr int LX = 240;
        static constexpr int LY = 320;
        static constexpr int MAX_WRITE_LINE = 120;
        static constexpr int MIN_SCANLINE_SPACE = 8;
    };

    using DiffBuffBase = T4Diff::DiffBuffBaseT<DiffBuffTraits>;
    using DiffBuff = T4Diff::DiffBuffT<DiffBuffTraits>;

    template<int SIZEBUF>
    using DiffBuffStatic = T4Diff::DiffBuffStaticT<DiffBuffTraits, SIZEBUF>;

    using DiffBuffDummy = T4Diff::DiffBuffDummyT<DiffBuffTraits>;

}

extern template class T4Diff::DiffBuffBaseT<ILI9341_T4::DiffBuffTraits>;
extern template class T4Diff::DiffBuffT<ILI9341_T4::DiffBuffTraits>;
extern template class T4Diff::DiffBuffDummyT<ILI9341_T4::DiffBuffTraits>;

#endif

#endif

/** end of file */
