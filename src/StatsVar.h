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

#include <stdint.h>
#include <Arduino.h>

namespace ILI9341_T4
{


/**
 * Class that stores some statistics about a sequence of int32 values.
 * It keeps track of:
 * 
 * - the min value of the sequence
 * - the max value of the sequence
 * - the average value of the sequence. 
 * - the standard deviation around the average.  
 * 
 **/
 class StatsVar
    {
    public:
  
        /** ctor. */
        StatsVar()
            {
            reset(); 
            }


        /**
         * Reset all statistics to their default values. 
         **/
        void reset()
            {
            _count = 0; 
            _min = INT32_MAX;
            _max = INT32_MIN;
            _sum = 0;
            _sumsqr = 0;
            }


        /**
         * Add a new value to the sequence.
         **/
        void push(int32_t val)  __attribute__((always_inline))            
            {
            _count++;
            _sum += val;
            _sumsqr += (val * val);
            if (val < _min) _min = val; 
            if (val > _max) _max = val;
            }


        /**
         * Output the statistics into a stream. 
         **/
        void print(const char* unit, const char * endl, Stream* outputStream, bool with_precision = false) const
            {
            if (with_precision)
                outputStream->printf("avg=%.2f%s [min=%d%s , max=%d%s] std=%.2f%s%s", avg(), unit, min(), unit, max(), unit, std(), unit, endl);
            else
                outputStream->printf("avg=%.0f%s [min=%d%s , max=%d%s] std=%.0f%s%s", avg(), unit, min(), unit, max(), unit, std(), unit, endl);
            }


        /**
         * Return the current number of record pushed (since the last reset);
         **/
        uint32_t count() const { return _count; }


        /**
         * Return the minimum value recorded (since the last reset). 
         **/
        int32_t min() const { return _min; }


        /**
         * Return the maximum value recorded (since the last reset).
         **/
        int32_t max() const { return _max; }


        /**
         * Return the average value of all records (since the last reset).
         **/
        double avg() const { return ((_count == 0) ? 0 : (((double)_sum) / _count)); }


        /**
         * Return the std around its average off all records (since the last reset).
         **/
        double std() const 
            {
            if (_count == 0) return 0.0;
            const double a = _sum;
            const double b = ((((uint64_t)_sumsqr) >> 32) * 4294967296.0) + (((uint64_t)_sumsqr) & 0xFFFFFFFF); // BUG: converting directly from uint64 to double fails so we do this dirty workaround...
            const double c = sqrt((b - ((a*a)/ _count))/_count);
            return c;
            }


        private:

            uint32_t _count; 
            int32_t _min;
            int32_t _max;
            int64_t _sum;
            int64_t _sumsqr;
    };




}


/** end of file */

