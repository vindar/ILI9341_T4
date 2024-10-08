/**
 * @file lv_demo_music.h
 *
 */

#ifndef LV_DEMO_MUSIC_H
#define LV_DEMO_MUSIC_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
//#include "../lv_demos.h"

#include <lvgl.h>
#include <Arduino.h>


/** Disable performance monitor because there is a bug in the code when enabled (with LVGL v9.2) */
#ifdef LV_USE_SYSMON
#undef LV_USE_SYSMON
#endif
#define LV_USE_SYSMON 0

#ifdef LV_USE_PERF_MONITOR
#undef LV_USE_PERF_MONITOR
#endif
#define LV_USE_PERF_MONITOR 0


/* overwrite predefined values */
#ifdef LV_USE_DEMO_MUSIC
#undef LV_USE_DEMO_MUSIC
#endif
#define LV_USE_DEMO_MUSIC 1

#ifdef LV_DEMO_MUSIC_SQUARE
#undef LV_DEMO_MUSIC_SQUARE
#endif
#define LV_DEMO_MUSIC_SQUARE 0

#ifdef LV_DEMO_MUSIC_LANDSCAPE
#undef LV_DEMO_MUSIC_LANDSCAPE
#endif
#define LV_DEMO_MUSIC_LANDSCAPE 1

#ifdef LV_DEMO_MUSIC_ROUND
#undef LV_DEMO_MUSIC_ROUND
#endif
#define LV_DEMO_MUSIC_ROUND 1

#ifdef LV_DEMO_MUSIC_LARGE
#undef LV_DEMO_MUSIC_LARGE
#endif
#define LV_DEMO_MUSIC_LARGE 0

#ifdef LV_DEMO_MUSIC_AUTO_PLAY
#undef LV_DEMO_MUSIC_AUTO_PLAY
#endif
#define LV_DEMO_MUSIC_AUTO_PLAY 1






#if LV_USE_DEMO_MUSIC

/*********************
 *      DEFINES
 *********************/

#if LV_DEMO_MUSIC_LARGE
#  define LV_DEMO_MUSIC_HANDLE_SIZE  40
#else
#  define LV_DEMO_MUSIC_HANDLE_SIZE  20
#endif

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void lv_demo_music(void);
const char * lv_demo_music_get_title(uint32_t track_id);
const char * lv_demo_music_get_artist(uint32_t track_id);
const char * lv_demo_music_get_genre(uint32_t track_id);
uint32_t lv_demo_music_get_track_length(uint32_t track_id);

/**********************
 *      MACROS
 **********************/

#endif /*LV_USE_DEMO_MUSIC*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_DEMO_MUSIC_H*/
