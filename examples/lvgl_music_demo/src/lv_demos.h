/**
 * @file lv_demos.h
 *
 */

#ifndef LV_DEMOS_H
#define LV_DEMOS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
//#include "../lvgl.h"
#include <lvgl.h>
#include <Arduino.h>

typedef struct _lv_demo_args lv_demo_args_t;



/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
struct _lv_demo_args {
    lv_obj_t * parent;
};

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Initialize the lv_demo_args_t structure with default values.
 * @param args Pointer to the lv_demo_args_t structure to be initialized.
 */
void lv_demo_args_init(lv_demo_args_t * args);

/**
 * Call lv_demo_xxx.
 * @param   info the information which contains demo name and parameters
 *               needs by lv_demo_xxx.
 * @size    size of information.
 */
bool lv_demos_create(char * info[], int size);

/**
 * Show help for lv_demos.
 */
void lv_demos_show_help(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_DEMOS_H*/
