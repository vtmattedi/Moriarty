// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: test32

#ifndef _TEST32_UI_H
#define _TEST32_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
  #if __has_include("lvgl.h")
    #include "lvgl.h"
  #elif __has_include("lvgl/lvgl.h")
    #include "lvgl/lvgl.h"
  #else
    #include "lvgl.h"
  #endif
#else
  #include "lvgl.h"
#endif

#include "ui_helpers.h"
#include "ui_events.h"
// SCREEN: ui_mainScreen
void ui_mainScreen_screen_init(void);
extern lv_obj_t *ui_mainScreen;
extern lv_obj_t *ui_arcTemp;
extern lv_obj_t *ui_lbTemp;
extern lv_obj_t *ui_lbTempSecondary;
extern lv_obj_t *ui_btAcPower;
extern lv_obj_t *ui_lbConstAcPower;
extern lv_obj_t *ui_lbConstAcMinus;
extern lv_obj_t *ui_lbConstAcPlus;
extern lv_obj_t *ui_btAcPlus;
extern lv_obj_t *ui_btAcMinus;
extern lv_obj_t *ui_btAcSet;
extern lv_obj_t *ui_btColorAuto;
extern lv_obj_t *ui_lbConstColorAuto;
void ui_event_btConfig( lv_event_t * e);
extern lv_obj_t *ui_btConfig;
extern lv_obj_t *ui_lbConstConfig;
extern lv_obj_t *ui_Bar1;
extern lv_obj_t *ui_lbInfo;
void ui_event_btLock( lv_event_t * e);
extern lv_obj_t *ui_btLock;
extern lv_obj_t *ui_pnConfig;
extern lv_obj_t *ui_lbConstDeviceName;
extern lv_obj_t *ui_lbConstLed;
extern lv_obj_t *ui_swLed;
extern lv_obj_t *ui_lbConstIP;
extern lv_obj_t *ui_lbIP;
extern lv_obj_t *ui_lbConstBoot;
extern lv_obj_t *ui_lbBoot;
void ui_event_btBack( lv_event_t * e);
extern lv_obj_t *ui_btBack;
extern lv_obj_t *ui_lbConstbtBack;
extern lv_obj_t *ui_btReset;
extern lv_obj_t *ui_lbConstbtReset;
extern lv_obj_t *ui_arcRam;
extern lv_obj_t *ui_lbRamValue;
extern lv_obj_t *ui_lbConstRam;
extern lv_obj_t *ui_arcFS;
extern lv_obj_t *ui_lbFSValue;
extern lv_obj_t *ui_lbConstFS;
// SCREEN: ui_lockScreen
void ui_lockScreen_screen_init(void);
extern lv_obj_t *ui_lockScreen;
void ui_event_imgLogo( lv_event_t * e);
extern lv_obj_t *ui_imgLogo;
extern lv_obj_t *ui_lbTime;
extern lv_obj_t *ui____initial_actions0;

LV_IMG_DECLARE( ui_img_1529088296);   // assets\logo-no-background240.png
LV_IMG_DECLARE( ui_img_634369039);   // assets\logo-no-background256.png

LV_FONT_DECLARE( ui_font_FontMono);
LV_FONT_DECLARE( ui_font_FontMono2);

void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
