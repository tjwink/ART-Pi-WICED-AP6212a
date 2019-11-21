/*
 * $ Copyright Cypress Semiconductor  $
 */

/** @file
 *
 */

#include "dpm.h"
#include "wss.h"
#include "timer.h"
#include "xhr.h"
#include "audio.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

 /******************************************************
 *               Function Definitions
 ******************************************************/

static const duk_function_list_entry dpm_funcs[] = {

  // wss.c
  {"new_wss", dpm_new_wss, 1},
  {"wss_start", dpm_wss_start, 3},
  {"wss_on_close", dpm_wss_on_close, 2},
  {"wss_on_message", dpm_wss_on_message, 2},
  {"wss_send", dpm_wss_send, 2},
  {"wss_close", dpm_wss_close, 1},
  {"wss_shutdown", dpm_wss_shutdown, 0},

  // timer.c
  {"new_timer", dpm_new_timer, 0},
  {"start_timer", dpm_timer_start, 4},
  {"clear_timer", dpm_timer_stop, 1},
  {"timer_shutdown", dpm_timer_shutdown, 0},

  // xhr.c
  {"new_xhr", dpm_new_xhr, 0},
  {"xhr_open", dpm_xhr_open, 4},
  {"xhr_send", dpm_xhr_send, 2},
  {"xhr_set_request_header", dpm_xhr_set_request_header, 3},
  {"xhr_on_load", dpm_xhr_on_load, 2},
  {"xhr_on_error", dpm_xhr_on_error, 2},
  {"xhr_shutdown", dpm_xhr_shutdown, 0},

  // audio.c
  {"new_audio", dpm_new_audio, 0},
  {"audio_load", dpm_audio_load, 1},
  {"audio_play", dpm_audio_play, 1},
  {"audio_pause", dpm_audio_pause, 1},
  {"audio_attr", dpm_audio_attribute, 3},
  {"audio_get_prop", dpm_audio_property_getter, 2},
  {"audio_set_prop", dpm_audio_property_setter, 3},
  {"audio_on_event", dpm_audio_on_event, 2},
  {"audio_shutdown", dpm_audio_shutdown, 0},

  {NULL, NULL, 0},
};

duk_ret_t dukopen_dpm(duk_context *ctx) {

  duv_ref_setup(ctx);

  // Create a dpm table on the global
  duk_push_object(ctx);
  duk_put_function_list(ctx, -1, dpm_funcs);
  return 1;
}
