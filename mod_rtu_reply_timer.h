#ifndef __MOD_RTU_REPLY_TIMER_H__
#define __MOD_RTU_REPLY_TIMER_H__

#include "mod.h"
#include "app_timer.h"
#include <inttypes.h>

typedef void (* mod_rtu_reply_timer_callback_t)(void *context);

typedef struct mod_rtu_reply_timer_s {
  app_timer_t timer;
  uint16_t timeout;
  mod_rtu_reply_timer_callback_t callback;
  void * callback_context;
} mod_rtu_reply_timer_t;

typedef struct mod_rtu_reply_timer_init_s {
  uint16_t timeout;
  mod_rtu_reply_timer_callback_t callback;
  void * callback_context;
} mod_rtu_reply_timer_init_t;

mod_rtu_error_t mod_rtu_reply_timer_init(mod_rtu_reply_timer_t *const me, const mod_rtu_reply_timer_init_t * const init);

mod_rtu_error_t mod_rtu_reply_timer_start(mod_rtu_reply_timer_t *const me);

mod_rtu_error_t mod_rtu_reply_timer_stop(mod_rtu_reply_timer_t *const me);

#endif
