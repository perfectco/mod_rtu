#include "mod_rtu_reply_timer.h"

#define NRF_LOG_MODULE_NAME mod_rtu_reply_timer
#define NRF_LOG_LEVEL 4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

static void app_timer_callback(void *context) {
  NRF_LOG_DEBUG("timer callback");
  if (!context) {
    return;
  }
  NRF_LOG_DEBUG("context OK");
  mod_rtu_reply_timer_t * me = (mod_rtu_reply_timer_t *)context;
  if (me->callback) {
    NRF_LOG_DEBUG("calling callback");
    me->callback(me->callback_context);
  }
}

mod_rtu_error_t mod_rtu_reply_timer_init(mod_rtu_reply_timer_t *const me, const mod_rtu_reply_timer_init_t *const init) {
  app_timer_t * timer_id = &me->timer;
  app_timer_create(&timer_id, APP_TIMER_MODE_SINGLE_SHOT, app_timer_callback);

  me->timeout = init->timeout;
  me->callback = init->callback;
  me->callback_context = init->callback_context;
  return mod_rtu_error_ok;
}

mod_rtu_error_t mod_rtu_reply_timer_start(mod_rtu_reply_timer_t *const me) {
  mod_rtu_reply_timer_stop(me);
  app_timer_start(&me->timer, APP_TIMER_TICKS(me->timeout), me);
  return mod_rtu_error_ok;
}

mod_rtu_error_t mod_rtu_reply_timer_stop(mod_rtu_reply_timer_t *const me) {
  app_timer_stop(&me->timer);
  return mod_rtu_error_ok;
}
