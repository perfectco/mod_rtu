#include "mod_rtu_reply_timer.h"

static void app_timer_callback(void *context) {
  if (!context) {
    return;
  }
  mod_rtu_reply_timer_t * me = (mod_rtu_reply_timer_t *)context;
  if (me->callback) {
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
  app_timer_start(&me->timer, APP_TIMER_TICKS(me->timeout), me->callback_context);
  return mod_rtu_error_ok;
}

mod_rtu_error_t mod_rtu_reply_timer_stop(mod_rtu_reply_timer_t *const me) {
  app_timer_stop(&me->timer);
  return mod_rtu_error_ok;
}
