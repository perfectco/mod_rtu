#ifndef __MOD_RTU_MASTER_H__
#define __MOD_RTU_MASTER_H__

#include "mod.h"
#include "mod_rtu_tx.h"
#include "mod_rtu_reply_timer.h"

typedef struct mod_rtu_master_init_s {
  uint16_t response_timeout_ms;
} mod_rtu_master_init_t;

typedef enum mod_rtu_master_state_e {
  mod_rtu_master_state_init,
  mod_rtu_master_state_idle,
  mod_rtu_master_wait_turnaround,
  mod_rtu_master_wait_reply,
} mod_rtu_master_state_t;

typedef struct mod_rtu_master_s {
  mod_rtu_master_state_t state;
  mod_rtu_tx_t rtu_tx; //serial port controller
  mod_rtu_reply_timer_t timer;
  mod_rtu_slave_addr_t expected_address;
  uint8_t max_retry;
  uint8_t retry_counter;
} mod_rtu_master_t;

mod_rtu_error_t mod_rtu_master_init(mod_rtu_master_t *const me, const mod_rtu_master_init_t *const init);
mod_rtu_error_t mod_rtu_master_read_coils(mod_rtu_master_t *const me, const uint8_t device_addr, const uint16_t coil_start_addr, const uint16_t count);

#endif
