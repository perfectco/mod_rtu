#ifndef __MOD_RTU_MASTER_H__
#define __MOD_RTU_MASTER_H__

#include "mod.h"
#include "mod_rtu_tx.h"
#include "mod_rtu_reply_timer.h"

typedef enum mod_rtu_master_event_type_e {
  mod_rtu_master_event_ready, //transitioned from init state to idle
  mod_rtu_master_event_response, //got a new message
  mod_rtu_master_event_response_timeout,
} mod_rtu_master_event_type_t;

//data for msg_rx_event or msg_rx_error events
typedef struct mod_rtu_master_response_data_s {
  const mod_rtu_msg_t *msg;
} mod_rtu_master_response_data_t;

typedef struct mod_rtu_master_event_s {
  mod_rtu_master_event_type_t type;
  mod_rtu_error_t error;
  union {
    mod_rtu_master_response_data_t msg_received;
  };
} mod_rtu_master_event_t;

typedef void (* mod_rtu_master_callback_t)(const mod_rtu_master_event_t *const event, void *const context);

typedef struct mod_rtu_master_init_s {
  uint16_t response_timeout_ms;
  mod_rtu_master_callback_t callback;
  void * callback_context;
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
  mod_rtu_master_callback_t callback;
  void * callback_context;
} mod_rtu_master_t;

mod_rtu_error_t mod_rtu_master_init(mod_rtu_master_t *const me, const mod_rtu_master_init_t *const init);
mod_rtu_error_t mod_rtu_master_read_coils(mod_rtu_master_t *const me, const uint8_t device_addr, const uint16_t start_addr, const uint16_t count);
mod_rtu_error_t mod_rtu_master_read_holding_registers(mod_rtu_master_t *const me, const uint8_t device_addr, const uint16_t start_addr, const uint16_t count);

#endif
