#ifndef __MOD_RTU_MASTER_H__
#define __MOD_RTU_MASTER_H__

#include "mod.h"
#include "mod_rtu_tx.h"
#include "mod_rtu_reply_timer.h"

//types of master events sent to callback
typedef enum mod_rtu_master_event_type_e {
  mod_rtu_master_event_ready, //transitioned from init state to idle
  mod_rtu_master_event_response, //got a new message
  mod_rtu_master_event_response_timeout,
  mod_rtu_master_event_read_holding_response,
  mod_rtu_master_event_read_coils_response,
  mod_rtu_master_event_write_single_response,
  mod_rtu_master_event_write_multiple_response,
} mod_rtu_master_event_type_t;

//Structs for various event types
//Generic response type, just a pointer to the received message
typedef struct mod_rtu_master_response_data_s {
  const mod_rtu_msg_t *msg;
} mod_rtu_master_response_data_t;

//
typedef struct mod_rtu_master_read_holding_response_data_s {
  uint16_t start_address;
  uint16_t count;
  uint16_t *data;
} mod_rtu_master_read_holding_response_data_t;

typedef struct mod_rtu_master_read_coils_response_data_s {
  uint16_t start_address;
  uint16_t count;
  bool *data;
} mod_rtu_master_read_coils_response_data_t;

typedef struct mod_rtu_master_write_single_response_data_s {
  uint16_t address;
  uint16_t value;
} mod_rtu_master_write_single_response_data_t;

typedef struct mod_rtu_master_write_multiple_response_data_s {
  uint16_t start_address;
  uint16_t count;
} mod_rtu_master_write_multiple_response_data_t;

typedef struct mod_rtu_master_event_s {
  mod_rtu_master_event_type_t type;
  mod_rtu_error_t error;
  union {
    mod_rtu_master_response_data_t response;
    mod_rtu_master_read_holding_response_data_t read_holding_response;
    mod_rtu_master_read_coils_response_data_t read_coils_response;
    mod_rtu_master_write_single_response_data_t write_single_response;
    mod_rtu_master_write_multiple_response_data_t write_multiple_response;
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
  mod_rtu_master_callback_t temp_callback;
  void * temp_callback_context;
  uint16_t last_start_address;
  uint16_t last_count;
} mod_rtu_master_t;

mod_rtu_error_t mod_rtu_master_init(mod_rtu_master_t *const me, const mod_rtu_master_init_t *const init);
mod_rtu_error_t mod_rtu_master_read_coils(mod_rtu_master_t *const me, const uint8_t device_addr, const uint16_t start_addr, const uint16_t count);
mod_rtu_error_t mod_rtu_master_read_holding_registers(mod_rtu_master_t *const me, const uint8_t device_addr, const uint16_t start_addr, const uint16_t count);
mod_rtu_error_t mod_rtu_master_read_holding_registers_cb(mod_rtu_master_t *const me, const uint8_t device_addr, const uint16_t start_addr, const uint16_t count, const mod_rtu_master_callback_t cb, void *const context);
mod_rtu_error_t mod_rtu_master_write_single_register(mod_rtu_master_t *const me, const uint8_t device_addr, const uint16_t addr, const uint16_t data, const mod_rtu_master_callback_t cb, void *const context);
mod_rtu_error_t mod_rtu_master_write_multiple_registers(mod_rtu_master_t *const me, const uint8_t device_addr, const uint16_t start_addr, const uint16_t count, const uint16_t *const data, const mod_rtu_master_callback_t cb, void *const context);

#endif
