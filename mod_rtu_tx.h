#ifndef __MOD_RTU_TX_H__
#define __MOD_RTU_TX_H__

#include <inttypes.h>
#include "mod.h"
#include "nrf_drv_uart.h"
#include "nrf_drv_timer.h"
#include "app_timer.h" //todo: need higher resolution
#include "nrf_drv_ppi.h"

#define MOD_RTU_TX_FRAME_OVERHEAD 4
#define MOD_RTU_TX_MAX_RAW_MSG_LENGTH (MOD_RTU_MAX_DATA_LENGTH + MOD_RTU_TX_FRAME_OVERHEAD)

typedef struct mod_rtu_tx_raw_msg_s {
  uint16_t data_length;
  uint8_t data[MOD_RTU_TX_MAX_RAW_MSG_LENGTH];
} mod_rtu_tx_raw_msg_t;

typedef enum mod_rtu_tx_state_e {
  mod_rtu_tx_state_init, //initial state, waiting for expiration of t35
  mod_rtu_tx_state_idle,
  mod_rtu_tx_state_emission,
  mod_rtu_tx_state_reception,
  mod_rtu_tx_state_ctrl_wait,
} mod_rtu_tx_state_t;

typedef struct mod_rtu_tx_s {
  mod_rtu_tx_state_t state;
  mod_rtu_tx_raw_msg_t tx_raw_msg; //current/last outgoing message
  mod_rtu_tx_raw_msg_t rx_raw_msg; //buffer for receiving message
  mod_rtu_msg_t last_rx_msg; //last validated received message
  bool rx_msg_ok;
  bool rx_35;
  bool rx_done;
  bool serial_init;
  bool timer_init;
  nrf_drv_uart_t uart;
  nrf_drv_timer_t timer;
  uint32_t tx_en_pin;
  uint32_t rx_en_pin;
  nrf_ppi_channel_group_t ppi_group;
  bool master_mode; //true if in master mode, false for slave mode
  uint8_t own_addr; //if in slave mode, our address
} mod_rtu_tx_t;

typedef enum mod_rtu_tx_event_type_e {
  mod_rtu_tx_event_ready, //transitioned from init state to idle
  mod_rtu_tx_event_msg_rx, //got a new message
  mod_rtu_tx_event_rx_error, //error while receving
  mod_rtu_tx_event_tx_done, //done sending
} mod_rtu_tx_event_type_t;

//data for msg_rx_event
typedef struct mod_rtu_tx_msg_received_data_s {
  const mod_rtu_msg_t *msg;
} mod_rtu_tx_msg_received_data_t;

typedef struct mod_rtu_tx_ready_data_s {
} mod_rtu_tx_ready_data_t;

typedef struct mod_rtu_tx_event_s {
  mod_rtu_tx_event_type_t type;
  mod_rtu_error_t error;
  union {
    mod_rtu_tx_msg_received_data_t msg_received;
    mod_rtu_tx_ready_data_t ready;
  };
} mod_rtu_tx_event_t;

typedef enum mod_rtu_tx_timer_type_e {
  mod_rtu_tx_timer_type_t15, //inter-character timeout, reception to control trigger
  mod_rtu_tx_timer_type_t35, //inter-message timeout, control to idle trigger
} mod_rtu_tx_timer_type_t;

typedef void (*mod_rtu_tx_event_callback_t)(const mod_rtu_tx_event_t * const event, void *const context);

/*
timer callback
The hardware port must implement a timer (or timers) for the inter-character and inter-frame delays.
The mod_rtu_tx module will call mod_rtu_port_tx_timer_reset() to start the t15 and t35 timers.
Upon expiration of the t15 timer, call mod_rtu_tx_timer_expired_callback_t with type = mod_rtu_tx_timer_type_t15,
and upon t35 expiration, with type = mod_rtu_tx_timer_type_t35
*/
void mod_rtu_tx_timer_expired_callback(mod_rtu_tx_t *const me, const mod_rtu_tx_timer_type_t type);

/*
Initialize mod_rtu_tx instance
*/
mod_rtu_error_t mod_rtu_tx_init(mod_rtu_tx_t *const me);

/*
Send a modbus RTU message
*/
mod_rtu_error_t mod_rtu_tx_send(mod_rtu_tx_t *const me, const mod_rtu_msg_t *const msg);


#endif
