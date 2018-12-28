#ifndef __MOD_H__
#define __MOD_H__

#include <inttypes.h>

#define MOD_RTU_ADDR_OFFSET 0
#define MOD_RTU_FUNCTION_OFFSET 1
#define MOD_RTU_DATA_OFFSET 2
#define MOD_RTU_CRC_LENGTH 2
#define MOD_RTU_MAX_DATA_LENGTH 252


typedef struct mod_rtu_msg_s {
  uint8_t address;
  uint8_t function;
  uint8_t data_length;
  uint8_t data[MOD_RTU_MAX_DATA_LENGTH];
} mod_rtu_msg_t;

typedef enum mod_rtu_error_e {
  mod_rtu_error_ok = 0,
  mod_rtu_error_unknown = 1,
  mod_rtu_error_invalid_state = 2,
  mod_rtu_error_parameter = 3,
} mod_rtu_error_t;

typedef struct mod_rtu_slave_s {
} mod_rtu_slave_t;

typedef struct mod_rtu_master_s {
} mod_rtu_master_t;

typedef struct mod_init_s {
} mod_init_t;

typedef enum mod_rtu_master_state_e {
  mod_rtu_master_state_idle,
  mod_rtu_master_wait_turnaround,
  mod_rtu_master_wait_reply,
} mod_rtu_master_state_t;

typedef uint8_t mod_rtu_slave_addr_t;

mod_rtu_error_t mod_init(mod_rtu_master_t *const me, const mod_init_t *const init);

#endif

