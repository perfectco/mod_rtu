#ifndef __MOD_H__
#define __MOD_H__

#define MOD_RTU_MAX_MSG_LENGTH 256

#include <inttypes.h>

typedef struct mod_rtu_msg_s {
  uint16_t length;
  uint8_t buf[MOD_RTU_MAX_MSG_LENGTH];
} mod_rtu_msg_t;

typedef enum mod_rtu_error_e {
  mod_rtu_error_ok = 0,
  mod_rtu_error_unknown = 1,
  mod_rtu_error_invalid_state,
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

