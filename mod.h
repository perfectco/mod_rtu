#ifndef __MOD_H__
#define __MOD_H__

#include <inttypes.h>

#define MOD_RTU_ADDR_OFFSET 0
#define MOD_RTU_FUNCTION_OFFSET 1
#define MOD_RTU_DATA_OFFSET 2
#define MOD_RTU_CRC_LENGTH 2
#define MOD_RTU_MAX_DATA_LENGTH 252

#define MOD_RTU_BROADCAST_ADDRESS 0
#define MOD_RTU_MIN_UNICAST_ADDRESS 1
#define MOD_RTU_MAX_UNICAST_ADDRESS 247

#define MOD_RTU_FUNCTION_READ_COILS_CODE 0x01
#define MOD_RTU_FUNCTION_READ_COILS_REQUEST_LENGTH 4
#define MOD_RTU_FUNCTION_READ_COILS_START_ADDR_OFFSET 0
#define MOD_RTU_FUNCTION_READ_COILS_COUNT_OFFSET 2
#define MOD_RTU_FUNCTION_READ_COILS_COUNT_OFFSET 2

#define MOD_RTU_FUNCTION_READ_COILS_MIN_ADDR 0x0000
#define MOD_RTU_FUNCTION_READ_COILS_MAX_ADDR 0xFFFF
#define MOD_RTU_FUNCTION_READ_COILS_MIN_COUNT 1
#define MOD_RTU_FUNCTION_READ_COILS_MAX_COUNT 2000

#define MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_CODE 0x03
#define MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_REQUEST_LENGTH 4
#define MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_START_ADDR_OFFSET 0
#define MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_COUNT_OFFSET 2
#define MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_MIN_ADDR 0x0000
#define MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_MAX_ADDR 0xFFFF
#define MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_MIN_COUNT 1
#define MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_MAX_COUNT 125

typedef uint8_t mod_rtu_slave_addr_t;

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
  mod_rtu_error_invalid_address = 4,
  mod_rtu_error_msg_crc = 5,
  mod_rtu_error_msg_frame = 6,
  mod_rtu_error_msg_exception = 7,
  mod_rtu_error_timeout = 8,
} mod_rtu_error_t;

#endif

