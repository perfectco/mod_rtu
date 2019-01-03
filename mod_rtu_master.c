#include "mod.h"
#include "mod_rtu_master.h"
#include "mod_rtu_tx.h"
//nrf52840 port
#include "nrf_serial.h"
#include "nrf_drv_timer.h"
#include "mod_rtu_reply_timer.h"

#define NRF_LOG_MODULE_NAME mod_rtu_master
#define NRF_LOG_LEVEL 4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


void mod_rtu_encode_uint16 (const uint16_t value, uint8_t *const target) {
    target[0] = (value >> 8) & 0xff;
    target[1] = value & 0xff;
}

void mod_rtu_encode_int16 (const int16_t value, uint8_t *const target) {
    target[0] = (value >> 8) & 0xff;
    target[1] = value & 0xff;
}

bool in_range_uint16(const uint16_t val, const uint16_t min, const uint16_t max) {
    return val >= min && val <= max;
}

static void rtu_tx_callback(const mod_rtu_tx_event_t * const event, void *const context) {
    mod_rtu_master_t *const me = context;
    if (me->state == mod_rtu_master_wait_reply) {
        switch (event->type) {
            case mod_rtu_tx_event_msg_rx:
            //received a valid reply
            //todo: do something with that reply
            if (event->error != mod_rtu_error_ok) {
                NRF_LOG_DEBUG("got reply with error");
            } else {
                NRF_LOG_DEBUG("got reply");
            }
            mod_rtu_reply_timer_stop(&me->timer);

            me->state = mod_rtu_master_state_idle;
            break;

            default:
            break;
        }
    } else if (me->state == mod_rtu_master_state_init) {
        switch (event->type) {
            case mod_rtu_tx_event_ready:
            //todo: send ready event to our callback
            NRF_LOG_DEBUG("got tx ready");
            me->state = mod_rtu_master_state_idle;
            //todo: testing
            mod_rtu_master_read_holding_registers(me, 1, 0, 1);
            break;

            default:
            break;
        }
    }
}

bool mod_rtu_is_broadcast_address(const uint8_t addr) {
    return addr == MOD_RTU_BROADCAST_ADDRESS;
}

bool mod_rtu_is_unicast_address(const uint8_t addr) {
    return (addr >= MOD_RTU_MIN_UNICAST_ADDRESS && addr <= MOD_RTU_MAX_UNICAST_ADDRESS);
}

mod_rtu_error_t mod_rtu_address_validate(const uint8_t address) {
    return (mod_rtu_is_broadcast_address(address) || mod_rtu_is_unicast_address(address)) ? 
        mod_rtu_error_ok : mod_rtu_error_invalid_address;
}

void reply_timer_callback(void * context) {
    NRF_LOG_DEBUG("timer callback");
}

/*
Initialize modbus library
*/
mod_rtu_error_t mod_rtu_master_init(mod_rtu_master_t *const me, const mod_rtu_master_init_t *const init) {
    me->state = mod_rtu_master_state_init;
    mod_rtu_tx_init_t tx_init = {
        .callback = rtu_tx_callback,
        .callback_context = me,
    };

    mod_rtu_reply_timer_init_t timer_init = {
        .timeout = 1000,
        .callback = reply_timer_callback,
        .callback_context = me,
    };

    mod_rtu_error_t timer_error = mod_rtu_reply_timer_init(&me->timer, &timer_init);
    if (timer_error != mod_rtu_error_ok) {
        return timer_error;
    }

    //todo: most settings hard-coded into rtu_tx, no hardware abstraction to speak of
    return mod_rtu_tx_init(&me->rtu_tx, &tx_init);
}

static mod_rtu_error_t master_request_start(mod_rtu_master_t *const me, const uint8_t device_addr, mod_rtu_msg_t *const msg) {
    if (me->state != mod_rtu_master_state_idle) {
        return mod_rtu_error_invalid_state;
    }

    const mod_rtu_error_t addr_err = mod_rtu_address_validate(device_addr);

    if (addr_err != mod_rtu_error_ok) {
        return addr_err;
    }

    msg->address = device_addr;

    me->expected_address = device_addr;

    return mod_rtu_error_ok;
}

static mod_rtu_error_t master_request_execute(mod_rtu_master_t *const me, mod_rtu_msg_t *const msg) {
    NRF_LOG_DEBUG("execute");
    const mod_rtu_error_t err = mod_rtu_tx_send(&me->rtu_tx, msg);

    if (err == mod_rtu_error_ok) {
        NRF_LOG_DEBUG("starting timers");
        me->state = mod_rtu_master_wait_reply;
        mod_rtu_reply_timer_start(&me->timer);
        return mod_rtu_error_ok;
    }

    return err;
}

mod_rtu_error_t mod_rtu_master_read_coils(mod_rtu_master_t *const me, const uint8_t device_addr, const uint16_t start_addr, const uint16_t count) {
    NRF_LOG_DEBUG("read coils");
    mod_rtu_msg_t msg;
    const mod_rtu_error_t setup_err = master_request_start(me, device_addr, &msg);

    if (setup_err != mod_rtu_error_ok) {
        return setup_err;
    }

    if (!in_range_uint16(start_addr, MOD_RTU_FUNCTION_READ_COILS_MIN_ADDR, MOD_RTU_FUNCTION_READ_COILS_MAX_ADDR)) {
        return mod_rtu_error_parameter;
    }

    if (!in_range_uint16(count, MOD_RTU_FUNCTION_READ_COILS_MIN_COUNT, MOD_RTU_FUNCTION_READ_COILS_MAX_COUNT)) {
        return mod_rtu_error_parameter;
    }

    msg.function = MOD_RTU_FUNCTION_READ_COILS_CODE;
    msg.data_length = MOD_RTU_FUNCTION_READ_COILS_REQUEST_LENGTH;

    //read coils request consists of address and count
    mod_rtu_encode_uint16(start_addr, &msg.data[MOD_RTU_FUNCTION_READ_COILS_START_ADDR_OFFSET]);
    mod_rtu_encode_uint16(count, &msg.data[MOD_RTU_FUNCTION_READ_COILS_COUNT_OFFSET]);

    return master_request_execute(me, &msg);
}

mod_rtu_error_t mod_rtu_master_read_holding_registers(mod_rtu_master_t *const me, const uint8_t device_addr, const uint16_t start_addr, const uint16_t count) {
    NRF_LOG_DEBUG("read holding registers");
    mod_rtu_msg_t msg;
    const mod_rtu_error_t setup_err = master_request_start(me, device_addr, &msg);

    if (setup_err != mod_rtu_error_ok) {
        return setup_err;
    }

    if (!in_range_uint16(start_addr, MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_MIN_ADDR, MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_MAX_ADDR)) {
        return mod_rtu_error_parameter;
    }
    
    if (!in_range_uint16(count, MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_MIN_COUNT, MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_MAX_COUNT)) {
        return mod_rtu_error_parameter;
    }

    msg.function = MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_CODE;
    msg.data_length = MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_REQUEST_LENGTH;

    //read coils request consists of address and count
    mod_rtu_encode_uint16(start_addr, &msg.data[MOD_RTU_FUNCTION_READ_COILS_START_ADDR_OFFSET]);
    mod_rtu_encode_uint16(count, &msg.data[MOD_RTU_FUNCTION_READ_COILS_COUNT_OFFSET]);

    return master_request_execute(me, &msg);
}


