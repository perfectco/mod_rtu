#include "mod.h"
#include "mod_rtu_master.h"
#include "mod_rtu_tx.h"
//nrf52840 port
#include "nrf_serial.h"
#include "nrf_drv_timer.h"
#include "mod_rtu_reply_timer.h"

#define NRF_LOG_MODULE_NAME mod_rtu_master
#define NRF_LOG_LEVEL 3
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


void mod_rtu_encode_uint16 (const uint16_t value, uint8_t *const target) {
    target[0] = (value >> 8) & 0xff;
    target[1] = value & 0xff;
}

uint16_t mod_rtu_decode_uint16 (const uint8_t *const source) {
    return (source[0] << 8) + source[1];
}

void mod_rtu_encode_int16 (const int16_t value, uint8_t *const target) {
    target[0] = (value >> 8) & 0xff;
    target[1] = value & 0xff;
}

bool in_range_uint16(const uint16_t val, const uint16_t min, const uint16_t max) {
    return val >= min && val <= max;
}

//call to process message response
static void process_received_msg(mod_rtu_master_t *const me, const mod_rtu_tx_event_t * const event) {
    if (event->msg_received.msg->address != me->expected_address) {
        NRF_LOG_DEBUG("unexpected slave address, ignoring");
        return;
    }

    const uint8_t original_type = event->msg_received.msg->function & 0x7F;
    const bool exception = (event->msg_received.msg->function & 0x80) != 0;

    mod_rtu_master_event_t master_event = {
        .type = mod_rtu_master_event_response, //preset as generic response
        .error = exception ? mod_rtu_error_msg_exception : mod_rtu_error_ok,
        .response = {
            .msg = event->msg_received.msg,
        },
    };

    //modify event for specific type, if possible
    const mod_rtu_master_callback_t cb = me->temp_callback ? me->temp_callback : (me->callback ? me->callback : NULL);
    void * const context = me->temp_callback ? me->temp_callback_context : (me->callback ? me->callback_context : NULL);
    me->temp_callback = NULL;
    me->temp_callback_context = NULL;

    switch (original_type) {
        case MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_CODE: {
            master_event.type = mod_rtu_master_event_read_holding_response;
            master_event.read_holding_response.start_address = me->last_start_address;
            if (exception) {
                master_event.read_holding_response.count = 0;
                master_event.read_holding_response.data = NULL;
                if (cb) {
                    cb(&master_event, context);
                }
            } else {
                master_event.read_holding_response.count = event->msg_received.msg->data[0] / 2;
                uint16_t data[master_event.read_holding_response.count];
                master_event.read_holding_response.data = data;
                memcpy (data, event->msg_received.msg->data + 1, master_event.read_holding_response.count * sizeof(data[0]));
                for (int i = 0; i < master_event.read_holding_response.count; i += 1) {
                    data[i] = (data[i] >> 8) + (data[i] << 8);
                }
                if (cb) {
                    cb(&master_event, context);
                }
            }
        }
        break;

        case MOD_RTU_FUNCTION_WRITE_SINGLE_REGISTER_CODE: {
            master_event.type = mod_rtu_master_event_write_single_response;
            if (exception) {
                if (cb) {
                    cb(&master_event, context);
                }
            } else {
                master_event.write_single_response.address = mod_rtu_decode_uint16(&event->msg_received.msg->data[MOD_RTU_FUNCTION_WRITE_SINGLE_REGISTER_ADDR_OFFSET]);
                master_event.write_single_response.value = mod_rtu_decode_uint16(&event->msg_received.msg->data[MOD_RTU_FUNCTION_WRITE_SINGLE_REGISTER_VALUE_OFFSET]);
                if (cb) {
                    cb(&master_event, context);
                }
            }
        }
        break;

        case MOD_RTU_FUNCTION_WRITE_MULTIPLE_REGISTERS_CODE: {
            master_event.type = mod_rtu_master_event_write_multiple_response;
            if (exception) {
                if (cb) {
                    cb(&master_event, context);
                }
            } else {
                master_event.write_multiple_response.start_address = mod_rtu_decode_uint16(&event->msg_received.msg->data[MOD_RTU_FUNCTION_WRITE_MULTIPLE_REGISTERS_START_ADDR_OFFSET]);
                master_event.write_multiple_response.count = mod_rtu_decode_uint16(&event->msg_received.msg->data[MOD_RTU_FUNCTION_WRITE_MULTIPLE_REGISTERS_COUNT_OFFSET]);
                if (cb) {
                    cb(&master_event, context);
                }
            }
        }
        break;

#if 0
        case MOD_RTU_FUNCTION_READ_COILS_CODE: {
            master_event.type = mod_rtu_master_event_read_coils_response;
            master_event.read_coils_response.start_address = me->last_start_address;
            master_event.read_coils_response.count = me->last_count;
            bool data[master_event.read_coils_response.count];
            master_event.read_coils_response.data = data;
            //set boolean data from bits
            for (int i = 0; i < master_event.read_coils_response.count; i += 1) {
                data[i] = (event->msg_received.msg->data[i/8] & (1 << (i % 8))) != 0;
            }

            if (me->callback) {
                me->callback(&master_event, me->callback_context);
            }
        }
        break;
#endif

        default: {
            if (cb) {
                cb(&master_event, context);
            }
        }
        break;
    }

    mod_rtu_reply_timer_stop(&me->timer);
    me->state = mod_rtu_master_state_idle;
}

static void rtu_tx_callback(const mod_rtu_tx_event_t * const event, void *const context) {
    mod_rtu_master_t *const me = context;
    if (me->state == mod_rtu_master_wait_reply) {
        switch (event->type) {
            case mod_rtu_tx_event_msg_rx:
            //received a valid reply
            //todo: do something with that reply
            if (event->error != mod_rtu_error_ok) {
                NRF_LOG_DEBUG("got reply with error, ignoring");
            } else {
                NRF_LOG_DEBUG("got reply");
                process_received_msg(me, event);
            }
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
            if (me->callback) {
                const mod_rtu_master_event_t event = {
                    .type = mod_rtu_master_event_ready,
                    .error = mod_rtu_error_ok,
                };
                me->callback(&event, me->callback_context);
            }
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
    mod_rtu_master_t *const me = context;
    if (me->state == mod_rtu_master_wait_reply) {
        NRF_LOG_DEBUG("timeout");
        //todo: send timeout event so app can retry, if desired
        mod_rtu_master_event_t event = {
            .type = mod_rtu_master_event_response_timeout,
            .error = mod_rtu_error_timeout,
        };
        if (me->callback) {
            me->callback(&event, me->callback_context);
        }
        me->state = mod_rtu_master_state_idle;
    }
}

/*
Initialize modbus library
*/
mod_rtu_error_t mod_rtu_master_init(mod_rtu_master_t *const me, const mod_rtu_master_init_t *const init) {
    me->state = mod_rtu_master_state_init;
    me->callback = init->callback;
    me->callback_context = init->callback_context;

    mod_rtu_tx_init_t tx_init = {
        .callback = rtu_tx_callback,
        .callback_context = me,
        .serial_config = {
            .tx_pin = MODBUS_TX_PIN,
            .rx_pin = MODBUS_RX_PIN,
            .tx_en_pin = MODBUS_TX_EN_PIN,
            .rx_en_pin = MODBUS_RX_EN_PIN,
            .parity = NRF_UARTE_PARITY_EXCLUDED,
            .baudrate = NRF_UARTE_BAUDRATE_19200,
            .stop = NRF_UARTE_STOP_TWO,
            .interrupt_priority = UART_DEFAULT_CONFIG_IRQ_PRIORITY,
            .uart_instance = 1,
        }
    };

    mod_rtu_reply_timer_init_t timer_init = {
        .timeout = init->response_timeout_ms,
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

    if (!in_range_uint16(start_addr, MOD_RTU_FUNCTION_READ_COILS_MIN_ADDR, MOD_RTU_FUNCTION_READ_COILS_MAX_ADDR)) {
        return mod_rtu_error_parameter;
    }

    if (!in_range_uint16(count, MOD_RTU_FUNCTION_READ_COILS_MIN_COUNT, MOD_RTU_FUNCTION_READ_COILS_MAX_COUNT)) {
        return mod_rtu_error_parameter;
    }

    mod_rtu_msg_t msg;
    const mod_rtu_error_t setup_err = master_request_start(me, device_addr, &msg);

    if (setup_err != mod_rtu_error_ok) {
        return setup_err;
    }

    msg.function = MOD_RTU_FUNCTION_READ_COILS_CODE;
    msg.data_length = MOD_RTU_FUNCTION_READ_COILS_REQUEST_LENGTH;

    //read coils request consists of address and count
    mod_rtu_encode_uint16(start_addr, &msg.data[MOD_RTU_FUNCTION_READ_COILS_START_ADDR_OFFSET]);
    mod_rtu_encode_uint16(count, &msg.data[MOD_RTU_FUNCTION_READ_COILS_COUNT_OFFSET]);

    me->last_start_address = start_addr;
    me->last_count = count;

    return master_request_execute(me, &msg);
}

mod_rtu_error_t mod_rtu_master_read_holding_registers(mod_rtu_master_t *const me, const uint8_t device_addr, const uint16_t start_addr, const uint16_t count) {
    return mod_rtu_master_read_holding_registers_cb(me, device_addr, start_addr, count, NULL, NULL);
}

mod_rtu_error_t mod_rtu_master_read_holding_registers_cb(mod_rtu_master_t *const me, const uint8_t device_addr, const uint16_t start_addr, const uint16_t count, const mod_rtu_master_callback_t cb, void *const context) {
    NRF_LOG_DEBUG("read holding registers");

    if (!in_range_uint16(start_addr, MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_MIN_ADDR, MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_MAX_ADDR)) {
        return mod_rtu_error_parameter;
    }
    
    if (!in_range_uint16(count, MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_MIN_COUNT, MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_MAX_COUNT)) {
        return mod_rtu_error_parameter;
    }

    me->temp_callback = cb;
    me->temp_callback_context = context;

    mod_rtu_msg_t msg;
    const mod_rtu_error_t setup_err = master_request_start(me, device_addr, &msg);

    if (setup_err != mod_rtu_error_ok) {
        return setup_err;
    }

    msg.function = MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_CODE;
    msg.data_length = MOD_RTU_FUNCTION_READ_HOLDING_REGISTERS_REQUEST_LENGTH;

    //read coils request consists of address and count
    mod_rtu_encode_uint16(start_addr, &msg.data[MOD_RTU_FUNCTION_READ_COILS_START_ADDR_OFFSET]);
    mod_rtu_encode_uint16(count, &msg.data[MOD_RTU_FUNCTION_READ_COILS_COUNT_OFFSET]);

    me->last_start_address = start_addr;
    me->last_count = count;

    return master_request_execute(me, &msg);
}

mod_rtu_error_t mod_rtu_master_write_single_register_cb(mod_rtu_master_t *const me, const uint8_t device_addr, const uint16_t addr, const uint16_t data, const mod_rtu_master_callback_t cb, void *const context) {
    NRF_LOG_DEBUG("write single register");

    if (!in_range_uint16(addr, MOD_RTU_FUNCTION_WRITE_SINGLE_REGISTER_MIN_ADDR, MOD_RTU_FUNCTION_WRITE_SINGLE_REGISTER_MAX_ADDR)) {
        return mod_rtu_error_parameter;
    }
    
    me->temp_callback = cb;
    me->temp_callback_context = context;

    mod_rtu_msg_t msg;
    const mod_rtu_error_t setup_err = master_request_start(me, device_addr, &msg);

    if (setup_err != mod_rtu_error_ok) {
        return setup_err;
    }

    msg.function = MOD_RTU_FUNCTION_WRITE_SINGLE_REGISTER_CODE;
    msg.data_length = MOD_RTU_FUNCTION_WRITE_SINGLE_REGISTER_REQUEST_LENGTH;

    //read coils request consists of address and count
    mod_rtu_encode_uint16(addr, &msg.data[MOD_RTU_FUNCTION_WRITE_SINGLE_REGISTER_ADDR_OFFSET]);
    mod_rtu_encode_uint16(data, &msg.data[MOD_RTU_FUNCTION_WRITE_SINGLE_REGISTER_VALUE_OFFSET]);

    me->last_start_address = addr;
    me->last_count = 1;

    return master_request_execute(me, &msg);
}

mod_rtu_error_t mod_rtu_master_write_multiple_registers_cb(mod_rtu_master_t *const me, const uint8_t device_addr, const uint16_t start_addr, const uint16_t count, const uint16_t *const data, 
                                                           const mod_rtu_master_callback_t cb, void *const context) {
    NRF_LOG_DEBUG("write multiple registers");

    if (!in_range_uint16(start_addr, MOD_RTU_FUNCTION_WRITE_MULTIPLE_REGISTERS_MIN_ADDR, MOD_RTU_FUNCTION_WRITE_MULTIPLE_REGISTERS_MAX_ADDR)) {
        return mod_rtu_error_parameter;
    }
    
    if (!in_range_uint16(count, MOD_RTU_FUNCTION_WRITE_MULTIPLE_REGISTERS_MIN_COUNT, MOD_RTU_FUNCTION_WRITE_MULTIPLE_REGISTERS_MAX_COUNT)) {
        return mod_rtu_error_parameter;
    }
    
    me->temp_callback = cb;
    me->temp_callback_context = context;

    mod_rtu_msg_t msg;
    const mod_rtu_error_t setup_err = master_request_start(me, device_addr, &msg);

    if (setup_err != mod_rtu_error_ok) {
        return setup_err;
    }

    msg.function = MOD_RTU_FUNCTION_WRITE_SINGLE_REGISTER_CODE;
    msg.data_length = MOD_RTU_FUNCTION_WRITE_SINGLE_REGISTER_REQUEST_LENGTH;

    //read coils request consists of address and count
    mod_rtu_encode_uint16(start_addr, &msg.data[MOD_RTU_FUNCTION_WRITE_MULTIPLE_REGISTERS_START_ADDR_OFFSET]);
    mod_rtu_encode_uint16(count, &msg.data[MOD_RTU_FUNCTION_WRITE_MULTIPLE_REGISTERS_COUNT_OFFSET]);
    msg.data[MOD_RTU_FUNCTION_WRITE_MULTIPLE_REGISTERS_BYTE_COUNT_OFFSET] = 2 * count;
    for (int i = 0; i < count; i += 1) {
        mod_rtu_encode_uint16(data[i], &msg.data[MOD_RTU_FUNCTION_WRITE_MULTIPLE_REGISTERS_VALUES_OFFSET + i * 2]);
    }
    
    me->last_start_address = start_addr;
    me->last_count = count;

    return master_request_execute(me, &msg);
}


