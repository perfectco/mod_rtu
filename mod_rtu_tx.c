#include "mod_rtu_tx.h"

#include "nrf_drv_uart.h"

#include "boards.h"
#define NRF_LOG_MODULE_NAME mod_rtu_tx
#define NRF_LOG_LEVEL 3
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include <string.h>

#include "nrf_drv_timer.h"

#include "mbcrc.h"

#define T15_US 1432
#define T35_US 2578

typedef struct mod_rtu_tx_serial_nrf52_config_s {
  uint32_t tx_pin;
  uint32_t rx_pin;
  uint16_t tx_en_pin;
  uint16_t rx_en_pin;
  nrf_uart_parity_t parity;
  nrf_uart_baudrate_t baudrate;
  uint8_t interrupt_priority;
  uint8_t uart_instance;
} mod_rtu_tx_serial_nrf52_config_t;

typedef struct mod_rtu_tx_timer_nrf52_config_s {
  uint8_t timer_instance;
} mod_rtu_tx_timer_nrf52_config_t;

static void serial_event_handler(nrf_drv_uart_event_t * p_event, void * p_context);
static void timer_event_handler(nrf_timer_event_t p_event, void * p_context);
static void set_rx_en(mod_rtu_tx_t * const me, const bool enable);
static void set_tx_en(mod_rtu_tx_t * const me, const bool enable);

static mod_rtu_error_t timer_init(mod_rtu_tx_t * const me, const mod_rtu_tx_timer_nrf52_config_t * const p_config) {
  NRF_LOG_DEBUG("rtu timer_init");
  nrf_drv_timer_config_t timer_config = (nrf_drv_timer_config_t)NRF_DRV_TIMER_DEFAULT_CONFIG;
  timer_config.p_context = me;

  switch (p_config->timer_instance) {
    case 0:
    me->timer = (nrf_drv_timer_t)NRF_DRV_TIMER_INSTANCE(0);
    break;

    case 1:
    me->timer = (nrf_drv_timer_t)NRF_DRV_TIMER_INSTANCE(1);
    break;

    case 2:
    me->timer = (nrf_drv_timer_t)NRF_DRV_TIMER_INSTANCE(2);
    break;

    case 3:
    me->timer = (nrf_drv_timer_t)NRF_DRV_TIMER_INSTANCE(3);
    break;

    case 4:
    me->timer = (nrf_drv_timer_t)NRF_DRV_TIMER_INSTANCE(4);
    break;

    default:
    return mod_rtu_error_parameter;
    break;
  }

  ret_code_t ret = nrf_drv_timer_init(&me->timer, &timer_config, timer_event_handler);
  if (ret != NRF_SUCCESS) {
    return mod_rtu_error_unknown;
  }

  //setup compare register for T1.5
  const uint32_t compare15 = nrf_drv_timer_us_to_ticks(&me->timer, T15_US);
  nrf_drv_timer_compare(&me->timer, 0, compare15, true);

  //setup compare register for T3.5. Also stops timer.
  const uint32_t compare35 = nrf_drv_timer_us_to_ticks(&me->timer, T35_US);
  nrf_drv_timer_extended_compare(&me->timer, 1, compare35, NRF_TIMER_SHORT_COMPARE1_STOP_MASK, true);

  //enable timer, which also starts it, so stop and clear as well.
  nrf_drv_timer_enable(&me->timer);
  nrf_drv_timer_pause(&me->timer);
  nrf_drv_timer_clear(&me->timer);

  return mod_rtu_error_ok;
}

static mod_rtu_error_t ppi_init(mod_rtu_tx_t *const me) {
  NRF_LOG_DEBUG("rtu ppi_init");
  //set up ppi channel for uart rx to start/reset timer
  const uint32_t timer_start_addr = nrf_drv_timer_task_address_get(&me->timer, NRF_TIMER_TASK_START);
  const uint32_t timer_clear_addr = nrf_drv_timer_task_address_get(&me->timer, NRF_TIMER_TASK_CLEAR);

  const uint32_t uart_rx_addr = nrf_drv_uart_event_address_get(&me->uart, NRF_UART_EVENT_RXDRDY);
  
  //ppi_init returns either success or already initialized, so ignore either way
  (void)nrf_drv_ppi_init();

  //nrf_drv_timer_resume is synonymous with start task
  nrf_ppi_channel_t channel_start;
  ret_code_t ret = nrf_drv_ppi_channel_alloc(&channel_start);
  if (ret != NRF_SUCCESS) {
    return mod_rtu_error_unknown;
  }

  nrf_ppi_channel_t channel_clear;
  ret = nrf_drv_ppi_channel_alloc(&channel_clear);
  if (ret != NRF_SUCCESS) {
    return mod_rtu_error_unknown;
  }

  ret = nrf_drv_ppi_channel_assign(channel_start, uart_rx_addr, timer_start_addr);
  if (ret != NRF_SUCCESS) {
    return mod_rtu_error_unknown;
  }

  ret = nrf_drv_ppi_channel_assign(channel_clear, uart_rx_addr, timer_clear_addr);
  if (ret != NRF_SUCCESS) {
    return mod_rtu_error_unknown;
  }

  nrf_ppi_channel_group_t group;

  ret = nrf_drv_ppi_group_alloc(&group);
  if (ret != NRF_SUCCESS) {
    return mod_rtu_error_unknown;
  }
  me->ppi_group = group;
  

  const uint32_t group_mask = nrf_drv_ppi_channel_to_mask(channel_start) | nrf_drv_ppi_channel_to_mask(channel_clear);
  ret = nrf_drv_ppi_channels_include_in_group(group_mask, group);
  if (ret != NRF_SUCCESS) {
    return mod_rtu_error_unknown;
  }

  return mod_rtu_error_ok;
}

static mod_rtu_error_t serial_init(mod_rtu_tx_t * const me,
                           const mod_rtu_tx_serial_nrf52_config_t * const p_config)
{
    NRF_LOG_DEBUG("rtu serial_init");
    ret_code_t ret;
    ASSERT(me && p_config);

    if (me->serial_init)
    {
        /*Already initialized.*/
        return mod_rtu_error_invalid_state;
    }

    //set port
    switch (p_config->uart_instance) {
      case 0:
      me->uart = (nrf_drv_uart_t)NRF_DRV_UART_INSTANCE(0);
      break;

      case 1:
      me->uart = (nrf_drv_uart_t)NRF_DRV_UART_INSTANCE(1);
      break;

      default:
      return mod_rtu_error_parameter;
    }

    nrf_drv_uart_config_t drv_config = {
      .pseltxd = p_config->tx_pin,
      .pselrxd = p_config->rx_pin,
      .pselcts = 0,
      .pselrts = 0,
      .p_context = me,
      .hwfc = NRF_UARTE_HWFC_DISABLED,
      .parity = p_config->parity,
      .baudrate = p_config->baudrate,
      .interrupt_priority = p_config->interrupt_priority,
      .use_easy_dma = true,
    };

    ret = nrf_drv_uart_init(&(me->uart),
                            &drv_config,
                            serial_event_handler);
    if (ret != NRF_SUCCESS)
    {
        return mod_rtu_error_unknown;
    }

    //configure enable pins
    nrf_gpio_cfg_output(p_config->tx_en_pin);
    nrf_gpio_cfg_output(p_config->rx_en_pin);
    me->tx_en_pin = p_config->tx_en_pin;
    me->rx_en_pin = p_config->rx_en_pin;

    set_rx_en(me, false);
    set_tx_en(me, false);

    return mod_rtu_error_ok;
}

static void set_rx_en(mod_rtu_tx_t * const me, const bool enable) {
  if (enable) {
    nrf_gpio_pin_clear(me->rx_en_pin);
  } else {
    nrf_gpio_pin_set(me->rx_en_pin);
  }
}

static void set_tx_en(mod_rtu_tx_t * const me, const bool enable) {
  if (enable) {
    nrf_gpio_pin_set(me->tx_en_pin);
  } else {
    nrf_gpio_pin_clear(me->tx_en_pin);
  }
}

//configure system to receive first character in a message
static void receive_first_character(mod_rtu_tx_t * const me) {
  NRF_LOG_DEBUG("rtu receive_first_character");

  //Disable ppi for first character. Will start manually.
  ret_code_t ret = nrf_drv_ppi_group_disable(me->ppi_group);
  //todo: error handling?
  (void)ret;

  //stop/clear timer
  nrf_drv_timer_pause(&me->timer);
  nrf_drv_timer_clear(&me->timer);

  //set up flags for next message. No timeouts, presume good message
  me->rx_msg_ok = true; //assume ok until we get an error
  me->rx_done = false;
  me->rx_35 = false;

  //configure rx and tx enables for receive
  set_tx_en(me, false);
  set_rx_en(me, true);

  //enable receive for first character
  nrf_drv_uart_rx(&me->uart, me->rx_raw_msg.data, 1);
}

static void receive_rest_of_message(mod_rtu_tx_t * const me) {
  NRF_LOG_DEBUG("rtu receive_rest_of_message");

  //enable uart->timer reset ppi group
  ret_code_t ret = nrf_drv_ppi_group_enable(me->ppi_group);
  //todo: error handling?
  (void)ret;

  //in case of single byte frame, start timer manually as well.
  nrf_drv_timer_resume(&me->timer);

  //enable reception of remaining bytes
  nrf_drv_uart_rx(&me->uart, me->rx_raw_msg.data + 1, 255);
}

static void start_t35_for_init(mod_rtu_tx_t * const me) {
  NRF_LOG_DEBUG("rtu start_t35_for_init");
  nrf_drv_timer_pause(&me->timer);
  nrf_drv_timer_clear(&me->timer);

  //configure for receive
  set_tx_en(me, false);
  set_rx_en(me, true);

  //enable receive so rx events will happen
  receive_rest_of_message(me);
}

static void return_to_idle(mod_rtu_tx_t * const me) {
  NRF_LOG_DEBUG("return to idle");
  me->state = mod_rtu_tx_state_idle;
  receive_first_character(me);
}

static mod_rtu_error_t process_rx_message(mod_rtu_tx_raw_msg_t *const raw_msg, mod_rtu_msg_t *const out_msg) {
  if (raw_msg->data_length < MOD_RTU_TX_FRAME_OVERHEAD) {
    return mod_rtu_error_msg_frame;
  }

  //check crc
  const uint16_t check_crc = usMBCRC16(raw_msg->data, raw_msg->data_length - MOD_RTU_CRC_LENGTH);
  const uint16_t msg_crc = *(uint16_t *)(raw_msg->data + raw_msg->data_length - MOD_RTU_CRC_LENGTH);
  if (check_crc != msg_crc) {
    return mod_rtu_error_msg_crc;
  }

  //unpack message fields
  out_msg->address = raw_msg->data[MOD_RTU_ADDR_OFFSET];
  out_msg->function = raw_msg->data[MOD_RTU_FUNCTION_OFFSET];
  out_msg->data_length = raw_msg->data_length - MOD_RTU_TX_FRAME_OVERHEAD;

  NRF_LOG_DEBUG("msg: addr = %d, function = %d, data_len = %d", out_msg->address, out_msg->function, out_msg->data_length);

  memcpy(out_msg->data, raw_msg->data + MOD_RTU_DATA_OFFSET, raw_msg->data_length - MOD_RTU_TX_FRAME_OVERHEAD);

  return mod_rtu_error_ok;
}

static void finish_rx(mod_rtu_tx_t * const me) {
  me->state = mod_rtu_tx_state_ctrl_wait; //in case we weren't already there
  //todo process the just-received messaged and send to callback
  NRF_LOG_DEBUG("got message, %d bytes", me->rx_raw_msg.data_length);
  char msgout[me->rx_raw_msg.data_length*2+1];
  int idx = 0;
  for (idx = 0; idx < me->rx_raw_msg.data_length; idx++) {
    sprintf(msgout + idx * 2, "%02x", me->rx_raw_msg.data[idx]);
  }
  msgout[me->rx_raw_msg.data_length * 2] = 0;
  NRF_LOG_DEBUG("msg: %s", msgout);

  const mod_rtu_error_t msg_error = process_rx_message(&me->rx_raw_msg, &me->last_rx_msg);

  const mod_rtu_tx_event_t event = {
    .type = mod_rtu_tx_event_msg_rx,
    .error = msg_error,
    .msg_received = {&me->last_rx_msg},
  };

  if (me->callback) {
    me->callback(&event, me->callback_context);
  }

  NRF_LOG_DEBUG("msg_error: %d", msg_error);
  return_to_idle(me);
}

static void serial_event_handler(nrf_drv_uart_event_t * p_event, void * p_context) {
  NRF_LOG_DEBUG("serial event, %d", p_event->type);
  mod_rtu_tx_t *const me = p_context;
  switch (p_event->type) {
    case NRF_DRV_UART_EVT_TX_DONE: {
      if (me->state == mod_rtu_tx_state_emission) {
        NRF_LOG_DEBUG("tx done");
        return_to_idle(me);
      }
    }
    break;

    case NRF_DRV_UART_EVT_RX_DONE: {
      if (me->state == mod_rtu_tx_state_init) {
        //rx_abort is triggered by timer, transition to idle on rx_done received
        //could also get rx_done if buffer fills up, so ignore and restart if no timer flag
        me->rx_done = true;
        if (me->rx_35) {
          NRF_LOG_DEBUG("init --> idle");
          return_to_idle(me);
          if (me->callback) {
            me->callback(mod_rtu_tx_event_ready, me->callback_context);
          }
        } else {
          start_t35_for_init(me);
        }
      } else if (me->state == mod_rtu_tx_state_idle) {
        NRF_LOG_DEBUG("**1st byte**");
        //got first byte
        //start of reception, re-enable rx for remainder of message
        me->state = mod_rtu_tx_state_reception;
        receive_rest_of_message(me);
      } else if (me->state == mod_rtu_tx_state_reception ||
                 me->state == mod_rtu_tx_state_ctrl_wait) {
        //we might get rx_done in reception if the buffer fills up
        //set message length for later processing, include the additional first byte we received
        me->rx_raw_msg.data_length = p_event->data.rxtx.bytes + 1;
        me->rx_done = true;
        //events may come out of order, check if timer flag already set
        if (me->rx_35) {
          finish_rx(me);
        } else {
          me->state = mod_rtu_tx_state_ctrl_wait;
        }
      }
      else {
      }
    }
    break;

    case NRF_DRV_UART_EVT_ERROR:
    NRF_LOG_DEBUG("uart error: %d", p_event->data.error.error_mask);
    nrf_drv_uart_rx_abort(&me->uart);
    switch (me->state) {
      //if this is idle or reception, bad byte
      //is part of message. Abort and go back to idle.
      case mod_rtu_tx_state_idle:
      case mod_rtu_tx_state_reception:
      return_to_idle(me);
      break;

      default:
      break;
    }
    break;

    default:
    break;
  }
}

mod_rtu_error_t mod_rtu_tx_init(mod_rtu_tx_t *const me, const mod_rtu_tx_init_t * const init) {
  //start with all fields set to 0/null
  *me = (mod_rtu_tx_t){0};

  //initial state is init. Will start timer and wait for t35 timeout.
  me->state = mod_rtu_tx_state_init;

  //save callback from init
  me->callback = init->callback;
  me->callback_context = init->callback_context;

  //timer_init
  const mod_rtu_tx_timer_nrf52_config_t timer_config = {
    .timer_instance = 1,
  };

  timer_init(me, &timer_config);

  const mod_rtu_tx_serial_nrf52_config_t serial_config = {
    .tx_pin = MODBUS_TX_PIN,
    .rx_pin = MODBUS_RX_PIN,
    .tx_en_pin = MODBUS_TX_EN_PIN,
    .rx_en_pin = MODBUS_RX_EN_PIN,
    .parity = NRF_UARTE_PARITY_INCLUDED,
    .baudrate = NRF_UARTE_BAUDRATE_19200,
    .interrupt_priority = UART_DEFAULT_CONFIG_IRQ_PRIORITY,
    .uart_instance = 1,
  };
  serial_init(me, &serial_config);
  me->serial_init = true;

  ppi_init(me);

  //enable serial receive and start timer to transition to idle
  start_t35_for_init(me);

  return mod_rtu_error_ok;
}

mod_rtu_error_t mod_rtu_tx_send(mod_rtu_tx_t *const me, const mod_rtu_msg_t *const msg) {
  if (me->state != mod_rtu_tx_state_idle) {
    return mod_rtu_error_invalid_state;
  }
  me->state = mod_rtu_tx_state_emission;
  //this will cause a rx_done event, but it will be ignored
  nrf_drv_uart_rx_abort(&me->uart);
  NRF_LOG_DEBUG("sending");

  //pack message into raw buffer and calculate crc
  me->tx_raw_msg.data[MOD_RTU_ADDR_OFFSET] = msg->address;
  me->tx_raw_msg.data[MOD_RTU_FUNCTION_OFFSET] = msg->function;
  memcpy(me->tx_raw_msg.data + MOD_RTU_DATA_OFFSET, msg->data, msg->data_length);
  me->tx_raw_msg.data_length = msg->data_length + MOD_RTU_TX_FRAME_OVERHEAD;
  const uint16_t crc = usMBCRC16(me->tx_raw_msg.data, msg->data_length+2);
  const size_t crc_offset = me->tx_raw_msg.data_length - 2;
  me->tx_raw_msg.data[crc_offset] = crc; //low order byte
  me->tx_raw_msg.data[crc_offset + 1] = crc >> 8; //high order byte
  NRF_LOG_DEBUG("orig len %d, raw_len %d, crc %04x", msg->data_length, me->tx_raw_msg.data_length, crc);

  set_rx_en(me, false);
  set_tx_en(me, true);
  nrf_drv_uart_tx(&me->uart, me->tx_raw_msg.data, me->tx_raw_msg.data_length);
  return mod_rtu_error_ok;
}

void mod_rtu_tx_timer_expired_callback(mod_rtu_tx_t *const me, const mod_rtu_tx_timer_type_t type) {
  NRF_LOG_DEBUG("rtu timer callback, %d", type);
  switch (me->state) {
    case mod_rtu_tx_state_init:
    if (type == mod_rtu_tx_timer_type_t35) {
      me->rx_35 = true;
      if (me->rx_done) {
        return_to_idle(me);
      } else {
        nrf_drv_uart_rx_abort(&me->uart);
      }

      //todo: testing
      mod_rtu_msg_t msg;
      strcpy((char *)msg.data, "this is a test string, dummy");
      msg.data_length = strlen((char *)msg.data) - 1;
      msg.address = 1;
      msg.function = 1;

      //mod_rtu_tx_send(me, &msg);
    }
    break;

    case mod_rtu_tx_state_idle:
    case mod_rtu_tx_state_reception:
    if (type == mod_rtu_tx_timer_type_t15) {
      //event timing is a little funny with use of DMA
      //in case of garbage single character message,
      //go directly to ctrl_wait from idle if we get t15
      //before rx_done
      NRF_LOG_DEBUG("t15 in idle/reception");
      me->state = mod_rtu_tx_state_ctrl_wait;
      nrf_drv_uart_rx_abort(&me->uart);
    }
    break;

    case mod_rtu_tx_state_ctrl_wait:
    NRF_LOG_DEBUG("timer expired in ctrl_wait");
    if (type == mod_rtu_tx_timer_type_t35) {
      NRF_LOG_DEBUG("timer35 in ctrl_wait");
      me->rx_35 = true;
      if (me->rx_done) {
        finish_rx(me);
      }
    }
    break;

    case mod_rtu_tx_state_emission:
    NRF_LOG_WARNING("Timer expired in emission. Ignoring.");
    break;
  }
}

static void timer_event_handler(nrf_timer_event_t event, void * p_context) {
  mod_rtu_tx_t * const me = p_context;
  (void)me;
  switch (event) {
    case NRF_TIMER_EVENT_COMPARE0: //t1.5
    NRF_LOG_DEBUG("timer1.5");
    mod_rtu_tx_timer_expired_callback((mod_rtu_tx_t *)p_context, mod_rtu_tx_timer_type_t15);
    break;

    case NRF_TIMER_EVENT_COMPARE1: //t3.5
    NRF_LOG_DEBUG("timer3.5");
    mod_rtu_tx_timer_expired_callback((mod_rtu_tx_t *)p_context, mod_rtu_tx_timer_type_t35);
    break;

    default:
    break;
  }
}
