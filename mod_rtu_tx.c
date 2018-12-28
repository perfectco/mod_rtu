#include "mod_rtu_tx.h"

#include "nrf_drv_uart.h"

#include "boards.h"
#define NRF_LOG_MODULE_NAME mod_rtu
#define NRF_LOG_LEVEL 4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include <string.h>

#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"

#define T15_US 1432
#define T35_US 2578

typedef struct mod_rtu_tx_serial_nrf52_config_s {
  uint32_t tx_pin;
  uint32_t rx_pin;
  nrf_uart_parity_t parity;
  nrf_uart_baudrate_t baudrate;
  uint8_t interrupt_priority;
  uint8_t uart_instance;
} mod_rtu_tx_serial_nrf52_config_t;

typedef struct mod_rtu_tx_timer_nrf52_config_s {
  uint8_t timer_instance;
} mod_rtu_tx_timer_nrf52_config_t;

typedef struct mod_rtu_tx_gpio_nrf52_config_s {
  uint16_t tx_en_pin;
  uint16_t rx_en_pin;
} mod_rtu_tx_gpio_nrf52_config_t;

static void serial_event_handler(nrf_drv_uart_event_t * p_event, void * p_context);
static void timer_event_handler(nrf_timer_event_t p_event, void * p_context);

static mod_rtu_error_t timer_init(mod_rtu_tx_t * const me, const mod_rtu_tx_timer_nrf52_config_t * const p_config) {
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

  nrf_drv_timer_enable(&me->timer);

  //setup compare register for T1.5
  const uint32_t compare15 = nrf_drv_timer_us_to_ticks(&me->timer, T15_US);
  nrf_drv_timer_compare(&me->timer, 0, compare15, true);

  //setup compare register for T3.5
  const uint32_t compare35 = nrf_drv_timer_us_to_ticks(&me->timer, T35_US);
  nrf_drv_timer_compare(&me->timer, 1, compare35, true);


  return mod_rtu_error_ok;
}

static mod_rtu_error_t ppi_init(mod_rtu_tx_t *const me) {
  //set up ppi channel for uart rx to start/reset timer
  const uint32_t timer_start_addr = nrf_drv_timer_task_address_get(&me->timer, NRF_TIMER_TASK_START);
  const uint32_t timer_clear_addr = nrf_drv_timer_task_address_get(&me->timer, NRF_TIMER_TASK_CLEAR);

  const uint32_t uart_rx_addr = nrf_drv_uart_event_address_get(&me->uart, NRF_UART_EVENT_RXDRDY);
  
  //ppi_init returns either success or already initialized, so ignore either way
  (void)nrf_drv_ppi_init();

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

  const uint32_t group_mask = nrf_drv_ppi_channel_to_mask(channel_start) | nrf_drv_ppi_channel_to_mask(channel_clear);
  ret = nrf_drv_ppi_channels_include_in_group(group_mask, group);
  if (ret != NRF_SUCCESS) {
    return mod_rtu_error_unknown;
  }

  ret = nrf_drv_ppi_group_enable(group);
  if (ret != NRF_SUCCESS) {
    return mod_rtu_error_unknown;
  }

  return mod_rtu_error_ok;
}

static mod_rtu_error_t serial_init(mod_rtu_tx_t * const me,
                           const mod_rtu_tx_serial_nrf52_config_t * const p_config)
{
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

static void gpio_init(mod_rtu_tx_t * const me,
                           const mod_rtu_tx_gpio_nrf52_config_t * const p_config) {
  nrf_gpio_cfg_output(p_config->tx_en_pin);
  nrf_gpio_cfg_output(p_config->rx_en_pin);
  me->tx_en_pin = p_config->tx_en_pin;
  me->rx_en_pin = p_config->rx_en_pin;
  set_rx_en(me, false);
  set_tx_en(me, false);
}

static void return_to_idle(mod_rtu_tx_t * const me) {
  //process message here!
  NRF_LOG_DEBUG("return to idle");
  me->state = mod_rtu_tx_state_idle;
  me->rx_msg_ok = true; //assume ok until we get an error
  me->rx_done = false;
  me->rx_35 = false;
  set_tx_en(me, false);
  set_rx_en(me, true);
  nrf_drv_uart_rx(&me->uart, me->rx_buf.buf, 255);
}

static void finish_rx(mod_rtu_tx_t * const me) {
  me->state = mod_rtu_tx_state_ctrl_wait; //in case we weren't already there
  //todo process the just-received messaged and send to callback
  return_to_idle(me);
}

static void serial_event_handler(nrf_drv_uart_event_t * p_event, void * p_context) {
  NRF_LOG_INFO("serial event, %d", p_event->type);
  mod_rtu_tx_t *const me = p_context;
  switch (p_event->type) {
    case NRF_DRV_UART_EVT_TX_DONE: {
      if (me->state == mod_rtu_tx_state_emission) {
        NRF_LOG_INFO("tx done");
        return_to_idle(me);
      }
    }
    break;

    case NRF_DRV_UART_EVT_RX_DONE: {
      //depending on how the timing works out, we
      //might finish in idle, reception, or ctrl_wait
      if (me->state == mod_rtu_tx_state_idle ||
          me->state == mod_rtu_tx_state_reception ||
          me->state == mod_rtu_tx_state_ctrl_wait) {
        //how long was the received data?
        me->rx_buf.length = p_event->data.rxtx.bytes;
        NRF_LOG_INFO("got message, %d bytes", me->rx_buf.length);
        me->rx_done = true;
        if (me->rx_35) {
          finish_rx(me);
        }
      }
    }
    break;

    case NRF_DRV_UART_EVT_ERROR:
    NRF_LOG_DEBUG("uart error: %d", p_event->data.error.error_mask);
    nrf_drv_uart_rx_abort(&me->uart);
    return_to_idle(me);
    break;

    default:
    break;
  }
}

mod_rtu_error_t mod_rtu_tx_init(mod_rtu_tx_t *const me) {
  *me = (mod_rtu_tx_t){0};
  me->state = mod_rtu_tx_state_init;

  //timer_init
  const mod_rtu_tx_timer_nrf52_config_t timer_config = {
    .timer_instance = 1,
  };

  timer_init(me, &timer_config);

  const mod_rtu_tx_serial_nrf52_config_t serial_config = {
    .tx_pin = MODBUS_TX_PIN,
    .rx_pin = MODBUS_RX_PIN,
    .parity = NRF_UARTE_PARITY_INCLUDED,
    .baudrate = NRF_UARTE_BAUDRATE_19200,
    .interrupt_priority = UART_DEFAULT_CONFIG_IRQ_PRIORITY,
    .uart_instance = 1,
  };
  serial_init(me, &serial_config);
  me->serial_init = true;

  ppi_init(me);

  const mod_rtu_tx_gpio_nrf52_config_t gpio_config = {
    .tx_en_pin = MODBUS_TX_EN_PIN,
    .rx_en_pin = MODBUS_RX_EN_PIN,
  };

  gpio_init(me, &gpio_config);

  return mod_rtu_error_ok;
}

mod_rtu_error_t mod_rtu_tx_send(mod_rtu_tx_t *const me, const mod_rtu_slave_addr_t address, const mod_rtu_msg_t *const msg) {
  if (me->state != mod_rtu_tx_state_idle) {
    return mod_rtu_error_invalid_state;
  }
  me->state = mod_rtu_tx_state_emission;
  //this will cause a rx_done event, but it will be ignored
  nrf_drv_uart_rx_abort(&me->uart);
  NRF_LOG_INFO("sending");
  memcpy(me->tx_buf.buf, msg->buf, msg->length);
  me->tx_buf.length = msg->length;

  set_rx_en(me, false);
  set_tx_en(me, true);
  nrf_drv_uart_tx(&(me->uart), me->tx_buf.buf, me->tx_buf.length);
  return mod_rtu_error_ok;
}

void mod_rtu_tx_timer_expired_callback(mod_rtu_tx_t *const me, const mod_rtu_tx_timer_type_t type) {
  switch (me->state) {
    case mod_rtu_tx_state_init:
    if (type == mod_rtu_tx_timer_type_t35) {
      NRF_LOG_INFO("init --> idle");
      return_to_idle(me);

      //todo: testing
      mod_rtu_msg_t msg;
      strcpy((char *)msg.buf, "this is a test string, dummy");
      msg.length = strlen((char *)msg.buf);

      mod_rtu_tx_send(me, 0, &msg);
    }
    break;

    case mod_rtu_tx_state_idle:
    case mod_rtu_tx_state_reception:
    if (type == mod_rtu_tx_timer_type_t15) {
      NRF_LOG_DEBUG("t15 in idle/reception");
      me->state = mod_rtu_tx_state_ctrl_wait;
    }
    break;

    case mod_rtu_tx_state_ctrl_wait:
    if (type == mod_rtu_tx_timer_type_t35) {
      NRF_LOG_DEBUG("timer35 in ctrl_wait");
      me->rx_35 = true;
      if (me->rx_done) {
        finish_rx(me);
      } else {
        nrf_drv_uart_rx_abort(&me->uart);
      }
    }
    NRF_LOG_DEBUG("timer expired in idle/reception");
    break;

    case mod_rtu_tx_state_emission:
    NRF_LOG_WARNING("timer expired in idle");
    break;
  }
}

static void timer_event_handler(nrf_timer_event_t event, void * p_context) {
  mod_rtu_tx_t * const me = p_context;
  (void)me;
  switch (event) {
    case NRF_TIMER_EVENT_COMPARE0: //t1.5
    NRF_LOG_INFO("timer1.5");
    mod_rtu_tx_timer_expired_callback((mod_rtu_tx_t *)p_context, mod_rtu_tx_timer_type_t15);
    break;

    case NRF_TIMER_EVENT_COMPARE1: //t3.5
    NRF_LOG_INFO("timer3.5");
    mod_rtu_tx_timer_expired_callback((mod_rtu_tx_t *)p_context, mod_rtu_tx_timer_type_t35);
    nrf_drv_timer_pause(&me->timer);
    nrf_drv_timer_clear(&me->timer);
    break;

    default:
    break;
  }
}
