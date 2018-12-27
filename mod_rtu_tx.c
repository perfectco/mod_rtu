#include "mod_rtu_tx.h"

#include "nrf_drv_uart.h"

#include "boards.h"

#include "nrf_log.h"

#include <string.h>

#include "app_timer.h"

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

static void serial_event_handler(nrf_drv_uart_event_t * p_event, void * p_context);
static void reset_timer_15(mod_rtu_tx_t *const me);
static void reset_timer_20(mod_rtu_tx_t *const me);
static void reset_timer_35(mod_rtu_tx_t *const me);

static ret_code_t serial_init(mod_rtu_tx_t * const me,
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
    me->uart = p_config->uart_instance == 0 ? (nrf_drv_uart_t)NRF_DRV_UART_INSTANCE(0) : (nrf_drv_uart_t)NRF_DRV_UART_INSTANCE(1);

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
        return ret;
    }

    return NRF_SUCCESS;
}

static void serial_event_handler(nrf_drv_uart_event_t * p_event, void * p_context) {
  NRF_LOG_INFO("serial event, %d", p_event->type);
  mod_rtu_tx_t *const me = p_context;
  if (p_event->type == NRF_DRV_UART_EVT_TX_DONE) {
    if (me->state == mod_rtu_tx_state_emission) {
      NRF_LOG_INFO("tx done, --> reception");
      reset_timer_15(me);
      me->state = mod_rtu_tx_state_reception;
    }
  }
}

static void reset_timer_15(mod_rtu_tx_t *const me) {
  app_timer_stop(&me->timer);
  app_timer_start(&me->timer, APP_TIMER_TICKS((T15_US + 500)/1000), me);
}

static void reset_timer_20(mod_rtu_tx_t *const me) {
  app_timer_stop(&me->timer);
  app_timer_start(&me->timer, APP_TIMER_TICKS((T35_US - T15_US + 500)/1000), me);
}

static void reset_timer_35(mod_rtu_tx_t *const me) {
  app_timer_stop(&me->timer);
  app_timer_start(&me->timer, APP_TIMER_TICKS((T35_US + 500)/1000), me);
}

static void timer_timeout_handler(void * p_context) {
  mod_rtu_tx_t * me = p_context;
  if (me->state == mod_rtu_tx_state_reception) {
    NRF_LOG_INFO("reception, timer1.5");
    reset_timer_20(me);
    mod_rtu_tx_timer_expired_callback((mod_rtu_tx_t *)p_context, mod_rtu_tx_timer_type_t15);
  } else if (me->state == mod_rtu_tx_state_ctrl_wait) {
    NRF_LOG_INFO("ctrl_wait, timer3.5");
    mod_rtu_tx_timer_expired_callback((mod_rtu_tx_t *)p_context, mod_rtu_tx_timer_type_t35);
  } else if (me->state == mod_rtu_tx_state_init) {
    NRF_LOG_INFO("init_wait, timer3.5");
    mod_rtu_tx_timer_expired_callback((mod_rtu_tx_t *)p_context, mod_rtu_tx_timer_type_t35);
  }
}

mod_rtu_error_t mod_rtu_tx_init(mod_rtu_tx_t *const me) {
  *me = (mod_rtu_tx_t){0};
  me->state = mod_rtu_tx_state_init;
  mod_rtu_tx_serial_nrf52_config_t serial_config = {
    .tx_pin = MODBUS_TX_PIN,
    .rx_pin = MODBUS_RX_PIN,
    .parity = NRF_UARTE_PARITY_INCLUDED,
    .baudrate = NRF_UARTE_BAUDRATE_19200,
    .interrupt_priority = UART_DEFAULT_CONFIG_IRQ_PRIORITY,
    .uart_instance = 1,
  };
  serial_init(me, &serial_config);
  me->serial_init = true;

  app_timer_t * temp_handle = &me->timer;

  app_timer_create(&temp_handle, APP_TIMER_MODE_SINGLE_SHOT, timer_timeout_handler);

  //initial reset timer
  reset_timer_35(me);

  return mod_rtu_error_ok;
}

mod_rtu_error_t mod_rtu_tx_send(mod_rtu_tx_t *const me, const mod_rtu_slave_addr_t address, const mod_rtu_msg_t *const msg) {
  if (me->state != mod_rtu_tx_state_idle) {
    return mod_rtu_error_invalid_state;
  }
  NRF_LOG_INFO("sending");
  memcpy(me->tx_buf.buf, msg->buf, msg->length);
  me->tx_buf.length = msg->length;

  me->state = mod_rtu_tx_state_emission;

  nrf_drv_uart_tx(&(me->uart), me->tx_buf.buf, me->tx_buf.length);
  return mod_rtu_error_ok;
}

void mod_rtu_tx_timer_expired_callback(mod_rtu_tx_t *const me, const mod_rtu_tx_timer_type_t type) {
  switch (me->state) {
    case mod_rtu_tx_state_init:
    NRF_LOG_INFO("init --> idle");
    me->state = mod_rtu_tx_state_idle;

    //todo: testing
    mod_rtu_msg_t msg;
    strcpy((char *)msg.buf, "this is a test string, dummy");
    msg.length = strlen((char *)msg.buf);

    mod_rtu_tx_send(me, 0, &msg);
    break;

    case mod_rtu_tx_state_idle:
    NRF_LOG_WARNING("timer expired in idle");
    break;

    case mod_rtu_tx_state_emission:
    NRF_LOG_WARNING("timer expired in idle");
    break;

    case mod_rtu_tx_state_reception:
    NRF_LOG_INFO("reception --> ctrl_wait");
    me->state = mod_rtu_tx_state_ctrl_wait;
    break;

    case mod_rtu_tx_state_ctrl_wait:
    NRF_LOG_INFO("ctrl_wait --> idle");
    me->state = mod_rtu_tx_state_idle;
    break;
  }
}

