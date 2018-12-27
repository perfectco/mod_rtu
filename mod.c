#include "mod.h"

//nrf52840 port
#include "nrf_serial.h"
#include "drv_timer.h"

static const nrf_drv_uart_config_t m_uart0_drv_config = {
    .pselrxd = MODBUS_RX_PIN,
    .pseltxd = MODBUS_TX_PIN,
    .pselrts = 0,
    .pselcts = 0,
    .hwfc = NRF_UART_HWFC_DISABLED,
    .parity = NRF_UART_PARITY_EXCLUDED,
    .baudrate = NRF_UART_BAUDRATE_19200,
    .interrupt_priority = UART_DEFAULT_CONFIG_IRQ_PRIORITY,
};



/*
Initialize the serial driver and 


/*
Initialize modbus library
*/
mod_error_t mod_platform_init(mod_t *const me, const mod_init_t *const init) {

}
