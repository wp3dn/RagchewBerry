#include "gateware_pico.h"

#include "pico/stdlib.h"

#define PIN_CONF_DONE 22u
#define PIN_NSTATUS   26u
#define PIN_NCONFIG   27u
#define PIN_DATA      13u
#define PIN_DCLK      24u

static void initialize(void *user)
{
    (void)user;
    gpio_init(PIN_CONF_DONE); gpio_set_dir(PIN_CONF_DONE, GPIO_IN);
    gpio_init(PIN_NSTATUS); gpio_set_dir(PIN_NSTATUS, GPIO_IN);
    gpio_init(PIN_NCONFIG); gpio_set_dir(PIN_NCONFIG, GPIO_OUT);
    gpio_init(PIN_DATA); gpio_set_dir(PIN_DATA, GPIO_OUT);
    gpio_init(PIN_DCLK); gpio_set_dir(PIN_DCLK, GPIO_OUT);
}
static void nconfig(void *u, bool v) { (void)u; gpio_put(PIN_NCONFIG, v); }
static void data(void *u, bool v) { (void)u; gpio_put(PIN_DATA, v); }
static void dclk(void *u, bool v) { (void)u; gpio_put(PIN_DCLK, v); }
static bool nstatus(void *u) { (void)u; return gpio_get(PIN_NSTATUS); }
static bool conf_done(void *u) { (void)u; return gpio_get(PIN_CONF_DONE); }
static void wait_us(void *u, uint32_t us) { (void)u; sleep_us(us); }

fpga_loader_io_t fpga_loader_pico_io(void)
{
    return (fpga_loader_io_t){
        .initialize = initialize, .set_nconfig = nconfig, .set_data = data,
        .set_dclk = dclk, .get_nstatus = nstatus, .get_conf_done = conf_done,
        .delay_us = wait_us
    };
}
