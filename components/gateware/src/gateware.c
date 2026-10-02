#include <radioberry/gateware.h>

#include <string.h>

#define RESET_HOLD_US       1000u
#define STATUS_POLL_US      10000u
#define STATUS_POLL_LIMIT   255u
#define CLOCK_DELAY_US      1u
#define FINAL_CLOCK_COUNT   2u

static void delay(fpga_loader_t *loader, uint32_t us)
{
    loader->io.delay_us(loader->io.user, us);
}

static void clock_pulse(fpga_loader_t *loader)
{
    loader->io.set_dclk(loader->io.user, true);
    delay(loader, CLOCK_DELAY_US);
    loader->io.set_dclk(loader->io.user, false);
    delay(loader, CLOCK_DELAY_US);
}

bool fpga_loader_init(fpga_loader_t *loader, const fpga_loader_io_t *io)
{
    if (loader == NULL || io == NULL || io->set_nconfig == NULL ||
        io->set_data == NULL || io->set_dclk == NULL ||
        io->get_nstatus == NULL || io->get_conf_done == NULL ||
        io->delay_us == NULL) {
        return false;
    }
    memset(loader, 0, sizeof(*loader));
    loader->io = *io;
    if (loader->io.initialize != NULL) {
        loader->io.initialize(loader->io.user);
    }
    return true;
}

fpga_loader_result_t fpga_loader_program(fpga_loader_t *loader,
                                         const uint8_t *bitstream,
                                         size_t bitstream_size)
{
    return fpga_loader_program_progress(loader, bitstream, bitstream_size,
                                        NULL, NULL);
}

fpga_loader_result_t fpga_loader_program_progress(fpga_loader_t *loader,
                                                  const uint8_t *bitstream,
                                                  size_t bitstream_size,
                                                  fpga_loader_progress_cb_t progress,
                                                  void *progress_user)
{
    if (loader == NULL || bitstream == NULL || bitstream_size == 0u) {
        return FPGA_LOADER_ERROR_ARGUMENT;
    }

    loader->io.set_nconfig(loader->io.user, false);
    loader->io.set_data(loader->io.user, false);
    loader->io.set_dclk(loader->io.user, false);
    delay(loader, RESET_HOLD_US);
    loader->io.set_nconfig(loader->io.user, true);

    bool ready = false;
    for (uint32_t attempt = 0; attempt < STATUS_POLL_LIMIT; ++attempt) {
        if (loader->io.get_nstatus(loader->io.user)) {
            ready = true;
            break;
        }
        delay(loader, STATUS_POLL_US);
    }
    if (!ready) return FPGA_LOADER_ERROR_PREPARE;

    if (progress) progress(0u, bitstream_size, progress_user);
    size_t next_progress = bitstream_size / 20u;
    if (next_progress < 1u) next_progress = 1u;
    size_t progress_mark = next_progress;

    for (size_t index = 0; index < bitstream_size; ++index) {
        uint8_t value = bitstream[index];
        for (uint32_t bit = 0; bit < 8u; ++bit) {
            loader->io.set_data(loader->io.user,
                                ((value >> bit) & 1u) != 0u);
            delay(loader, CLOCK_DELAY_US);
            clock_pulse(loader);
        }
        if (progress && (index + 1u >= progress_mark || index + 1u == bitstream_size)) {
            progress(index + 1u, bitstream_size, progress_user);
            progress_mark += next_progress;
        }
    }

    if (!loader->io.get_nstatus(loader->io.user)) return FPGA_LOADER_ERROR_NSTATUS;
    if (!loader->io.get_conf_done(loader->io.user)) return FPGA_LOADER_ERROR_CONF_DONE;
    for (uint32_t count = 0; count < FINAL_CLOCK_COUNT; ++count) clock_pulse(loader);
    return FPGA_LOADER_OK;
}
