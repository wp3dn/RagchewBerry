#ifndef RADIOBERRY_GATEWARE_H
#define RADIOBERRY_GATEWARE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    FPGA_LOADER_OK = 0,
    FPGA_LOADER_ERROR_ARGUMENT = -1,
    FPGA_LOADER_ERROR_PREPARE = -2,
    FPGA_LOADER_ERROR_NSTATUS = -3,
    FPGA_LOADER_ERROR_CONF_DONE = -4
} fpga_loader_result_t;

typedef struct {
    void *user;
    void (*initialize)(void *user);
    void (*set_nconfig)(void *user, bool high);
    void (*set_data)(void *user, bool high);
    void (*set_dclk)(void *user, bool high);
    bool (*get_nstatus)(void *user);
    bool (*get_conf_done)(void *user);
    void (*delay_us)(void *user, uint32_t microseconds);
} fpga_loader_io_t;

typedef struct {
    fpga_loader_io_t io;
} fpga_loader_t;

bool fpga_loader_init(fpga_loader_t *loader, const fpga_loader_io_t *io);
fpga_loader_result_t fpga_loader_program(fpga_loader_t *loader,
                                         const uint8_t *bitstream,
                                         size_t bitstream_size);

typedef void (*fpga_loader_progress_cb_t)(size_t done, size_t total, void *user);

fpga_loader_result_t fpga_loader_program_progress(fpga_loader_t *loader,
                                                  const uint8_t *bitstream,
                                                  size_t bitstream_size,
                                                  fpga_loader_progress_cb_t progress,
                                                  void *progress_user);

#endif /* RADIOBERRY_GATEWARE_H */
