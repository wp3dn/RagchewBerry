#include "radio_spi.h"

// SPI Controller configuration:
#define SPI_PORT spi1
#define SPI_CLK  10     

//GPIO config for SPI1:
#define PIN_SCK 10
#define PIN_MOSI 11     //TX
#define PIN_MISO 12     //RX
#define PIN_CS 8

void radioberry_spi_init(void) {
   uint actual_spi = spi_init(SPI_PORT, SPI_CLK * 1000 * 1000);

    printf("SPI requested: %d MHz\r\n", SPI_CLK);
    printf("SPI actual: %u Hz\r\n", actual_spi);

    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    // SPI mode 3: CPOL=1, CPHA=1
    spi_set_format(
        SPI_PORT,
        8,
        SPI_CPOL_1,
        SPI_CPHA_1,
        SPI_MSB_FIRST
    );

}

int radioberry_spi_write_read_blocking(const uint8_t *src, uint8_t *dst, size_t len){
    gpio_put(PIN_CS, 0);
    int result = spi_write_read_blocking(SPI_PORT, src, dst, len);
    gpio_put(PIN_CS, 1);
    return result;
}

//  end of source file radioberry_spi.c
