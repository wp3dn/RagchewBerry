add_library(rb_radio STATIC
    src/radio.c src/radio_spi.c
    stream_rx/stream_rx_dma.c stream_tx/stream_tx_dma.c)
add_library(radioberry::radio ALIAS rb_radio)
pico_generate_pio_header(rb_radio
    ${CMAKE_CURRENT_LIST_DIR}/pio/rb-rx.pio
    ${CMAKE_CURRENT_LIST_DIR}/pio/rb-tx.pio)
target_include_directories(rb_radio PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/stream_rx
    ${CMAKE_CURRENT_LIST_DIR}/stream_tx)
target_include_directories(rb_radio PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)
target_link_libraries(rb_radio PUBLIC pico_stdlib hardware_spi hardware_pio
    hardware_dma hardware_irq hardware_sync)
