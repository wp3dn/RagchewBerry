cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350)

include(pico_sdk_import.cmake)
project(radioberrycore_local C CXX ASM)

set(RADIOBERRY_FPGA_VARIANT "CL025" CACHE STRING
    "Cyclone 10 LP FPGA variant (CL016 or CL025)")
set_property(CACHE RADIOBERRY_FPGA_VARIANT PROPERTY STRINGS CL016 CL025)

pico_sdk_init()

add_compile_options(
    -Wall
    -Wno-format
    -Wno-unused-function
    -Wno-maybe-uninitialized
)

add_subdirectory(components/gateware)
add_subdirectory(components/radio)
add_subdirectory(components/dsp)

set(USB_HEADSET_TUSB_CONFIG_DIR
    ${CMAKE_CURRENT_LIST_DIR}/usb_headset/config)
add_subdirectory(usb_headset)

add_executable(radioberrycore_local
    applications/radioberry_local/main.c
    applications/radioberry_local/remote_ui.c
    applications/radioberry_local/settings_store.c
    applications/radioberry_local/usb_descriptors.c
    applications/radioberry_local/p1/p1_protocol.c
    applications/radioberry_local/p1/p1_network.c
    applications/radioberry_local/p1/usb_network.c
    applications/radioberry_local/p1/dhcpserver/dhcpserver.c
    ${PICO_TINYUSB_PATH}/lib/networking/rndis_reports.c
)

target_include_directories(radioberrycore_local PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/applications/radioberry_local
    ${CMAKE_CURRENT_LIST_DIR}/applications/radioberry_local/p1
    ${PICO_TINYUSB_PATH}/lib/networking
)

target_link_libraries(radioberrycore_local PRIVATE
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_pio
    hardware_dma
    hardware_irq
    hardware_sync
    hardware_flash
    hardware_i2c
    hardware_uart
    tinyusb_device
    pico_lwip
    pico_lwip_nosys
    pico_unique_id
    radioberry::gateware
    radioberry::gateware_image
    radioberry::radio
    radioberry::dsp
    wp3dn_usb_headset
)

pico_enable_stdio_uart(radioberrycore_local 0)
pico_enable_stdio_usb(radioberrycore_local 0)

pico_set_program_name(radioberrycore_local "WP3DN RadioberryCore Local")
pico_set_program_version(radioberrycore_local "0.8-port2")
pico_add_extra_outputs(radioberrycore_local)
