# WP3DN reusable RP2350 PIO-USB headset module.
#
# Parent project may set:
#   PICO_PIO_USB_PATH=/path/to/Pico-PIO-USB
#   USB_HEADSET_TUSB_CONFIG_DIR=/path/containing/tusb_config.h
#
# The upstream Pico-PIO-USB clone is NEVER modified.  It is copied into the
# build directory and the proven WP3DN UAC1 ISO fixes are applied there.

if(NOT DEFINED PICO_PIO_USB_PATH)
    set(PICO_PIO_USB_PATH ${CMAKE_CURRENT_LIST_DIR}/../lib/Pico-PIO-USB)
endif()

if(NOT EXISTS ${PICO_PIO_USB_PATH}/CMakeLists.txt)
    message(FATAL_ERROR
        "Pico-PIO-USB not found at ${PICO_PIO_USB_PATH}.\n"
        "Set PICO_PIO_USB_PATH when configuring CMake.")
endif()

if(NOT DEFINED USB_HEADSET_TUSB_CONFIG_DIR)
    set(USB_HEADSET_TUSB_CONFIG_DIR ${CMAKE_CURRENT_LIST_DIR}/config)
endif()

if(NOT EXISTS ${USB_HEADSET_TUSB_CONFIG_DIR}/tusb_config.h)
    message(FATAL_ERROR
        "No tusb_config.h in USB_HEADSET_TUSB_CONFIG_DIR=${USB_HEADSET_TUSB_CONFIG_DIR}")
endif()

find_package(Python3 COMPONENTS Interpreter REQUIRED)

if(NOT TARGET pico_pio_usb)
    set(WP3DN_PIO_USB_LOCAL ${CMAKE_BINARY_DIR}/_wp3dn_pico_pio_usb)
    file(REMOVE_RECURSE ${WP3DN_PIO_USB_LOCAL})
    file(COPY ${PICO_PIO_USB_PATH}/ DESTINATION ${WP3DN_PIO_USB_LOCAL})

    execute_process(
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_LIST_DIR}/cmake/integrate_iso_audio.py
                ${WP3DN_PIO_USB_LOCAL}
        RESULT_VARIABLE WP3DN_ISO_FIX_RESULT
        OUTPUT_VARIABLE WP3DN_ISO_FIX_OUT
        ERROR_VARIABLE WP3DN_ISO_FIX_ERR
    )

    message(STATUS "${WP3DN_ISO_FIX_OUT}")

    if(NOT WP3DN_ISO_FIX_RESULT EQUAL 0)
        message(FATAL_ERROR
            "Could not integrate WP3DN ISO audio support:\n${WP3DN_ISO_FIX_ERR}")
    endif()

    add_subdirectory(${WP3DN_PIO_USB_LOCAL} wp3dn_pico_pio_usb_build)
endif()

add_library(wp3dn_usb_headset
    src/usb_headset.c
    ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
)

target_include_directories(wp3dn_usb_headset PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${USB_HEADSET_TUSB_CONFIG_DIR}
)

target_compile_definitions(wp3dn_usb_headset PUBLIC
    PIO_USB_USE_TINYUSB
)

target_compile_options(wp3dn_usb_headset PRIVATE
    -Wall
    -Wextra
)

target_link_libraries(wp3dn_usb_headset PUBLIC
    pico_stdlib
    pico_sync
    pico_pio_usb
    tinyusb_host
)
