add_library(rb_gateware STATIC
    src/gateware.c
    platform/pico/gateware_pico.c)
add_library(radioberry::gateware ALIAS rb_gateware)
target_include_directories(rb_gateware PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/platform/pico)
target_link_libraries(rb_gateware PUBLIC pico_stdlib)

string(TOUPPER "${RADIOBERRY_FPGA_VARIANT}" RADIOBERRY_FPGA_VARIANT_NORMALIZED)
if(NOT RADIOBERRY_FPGA_VARIANT_NORMALIZED MATCHES "^(CL016|CL025)$")
    message(FATAL_ERROR
        "Invalid RADIOBERRY_FPGA_VARIANT='${RADIOBERRY_FPGA_VARIANT}'. "
        "Choose CL016 or CL025.")
endif()

set(RADIOBERRY_RBF_INPUT
    ${CMAKE_CURRENT_LIST_DIR}/bitstreams/${RADIOBERRY_FPGA_VARIANT_NORMALIZED}/radioberry.rbf)
if(NOT EXISTS ${RADIOBERRY_RBF_INPUT})
    message(FATAL_ERROR
        "No gateware for ${RADIOBERRY_FPGA_VARIANT_NORMALIZED}: "
        "${RADIOBERRY_RBF_INPUT}")
endif()

set(RADIOBERRY_RBF_GENERATED_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/generated/${RADIOBERRY_FPGA_VARIANT_NORMALIZED})
set(RADIOBERRY_RBF_SOURCE ${RADIOBERRY_RBF_GENERATED_DIR}/radioberry_rbf.c)
set(RADIOBERRY_RBF_HEADER ${RADIOBERRY_RBF_GENERATED_DIR}/radioberry_rbf.h)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

add_custom_command(
    OUTPUT ${RADIOBERRY_RBF_SOURCE} ${RADIOBERRY_RBF_HEADER}
    COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_SOURCE_DIR}/cmake/embed_binary_as_c.py
        --input ${RADIOBERRY_RBF_INPUT}
        --out-c ${RADIOBERRY_RBF_SOURCE}
        --out-h ${RADIOBERRY_RBF_HEADER}
        --symbol radioberry_rbf
    DEPENDS ${RADIOBERRY_RBF_INPUT}
            ${CMAKE_SOURCE_DIR}/cmake/embed_binary_as_c.py
    COMMENT "Embedding ${RADIOBERRY_FPGA_VARIANT_NORMALIZED} gateware"
    VERBATIM)

add_library(rb_gateware_image STATIC ${RADIOBERRY_RBF_SOURCE})
add_library(radioberry::gateware_image ALIAS rb_gateware_image)
target_include_directories(rb_gateware_image PUBLIC ${RADIOBERRY_RBF_GENERATED_DIR})
target_compile_definitions(rb_gateware_image PUBLIC
    RADIOBERRY_FPGA_VARIANT_${RADIOBERRY_FPGA_VARIANT_NORMALIZED}=1)
