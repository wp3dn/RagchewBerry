add_library(rb_dsp STATIC src/fft.c)
add_library(radioberry::dsp ALIAS rb_dsp)
target_include_directories(rb_dsp PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(rb_dsp PUBLIC m)
