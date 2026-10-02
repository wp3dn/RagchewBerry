#ifndef RADIOBERRY_DSP_H
#define RADIOBERRY_DSP_H

#include <stddef.h>

typedef struct { float i; float q; } rb_iq_f32_t;

typedef struct {
    rb_iq_f32_t *samples;
    size_t count;
    unsigned sample_rate;
} rb_iq_block_t;

typedef struct {
    void *context;
    void (*reset)(void *context);
    size_t (*process)(void *context,
                      const rb_iq_block_t *input,
                      rb_iq_block_t *output);
} rb_dsp_stage_t;

/* In-place radix-2 FFT. Count must be a non-zero power of two. */
int rb_fft_forward(rb_iq_f32_t *samples, size_t count);
int rb_fft_inverse(rb_iq_f32_t *samples, size_t count);

#endif /* RADIOBERRY_DSP_H */
