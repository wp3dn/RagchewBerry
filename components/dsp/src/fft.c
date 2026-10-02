#include <radioberry/dsp.h>

#include <math.h>

#define RB_PI 3.14159265358979323846f

static int transform(rb_iq_f32_t *x, size_t n, int inverse)
{
    if (x == NULL || n == 0u || (n & (n - 1u)) != 0u) return -1;
    for (size_t i = 1u, j = 0u; i < n; ++i) {
        size_t bit = n >> 1u;
        for (; (j & bit) != 0u; bit >>= 1u) j ^= bit;
        j ^= bit;
        if (i < j) { rb_iq_f32_t t = x[i]; x[i] = x[j]; x[j] = t; }
    }
    for (size_t length = 2u; length <= n; length <<= 1u) {
        float angle = (inverse ? 2.0f : -2.0f) * RB_PI / (float)length;
        rb_iq_f32_t step = {cosf(angle), sinf(angle)};
        for (size_t base = 0u; base < n; base += length) {
            rb_iq_f32_t w = {1.0f, 0.0f};
            for (size_t k = 0u; k < length / 2u; ++k) {
                rb_iq_f32_t even = x[base + k];
                rb_iq_f32_t odd = x[base + k + length / 2u];
                rb_iq_f32_t v = {odd.i * w.i - odd.q * w.q,
                                 odd.i * w.q + odd.q * w.i};
                x[base + k] = (rb_iq_f32_t){even.i + v.i, even.q + v.q};
                x[base + k + length / 2u] =
                    (rb_iq_f32_t){even.i - v.i, even.q - v.q};
                w = (rb_iq_f32_t){w.i * step.i - w.q * step.q,
                                  w.i * step.q + w.q * step.i};
            }
        }
    }
    if (inverse) {
        for (size_t i = 0u; i < n; ++i) { x[i].i /= n; x[i].q /= n; }
    }
    return 0;
}

int rb_fft_forward(rb_iq_f32_t *samples, size_t count)
{ return transform(samples, count, 0); }
int rb_fft_inverse(rb_iq_f32_t *samples, size_t count)
{ return transform(samples, count, 1); }
