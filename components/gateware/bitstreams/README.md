# RadioBerry gateware images

Each assembled FPGA has its own image:

- `CL016/radioberry.rbf`
- `CL025/radioberry.rbf`

Select it with the CMake cache variable `RADIOBERRY_FPGA_VARIANT`. These images
are hardware-specific and must not be interchanged.


## v0.7.9 Velvet Spartan edge-only test

Based on v0.7.8. Restores the Ragchew Machine fractional FFT-bin overlap rule, but computes weights only for the small passband candidate range and multiplies only the fractional boundary bins. TUNE remains free-running and TX monitor RF attenuation remains unchanged.
