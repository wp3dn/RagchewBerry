# CL025 gateware

The `radioberry.rbf` in this directory is for a RadioBerry assembled with the
Cyclone 10 LP 10CL025 FPGA. Do not replace it with the CL016 image.


## v0.7.9 Velvet Spartan edge-only test

Based on v0.7.8. Restores the Ragchew Machine fractional FFT-bin overlap rule, but computes weights only for the small passband candidate range and multiplies only the fractional boundary bins. TUNE remains free-running and TX monitor RF attenuation remains unchanged.
