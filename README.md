RagchewBerry

Standalone RP2350 + Radioberry HF SDR Transceiver

RagchewBerry is an experimental standalone HF SDR transceiver built
around the Radioberry Board and a Waveshare RP2350 pi zero.

The RP2350 performs the local receive and transmit DSP, hosts a USB
audio headset directly, controls the Radioberry and communicates
with a separate waveshare RP2350 zero front-panel controller.

In normal LOCAL operation, RagchewBerry does not require a Raspberry Pi,
PC, or operating system. Connect the radio hardware, front panel, and
USB headset and it operates as a self-contained HF transceiver.

RagchewBerry can also switch to very limited HPSDR Protocol 1 over USB, allowing the
same hardware to operate with an external SDR application such as
PiHPSDR or Quisk.

ACKNOWLEDGEMENTS

Johan - PA3GSB

RagchewBerry is based on Johan PA3GSB’s RP2350 Hub.

Johan’s RP2350 Hub is the foundation of this project. The
RP2350/Radioberry interface and original architecture upon which
RagchewBerry was developed come from his work.

RagchewBerry extends that foundation with standalone radio DSP,
USB-headset integration, the front-panel system, and the additional
radio features described below.

Many thanks to Johan for his Radioberry work and for making the RP2350
Hub foundation available for this project.

Pico-PIO-USB

Direct USB-headset hosting is made possible by Pico-PIO-USB by
sekigon-gonnoc.

This library allows the RP2350 to host the USB audio/HID headset through
PIO while leaving the native USB peripheral available for other
RagchewBerry functions.

Other Projects

RagchewBerry also depends upon, incorporates, or has benefited from the
work of the developers and contributors behind:

-   Radioberry
-   HPSDR / Hermes / Hermes-Lite
-   Pico-PIO-USB
-   TinyUSB
-   Raspberry Pi Pico SDK
-   PiHPSDR

All upstream projects remain the work of their respective authors and
are subject to their respective licenses.

WP3DN + Bug

RagchewBerry was developed by WP3DN in collaboration with Bug / OpenAI.

Bug participated extensively in the architecture, DSP work, firmware
implementation, debugging, USB-headset integration, front-panel
development, Protocol 1 integration, and the many experiments required
to make all of these pieces coexist on the RP2350.

Much of the local-radio work also builds upon techniques previously
developed and tested during the WP3DN Ragchew Machine project.

FEATURES

RagchewBerry currently provides:
-   
-   RP2350 local RX and TX DSP
-   LSB, USB, CW and AM
-   FFT-based receive filtering
-   Adjustable receive bandwidth
-   Adjustable AM bandwidth
-   AGC
-   Adjustable RF gain
-   RIT
-   Selectable spectrum span
-   S-meter
-   Direct USB audio-headset host
-   USB HID PTT support
-   USB HID TUNE support
-   Three-band TX audio EQ
-   TX compression
-   Per-band TX IQ gain
-   Shaped CW RF envelope
-   Separate RP2350 front-panel controller
-   Seven programmable RF switching outputs with per-band masks
-   Forward power measurement
-   Reflected power measurement
-   SWR calculation and display
-   Only CL025 Radioberry has been tested
-   Basic HPSDR Protocol 1 operation over USB

HARDWARE

RagchewBerry uses two different Waveshare RP2350 controllers:

Main radio / DSP: Waveshare RP2350-PiZero

Display / front panel: Waveshare RP2350-Zero


All GPIO numbers below are RP2350 GPxx GPIO numbers, not physical
connector pin numbers.

RADIO CONTROLLER PINOUT

Waveshare RP2350-PiZero

The RP2350-PiZero is the main radio processor. It handles the Radioberry
interface, FPGA loading, RX/TX DSP, USB headset, CW, Protocol 1, and
external PTT.

  GPIO   Direction   Function
  ------ ----------- -------------------------------
  GP0    IN          CW KEY - active LOW
  GP4    OUT         UART TX to display
  GP5    IN          UART RX from display
  GP8    OUT         Radioberry SPI CS
  GP10   OUT         Radioberry SPI SCK
  GP11   OUT         Radioberry SPI MOSI
  GP12   IN          Radioberry SPI MISO
  GP13   OUT         FPGA configuration DATA
  GP16   OUT         External RF PTT - active HIGH
  GP22   IN          FPGA CONF_DONE
  GP24   OUT         FPGA DCLK
  GP26   IN          FPGA NSTATUS
  GP27   OUT         FPGA NCONFIG
  GP28   PIO USB     USB headset D+
  GP29   PIO USB     USB headset D-

Radioberry SPI

  Signal   RP2350-PiZero
  -------- ---------------
  CS       GP8
  SCK      GP10
  MOSI     GP11
  MISO     GP12

FPGA Configuration

  Signal      RP2350-PiZero
  ----------- ---------------
  DATA        GP13
  CONF_DONE   GP22
  DCLK        GP24
  NSTATUS     GP26
  NCONFIG     GP27

The RP2350 loads the selected Radioberry FPGA gateware at startup.


USB Headset

  Signal   RP2350-PiZero
  -------- ---------------
  USB D+   GP28
  USB D-   GP29

The USB headset is hosted using Pico-PIO-USB.

External RF PTT

GP16 provides the external RF-deck PTT output.

The output is active HIGH.

DISPLAY / FRONT-PANEL CONTROLLER PINOUT

Waveshare RP2350-Zero

The RP2350-Zero handles the display, encoder, buttons, CW paddles/keyer,
programmable RF outputs, and FWD/REV measurements.

  GPIO          Direction    Function
  ------------- ------------ --------------------
  GP0           OUT          RF OUT7
  GP1           IN           CONFIRM button
  GP2           I/O          OLED SDA
  GP3           OUT          OLED SCL
  GP4           IN           Encoder push
  GP5           IN           Encoder A
  GP6           IN           Encoder B
  GP7           IN           BACK button
  GP8           OUT          UART TX to radio
  GP9           IN           UART RX from radio
  GP10          OUT          RF OUT1
  GP11          OUT          RF OUT2
  GP12          OUT          RF OUT3
  GP13          OUT          RF OUT4
  GP14          OUT          RF OUT5
  GP15          OUT          RF OUT6
  GP23          OUT / Hi-Z   CW KEY to radio
  GP24          IN           DAH paddle
  GP25          IN           DIT paddle
  GP26 / ADC0   ANALOG IN    Forward power
  GP27 / ADC1   ANALOG IN    Reflected power

OLED

  Signal   RP2350-Zero
  -------- -------------
  SDA      GP2
  SCL      GP3

Encoder and Buttons

  Control        GPIO
  -------------- ------
  Encoder push   GP4
  Encoder A      GP5
  Encoder B      GP6
  BACK           GP7
  CONFIRM        GP1

CW Paddle / Keyer

  Function   GPIO
  ---------- ------
  DIT        GP25
  DAH        GP24
  KEY OUT    GP23

DIT and DAH are active LOW and use internal pull-ups.

Display GP23 connects directly to Radio GP0.

The CW key output operates in an open-drain-like manner:

-   Key down: GP23 drives LOW
-   Key up: GP23 becomes high impedance

The radio processor provides the pull-up on GP0.

This keeps the final CW keying path independent of UART timing.

RADIO / DISPLAY CONNECTIONS

The control UART operates at 2,000,000 baud.

  Radio RP2350-PiZero   Display RP2350-Zero F   unction
  --------------------- ----------------------- ------------------
  GP4 TX                GP9 RX                  Radio -> Display
  GP5 RX                GP8 TX                  Display -> Radio
  GND                   GND                     Common ground

CW has an additional direct hardware connection:

  Display   Radio   Function
  --------- ------- ----------
  GP23      GP0     CW KEY

FRONT-PANEL CONTROLS

The RagchewBerry front panel uses a rotary encoder with push switch plus
BACK and CONFIRM buttons.

  Control               Action
  --------------------- ----------------------------------------
  Encoder rotate        Tune frequency / change selected value
  Encoder press         Enter menu / select menu item
  BACK short press      Back / normal navigation
  BACK long press       Enter / exit RIT
  CONFIRM short press   Confirm / select
  CONFIRM long press    Enter LPF / RF-output setup


LPF / RF OUTPUT SETUP

Press and hold CONFIRM to enter the LPF / RF-output setup screen.

The display controller provides seven generic outputs for external RF
hardware:

  Output   GPIO
  -------- ------
  OUT1     GP10
  OUT2     GP11
  OUT3     GP12
  OUT4     GP13
  OUT5     GP14
  OUT6     GP15
  OUT7     GP0

Each amateur band can store its own seven-bit output mask.


FWD / REV / SWR

  Measurement   GPIO
  ------------- -------------
  Forward       GP26 / ADC0
  Reflected     GP27 / ADC1

Ragchewberry Build

mkdir -p lib
git clone https://github.com/sekigon-gonnoc/Pico-PIO-USB.git lib/Pico-PIO-USB
mkdir build
cd build
cmake .. -DPICO_BOARD=pico2
cmake --build . -j


Display Controller Build


mkdir build
cd build
cmake .. -DPICO_BOARD=pico2
make -j4

THE LAB

RagchewBerry was developed the traditional homebrew way:

Not much test equipment, none of it particularly fancy, a soldering
iron, parts rescued from the junk box, occasional smoke, and repeated
attempts by WP3DN and Bug to leave perfectly functional radios alone.

Success rate on that last item: 0%.



WP3DN RagchewBerry

Radioberry + RP2350 + a junk box.

