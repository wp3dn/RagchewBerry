#!/usr/bin/env python3
from pathlib import Path
import sys, re

if len(sys.argv) != 2:
    raise SystemExit("usage: integrate_iso_audio.py <Pico-PIO-USB source dir>")

root = Path(sys.argv[1])
host = root / "src" / "pio_usb_host.c"
llh  = root / "src" / "pio_usb_ll.h"
defs = root / "src" / "usb_definitions.h"
core = root / "src" / "pio_usb.c"
cfg  = root / "src" / "pio_usb_configuration.h"

for p in (host, llh, defs, core, cfg):
    if not p.exists():
        print(f"ERROR: missing {p}", file=sys.stderr)
        raise SystemExit(2)

MARK = "WP3DN UAC1 ISO LARGE-PACKET SUPPORT"

# ------------------------------------------------------------------
# Large full-speed audio packet support.
# ------------------------------------------------------------------
s = cfg.read_text()
if "PIO_USB_EP_SIZE 256" not in s:
    s2, n = re.subn(r'#define\s+PIO_USB_EP_SIZE\s+64\b',
                    '#define PIO_USB_EP_SIZE 256  // WP3DN: full-speed UAC packets',
                    s, count=1)
    if n != 1:
        print("ERROR: could not expand PIO_USB_EP_SIZE from 64 to 256.",
              file=sys.stderr)
        raise SystemExit(2)
    cfg.write_text(s2)

s = defs.read_text()

old_buf = "uint8_t buffer[(64 + 4) * 2 * 7 / 6 + 2];"
new_buf = "uint8_t buffer[(PIO_USB_EP_SIZE + 4) * 2 * 7 / 6 + 2];"
if old_buf in s:
    s = s.replace(old_buf, new_buf, 1)
elif new_buf not in s:
    print("ERROR: endpoint encoded TX buffer declaration not found.",
          file=sys.stderr)
    raise SystemExit(2)

if MARK not in s:
    old = "uint8_t encoded_data_len;"
    if old not in s:
        print("ERROR: encoded_data_len field not found.", file=sys.stderr)
        raise SystemExit(2)
    s = s.replace(old, f"uint16_t encoded_data_len; // {MARK}", 1)

defs.write_text(s)

s = llh.read_text()
s, n = re.subn(
    r'uint8_t\s+pio_usb_ll_encode_tx_data\(uint8_t const \*buffer,\s*uint8_t buffer_len,',
    'uint16_t pio_usb_ll_encode_tx_data(uint8_t const *buffer, uint16_t buffer_len,',
    s, count=1
)
if n != 1 and "uint16_t pio_usb_ll_encode_tx_data" not in s:
    print("ERROR: encoder prototype not found.", file=sys.stderr)
    raise SystemExit(2)
llh.write_text(s)

s = core.read_text()
s, n = re.subn(
    r'uint8_t\s+__no_inline_not_in_flash_func\(pio_usb_ll_encode_tx_data\)\(\s*'
    r'uint8_t const \*buffer,\s*uint8_t buffer_len,\s*uint8_t \*encoded_data\)',
    'uint16_t __no_inline_not_in_flash_func(pio_usb_ll_encode_tx_data)(\n'
    '    uint8_t const *buffer, uint16_t buffer_len, uint8_t *encoded_data)',
    s, count=1
)
if n != 1 and \
   "uint16_t __no_inline_not_in_flash_func(pio_usb_ll_encode_tx_data)" not in s:
    print("ERROR: encoder implementation signature not found.", file=sys.stderr)
    raise SystemExit(2)

s = s.replace("uint8_t byte_idx = bit_idx >> 2;",
              "uint16_t byte_idx = bit_idx >> 2;")

# ------------------------------------------------------------------
# Add a "no handshake" receive mode.
# handshake == 0 means validate CRC and return payload length but send
# absolutely nothing after the isochronous IN packet.
# ------------------------------------------------------------------
if "WP3DN ISO IN: receive without ACK" not in s:
    old = """      if (handshake == USB_PID_ACK) {
        // Only ACK if crc matches
        if (idx >= 4 && crc_match) {
          pio_usb_bus_usb_transfer(pp, ack_encoded, 5);
          return idx - 4;
        }
      } else if (handshake == USB_PID_NAK) {"""

    new = """      // WP3DN ISO IN: receive without ACK.
      // UAC isochronous IN has no handshake phase. A handshake value of
      // zero means validate the packet CRC and return the payload only.
      if (handshake == 0) {
        if (idx >= 4 && crc_match) {
          return idx - 4;
        }
      } else if (handshake == USB_PID_ACK) {
        // Only ACK if crc matches
        if (idx >= 4 && crc_match) {
          pio_usb_bus_usb_transfer(pp, ack_encoded, 5);
          return idx - 4;
        }
      } else if (handshake == USB_PID_NAK) {"""

    if old not in s:
        print("ERROR: receive handshake block not found in pio_usb.c.",
              file=sys.stderr)
        raise SystemExit(2)
    s = s.replace(old, new, 1)

core.write_text(s)

# ------------------------------------------------------------------
# Host scheduler / ISO transaction behavior.
# ------------------------------------------------------------------
s = host.read_text()

# Treat both interrupt and ISO endpoints as periodic.
s = s.replace(
    "bool const is_periodic = ((ep->attr & 0x03) == EP_ATTR_INTERRUPT);",
    "bool const is_periodic = (((ep->attr & 0x03) == EP_ATTR_INTERRUPT) || "
    "((ep->attr & 0x03) == EP_ATTR_ISOCHRONOUS));"
)

# ISO OUT support preserved from the proven Stage 2 baseline.
if "WP3DN ISO OUT: no handshake" not in s:
    name = "usb_out_transaction"
    pos = s.find(name)
    while pos >= 0:
        brace = s.find("{", pos)
        semi = s.find(";", pos)
        if brace >= 0 and (semi < 0 or brace < semi):
            break
        pos = s.find(name, pos + len(name))
    if pos < 0:
        print("ERROR: usb_out_transaction() definition not found.",
              file=sys.stderr)
        raise SystemExit(2)

    depth = 0
    end = None
    for i in range(brace, len(s)):
        if s[i] == "{":
            depth += 1
        elif s[i] == "}":
            depth -= 1
            if depth == 0:
                end = i + 1
                break
    if end is None:
        print("ERROR: malformed usb_out_transaction().", file=sys.stderr)
        raise SystemExit(2)

    func = s[brace:end]
    needle = "int res = 0;"
    rel = func.find(needle)
    if rel < 0:
        print("ERROR: usb_out_transaction() prologue not found.",
              file=sys.stderr)
        raise SystemExit(2)

    insert_at = brace + rel + len(needle)
    code = """
  // WP3DN ISO OUT: no handshake. Do not arm the RX state machine.
  if ((ep->attr & 0x03) == EP_ATTR_ISOCHRONOUS) {
    uint16_t const iso_xact_len = pio_usb_ll_get_transaction_len(ep);
    pio_usb_bus_send_token(pp, USB_PID_OUT, ep->dev_addr, ep->ep_num);
    pio_usb_bus_usb_transfer(pp, ep->buffer, ep->encoded_data_len);
    pio_usb_ll_transfer_continue(ep, iso_xact_len);
    ep->data_id = 0;
    return 0;
  }
"""
    s = s[:insert_at] + code + s[insert_at:]

# ISO IN: call receiver with handshake=0 and keep DATA0 every frame.
if "WP3DN ISO IN: no ACK" not in s:
    old_call = """  int receive_len = pio_usb_bus_receive_packet_and_handshake(pp, USB_PID_ACK);"""
    new_call = """  // WP3DN ISO IN: no ACK/NAK/STALL handshake is legal.
  uint8_t const rx_handshake =
      ((ep->attr & 0x03) == EP_ATTR_ISOCHRONOUS) ? 0 : USB_PID_ACK;
  int receive_len =
      pio_usb_bus_receive_packet_and_handshake(pp, rx_handshake);"""
    if old_call not in s:
        print("ERROR: usb_in_transaction receive call not found.",
              file=sys.stderr)
        raise SystemExit(2)
    s = s.replace(old_call, new_call, 1)

    old_continue = """      pio_usb_ll_transfer_continue(ep, receive_len);"""
    new_continue = """      pio_usb_ll_transfer_continue(ep, receive_len);
      // WP3DN ISO IN: no ACK and every full-speed ISO frame uses DATA0.
      if ((ep->attr & 0x03) == EP_ATTR_ISOCHRONOUS) {
        ep->data_id = 0;
      }"""
    if old_continue not in s:
        print("ERROR: usb_in_transaction transfer_continue not found.",
              file=sys.stderr)
        raise SystemExit(2)
    s = s.replace(old_continue, new_continue, 1)

host.write_text(s)

print("Integrated WP3DN UAC1 ISO OUT + ISO IN support into private Pico-PIO-USB build copy")
