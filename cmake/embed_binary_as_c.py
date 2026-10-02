#!/usr/bin/env python3
import argparse
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("--input", required=True)
p.add_argument("--out-c", required=True)
p.add_argument("--out-h", required=True)
p.add_argument("--symbol", required=True)
a = p.parse_args()

data = Path(a.input).read_bytes()
sym = a.symbol
Path(a.out_h).parent.mkdir(parents=True, exist_ok=True)

Path(a.out_h).write_text(
    "#pragma once\n"
    "#include <stddef.h>\n"
    "#include <stdint.h>\n"
    f"extern const uint8_t {sym}_data[];\n"
    f"extern const size_t {sym}_size;\n"
    f"extern const char {sym}_name[];\n"
)

rows = []
for i in range(0, len(data), 12):
    rows.append("    " + ", ".join(f"0x{b:02X}" for b in data[i:i+12]) + ",")

Path(a.out_c).write_text(
    f'#include "{Path(a.out_h).name}"\n'
    f'const uint8_t {sym}_data[] = {{\n' +
    "\n".join(rows) +
    f'\n}};\n'
    f'const size_t {sym}_size = sizeof({sym}_data);\n'
    f'const char {sym}_name[] = "{Path(a.input).name}";\n'
)
