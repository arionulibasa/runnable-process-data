"""Read-only check of the ESI/SII inputs used to define this bench example."""

import argparse
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

IDENTITY = (0x232, 0xAB123, 9)
EXPECTED = {
    "RxPdo": [(0x1400, 2, [(0x7000, 1, 8)]), (0x1401, 2, [(0x7001, 0, 8)])],
    "TxPdo": [
        (0x1A00, 3, [(0x6000, sub, 8) for sub in range(1, 9)]),
        (0x1A01, 3, [(0x6001, 1, 32), (0x6001, 2, 8), (0x6001, 3, 1), (0, 0, 7)]),
    ],
}


def number(text):
    value = text.strip()
    return int(value[2:], 16) if value.lower().startswith("#x") else int(value, 10)


def sii_categories(image):
    if len(image) < 132:
        raise ValueError("truncated SII image")
    cursor = 128
    categories = {}
    while cursor + 2 <= len(image):
        kind = struct.unpack_from("<H", image, cursor)[0]
        if kind == 0xFFFF:
            return categories
        if cursor + 4 > len(image):
            raise ValueError("truncated category header")
        words = struct.unpack_from("<H", image, cursor + 2)[0]
        end = cursor + 4 + words * 2
        if end > len(image):
            raise ValueError("category extends beyond image")
        if kind in categories:
            raise ValueError(f"duplicate category {kind}")
        categories[kind] = image[cursor + 4:end]
        cursor = end
    raise ValueError("no category terminator")


def inspect(directory, require_sii):
    paths = list(directory.glob("*.xml"))
    if len(paths) != 1:
        raise ValueError("expected one ESI XML in the directory")
    root = ET.parse(paths[0]).getroot()
    devices = root.findall("./Descriptions/Devices/Device")
    if len(devices) != 1:
        raise ValueError("expected one ESI device")
    device = devices[0]
    kind = device.find("Type")
    identity = (number(root.findtext("Vendor/Id")), number(kind.get("ProductCode")), number(kind.get("RevisionNo")))
    if identity != IDENTITY:
        raise ValueError(f"ESI identity differs from Rust contract: {identity}")
    print(f"ESI: {paths[0].name}; vendor=0x{identity[0]:08x}, product=0x{identity[1]:08x}, revision={identity[2]}")
    for direction in EXPECTED:
        pdos = []
        bits = 0
        for pdo in device.findall(direction):
            entries = [(number(e.findtext("Index")), number(e.findtext("SubIndex")), number(e.findtext("BitLen"))) for e in pdo.findall("Entry")]
            pdos.append((number(pdo.findtext("Index")), number(pdo.get("Sm")), entries))
            for index, sub, width in entries:
                print(f"  {direction} bit {bits:3}: 0x{index:04x}:{sub:02x}, {width} bits")
                bits += width
        if pdos != EXPECTED[direction]:
            raise ValueError(f"{direction} order/entries differ from Rust contract")
        print(f"{direction}: {bits} bits, {(bits + 7) // 8} bytes")
    sm_addresses = [number(sm.get("StartAddress")) for sm in device.findall("Sm")]
    if sm_addresses != [0x1000, 0x1200, 0x1400, 0x1A00]:
        raise ValueError("unexpected ESI SM addresses")
    image = (directory / "eeprom.bin").read_bytes()
    if len(image) < 32:
        raise ValueError("truncated SII identity")
    binary_identity = struct.unpack_from("<4I", image, 16)
    if binary_identity[:3] != identity:
        raise ValueError("EEPROM and ESI identity do not match")
    categories = sii_categories(image)
    print(f"EEPROM: {len(image)} bytes, serial={binary_identity[3]}, categories={list(categories)}")
    for category, direction in [(50, "TxPdo"), (51, "RxPdo")]:
        if category not in categories:
            print(f"WARNING: missing {direction} category {category}; SII fallback cannot provide this direction.")
            if require_sii:
                raise ValueError("regenerate EEPROM with PDO categories before using the sii feature")
    print("CoE example contract matches the ESI. This does not verify the flashed slave or its timing.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--require-sii", action="store_true")
    args = parser.parse_args()
    try:
        inspect(args.directory, args.require_sii)
    except (ValueError, OSError, ET.ParseError, TypeError, AttributeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
