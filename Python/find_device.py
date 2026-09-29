#!/usr/bin/env python3
"""Find the Dynamixel USB adapter and save its port for MyRobot."""

import sys
from pathlib import Path

DEVICE_FILE = Path(__file__).resolve().parent / "device_name.txt"
FTDI_VID = "0403"  # U2D2 and USB2Dynamixel


def usb_ids(tty_name):
    device = Path("/sys/class/tty") / tty_name / "device"
    for parent in [device, *device.parents]:
        vendor = parent / "idVendor"
        if vendor.is_file():
            product = (parent / "idProduct").read_text(encoding="utf-8").strip()
            return vendor.read_text(encoding="utf-8").strip().lower(), product.lower()
    return "", ""


def discover_ports():
    by_id = {}
    by_id_dir = Path("/dev/serial/by-id")
    if by_id_dir.is_dir():
        for link in by_id_dir.iterdir():
            try:
                by_id[str(link.resolve())] = str(link)
            except OSError:
                continue

    ports = []
    for pattern in ("ttyUSB*", "ttyACM*"):
        for node in sorted(Path("/dev").glob(pattern)):
            resolved = str(node.resolve())
            vid, product = usb_ids(node.name)
            ports.append(
                {
                    "device": by_id.get(resolved, str(node)),
                    "node": str(node),
                    "vid": vid,
                    "product": product,
                }
            )
    return ports


def choose_port(ports):
    if not ports:
        return None
    ftdi = [port for port in ports if port["vid"] == FTDI_VID]
    if len(ftdi) == 1:
        return ftdi[0]
    if len(ports) == 1:
        return ports[0]
    return None


def describe(port):
    label = port["device"]
    if port["node"] != port["device"]:
        label = f"{port['device']} -> {port['node']}"
    if port["vid"]:
        label = f"{label} (usb {port['vid']}:{port['product']})"
    return label


def write_device(path):
    DEVICE_FILE.write_text(path + "\n", encoding="utf-8")
    print(f"Saved {path} to {DEVICE_FILE}")


def main(argv):
    if len(argv) > 2:
        print(f"Usage: {argv[0]} [/dev/ttyUSB0]", file=sys.stderr)
        return 2

    if len(argv) == 2:
        chosen = Path(argv[1])
        if not chosen.exists():
            print(f"No such device: {chosen}", file=sys.stderr)
            return 1
        write_device(str(chosen))
        return 0

    ports = discover_ports()
    chosen = choose_port(ports)
    if chosen is not None:
        print(f"Found {describe(chosen)}")
        write_device(chosen["device"])
        return 0

    if not ports:
        print("No serial device found. Plug in the robot adapter and run this again.", file=sys.stderr)
        return 1

    print("Several serial devices found. Re-run with the one you want:", file=sys.stderr)
    for port in ports:
        print(f"  {Path(sys.argv[0]).name} {port['node']}    {describe(port)}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
