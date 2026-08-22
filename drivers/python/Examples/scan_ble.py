"""Diagnostic: list every BLE advertiser this PC can see.

Use this when SineWaveBLE.py reports "No BLE device named ... found". It dumps
each advertiser's name, address, RSSI and advertised service UUIDs, and flags
any that expose the Nordic UART Service (NUS) -- which is what Eris BLE firmware
advertises. If your board shows up here (even with no/oddly-cased name), connect
by the exact name or address it prints.

    python scan_ble.py [--seconds 8]
"""
import argparse
import asyncio

from bleak import BleakScanner

NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"


async def main(seconds):
    print(f"Scanning for {seconds}s ...\n")
    # return_adv=True -> {address: (BLEDevice, AdvertisementData)}
    found = await BleakScanner.discover(timeout=seconds, return_adv=True)

    if not found:
        print("Nothing seen. The radio adapter may be off, or no device is "
              "advertising. (On the board: confirm ERIS_USE_BLE is enabled and "
              "it actually booted into the BLE path.)")
        return

    for address, (device, adv) in sorted(found.items()):
        uuids = [u.lower() for u in (adv.service_uuids or [])]
        is_nus = NUS_SERVICE_UUID in uuids
        name = adv.local_name or device.name or "(no name)"
        tag = "  <-- Nordic UART Service (Eris?)" if is_nus else ""
        print(f"{name!r:24} {address}  rssi={adv.rssi}{tag}")
        if uuids:
            print(f"    services: {', '.join(uuids)}")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="List BLE advertisers visible to this PC.")
    p.add_argument("--seconds", type=float, default=8.0, help="Scan duration (default: 8)")
    args = p.parse_args()
    asyncio.run(main(args.seconds))
