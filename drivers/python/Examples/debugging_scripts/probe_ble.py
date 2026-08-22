"""Diagnostic: connect to a BLE device by address and list its GATT table.

Confirms whether an advertiser is an Eris board: it should expose the Nordic
UART Service (NUS) with an RX (write) and TX (notify) characteristic. Use the
address printed by scan_ble.py.

    python probe_ble.py BC:45:5B:85:15:29
"""
import argparse
import asyncio

from bleak import BleakClient, BleakScanner

NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # device -> host (notify)


async def main(address):
    # Windows/WinRT can't connect cold by address string -- it must have seen the
    # device in a scan first, then connect via the discovered device object.
    print(f"Scanning for {address} ...")
    device = await BleakScanner.find_device_by_address(address, timeout=10.0)
    if device is None:
        print(f"Address {address} not seen while scanning. Re-run scan_ble.py "
              "to confirm it's still advertising (addresses can rotate).")
        return
    print(f"Connecting to {address} ...")
    async with BleakClient(device) as client:
        print(f"Connected: {client.is_connected}\n")
        nus = False
        for service in client.services:
            print(f"service {service.uuid}  ({service.description})")
            for ch in service.characteristics:
                print(f"    char {ch.uuid}  {ch.properties}")
                if ch.uuid.lower() == NUS_SERVICE_UUID:
                    nus = True
            if service.uuid.lower() == NUS_SERVICE_UUID:
                nus = True
        print()
        if nus:
            print("Nordic UART Service present -> this is an Eris BLE board.")
            print(f"Connect with: python SineWaveBLE.py --name {address}")
        else:
            print("No Nordic UART Service -> not an Eris BLE endpoint "
                  "(or BLE didn't fully start on the board).")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Connect to a BLE address and dump its GATT services.")
    p.add_argument("address", help="BLE address from scan_ble.py, e.g. BC:45:5B:85:15:29")
    args = p.parse_args()
    asyncio.run(main(args.address))
