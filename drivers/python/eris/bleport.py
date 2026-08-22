"""BLE transport adapter for the Eris host driver.

Presents a small pyserial-compatible surface (``in_waiting`` / ``read`` /
``write`` / ``flush`` / ``close``) over a Bluetooth LE Nordic UART Service
(NUS), so the existing :class:`eris.eris.Eris` driver can talk to nRF52 firmware
built with ``-DERIS_USE_BLE`` without changing any of its COBS packet-parsing
code. The firmware frames everything with COBS, so notifications that split or
coalesce across the radio reassemble transparently on the ``0x00`` delimiter.

Uses the cross-platform ``bleak`` library, which is installed automatically as a
dependency of the eris driver (``pip install -e .``).
"""

import asyncio
import threading

try:
    from bleak import BleakClient, BleakScanner
except ImportError as _exc:   # pragma: no cover - import-time guard
    raise ImportError(
        "The Eris BLE transport requires the 'bleak' package. It ships as a "
        "dependency of the eris driver, so reinstalling should pull it in:\n"
        "    pip install -e .     # from drivers/python\n"
        "    pip install bleak    # or just the dependency"
    ) from _exc


# Nordic UART Service UUIDs -- the de-facto "serial port over BLE".
# Naming is from the central's (host's) point of view.
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # host -> device  (we write commands)
NUS_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # device -> host  (we get notifications)


def _looks_like_address(s):
    """Heuristic: a BLE MAC (``AA:BB:CC:DD:EE:FF``) or a macOS UUID address.

    Anything else is treated as an advertised device name to scan for. Pass
    ``name=`` to :class:`BLEPort` to force scan-by-name if your name trips this.
    """
    if not isinstance(s, str):
        return False
    if s.count(":") >= 5:                       # Windows / Linux MAC
        return True
    if len(s) >= 32 and s.count("-") >= 4:      # macOS CBPeripheral UUID
        return True
    return False


class BLEPort:
    """A ``serial.Serial`` look-alike backed by a BLE NUS link.

    Only the subset used by the Eris driver is implemented: ``in_waiting``,
    ``read(size)``, ``write(data)``, ``flush()`` and ``close()``. A background
    thread runs the asyncio/bleak event loop; received notifications accumulate
    in a buffer that ``read()`` drains, mirroring pyserial's RX buffer.
    """

    def __init__(self, address, timeout=10.0, name=None):
        """Connect to an Eris BLE device.

        address: a BLE address (MAC on Windows/Linux, UUID on macOS) **or** an
                 advertised device name to scan for (e.g. ``"Eris-Bare"``).
        timeout: scan/connect timeout in seconds.
        name:    force scan-by-name; pass the advertised name here if the
                 ``address`` heuristic guesses wrong.
        """
        self._rx = bytearray()                 # bytes received from the device
        self._lock = threading.Lock()
        self._client = None

        self._target = address
        self._scan_by_name = (name is not None) or (not _looks_like_address(address))
        self._name = name if name is not None else address

        # Run the bleak event loop on its own thread; the Eris driver is sync.
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, name="eris-ble", daemon=True)
        self._thread.start()

        fut = asyncio.run_coroutine_threadsafe(self._connect(timeout), self._loop)
        fut.result(timeout=timeout + 5.0)      # surface connection errors to the caller

    # ------------------------------------------------------------------ #
    # asyncio plumbing (runs on the background thread)
    # ------------------------------------------------------------------ #
    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    async def _connect(self, timeout):
        # Always resolve to a discovered device object before connecting. Windows
        # / WinRT refuses a cold connect by bare address; it must have seen the
        # device in a scan first. Scanning by name or by address both yield the
        # device object that connect() needs.
        if self._scan_by_name:
            device = await BleakScanner.find_device_by_name(self._name, timeout=timeout)
            if device is None:
                raise RuntimeError(
                    "No BLE device named %r found while scanning." % (self._name,)
                )
        else:
            device = await BleakScanner.find_device_by_address(self._target, timeout=timeout)
            if device is None:
                raise RuntimeError(
                    "No BLE device with address %r found while scanning." % (self._target,)
                )

        self._client = BleakClient(device, timeout=timeout)
        await self._client.connect()
        await self._client.start_notify(NUS_TX_CHAR_UUID, self._on_notify)

    def _on_notify(self, _sender, data):
        with self._lock:
            self._rx.extend(data)

    async def _write(self, data, chunk):
        for i in range(0, len(data), chunk):
            await self._client.write_gatt_char(
                NUS_RX_CHAR_UUID, data[i:i + chunk], response=False
            )

    # ------------------------------------------------------------------ #
    # pyserial-compatible surface (called from the main thread)
    # ------------------------------------------------------------------ #
    @property
    def in_waiting(self):
        with self._lock:
            return len(self._rx)

    @property
    def is_open(self):
        return self._client is not None and self._client.is_connected

    def read(self, size=1):
        """Return up to ``size`` buffered bytes immediately (non-blocking).

        The Eris driver always calls ``read(in_waiting)``, so it only ever asks
        for bytes already received -- no blocking read is needed.
        """
        with self._lock:
            chunk = bytes(self._rx[:size])
            del self._rx[:size]
        return chunk

    def write(self, data):
        # NUS RX is write-without-response; chunk to the negotiated MTU minus the
        # 3-byte ATT header (fall back to the 23-byte default MTU -> 20 B payload).
        mtu = getattr(self._client, "mtu_size", 23) or 23
        chunk = max(20, mtu - 3)
        fut = asyncio.run_coroutine_threadsafe(
            self._write(bytes(data), chunk), self._loop
        )
        fut.result(timeout=5.0)
        return len(data)

    def flush(self):
        # write() awaits each GATT write, so nothing is buffered locally.
        pass

    def close(self):
        if self._client is not None:
            try:
                fut = asyncio.run_coroutine_threadsafe(
                    self._client.disconnect(), self._loop
                )
                fut.result(timeout=5.0)
            except Exception:
                pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=2.0)
