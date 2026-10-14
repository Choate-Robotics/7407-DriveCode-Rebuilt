"""Connection health for every Phoenix device, the CANivore and the vision coprocessor.

Dashboard (SmartDashboard):
    Health/All Connected   one boolean for a big green/red box
    Health/Status          "OK" or a summary of what's missing
    Health/Missing         list of missing devices

Plus an Elastic pop-up notification whenever something disconnects or comes back.

Cost: one device is checked per loop, round-robin (is_connected is a single
cached-signal read), so with ~20 devices each is checked about twice a second.
Devices are found automatically by scanning the subsystems for Phoenix devices.

Note this only catches devices that stop talking. A device that stays connected but
reports bad data (e.g. the Pigeon reading zeros) is gyro_broken's job.
"""

import json
import time

import ntcore
from phoenix6.hardware import ParentDevice
from wpilib import SmartDashboard

_MODULE_NAMES = ("FL", "FR", "BL", "BR")  # order passed to create_drivetrain()
_STARTUP_GRACE_S = 5.0  # devices are still coming up right after boot


def _bus(device: ParentDevice) -> str:
    return device._device_identifier.network.name or "rio"


def _label(name: str, device: ParentDevice) -> str:
    return f"{name} ({type(device).__name__} {device.device_id}, {_bus(device)})"


def find_devices(container) -> dict[str, ParentDevice]:
    """All Phoenix devices on the robot, keyed by a readable label."""
    found: dict[str, ParentDevice] = {}
    seen: set[int] = set()

    def add(name, device):
        if isinstance(device, ParentDevice) and id(device) not in seen:
            seen.add(id(device))
            found[_label(name, device)] = device

    dt = container.drivetrain
    for module_name, module in zip(_MODULE_NAMES, dt.modules):
        add(f"{module_name} drive", module.drive_motor)
        add(f"{module_name} steer", module.steer_motor)
        add(f"{module_name} encoder", module.encoder)
    add("Pigeon", dt.pigeon2)
    add("Backup Pigeon", getattr(container, "backup_gyro", None))

    for attr in ("shooter", "intake", "indexer", "climber"):
        subsystem = getattr(container, attr, None)
        if subsystem is None:
            continue
        for field, value in vars(subsystem).items():
            add(f"{type(subsystem).__name__} {field}", value)
    return found


class DeviceHealth:
    def __init__(self, devices: dict[str, ParentDevice], vision=None):
        self._devices = list(devices.items())
        self._connected = {name: True for name, _ in self._devices}
        self._bus_of = {name: _bus(dev) for name, dev in self._devices}
        self._vision = vision
        self._next = 0
        self._check = lambda device: device.is_connected  # a property in Phoenix 6
        self._start = time.monotonic()
        self._last_publish = 0.0
        self._last_missing: tuple[str, ...] = ()

        inst = ntcore.NetworkTableInstance.getDefault()
        self._inst = inst
        self._camera_subs: dict[str, ntcore.BooleanSubscriber] = {}
        self._last_camera_scan = 0.0
        self._notify_pub = inst.getStringTopic("/Elastic/RobotNotifications").publish(
            ntcore.PubSubOptions(sendAll=True, keepDuplicates=True))

        SmartDashboard.putBoolean("Health/All Connected", True)
        SmartDashboard.putString("Health/Status", "starting up")
        SmartDashboard.putStringArray("Health/Missing", [])

    def _notify(self, level: str, title: str, description: str, display_ms: int):
        # Elastic's notification format (same as elasticlib's Notification)
        self._notify_pub.set(json.dumps({
            "level": level, "title": title, "description": description,
            "displayTime": display_ms, "width": 350, "height": -1,
        }))

    def _scan_cameras(self, now: float):
        """Subscribe to each camera's connected flag from the coprocessor (found by topic name)."""
        if now - self._last_camera_scan < 2.0:
            return
        self._last_camera_scan = now
        for topic in self._inst.getTopics("/Vision/cameras/"):
            name = topic.getName()
            if name.endswith("/connected") and name not in self._camera_subs:
                self._camera_subs[name] = self._inst.getBooleanTopic(name).subscribe(True)

    def _missing(self) -> list[str]:
        missing = [name for name, ok in self._connected.items() if not ok]
        if self._vision is not None:
            if not self._vision.is_connected():
                missing.append("Vision coprocessor (no heartbeat)")
            else:
                for topic, sub in self._camera_subs.items():
                    if not sub.get():
                        missing.append(f"Camera {topic.split('/')[3]}")
        return missing

    def _summary(self, missing: list[str]) -> str:
        if not missing:
            return "OK"
        # A whole bus gone at once is one problem (e.g. the CANivore's USB link), not N
        lost_buses = []
        for bus in sorted(set(self._bus_of.values())):
            on_bus = [n for n, b in self._bus_of.items() if b == bus]
            if len(on_bus) > 1 and all(not self._connected[n] for n in on_bus):
                lost_buses.append(bus)
        bus_msgs = [f"{'roboRIO CAN bus' if bus == 'rio' else 'CANivore ' + repr(bus)} LOST "
                    f"({sum(b == bus for b in self._bus_of.values())} devices)" for bus in lost_buses]
        others = [m for m in missing if self._bus_of.get(m) not in lost_buses]
        return "; ".join(bus_msgs + others)

    def update(self):
        """Call once per loop."""
        now = time.monotonic()
        if self._devices:
            name, device = self._devices[self._next]
            self._next = (self._next + 1) % len(self._devices)
            self._connected[name] = self._check(device)
        if self._vision is not None:
            self._scan_cameras(now)

        if now - self._last_publish < 0.25:
            return
        self._last_publish = now
        missing = self._missing()
        status = self._summary(missing)
        SmartDashboard.putBoolean("Health/All Connected", not missing)
        SmartDashboard.putString("Health/Status", status)
        SmartDashboard.putStringArray("Health/Missing", missing)

        current = tuple(missing)
        if current != self._last_missing and now - self._start > _STARTUP_GRACE_S:
            lost = [m for m in current if m not in self._last_missing]
            back = [m for m in self._last_missing if m not in current]
            if lost:
                self._notify("ERROR", "Disconnected", status, 10000)
            if back:
                self._notify("INFO", "Reconnected", ", ".join(back), 4000)
        self._last_missing = current
