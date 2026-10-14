from types import SimpleNamespace

import pytest

from picar_core import FakeRobot
from picar_core import picar4wd_adapter
from picar_core.picar4wd_adapter import Picar4WDHardware
from picar_ros.hardware import make_hardware


def test_fake_plays_back_scripted_distances():
    hardware = make_hardware("fake", fake_distances_cm=[30.0, 10.0])

    assert isinstance(hardware, FakeRobot)
    assert [hardware.read_distance_cm() for _ in range(3)] == [30.0, 10.0, 10.0]


def test_picar4wd_builds_the_adapter(monkeypatch):
    # The real library needs a Raspberry Pi, so a stand-in replaces the import.
    monkeypatch.setattr(picar4wd_adapter, "_import_library", lambda: SimpleNamespace())

    assert isinstance(make_hardware("picar4wd"), Picar4WDHardware)


def test_unknown_name_is_rejected():
    with pytest.raises(ValueError, match="unknown hardware 'gopigo'"):
        make_hardware("gopigo")
