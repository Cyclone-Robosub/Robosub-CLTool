# tests/test_cltool.py

import io
import os
import threading
import pytest

import rclpy
from pwm_cltool.pwm_cltool import Pwm_Cltool
from pwm_cltool.pwm_publisher import Pwm_Publisher
from pwm_cltool.plant import Plant

from std_msgs.msg import Int32MultiArray, Int64, Bool

@pytest.fixture
def pwm_cltool(pwm_cltool_node, monkeypatch):
    """
    Ensure we don't actually spin ROS or sleep during tests.
    Replace sleeps with no-ops and spinning with immediate return.
    """
    # Patch sleep so override/exitCLTool/timed_pwm don't actually wait
    monkeypatch.setattr("pwm_cltool.cltool.sleep", lambda *_: None)

    # Stop the background ROS spin thread from blocking tests
    if hasattr(pwm_cltool_node, "ros_thread"):
        # monkey-patch spin_ros to exit immediately
        monkeypatch.setattr(pwm_cltool_node, "spin_ros", lambda: None)

    yield pwm_cltool_node

    # cleanup
    if rclpy.ok():
        rclpy.shutdown()
    if hasattr(pwm_cltool_node, "ros_thread") and pwm_cltool_node.ros_thread.is_alive():
        pwm_cltool_node.ros_thread.join(timeout=1)


def test_scaled_pwm_basics(pwm_cltool):
    zero = [1500] * 8
    # scaling by 1 gives identical list
    assert pwm_cltool.scaled_pwm(zero, 1.0) == zero
    # scaling by 0 moves everything to stop_pulse (1500)
    some = [1100, 1900] + [1500] * 6
    scaled = pwm_cltool.scaled_pwm(some, 0.0)
    assert scaled == zero
    # scaling by 2 doubles the distance from 1500
    sample = [1600, 1400]
    expected = [1500 + 2*(1600-1500), 1500 + 2*(1400-1500)]
    assert pwm_cltool.scaled_pwm(sample + [1500]*6, 2.0)[:2] == expected

def test_pwm_method_calls_publish_array_and_duration(pwm_cltool, monkeypatch):
    # patch publisher
    pub = pwm_cltool.publishCommandDurationObject
    calls = {"array": [], "duration": []}
    monkeypatch.setattr(pub, "publish_array", lambda arr: calls["array"].append(list(arr)))
    monkeypatch.setattr(pub, "publish_duration", lambda d: calls["duration"].append(d))

    # wrong length: should early-return and not publish
    bad = [1,2,3]
    pwm_cltool.pwm(bad)
    assert calls["array"] == [] and calls["duration"] == []

    # correct length, default scale=1
    good = [1500]*8
    pwm_cltool.pwm(good)
    assert calls["array"] == [good]
    assert calls["duration"] == [-1]

    # reset
    calls["array"].clear(); calls["duration"].clear()

    # with scale != 1
    pwm_cltool.pwm(good, scale=0.5)
    # array should equal scaled_pwm(good,0.5)
    assert calls["array"][0] == pwm_cltool.scaled_pwm(good, 0.5)
    assert calls["duration"][0] == -1

def test_timed_pwm_and_override_publish_sequence(pwm_cltool, monkeypatch):
    pub = pwm_cltool.publishCommandDurationObject
    seq = []
    monkeypatch.setattr(pub, "publish_array", lambda arr: seq.append(("array", list(arr))))
    monkeypatch.setattr(pub, "publish_duration", lambda d: seq.append(("duration", d)))
    monkeypatch.setattr(pub, "publish_manual_override", lambda f: seq.append(("override", f)))

    # test override: override sends override, then array, then duration
    pwm_cltool.override(durationMS=42, pwm_set=[1]*8)
    assert seq == [
        ("override", True),
        ("array", [1]*8),
        ("duration", 42)
    ]

    seq.clear()
    # test timed_pwm: array then duration
    pwm_cltool.timed_pwm(time_s=7, pwm_set=[2]*8, scale=1.0)
    assert seq == [
        ("array", [2]*8),
        ("duration", 7)
    ]

def test_exitCLTool_calls_manual_switch_false(pwm_cltool, monkeypatch):
    pub = pwm_cltool.publishCommandDurationObject
    called = []
    monkeypatch.setattr(pub, "publish_manual_switch", lambda f: called.append(f))
    # simulate rclpy.ok() true
    monkeypatch.setattr("pwm_cltool.cltool.rclpy.ok", lambda: True)

    pwm_cltool.exitCLTool()
    assert called == [False]

def test_reaction_uses_plant_pwm_force(pwm_cltool, monkeypatch):
    # patch plant.pwm_force to capture input
    got = []
    monkeypatch.setattr(pwm_cltool.plant, "pwm_force", lambda arr: got.append(arr))
    sample = [1500, 1600, 1400] + [1500]*5
    pwm_cltool.reaction(sample, scale=2.0)
    # reaction scales internally via same formula as scaled_pwm
    expected = [2.0*(x-1500)+1500 for x in sample]
    assert got == [expected]

def test_read_prints_file_contents(tmp_path, pwm_cltool, capsys):
    # create a temporary pwm_file
    content = "line1\nline2"
    p = tmp_path / "pwm_file.csv"
    p.write_text(content)
    # monkeypatch the filename global
    monkeypatch = pytest.MonkeyPatch()
    monkeypatch.setenv("PWM_CTOOL_PWMDIR", str(tmp_path))
    # override the global pwm_file path
    import pwm_cltool.pwm_cltool as cltool_mod
    cltool_mod.pwm_file = str(p)

    pwm_cltool.read()
    out = capsys.readouterr().out
    assert content in out
    monkeypatch.undo()
