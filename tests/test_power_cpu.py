"""Power node CPU governor toggle (nodes/power/cpu.py): a fake sysfs tree and an injected runner; no test can
reach the real cpufreq files or run a process."""
import json

import pytest

from robot_utils.nodes.common.lease import RequestError
from robot_utils.nodes.power.cpu import UNIT, CpuGovernor, refuse_run

GOVS = "conservative ondemand userspace powersave performance schedutil"


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


def fake_sysfs(tmp_path, gov="ondemand", policies=1):
    root = tmp_path / "cpufreq"
    for i in range(policies):
        p = root / f"policy{i * 4}"
        (p / "stats").mkdir(parents=True)
        (p / "scaling_available_governors").write_text(GOVS + " \n")
        (p / "scaling_governor").write_text(gov + "\n")
        (p / "scaling_cur_freq").write_text("1900000\n")
        (p / "scaling_min_freq").write_text("1500000\n")
        (p / "scaling_max_freq").write_text("2400000\n")
        (p / "stats" / "total_trans").write_text("1000\n")
    helper = tmp_path / "robot-cpu-governor@.service"
    helper.write_text("[Service]\n")
    return root, helper


def helper_runner(root, calls):
    """Stands in for `systemctl start robot-cpu-governor@X.service`: writes X to every policy."""
    def run(argv):
        calls.append(argv)
        g = argv[-1].split("@", 1)[1].rsplit(".service", 1)[0]
        for p in root.glob("policy*"):
            (p / "scaling_governor").write_text(g + "\n")
    return run


def test_read_reports_governor_clock_and_transition_rate(tmp_path):
    root, helper = fake_sysfs(tmp_path)
    clock = Clock()
    cg = CpuGovernor(None, sysfs=root, helper_unit_file=helper, clock=clock)
    st = cg.read()
    assert st["governor"] == "ondemand" and "schedutil" in st["governors"] and st["ghz"] == 1.9
    assert st["ghz_range"] == [1.5, 2.4] and st["helper_installed"] and st["transitions_s"] is None
    (root / "policy0" / "stats" / "total_trans").write_text("1620\n")
    clock.t += 10
    assert cg.read()["transitions_s"] == 62.0


def test_set_switches_validates_persists_and_reapplies(tmp_path):
    root, helper = fake_sysfs(tmp_path, policies=2)
    calls = []
    settings = tmp_path / "power.json"
    settings.write_text(json.dumps({"low_voltage": {"enabled": True}}))
    cg = CpuGovernor(settings, run=helper_runner(root, calls), sysfs=root, helper_unit_file=helper, clock=Clock())
    out = cg.set("schedutil")
    assert out["governor"] == "schedutil" and out["saved"] == "schedutil"
    assert calls == [["systemctl", "start", UNIT.format("schedutil")]]
    data = json.loads(settings.read_text())
    assert data["cpu"] == {"governor": "schedutil"} and data["low_voltage"] == {"enabled": True}   # kept
    with pytest.raises(RequestError) as e:
        cg.set("turbo")
    assert e.value.status == 422 and len(calls) == 1
    # after a reboot the kernel is back on its default: the saved choice is re-applied
    for p in root.glob("policy*"):
        (p / "scaling_governor").write_text("ondemand\n")
    again = CpuGovernor(settings, run=helper_runner(root, calls), sysfs=root, helper_unit_file=helper, clock=Clock())
    assert "ondemand -> schedutil" in again.apply_saved() and again.read()["governor"] == "schedutil"
    assert again.apply_saved() is None                 # already there: nothing to do


def test_set_refuses_without_helper_or_runner_and_reports_failures(tmp_path):
    root, helper = fake_sysfs(tmp_path)
    helper.unlink()
    cg = CpuGovernor(None, run=helper_runner(root, []), sysfs=root, helper_unit_file=helper, clock=Clock())
    with pytest.raises(RequestError) as e:
        cg.set("performance")
    assert e.value.status == 503 and "install.sh" in str(e.value)
    helper.write_text("[Service]\n")
    cg = CpuGovernor(None, sysfs=root, helper_unit_file=helper, clock=Clock())   # default runner refuses
    assert cg.run is refuse_run
    with pytest.raises(RequestError) as e:
        cg.set("performance")
    assert e.value.status == 502 and "no system access" in cg.read()["error"]
    lazy = CpuGovernor(None, run=lambda argv: None, sysfs=root, helper_unit_file=helper, clock=Clock())
    with pytest.raises(RequestError) as e:                # the command "worked" but nothing changed
        lazy.set("performance")
    assert e.value.status == 502 and "kernel reports ondemand" in str(e.value)


def test_no_cpufreq(tmp_path):
    cg = CpuGovernor(None, sysfs=tmp_path / "nothing", helper_unit_file=tmp_path / "x", clock=Clock())
    assert cg.read()["available"] is False
    with pytest.raises(RequestError):
        cg.set("performance")
