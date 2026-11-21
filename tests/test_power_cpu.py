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


# -- the node's API (power/server.py) --------------------------------------------------------------

def cpu_api(tmp_path, gov="ondemand", installed=True):
    from fastapi.testclient import TestClient

    from robot_utils.nodes.power.node import PowerNode
    from robot_utils.nodes.power.server import create_app
    from robot_utils.nodes.power.sources import TetherSource, UpsSource
    from robot_utils.nodes.power.ups import FakeBus, UpsHat
    root, helper = fake_sysfs(tmp_path, gov=gov)
    if not installed:
        helper.unlink()
    calls = []
    cg = CpuGovernor(tmp_path / "power.json", run=helper_runner(root, calls), sysfs=root, helper_unit_file=helper,
                     clock=Clock())
    node = PowerNode([UpsSource(UpsHat(FakeBus())), TetherSource("main")], power_off=refuse_run,
                     settings_path=tmp_path / "power.json", audit_path=tmp_path / "shutdown.jsonl",
                     spawn=lambda fn: fn(), cpu=cg)
    return TestClient(create_app(node, start=False, identify=lambda ip: ip)), node, root, calls


def test_api_get_and_post_cpu_with_audit_and_event(tmp_path):
    c, node, root, calls = cpu_api(tmp_path)
    with c:
        assert c.get("/cpu").json()["governor"] == "ondemand"
        assert c.get("/state").json()["cpu"]["governors"] == sorted(GOVS.split())
        r = c.post("/cpu", json={"governor": "schedutil", "by": "avery"})
        assert r.status_code == 200 and r.json()["governor"] == "schedutil"
        assert calls == [["systemctl", "start", UNIT.format("schedutil")]]
        assert (root / "policy0" / "scaling_governor").read_text().strip() == "schedutil"
        assert any("CPU governor ondemand -> schedutil by avery" in e["msg"] for e in c.get("/state").json()["events"])
        log = [x for x in c.get("/log").json()["log"] if x["event"] == "cpu_governor"]
        assert log[-1]["before"] == "ondemand" and log[-1]["after"] == "schedutil" and log[-1]["by"] == "avery"
        r = c.post("/cpu", json={"governor": "turbo", "by": "avery"})
        assert r.status_code == 422 and "unknown governor" in r.json()["detail"] and len(calls) == 1
        assert [x for x in c.get("/log").json()["log"] if x["event"] == "cpu_governor"][-1]["status"] == "failed"
        assert json.loads((tmp_path / "power.json").read_text())["cpu"] == {"governor": "schedutil"}


def test_api_cpu_without_helper_and_without_cpu_control(tmp_path):
    c, node, root, calls = cpu_api(tmp_path, installed=False)
    with c:
        st = c.get("/cpu").json()
        assert st["helper_installed"] is False
        r = c.post("/cpu", json={"governor": "performance"})
        assert r.status_code == 503 and "install.sh" in r.json()["detail"] and calls == []
        node.cpu = None
        assert c.get("/cpu").status_code == 404 and "cpu" not in c.get("/state").json()


def test_node_reapplies_saved_governor_at_start_and_notes_it(tmp_path):
    (tmp_path / "power.json").write_text(json.dumps({"cpu": {"governor": "performance"}}))
    c, node, root, calls = cpu_api(tmp_path)              # the kernel booted on ondemand
    node.apply_cpu_saved()
    assert calls == [["systemctl", "start", UNIT.format("performance")]]
    assert any("ondemand -> performance (saved choice)" in e["msg"] for e in node.events.list())


def test_state_reads_cpu_at_most_once_a_second(tmp_path):
    c, node, root, calls = cpu_api(tmp_path)
    reads = []
    orig = node.cpu.read
    node.cpu.read = lambda: reads.append(1) or orig()
    t = [10.0]
    node.clock = lambda: t[0]
    for _ in range(4):
        node.state()
        t[0] += 0.25
    assert len(reads) == 1
    t[0] += 1
    node.state()
    assert len(reads) == 2
