"""WBC --timing (nodes/wbc/__main__.py): lenient stall limits only for a sim on macOS, never lowered."""
from robot_utils.nodes.wbc.__main__ import LENIENT_TIMING, apply_timing


def test_timing_lenient_only_on_macos_or_asked_and_never_lowers():
    assert apply_timing({}, "auto", "linux") == {}
    assert apply_timing({"arm_stale": 0.2}, "strict", "darwin") == {"arm_stale": 0.2}
    mac = apply_timing({"arm_stale": 0.2}, "auto", "darwin")
    assert mac["arm_stale"] == LENIENT_TIMING["arm_stale"] and mac["ctl_heartbeat"] == LENIENT_TIMING["ctl_heartbeat"]
    assert apply_timing({"arm_stale": 9.0}, "lenient", "linux")["arm_stale"] == 9.0
