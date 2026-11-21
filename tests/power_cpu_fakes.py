"""A fake cpufreq sysfs tree and a stand-in for the governor helper unit, for the power node's CPU tests."""

GOVS = "conservative ondemand userspace powersave performance schedutil"


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
