"""python -m robot_utils.nodes.power [--fake] [--name power] [--bus 1] [--addr 0x2d] [--config nodes.yaml]

Runs the power node (docs/power.md): the Pi's UPS HAT (E) on I2C (battery, input, charger), the
main supply (a bench tether today: no telemetry), low-voltage auto-shutdown and remote shutdown.

--fake uses a simulated HAT (the captured reading, animated) with a back-filled 24 h history, and
never powers anything off: at the end of a shutdown countdown it e-stops the nodes it can reach,
then only logs "would power off". Without --fake, the real power-off arms the HAT (0x01 <- 0x55)
and runs `systemctl poweroff`.

Every UPS reading (1 Hz) is logged to ~/.local/state/robot_utils/power/log/ups-YYYY-MM-DD.csv (fsynced
every 5 s, kept 30 days), and input loss, draining on input, a low cell or unbalanced cells go to the journal.

The CPU governor toggle (GET/POST /cpu) switches through a root helper unit (deploy/power/install.sh, once);
the saved choice is re-applied at startup. --fake reads the local cpufreq but never switches it.

Serves on <runtime dir>/robot/<name>.sock = http://<robot>/nodes/<name>/; --port is for debugging.
"""
import argparse
import logging
import time

from ..common.serve import add_serve_args, serve


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--fake", action="store_true", help="simulated UPS HAT; never powers off")
    p.add_argument("--bus", type=int, default=1, help="I2C bus of the UPS HAT (default 1)")
    p.add_argument("--addr", type=lambda s: int(s, 0), default=0x2d, help="I2C address (default 0x2d)")
    p.add_argument("--config", default=None,
                   help="node registry for the e-stop before power-off (default: the packaged nodes.yaml)")
    p.add_argument("-v", "--verbose", action="store_true")
    add_serve_args(p, "power")
    a = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("httpx").setLevel(logging.WARNING)
    from .cpu import CpuGovernor, refuse_run, system_run
    from .node import HISTORY_S, HISTORY_LEN, PowerNode, system_power_off
    from .server import create_app
    from .sources import TetherSource, UpsSource, history_row_of
    from .ups import BusUnavailable, SimBus, UpsHat, decode, open_bus

    log = logging.getLogger("power")
    bus = None
    if a.fake:
        bus = SimBus()
        ups, err = UpsHat(bus, a.addr), None

        def power_off():
            log.warning("--fake: would arm the UPS power-off and run systemctl poweroff now (doing nothing)")
    else:
        try:
            ups, err = UpsHat(open_bus(a.bus), a.addr), None
        except BusUnavailable as e:
            ups, err = None, str(e)
            log.error("UPS HAT unavailable: %s", e)
        power_off = system_power_off(ups)
    sources = [UpsSource(ups, error=err), TetherSource("main", role="main")]
    paths = PowerNode.default_paths()
    if a.fake:
        paths["log_dir"] = None             # a simulated HAT must not write the robot's power log
    # the CPU governor toggle: the real runner only without --fake (it reads this machine's cpufreq either way)
    cpu = CpuGovernor(paths["settings_path"], run=refuse_run if a.fake else system_run)
    node = PowerNode(sources, power_off=power_off, fake=a.fake, name=a.name, config=a.config, cpu=cpu, **paths)
    if a.fake:      # a 24 h history on the simulated curve, so the plots have something to show
        now, wall, rows = time.monotonic(), time.time(), []
        for k in range(HISTORY_LEN, 0, -1):
            t = now - k * HISTORY_S
            rows.append(history_row_of(t, wall - k * HISTORY_S, decode(bus.regs_at(t))))
        node.backfill("ups", rows)
        from .powerloss import KEEP_S, sample_of     # the power-loss estimate's minutes, on the same curve
        node.loss.extend(sample_of(wall - k, decode(bus.regs_at(now - k))) for k in range(int(KEEP_S), 0, -5))
    else:
        try:
            n = node.restore_history()
            log.info("power log: %s (history: %d rows restored)", node.disklog.dir if node.disklog else "off", n)
        except Exception as e:  # noqa: BLE001 - an unreadable old log must not stop the node
            log.warning("power log: history not restored: %s", e)
    app = create_app(node)
    serve(app, a, banner=f"power node {a.name!r} ({'fake UPS' if a.fake else f'UPS HAT on i2c-{a.bus} 0x{a.addr:02x}'})")


if __name__ == "__main__":
    main()
