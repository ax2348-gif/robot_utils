#!/bin/bash
# Root-only measurements of the arm's control-loop I/O (run on the Pi with sudo). CAPTURE ONLY: starts nothing,
# moves nothing; the load scenario (arm servoing, base, recorder) is run separately while these capture.
#
#   sudo bash ~/profiling/root_tests.sh setup                 # once: load usbmon; check/install tools (asks apt)
#   sudo bash ~/profiling/root_tests.sh usbmon LABEL [SEC]    # USB bus 2 (the arm adapter alone) -> usbmon_LABEL.txt
#   sudo bash ~/profiling/root_tests.sh perftrace LABEL [SEC] # control thread syscall summary + slow syscalls
#   sudo bash ~/profiling/root_tests.sh offcpu LABEL [SEC]    # where the control thread blocks (kernel stacks)
#   sudo bash ~/profiling/root_tests.sh sched LABEL [SEC]     # scheduler latency per task (kworkers, poller)
#   sudo bash ~/profiling/root_tests.sh irq CPU|reset         # move the arm adapter's xHCI IRQ (133) / back to CPU0
#   sudo bash ~/profiling/root_tests.sh all LABEL             # usbmon + perftrace + offcpu, 20 s each, in sequence
#
# Outputs land in ~botcore/profiling/root_<LABEL>_*.txt, owned by botcore.
set -eu
CMD=${1:?usage: setup | usbmon | perftrace | offcpu | sched | irq | all}
LABEL=${2:-test}
SEC=${3:-20}
OUT=/home/botcore/profiling
IRQ=133            # xhci-hcd:usb2: the arm's HDSC adapter is alone on USB bus 2 (2e88:4603, device 002)
cd "$OUT"

arm_pid() { pgrep -f "robot_utils.nodes.arm --name arm" | head -1; }
ctl_tid() {     # the arm node's SCHED_FIFO control thread
    local p; p=$(arm_pid); [ -n "$p" ] || { echo "arm node not running" >&2; exit 1; }
    for t in $(ls /proc/$p/task); do chrt -p $t 2>/dev/null | grep -q SCHED_FIFO && { echo $t; return; }; done
    echo "no SCHED_FIFO thread in the arm node" >&2; exit 1
}
own() { chown botcore:botcore "$@"; }

case $CMD in
setup)
    modprobe usbmon && echo "usbmon loaded"
    ls /sys/kernel/debug/usb/usbmon/ | head
    command -v perf >/dev/null || echo "perf missing: apt install linux-tools-$(uname -r) linux-tools-raspi"
    command -v offcputime-bpfcc >/dev/null || echo "offcputime missing: apt install bpfcc-tools (for 'offcpu')"
    ;;
usbmon)
    [ -e /sys/kernel/debug/usb/usbmon/2u ] || modprobe usbmon
    f=$OUT/root_${LABEL}_usbmon2.txt
    echo "capturing USB bus 2 for $SEC s -> $f"
    timeout "$SEC" cat /sys/kernel/debug/usb/usbmon/2u > "$f" || true
    own "$f"; wc -l "$f"
    ;;
perftrace)
    t=$(ctl_tid); f=$OUT/root_${LABEL}_perftrace.txt
    echo "perf trace on control thread $t for $SEC s -> $f"
    { echo "== summary"; perf trace -s -t "$t" -- sleep "$SEC" 2>&1
      echo "== syscalls over 1 ms"; perf trace --duration 1 -t "$t" -- sleep "$SEC" 2>&1 | head -400; } > "$f"
    own "$f"; grep -A40 "== summary" "$f" | head -50
    ;;
offcpu)
    t=$(ctl_tid); f=$OUT/root_${LABEL}_offcpu.txt
    echo "off-CPU kernel stacks of control thread $t for $SEC s -> $f"
    offcputime-bpfcc -K -t "$t" "$SEC" > "$f" 2>&1 || true
    own "$f"; tail -60 "$f"
    ;;
sched)
    f=$OUT/root_${LABEL}_sched.txt
    perf sched record -o /tmp/perf_sched.data -- sleep "$SEC" >/dev/null 2>&1
    perf sched latency -i /tmp/perf_sched.data --sort max 2>/dev/null | head -60 > "$f"
    rm -f /tmp/perf_sched.data; own "$f"; cat "$f"
    ;;
irq)
    if [ "$LABEL" = reset ]; then echo 1 > /proc/irq/$IRQ/smp_affinity; else
        echo $(printf '%x' $((1 << LABEL))) > /proc/irq/$IRQ/smp_affinity; fi
    echo "IRQ $IRQ affinity: $(cat /proc/irq/$IRQ/smp_affinity) (1 = CPU0, the default)"
    ;;
all)
    "$0" usbmon "$LABEL" 20; "$0" perftrace "$LABEL" 10; "$0" offcpu "$LABEL" 20
    ;;
*) echo "unknown command $CMD" >&2; exit 2 ;;
esac
