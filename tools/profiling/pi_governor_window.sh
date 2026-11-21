#!/bin/bash
# One governor measurement window on the Pi: pi_governor_window.sh LABEL SECONDS
# Runs pi_load_sample.py (CPU, PSI, /health latency, arm/swerve/estimator loop and lag) and alongside it records
# the cpufreq transitions/s, the firmware mailbox IRQs/s, the clock range, and the UPS pack / input power over
# the same window from the power node's disk log. Read-only.
LABEL=${1:?label}; SEC=${2:-120}
cd ~/profiling
P=/sys/devices/system/cpu/cpufreq/policy0
gov=$(cat $P/scaling_governor)
tr0=$(cat $P/stats/total_trans 2>/dev/null || echo 0)
mb0=$(awk '/mailbox/ {s=0; for(i=2;i<=5;i++) s+=$i; print s}' /proc/interrupts)
w0=$(date +%s)
( for i in $(seq 1 $SEC); do cat $P/scaling_cur_freq; sleep 1; done ) > freq_$LABEL.txt &
~/robot_utils/.venv/bin/python pi_load_sample.py $SEC $LABEL > gov_$LABEL.json
wait
w1=$(date +%s)
tr1=$(cat $P/stats/total_trans 2>/dev/null || echo 0)
mb1=$(awk '/mailbox/ {s=0; for(i=2;i<=5;i++) s+=$i; print s}' /proc/interrupts)
f=~/.local/state/robot_utils/power/log/ups-$(date +%F).csv
pw=$(awk -F, -v a=$w0 -v b=$w1 'NR>1 && $1>=a && $1<=b && $10!="" {n++; p+=$10; iv+=$11*$12} END{if(n) printf "pack %.2f W, input %.2f W, n=%d", p/n, iv/n, n}' $f)
fr=$(sort -n freq_$LABEL.txt | awk '{a[NR]=$1} END{printf "min %.2f GHz, median %.2f, max %.2f", a[1]/1e6, a[int(NR/2)+1]/1e6, a[NR]/1e6}')
echo "{\"label\":\"$LABEL\",\"governor\":\"$gov\",\"seconds\":$((w1-w0)),\"cpufreq_transitions_s\":$(( (tr1-tr0)/(w1-w0) )),\"mailbox_irq_s\":$(( (mb1-mb0)/(w1-w0) )),\"clock\":\"$fr\",\"power\":\"$pw\"}" > govmeta_$LABEL.json
cat govmeta_$LABEL.json
