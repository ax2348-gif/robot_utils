"""Client-side data collection (docs/collect.md): record a session's streams continuously, index the
episodes from the operator stream's events (reconciled against the Pi event log), cut them by Pi time.

    python -m robot_utils.collect run --robot http://botcore --out data --task "pick the cup" --start-teleop
    python -m robot_utils.collect cut data/20261121-143000
"""
