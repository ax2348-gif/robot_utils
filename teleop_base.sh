#!/bin/bash
# Start the swerve teleop web page: open http://botcore-desktop:8080. Ctrl+C stops it.
cd "$HOME/robot_utils/rev_ctl/examples" && PYTHONPATH="$HOME/robot_utils/.venv/bin/python" exec python3 teleop.py slcan:/dev/ttyACM0
