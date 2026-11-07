"""Landing page for the robot's HTTP nodes: one tile per node, global E-STOP.

Stateless and off the control path: every node serves its own page and e-stop; this process
only indexes them (``python -m robot_utils.frontend``, see docs/frontend.md).
"""
