"""REV MAXSwerve drive base (SPARK Flex over CAN via sparkcan), as a library for a swerve node.

Modules (see README.md):
    constants             geometry, CAN IDs, gains, limits
    kinematics            pure twist <-> wheel math
    drive                 SwerveDrive: control-loop thread, command timeout, error reporting
    setup_sparks          python -m robot_utils.nodes.swerve.setup_sparks <iface> [--apply]
    steering_calibration  python -m robot_utils.nodes.swerve.steering_calibration <iface>

Submodules are not imported here, so ``python -m`` on them runs cleanly and importing the
package does not need sparkcan.
"""
