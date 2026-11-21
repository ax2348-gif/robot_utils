"""Clients of the sim world for the nodes' hardware backends (docs/sim.md, "Backends"): ``arm.WorldArmBus``
(an ArmBus), ``swerve.start_world_sim`` (world-coupled SimulatedSparks on a virtual CAN bus) and
``camera.WorldCameraBackend`` (the orbbec backend interface). Each talks to the world over its own socket."""
