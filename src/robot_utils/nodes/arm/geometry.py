"""The arm's shape as boxes, shared by the page (page.py) and the agent render (render.py)."""

# The arm drawn as wireframe boxes: [frame, cx, cy, cz, sx, sy, sz], centre and size in mm, axis-aligned
# in the frame. Derived offline from the visual meshes of rebot_ctl/urdf/DM/urdf/ReBot_Arm_DM_mount_right.urdf
# (meshes/visual + meshes/shared, read with numpy): every vertex in its link frame (the visual origin
# applied: base_link yaw -pi/2; link6's meshes moved into end_link through end_joint), and each box the
# bounding box of the vertices in one region of the link:
#   base:  plate = vertices outside |x|, |y| <= 47 mm; column = inside (up to joint 1)
#   link1, link2, link5: the whole link
#   link3, link4 (L-shaped): hub at the joint = y > -21 mm (link3) / y > -30 mm (link4); beam = the rest
#   gripper (end_link frame; x points out between the jaws, origin at the fingertips, y = jaw travel):
#     motor block = link6 (the gripper motor and its flange; it turns with joint 6) plus the gripper
#       base's central housing (|y| < 32 mm, x < -81 mm);
#     rail = the gripper base's bar (|y| >= 32 mm), its full width;
#   finger (finger_left_link at gap 0; the right one is its mirror in y), moved along end_link y by
#     half the measured gap:
#     jaw root = the carriage and travel stop + the finger mesh behind x = -55 mm (the rear plate);
#     jaw = the finger mesh in front of it (the prong), extended back to the root's front face (x -59
#       mm: the mesh has no vertices in between). The prong tapers from +-16.5 mm (z) at its root
#       to +-4 mm at the tip; the box bounds it. The crossed rack moves with the finger but runs
#       inside the rail, so it is not drawn.
ARM_BOXES = [
    ["base", 0.0, 0.0, 7.0, 200.0, 140.0, 14.0],        # mounting plate
    ["base", 0.0, -0.2, 43.8, 92.0, 92.3, 87.7],        # column, joint 1 motor
    ["link1", 6.7, 0.0, 42.0, 83.8, 77.2, 83.9],        # shoulder bracket, joint 2 motor
    ["link2", -132.0, 0.0, -31.6, 321.0, 57.0, 67.4],   # upper arm
    ["link3", 6.5, 2.6, -31.1, 57.8, 39.9, 80.2],       # elbow hub
    ["link3", 122.2, -52.0, -31.1, 297.8, 60.9, 80.2],  # forearm, joint 4 motor
    ["link4", 5.1, -0.2, -30.0, 66.3, 56.2, 66.8],      # wrist hub
    ["link4", 54.0, -59.4, -30.0, 127.5, 57.3, 66.8],   # wrist beam, joint 5 motor
    ["link5", -2.8, 0.0, 35.5, 70.0, 77.0, 85.0],       # joint 6 motor housing
    ["end", -121.0, 0.0, 0.0, 78.5, 63.6, 77.0],        # gripper motor block
    ["end", -88.2, 0.0, 0.0, 30.0, 184.1, 20.6],        # gripper linear rail
    ["finger", -70.8, 17.1, 0.0, 23.7, 34.3, 39.2],     # jaw root (carriage)
    ["finger", -29.4, 6.8, 0.0, 58.9, 13.6, 32.9],      # jaw (prong; inner face at y = 0)
]
