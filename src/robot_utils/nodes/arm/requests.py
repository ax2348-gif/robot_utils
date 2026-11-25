"""The arm node's requests that need no access to the bus or the control state beyond a consistent copy of it:
validation, the track IK, planning (in the planner process), the read-only lookups. ``ArmRequests`` is a mixin of
both the in-process node (``ArmNode``: the control thread in the same process) and the server side of the
two-process node (``proc_node.ProcNode``: the control loop in its own process, docs/arm.md "Processes"), so both run
the very same code; each supplies the hooks:

- ``_ready(token)``: the lease renewed, no e-stop, enabled with the motors on (else ``RequestError``);
- ``_new_command()``: a command arrived (the jam detector's refire counts commands);
- ``_track_context()``: (the seq of the track in force or about to engage, or None; the measured pose to start from);
- ``_track_apply(q, pose, twist, twist_frame, seq, gripper_args)``: put a solved track in force;
- ``_arm_start_now()``: the measured pose a new command starts from (``RequestError`` if it cannot start there);
- ``_start(traj, by, q0)``: put a planned motion in force; ``_stowed(why)``: stowed, motors off;
- ``_servo_cmd()``: a copy of the command while servoing, else None; ``_bus_names()``: the motors' names or None;
- attributes ``kin``, ``reading``, ``soft``, ``settings``, ``state``, ``hub``, ``clock``, ``description_node``,
  ``_aib``, ``_limited``, ``_plan_lock``.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import pinocchio as pin

from ..common.lease import RequestError
from . import config as C
from . import queries as Q
from .config import N
from .gripper import check_gripper
from .kin import ArmKinematics
from .motion import PlanError
from .planner import Planner
from .telemetry import tracking as tracking_of_row

ENGAGE_LIMIT_TOL = math.radians(3.0)   # servoing may start with a joint this far outside its limits
TRACK_IK_ITER = 30            # the Quest teleop's per-command IK iterations (warm-started)
TRACK_MAX_JUMP = math.radians(20.0)    # a track goal this far (any joint) from the measured pose is not taken (use servo)
STOW_NEAR = math.radians(3.0)  # every joint this close to home: stow turns the motors off at once


class ArmRequests:
    """Mixin (module docstring)."""

    def _limit_problem(self, q) -> Optional[str]:
        """Why servoing must not start from ``q`` (a joint further outside its limits than
        ENGAGE_LIMIT_TOL: the command would have to snap it back), or None."""
        kin = self.kin
        out = np.maximum(kin.lo - q, q - kin.hi)
        for i in np.flatnonzero(out > ENGAGE_LIMIT_TOL):
            return (f"joint{i + 1} is at {math.degrees(q[i]):+.1f}°, {math.degrees(out[i]):.1f}° outside its limits "
                    f"{math.degrees(kin.lo[i]):+.1f}..{math.degrees(kin.hi[i]):+.1f}°: move it back by hand first "
                    "(the arm floats while enabled)")
        return None

    _check_gripper = staticmethod(check_gripper)

    def track_cmd(self, token, pose: pin.SE3, twist=None, twist_frame: str = "arm", seq: int = 0,
                  gripper: Optional[dict] = None) -> dict:
        self._ready(token)
        self._new_command()
        if twist is not None:
            twist = np.asarray(twist, dtype=float)
            if twist.shape != (6,) or not np.all(np.isfinite(twist)):
                raise ValueError("twist must be 6 finite numbers [vx, vy, vz, wx, wy, wz]")
        twist_frame = "arm" if twist_frame == "base" else twist_frame      # "base": the old name of the arm frame
        if twist_frame not in ("arm", "tcp"):
            raise ValueError("twist_frame must be arm or tcp")
        scaled = False
        if twist is not None:          # bounded by the soft servo speeds
            lin, ang = float(np.linalg.norm(twist[:3])), float(np.linalg.norm(twist[3:]))
            if lin > self.soft.lin_vel:
                twist[:3] *= self.soft.lin_vel / lin
                scaled = True
            if ang > self.soft.ang_vel:
                twist[3:] *= self.soft.ang_vel / ang
                scaled = True
        g_args = self._check_gripper(**gripper) if gripper is not None else None
        prev_seq, seed = self._track_context()       # the measured pose: IK seed and jump reference
        if prev_seq is not None and seq and prev_seq and seq < prev_seq:
            return {"ok": False, "seq": seq, "ik": None, "limited": self._limited,
                    "reason": f"stale: seq {seq} is older than {prev_seq} already applied"}
        r = self.kin.ik(pose, seed, max_iter=TRACK_IK_ITER)
        out = {"ok": r.ok, "seq": seq, "ik": r.public(), "limited": self._limited, "twist_scaled": scaled,
               "reason": r.reason}
        if r.ok:
            jump = float(np.max(np.abs(r.q - seed)))
            if jump > TRACK_MAX_JUMP:
                out.update(ok=False, reason=f"too far for track: a joint is {math.degrees(jump):.0f}° from where it is "
                                            f"(max {math.degrees(TRACK_MAX_JUMP):.0f}°); use servo for large moves")
        if not out["ok"]:
            return out
        self._track_apply(r.q, pose, twist, twist_frame, int(seq), g_args)
        return out

    def _plan(self, job: str, *args, **kwargs):
        """Plans run in the planner process (planner.py), one at a time: in this process they
        would hold the interpreter long enough to starve the control loop."""
        if not self._plan_lock.acquire(blocking=False):
            raise RequestError(409, "already planning another motion; retry")
        try:
            return Planner.run(job, self.kin, *args, **kwargs)
        except PlanError as e:
            raise RequestError(422, f"not planned (nothing moved): {e}", extra=getattr(e, "extra", None))
        finally:
            self._plan_lock.release()

    def servo_cmd(self, token, pose: Optional[pin.SE3] = None, name: Optional[str] = None,
                  lin_vel=None, lin_acc=None, ang_vel=None, ang_acc=None, duration=None,
                  max_rot_deg: Optional[float] = None) -> dict:
        """Blocking (planning): call from a worker thread."""
        self._ready(token)
        h, s = self.settings.hard, self.soft
        lim = {}
        for k, v in (("lin_vel", lin_vel), ("lin_acc", lin_acc), ("ang_vel", ang_vel), ("ang_acc", ang_acc)):
            cap = getattr(s, k)
            if v is not None and not (math.isfinite(float(v)) and 0 < float(v) <= getattr(h, k)):
                raise ValueError(f"{k} must be within 0..{getattr(h, k):g}")
            lim[k] = min(float(v), cap) if v is not None else cap
        if name is not None and name not in self.settings.named_poses:
            raise RequestError(422, f"unknown named pose {name!r} (have: {', '.join(self.settings.named_poses)})")
        q0 = self._arm_start_now()
        if name is not None:
            traj = self._plan("joint", q0, [self.settings.named_poses[name]], s.home_vel, s.joint_vel,
                              kind="pose", name=name)
        else:
            rot_tol = None if max_rot_deg is None else math.radians(max_rot_deg)
            traj = self._plan("servo", q0, pose, joint_vel=s.joint_vel, duration=duration, rot_tol=rot_tol, **lim)
        return self._start(traj, "servo", q0)

    def _home_traj(self, kind: str):
        st = self.settings
        q0 = self._arm_start_now()
        way = [st.named_poses[n] for n in st.home_via] + [st.named_poses["home"]]
        return q0, self._plan("joint", q0, way, self.soft.home_vel, self.soft.joint_vel, kind=kind, name="home")

    def home(self, token) -> dict:
        self._ready(token)
        q0, traj = self._home_traj("home")
        return self._start(traj, "home", q0)

    def stow(self, token) -> dict:
        """Safe homing, then motors off (damping) at home. Already within STOW_NEAR of home on
        every joint (measured): motors off at once."""
        self._ready(token)
        r = self.reading
        if r is None:
            raise RequestError(503, "no reading yet")
        home = self.settings.named_poses["home"]
        if np.all(np.abs(r.q - home) <= STOW_NEAR):
            self._stowed("stowed: already at home")
            return {"stowed": True, "motion": None}
        q0, traj = self._home_traj("stow")
        return {"stowed": False, "motion": self._start(traj, "stow", q0)}

    def chunk(self, token, poses: list, dt: float, t0: Optional[float] = None, gaps_mm=None,
              blend_s: float = 0.1) -> dict:
        self._ready(token)
        if not 0 < len(poses) <= 1000:
            raise ValueError("a chunk has 1..1000 poses")
        if gaps_mm is not None and len(gaps_mm) != len(poses):
            raise ValueError("gaps_mm must have one value per pose")
        if gaps_mm is not None and not all(math.isfinite(g) and 0.0 <= g / 1000 <= C.GAP_MAX + 1e-9 for g in gaps_mm):
            raise ValueError(f"gaps_mm must be within 0..{C.GAP_MAX * 1000:.1f}")
        if not 0.0 <= blend_s <= 1.0:
            raise ValueError("blend_s must be within 0..1")
        if t0 is not None and not math.isfinite(t0):
            raise ValueError("t0 must be finite")
        q_now = self._arm_start_now()
        now = self.clock()
        traj = self._plan("chunk", q_now, poses, None if gaps_mm is None else [g / 1000 for g in gaps_mm],
                          t0=now if t0 is None else float(t0), dt=dt, now=now, blend_s=blend_s,
                          joint_vel=self.soft.joint_vel, q_now=q_now)
        return self._start(traj, "chunk", q_now)

    def limits(self) -> dict:
        return {"hard": self.settings.hard.public(), "soft": self.soft.public()}

    # reads -----------------------------------------------------------------------------------------

    def _need_kin(self) -> ArmKinematics:
        return Q.need_kin(self.kin)

    def ik_check(self, pose: pin.SE3, seed: str = "command", max_rot_deg: Optional[float] = None,
                 render: Optional[dict] = None) -> dict:
        """IK seed only (never taken as where the arm is): "command" = the command while servoing,
        else the measured pose; "measured" = the measured pose. With ``render`` (view options,
        render.VIEW_DEFAULTS) the answer also carries ``image``: a PNG data URL of the query."""
        return Q.ik_check(self.kin, self.reading, self._servo_cmd(), pose, seed, max_rot_deg, render)

    def ik_image(self, pose: pin.SE3, seed: str = "command", max_rot_deg: Optional[float] = None,
                 view: Optional[dict] = None) -> tuple[dict, bytes]:
        """The IK check and its picture (render.py), both in the planner process: (IK answer, PNG).
        Shows the measured arm (and gripper gap), so it needs a reading. No lease; nothing moves."""
        return Q.ik_image(self.kin, self.reading, self._servo_cmd(), pose, seed, max_rot_deg, view)

    def kinematics(self, q, frame: str = "arm") -> dict:
        return Q.kinematics(self.kin, q, frame)

    def kinematics_batch(self, qs: list, frame: str = "arm") -> list:
        """Lookups at many configurations, in the planner process (no lease; nothing moves)."""
        return Q.kinematics_batch(self.kin, qs, frame)

    def kinematics_now(self, frame: str = "arm", which: str = "measured") -> dict:
        """which "command": the command, only while servoing (409 otherwise)."""
        r = self.reading
        if r is None:
            raise RequestError(503, "no reading yet")
        if which == "command":
            q = self._servo_cmd()
            if q is None:
                raise RequestError(409, f"no command: the arm is not servoing ({self.state})")
        else:
            q = r.q
        out = self.kinematics(q, frame)
        out["which"] = which
        return out

    # -- geometry as numbers (read only) -------------------------------------------------------------

    def arm_in_base(self) -> dict:
        """The arm frame's pose in the robot base frame (p_base = T p_arm), from the description node's
        ``arm`` frame (cached 10 s); its shipped default if that node is unreachable. Says which."""
        v, self._aib = Q.arm_in_base(self.description_node, self._aib)
        return v

    def geometry(self, q=None, pose: Optional[pin.SE3] = None, seed: str = "measured", max_rot_deg: Optional[float] = None,
                 gap_mm: Optional[float] = None) -> dict:
        """Gripper and finger geometry, and every box of the arm, as numbers in both the arm and the robot
        base frame: at the measured configuration, at ``q``, or at the IK solution of a TCP ``pose``."""
        return Q.geometry(self.kin, self.reading, self.ik_check, self.arm_in_base, q, pose, seed, max_rot_deg, gap_mm)

    def links(self, q=None) -> list:
        """The joint frames, end_link and tcp at ``q`` (default measured), for drawing."""
        return Q.links(self.kin, self.reading, q)

    def joints(self) -> dict:
        r = self.reading
        if r is None:
            raise RequestError(503, "no reading yet")
        names = self._bus_names() or [f"joint{i}" for i in range(1, N + 1)]
        lo = hi = None
        if self.kin is not None:
            lo, hi = self.kin.lo.tolist(), self.kin.hi.tolist()
        q_cmd = self._servo_cmd()
        q_cmd = None if q_cmd is None else q_cmd.tolist()
        return {"names": names[:N], "q": r.q.tolist(), "q_mot": r.q_mot.tolist(), "qd": r.qd.tolist(),
                "tau": r.tau.tolist(), "q_cmd": q_cmd,
                "status": r.status.tolist(), "t_mos": r.t_mos.tolist(), "t_rotor": r.t_rotor.tolist(),
                "limits": {"lo": lo, "hi": hi}, "t": r.t}

    def tracking(self) -> dict:
        last = self.hub.recent(1, "tick")
        if not len(last):
            raise RequestError(503, "no tick yet")
        return tracking_of_row(last[0])
