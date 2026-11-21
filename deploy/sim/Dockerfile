# The sim stack's image (docs/sim.md, "Docker"): robot_utils with the MuJoCo sim extra, nginx, software OpenGL.
# Built by docker-compose.yml next to this file (context: the repository root; the rev_ctl and rebot_ctl submodules
# come in as named build contexts). One image for every service: the stack launcher and nginx.
#
#   docker compose -f deploy/sim/docker-compose.yml up --build
#
# Rendering: OSMesa (software, any machine) by default; EGL on an NVIDIA GPU with docker-compose.gpu.yml (erid).
FROM python:3.12-slim-bookworm

ENV DEBIAN_FRONTEND=noninteractive PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1 PYTHONUNBUFFERED=1
# nginx-light: the port-80 proxy (frontend/proxy.py writes its config); libosmesa6: MuJoCo's software renderer;
# libegl1 + libgl1 (glvnd): EGL / GLX, used with an NVIDIA GPU (the container toolkit adds the vendor library)
RUN apt-get update && apt-get install -y --no-install-recommends \
        nginx-light libosmesa6 libegl1 libgl1 libglib2.0-0 libgomp1 ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies of the nodes that run in the sim (not lerobot / torch: no node imports them). motorbridge is the
# arm's CAN driver: reBotArm_control_py imports it, the sim never calls it (PyPI's is enough here; the robot uses
# the fork's wheel). PyOpenGL: MuJoCo's OSMesa / EGL backends.
RUN pip install "fastapi>=0.110" "uvicorn>=0.29" "uvloop>=0.19" "httptools>=0.6.3" "httpx>=0.27" "pyyaml>=6" \
        numpy pillow opencv-python-headless "pin>=3.9" "mujoco>=3.2" PyOpenGL pyserial "motorbridge>=0.4.9"

# the submodules (named build contexts: their checkouts must be populated, git submodule update --init rev_ctl rebot_ctl)
COPY --from=rev_ctl . /opt/robot_utils/rev_ctl
COPY --from=rebot_ctl . /opt/robot_utils/rebot_ctl
RUN test -f /opt/robot_utils/rev_ctl/pyproject.toml && test -f /opt/robot_utils/rebot_ctl/pyproject.toml \
        || (echo "rev_ctl / rebot_ctl are empty: run git submodule update --init rev_ctl rebot_ctl" && false) \
    && pip install --no-deps -e /opt/robot_utils/rev_ctl -e /opt/robot_utils/rebot_ctl

COPY pyproject.toml README.md /opt/robot_utils/
COPY src /opt/robot_utils/src
COPY deploy/sim /opt/robot_utils/deploy/sim
RUN pip install --no-deps -e /opt/robot_utils

# Run as a user (uid 1000), like the robot's user services. The sockets' directory is a volume shared with nginx.
RUN useradd -m -u 1000 robot && mkdir -p /run/robot-rt/robot /home/robot/.local/share /home/robot/.local/state \
    && chown -R robot:robot /run/robot-rt /opt/robot_utils /home/robot
USER robot
WORKDIR /home/robot
ENV XDG_RUNTIME_DIR=/run/robot-rt MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
CMD ["python", "-m", "robot_utils.nodes.sim.stack", "--host", "127.0.0.1", "--port", "8090"]
