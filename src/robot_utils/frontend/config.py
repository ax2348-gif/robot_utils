"""Node registry: which nodes the landing page shows (``nodes.yaml``)."""

from __future__ import annotations

import re
from dataclasses import asdict, dataclass
from importlib import resources
from pathlib import Path

import yaml

DEFAULT_CONFIG = "nodes.yaml"  # packaged next to this module
_NAME = re.compile(r"^[A-Za-z0-9_.-]+$")
_MODULE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)*$")


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class StartRecipe:
    """How the frontend may start a node: ``python -m <module> --name <instance> <args...>``
    (the frontend's own interpreter, i.e. the robot venv)."""
    module: str
    args: tuple[str, ...] = ()

    def argv(self, python: str, name: str) -> list[str]:
        return [python, "-m", self.module, "--name", name, *self.args]


@dataclass(frozen=True)
class NodeEntry:
    """One node instance. On this machine (no ``host``) it is reached over its unix socket
    ``<runtime dir>/robot/<name>.sock``; ``port`` is then optional (a debug TCP port the node was
    told to open, checked as "already up"). On another machine, ``host`` and ``port`` are needed."""
    name: str
    port: int | None = None
    host: str | None = None
    enabled: bool = True
    start: StartRecipe | None = None  # None: the frontend cannot start this node
    kind: str | None = None           # camera, swerve, estimator, ... (default: the name)

    @property
    def node_kind(self) -> str:
        return self.kind or self.name

    def as_dict(self) -> dict:
        d = asdict(self)
        d["start"] = None if self.start is None else {"module": self.start.module, "args": list(self.start.args)}
        return d


def parse_config(data: object, source: str = "<config>") -> list[NodeEntry]:
    if data is None:
        return []
    if not isinstance(data, dict) or not isinstance(data.get("nodes", []), (list, type(None))):
        raise ConfigError(f"{source}: expected a mapping with a 'nodes:' list")
    nodes: list[NodeEntry] = []
    seen: set[str] = set()
    for i, raw in enumerate(data.get("nodes") or []):
        where = f"{source}: nodes[{i}]"
        if not isinstance(raw, dict):
            raise ConfigError(f"{where}: expected a mapping like {{name: swerve, kind: swerve}}")
        unknown = set(raw) - {"name", "kind", "port", "host", "enabled", "start"}
        if unknown:
            raise ConfigError(f"{where}: unknown keys {sorted(unknown)}")
        name = raw.get("name")
        if not isinstance(name, str) or not _NAME.match(name):
            raise ConfigError(f"{where}: 'name' must be a short identifier, got {name!r}")
        if name in seen:
            raise ConfigError(f"{where}: duplicate name {name!r}")
        kind = raw.get("kind")
        if kind is not None and (not isinstance(kind, str) or not _NAME.match(kind)):
            raise ConfigError(f"{where}: 'kind' must be a short identifier, got {kind!r}")
        port = raw.get("port")
        if port is not None and (isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535):
            raise ConfigError(f"{where}: 'port' must be an integer 1-65535, got {port!r}")
        host = raw.get("host")
        if host is not None and (not isinstance(host, str) or not host.strip()):
            raise ConfigError(f"{where}: 'host' must be a hostname or omitted, got {host!r}")
        if host is not None and port is None:
            raise ConfigError(f"{where}: a node on another machine ('host') needs a 'port'")
        enabled = raw.get("enabled", True)
        if not isinstance(enabled, bool):
            raise ConfigError(f"{where}: 'enabled' must be true or false, got {enabled!r}")
        start = _parse_start(raw.get("start"), where)
        if start is not None and host is not None:
            raise ConfigError(f"{where}: 'start' only works for nodes on this machine (no 'host')")
        seen.add(name)
        nodes.append(NodeEntry(name, port, host.strip() if host else None, enabled, start, kind))
    return nodes


def _parse_start(raw: object, where: str) -> StartRecipe | None:
    if raw is None:
        return None
    if not isinstance(raw, dict) or set(raw) - {"module", "args"}:
        raise ConfigError(f"{where}: 'start' must be {{module: robot_utils.nodes.x, args: [...]}}")
    module, args = raw.get("module"), raw.get("args", [])
    if not isinstance(module, str) or not _MODULE.match(module):
        raise ConfigError(f"{where}: 'start.module' must be a Python module path, got {module!r}")
    if not isinstance(args, list) or not all(isinstance(a, str) for a in args):
        raise ConfigError(f"{where}: 'start.args' must be a list of strings, got {args!r}")
    return StartRecipe(module, tuple(args))


def load_config(path: str | Path | None = None) -> list[NodeEntry]:
    """Read the registry from ``path``, or the packaged default ``nodes.yaml``."""
    if path is None:
        ref = resources.files(__package__).joinpath(DEFAULT_CONFIG)
        text, source = ref.read_text(encoding="utf-8"), f"(packaged) {DEFAULT_CONFIG}"
    else:
        p = Path(path).expanduser()
        try:
            text = p.read_text(encoding="utf-8")
        except OSError as e:
            raise ConfigError(f"cannot read {p}: {e.strerror or e}") from e
        source = str(p)
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as e:
        raise ConfigError(f"{source}: invalid YAML: {e}") from e
    return parse_config(data, source)
