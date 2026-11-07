"""Node registry: which nodes the landing page shows (``nodes.yaml``)."""

from __future__ import annotations

import re
from dataclasses import asdict, dataclass
from importlib import resources
from pathlib import Path

import yaml

DEFAULT_CONFIG = "nodes.yaml"  # packaged next to this module
_NAME = re.compile(r"^[A-Za-z0-9_.-]+$")


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class NodeEntry:
    name: str
    port: int
    host: str | None = None  # None: same host as the frontend (the browser's hostname)
    enabled: bool = True

    def as_dict(self) -> dict:
        return asdict(self)


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
            raise ConfigError(f"{where}: expected a mapping like {{name: swerve, port: 8102}}")
        unknown = set(raw) - {"name", "port", "host", "enabled"}
        if unknown:
            raise ConfigError(f"{where}: unknown keys {sorted(unknown)}")
        name = raw.get("name")
        if not isinstance(name, str) or not _NAME.match(name):
            raise ConfigError(f"{where}: 'name' must be a short identifier, got {name!r}")
        if name in seen:
            raise ConfigError(f"{where}: duplicate name {name!r}")
        port = raw.get("port")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ConfigError(f"{where}: 'port' must be an integer 1-65535, got {port!r}")
        host = raw.get("host")
        if host is not None and (not isinstance(host, str) or not host.strip()):
            raise ConfigError(f"{where}: 'host' must be a hostname or omitted, got {host!r}")
        enabled = raw.get("enabled", True)
        if not isinstance(enabled, bool):
            raise ConfigError(f"{where}: 'enabled' must be true or false, got {enabled!r}")
        seen.add(name)
        nodes.append(NodeEntry(name, port, host.strip() if host else None, enabled))
    return nodes


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
