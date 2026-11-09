"""Shared fixtures. Every test gets its own short runtime dir, so node sockets
(<XDG_RUNTIME_DIR>/robot/<name>.sock) never touch the real ones and stay under the
104-byte unix socket path limit (pytest's tmp_path is too long on macOS)."""
import shutil
import tempfile

import pytest


@pytest.fixture(autouse=True)
def tmp_locks_dir(monkeypatch, tmp_path):
    """Lock files (common/locks.py) of a test never touch the real ones."""
    d = tmp_path / "locks"
    monkeypatch.setenv("ROBOT_LOCKS_DIR", str(d))
    yield d


@pytest.fixture(autouse=True)
def tmp_sock_dir(monkeypatch):
    d = tempfile.mkdtemp(prefix="ru")
    monkeypatch.setenv("XDG_RUNTIME_DIR", d)
    yield d
    shutil.rmtree(d, ignore_errors=True)
