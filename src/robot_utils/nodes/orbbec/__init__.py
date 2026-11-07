"""Orbbec depth camera node (Gemini 336L): FastAPI server, owner thread, client library.

Run on the machine the camera is plugged into::

    python -m robot_utils.nodes.orbbec --host 0.0.0.0 --port 8000

Client side::

    from robot_utils.nodes.orbbec.client import OrbbecClient

See docs/orbbec_node.md.
"""
__version__ = "0.1.0"
