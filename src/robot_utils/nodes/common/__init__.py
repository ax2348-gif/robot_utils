"""Code shared by the robot's nodes (docs/nodes.md).

protocol   binary record framing, schema + rows records for every-sample data
timesync   server monotonic clock, client NTP-style sync (pure Python)
hub        thread -> asyncio handoff for every-sample rows and state changes
lease      single-controller lease (actuated nodes only)
estop      latching e-stop (actuated nodes only)
server     FastAPI pieces: /time, /health, CORS, event log, rows stream, SSE
client     Python client base (httpx): sync, JSON calls, streams, recording
"""
