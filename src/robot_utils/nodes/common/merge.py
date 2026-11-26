"""The one deep merge of every YAML layer (description defaults + profile overlay + data-dir overrides; default_sim.yaml
+ the robot profile's ``sim:`` + ``--config`` + overrides; an arm's ``components.arms.<name>`` over ``components.arm``).

Rule: ``b`` is merged over ``a`` key by key; a dict merges into a dict, anything else replaces; a ``null`` (None)
value in ``b`` **deletes** that key from the result (readers then use their own default). So ``null`` means the same
in every layer: "not set here, and not inherited either". Neither input is modified."""
from __future__ import annotations

import copy


def deep_merge(a: dict, b: dict) -> dict:
    out = copy.deepcopy(a or {})
    for k, v in (b or {}).items():
        if v is None:
            out.pop(k, None)
        elif isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out
