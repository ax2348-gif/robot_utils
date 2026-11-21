"""Synthetic depth images for the description node's tests."""
import numpy as np


def floor_depth(T, K, h=120, w=212, noise=0.0, seed=0):
    """Depth image (m) of an infinite floor (base z = 0) seen by a camera at pose T."""
    R, t = T[:3, :3], T[:3, 3]
    v, u = np.mgrid[0:h, 0:w]
    ray = np.stack([(u - K["cx"] * w / K["width"]) / (K["fx"] * w / K["width"]),
                    (v - K["cy"] * h / K["height"]) / (K["fy"] * h / K["height"]), np.ones((h, w))], -1)
    rz = ray @ R[2]                      # base z of the ray per unit depth
    z = np.where(rz < -1e-6, -t[2] / np.where(rz < -1e-6, rz, -1), 0.0)
    z[z > 8] = 0
    return z + np.random.default_rng(seed).normal(0, noise, z.shape) * (z > 0)
