"""python test_bench.py: synthetic checks for ring derivation and the degradations (no KITTI needed)."""
import numpy as np

from bench import degrade, rings

# 4 lasers stored one after another, each sweep starting just after forward (+0 rad) and ending just before it
az = np.linspace(0.01, 2 * np.pi - 0.01, 500)
az = np.tile(np.where(az > np.pi, az - 2 * np.pi, az), 4)
el = np.repeat(np.radians([2.0, 0.0, -2.0, -4.0]), 500)
pts = 20 * np.c_[np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)]
assert (rings(pts) == np.repeat(np.arange(4), 500)).all()

rng = np.random.default_rng(0)
cloud = rng.uniform(-50, 50, (200_000, 3))
cloud = cloud[np.linalg.norm(cloud, axis=1) > 2]
ring = rng.integers(0, 64, len(cloud))
assert len(degrade(cloud, ring, "beams", 16, rng)) == (ring % 4 == 0).sum()
assert abs(len(degrade(cloud, ring, "dropout", 0.9, rng)) / len(cloud) - 0.1) < 0.005
noisy = degrade(cloud, ring, "noise", 0.05, rng)
r0, r1 = np.linalg.norm(cloud, axis=1), np.linalg.norm(noisy, axis=1)
assert abs((r1 - r0).std() - 0.05) < 0.001 and np.allclose(cloud / r0[:, None], noisy / r1[:, None])  # along the ray
assert np.linalg.norm(degrade(cloud, ring, "maxrange", 20, rng), axis=1).max() <= 20
print("ok")
