#!/usr/bin/env python3
"""LiDAR odometry under synthetic degradation on KITTI: KISS-ICP vs small_gicp.

usage: python bench.py --seq 04 [--methods kiss gicp icp] [--only clean]
Writes results/<seq>.csv (one row per run) and results/traj/<run>.txt (KITTI format).
"""
import argparse
import copy
import csv
import os
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import small_gicp
from evo.core import metrics
from evo.core.filters import FilterException
from evo.core.trajectory import PosePath3D
from kiss_icp.config import load_config
from kiss_icp.kiss_icp import KissICP
from kiss_icp.metrics import sequence_error
from kiss_icp.pybind import kiss_icp_pybind

KITTI = Path(os.environ.get("KITTI_ROOT", Path.home() / "data/kitti"))
OUT = Path(__file__).parent / "results"
# odometry sequence -> (raw drive, first frame, last frame), from the KITTI odometry devkit readme
SEQS = {"04": ("2011_09_30_drive_0016", 0, 270), "07": ("2011_09_30_drive_0027", 0, 1100)}
THREADS = 8  # both methods; 16 OpenMP threads on this 16-thread hybrid laptop stalled on background load
SWEEP = {  # the shared clean run is level 64 / 0 / 0 / inf / 1 of each
    "beams": [32, 16, 8],  # rings kept out of 64
    "dropout": [0.25, 0.5, 0.75, 0.9],  # fraction of points removed
    "noise": [0.01, 0.03, 0.05, 0.10],  # Gaussian range sigma along the ray [m]
    "maxrange": [80, 40, 20, 10],  # [m]
    "rate": [2, 4, 8],  # keep every k-th scan: 10 Hz -> 5, 2.5, 1.25 Hz
    "rate_gtseed": [2, 4, 8],  # same, but the first registration starts from the GT relative motion
}
SEEDS = {"dropout": [0, 1], "noise": [0, 1]}  # deterministic degradations run once
DIVERGE_PCT = 10.0  # flag a run whose KITTI drift exceeds 10 % (defined at every scan rate, unlike 10 m RPE)


def rings(pts):
    """Laser index per point. KITTI .bin has no ring field; points are stored laser by laser,
    each sweep cut at the forward direction, so a new ring starts at every -0 -> +0 azimuth step.
    Checked against elevation in load(): 64 rings, median elevation decreasing ring by ring."""
    az = np.arctan2(pts[:, 1], pts[:, 0])
    return np.concatenate([[0], np.cumsum((az[1:] >= 0) & (az[:-1] < 0))])


def gt_velo(seq):
    """Odometry GT (camera-0 frame) -> velodyne frame: T_velo_i = Tr^-1 T_cam_i Tr."""
    tr = np.eye(4)
    calib = (KITTI / f"dataset/sequences/{seq}/calib.txt").read_text().splitlines()
    tr[:3] = np.array(next(l for l in calib if l.startswith("Tr:")).split()[1:], float).reshape(3, 4)
    gt_cam = np.loadtxt(KITTI / f"dataset/poses/{seq}.txt").reshape(-1, 3, 4)
    T_cam = np.tile(np.eye(4), (len(gt_cam), 1, 1))
    T_cam[:, :3] = gt_cam
    return np.linalg.inv(tr) @ T_cam @ tr


def load(seq):
    drive, first, last = SEQS[seq]
    vel = KITTI / "raw" / drive[:10] / f"{drive}_sync" / "velodyne_points"
    files = sorted((vel / "data").glob("*.bin"))[first : last + 1]
    gt = gt_velo(seq)
    times = np.loadtxt(KITTI / f"dataset/sequences/{seq}/times.txt")
    assert len(files) == len(gt) == len(times) == last - first + 1, "raw/odometry frame count mismatch"
    stamps = [datetime.strptime(s[:26], "%Y-%m-%d %H:%M:%S.%f") for s in (vel / "timestamps.txt").read_text().split("\n")[first : last + 1]]
    rel = np.array([(s - stamps[0]).total_seconds() for s in stamps])
    assert np.abs(rel - (times - times[0])).max() < 0.01, "raw/odometry timestamps disagree (dropped frame?)"

    scans, ring_ids = [], []
    for f in files:
        p = np.fromfile(f, np.float32).reshape(-1, 4)[:, :3].astype(np.float64)
        r = rings(p)
        el = np.arctan2(p[:, 2], np.linalg.norm(p[:, :2], axis=1))
        med = np.array([np.median(el[r == k]) for k in range(r[-1] + 1)])
        # 64 rings, median elevation falling ring by ring; adjacent rings may swap by < 0.5 deg (seq 07: 21 of 1101 scans, max 0.37 deg)
        assert r[-1] == 63 and np.diff(med).max() < np.radians(0.5), f"ring derivation failed on {f.name}"
        # KISS-ICP's KITTI loader applies this vertical-angle intrinsic fix; applied to every method's input
        scans.append(np.asarray(kiss_icp_pybind._correct_kitti_scan(kiss_icp_pybind._Vector3dVector(p))))
        ring_ids.append(r)
    return scans, ring_ids, gt


def degrade(pts, ring, kind, level, rng):
    if kind == "beams":
        return pts[ring % (64 // level) == 0]
    if kind == "dropout":
        return pts[rng.random(len(pts)) >= level]
    if kind == "noise":
        r = np.linalg.norm(pts, axis=1)
        return pts * ((r + rng.normal(0.0, level, len(r))) / r)[:, None]
    if kind == "maxrange":
        return pts[np.linalg.norm(pts, axis=1) <= level]
    return pts  # clean, rate (handled by frame selection)


def run_kiss(scans, deskew=False, step=1, seed=None):
    """KISS-ICP, default config (max_range 100 m, voxel 1 m, adaptive threshold) except the thread
    count. KITTI .bin has no per-point time and kiss-icp 1.3.0's KITTI loader passes none, so its
    deskew is a no-op there (poses match its own pipeline to 1e-12). deskew=True instead passes
    t = 0.5 (1 - atan2(y, x) / pi), a clockwise sweep wrapping at the back, rescaled to the k-scan
    interval when frames are skipped."""
    cfg = load_config(None)
    cfg.registration.max_num_threads = THREADS
    odom = KissICP(cfg)
    for i, s in enumerate(scans):
        t0 = time.perf_counter()
        t = 0.5 * (1 - np.arctan2(s[:, 1], s[:, 0]) / np.pi)
        odom.register_frame(s, 0.5 + (t - 0.5) / step if deskew else np.array([]))
        if i == 0 and seed is not None:
            U, _, Vt = np.linalg.svd(seed[:3, :3])  # KITTI GT is rounded; Sophus aborts on a non-orthonormal R
            odom.last_delta = np.block([[U @ Vt, seed[:3, 3:]], [np.zeros((1, 3)), np.ones((1, 1))]])
        yield odom.last_pose.copy(), time.perf_counter() - t0


def run_small_gicp(scans, reg_type, seed=None, **_):
    """Scan-to-scan small_gicp: scan k registered to scan k-1 (KdTree target), 0.25 m voxel
    downsampling, 10-NN covariances, 1.0 m max correspondence, <= 20 iterations,
    constant-velocity initial guess (last relative motion)."""
    T, delta, prev = np.eye(4), np.eye(4) if seed is None else seed, None
    for s in scans:
        t0 = time.perf_counter()
        cur, tree = small_gicp.preprocess_points(s, downsampling_resolution=0.25, num_neighbors=10, num_threads=THREADS)
        if prev is not None:
            res = small_gicp.align(prev[0], cur, prev[1], init_T_target_source=delta, registration_type=reg_type,
                                   max_correspondence_distance=1.0, num_threads=THREADS, max_iterations=20)
            delta = res.T_target_source
            T = T @ delta
        prev = (cur, tree)
        yield T.copy(), time.perf_counter() - t0


def run_vgicp(scans, seed=None, **_):
    """Scan-to-model small_gicp, after its README odometry example: VGICP against a GaussianVoxelMap
    (1.0 m voxels, LRU horizon 100 / clear cycle 10) built from 0.25 m-downsampled scans,
    constant-velocity initial guess, default 20 iterations."""
    vmap = small_gicp.GaussianVoxelMap(1.0)
    vmap.set_lru(horizon=100, clear_cycle=10)
    T, delta = np.eye(4), np.eye(4) if seed is None else seed
    for s in scans:
        t0 = time.perf_counter()
        cur, _ = small_gicp.preprocess_points(s, downsampling_resolution=0.25, num_neighbors=10, num_threads=THREADS)
        if vmap.size():
            T_new = small_gicp.align(vmap, cur, T @ delta, max_correspondence_distance=1.0, num_threads=THREADS).T_target_source
            delta, T = np.linalg.inv(T) @ T_new, T_new
        vmap.insert(cur, T)
        yield T.copy(), time.perf_counter() - t0


METHODS = {"kiss": run_kiss, "kiss_deskew": lambda s, **kw: run_kiss(s, deskew=True, **kw), "vgicp": run_vgicp,
           "gicp": lambda s, **kw: run_small_gicp(s, "GICP", **kw), "icp": lambda s, **kw: run_small_gicp(s, "ICP", **kw)}


def evaluate(gt, est):
    if not np.isfinite(est).all():
        return {"diverged": True, "error": "non-finite pose"}
    ref, traj = PosePath3D(poses_se3=list(gt)), PosePath3D(poses_se3=list(est))
    aligned = copy.deepcopy(traj)
    aligned.align(ref)  # SE(3) Umeyama, no scale
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((ref, aligned))
    out = {"ate_rmse_m": ape.get_statistic(metrics.StatisticsType.rmse)}
    for d in (10, 50):
        rpe = metrics.RPE(metrics.PoseRelation.translation_part, d, metrics.Unit.meters, all_pairs=True, pairs_from_reference=True)
        try:
            rpe.process_data((ref, traj))
        except FilterException:  # low scan rates: no GT pose pair within +-10 % of d
            continue
        out[f"rpe{d}_m"] = rpe.get_statistic(metrics.StatisticsType.rmse)
        if d == 10:
            out["rpe10_max_m"] = rpe.get_statistic(metrics.StatisticsType.max)
    # KITTI devkit metric (as shipped in kiss_icp): mean over 100..800 m segments that fit the sequence
    out["kitti_drift_pct"], out["kitti_rot_deg_per_m"] = sequence_error(gt, est)
    out["diverged"] = out["kitti_drift_pct"] > DIVERGE_PCT
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seq", default="04", choices=SEQS)
    ap.add_argument("--methods", nargs="+", default=["kiss", "gicp"], choices=METHODS)
    ap.add_argument("--only", nargs="+", help="subset of: clean " + " ".join(SWEEP))
    args = ap.parse_args()

    scans, ring_ids, gt = load(args.seq)
    runs = [("clean", 0, 0)] + [(k, lv, sd) for k, lvs in SWEEP.items() for lv in lvs for sd in SEEDS.get(k, [0])]
    runs = [r for r in runs if not args.only or r[0] in args.only]
    (OUT / "traj").mkdir(parents=True, exist_ok=True)
    csv_path = OUT / f"{args.seq}.csv"
    new = not csv_path.exists()
    with open(csv_path, "a", newline="") as fh:
        w = csv.DictWriter(fh, ["seq", "method", "degradation", "level", "seed", "n_scans", "mean_pts",
                                "ate_rmse_m", "rpe10_m", "rpe50_m", "rpe10_max_m", "kitti_drift_pct",
                                "kitti_rot_deg_per_m", "ms_per_scan", "diverged", "error"])
        if new:
            w.writeheader()
        for kind, level, seed in runs:
            step = level if kind.startswith("rate") else 1
            idx = list(range(0, len(scans), step))
            seed_delta = np.linalg.inv(gt[idx[0]]) @ gt[idx[1]] if kind == "rate_gtseed" else None
            for m in args.methods:
                rng = np.random.default_rng(seed)  # same draws for every method
                inputs = [degrade(scans[i], ring_ids[i], kind, level, rng) for i in idx]
                row = {"seq": args.seq, "method": m, "degradation": kind, "level": level, "seed": seed,
                       "n_scans": len(inputs), "mean_pts": round(np.mean([len(p) for p in inputs]))}
                try:
                    poses, dts = zip(*METHODS[m](inputs, step=step, seed=seed_delta))
                    est = np.array(poses)
                    row.update(evaluate(gt[idx], est), ms_per_scan=1e3 * np.mean(dts))
                    np.savetxt(OUT / "traj" / f"{args.seq}_{m}_{kind}_{level}_{seed}.txt", est[:, :3].reshape(len(est), 12))
                except Exception as e:  # a crash is a result too
                    row.update(diverged=True, error=repr(e)[:200])
                w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in row.items()})
                fh.flush()
                print({k: row.get(k) for k in ("method", "degradation", "level", "seed", "ate_rmse_m", "rpe10_m",
                                               "kitti_drift_pct", "ms_per_scan", "diverged", "error")}, flush=True)


if __name__ == "__main__":
    main()
