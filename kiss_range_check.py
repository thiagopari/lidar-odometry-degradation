# Does KISS-ICP still fail at short max range when its config is told the sensor range (voxel = range/100)?
import sys, time, numpy as np
sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
import bench
from kiss_icp.config import load_config
from kiss_icp.kiss_icp import KissICP
for seq in sys.argv[1:]:
    scans, ring_ids, gt = bench.load(seq)
    for rmax in (20.0, 10.0):
        rng = np.random.default_rng(0)
        inputs = [bench.degrade(s, r, "maxrange", rmax, rng) for s, r in zip(scans, ring_ids)]
        cfg = load_config(None); cfg.registration.max_num_threads = bench.THREADS
        cfg.data.max_range = rmax; cfg.mapping.voxel_size = rmax / 100.0
        odom, est = KissICP(cfg), []
        for s in inputs:
            odom.register_frame(s, np.array([])); est.append(odom.last_pose.copy())
        m = bench.evaluate(gt, np.array(est))
        print(seq, f"max_range {rmax:.0f} m, voxel {rmax/100:.2f} m: drift {m['kitti_drift_pct']:.2f} %, ATE {m['ate_rmse_m']:.2f} m", flush=True)
