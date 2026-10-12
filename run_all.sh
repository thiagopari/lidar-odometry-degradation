#!/usr/bin/env bash
# Fetch KITTI (CC BY-NC-SA 3.0; never committed here), extract the velodyne scans, run the sweep, plot.
# usage: SEQS="04 07" ./run_all.sh     (default: 04)
set -euo pipefail
cd "$(dirname "$0")"
K=${KITTI_ROOT:-$HOME/data/kitti}
PY=${PY:-$HOME/.venvs/lidar_bench/bin/python}
B=https://s3.eu-central-1.amazonaws.com/avg-kitti
declare -A DRIVE=([04]=2011_09_30_drive_0016 [07]=2011_09_30_drive_0027)
fetch() { unzip -l "$K/dl/$1" >/dev/null 2>&1 || curl -C - -o "$K/dl/$1" "$2"; }  # resumes partial zips

mkdir -p "$K/dl" "$K/raw" results
$PY test_bench.py
for f in data_odometry_poses data_odometry_calib; do
  fetch $f.zip $B/$f.zip && unzip -qo "$K/dl/$f.zip" -d "$K"
done
for s in ${SEQS:-04}; do
  d=${DRIVE[$s]}
  fetch ${d}_sync.zip $B/raw_data/$d/${d}_sync.zip
  unzip -qo "$K/dl/${d}_sync.zip" '*/velodyne_points/*' -d "$K/raw"
  rm -f results/$s.csv
  $PY bench.py --seq $s --methods kiss gicp icp | tee results/$s.log
  $PY plot.py $s
done
