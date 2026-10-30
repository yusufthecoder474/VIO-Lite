#!/usr/bin/env python3
"""Offline only: camera-centre EuRoC ground truth, SE(3) ATE, optional Sim(3)."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml


def alignment(source, target, scale=False):
    if len(source) < 3:
        raise ValueError('at least three associated poses required')
    x, y = source - source.mean(0), target - target.mean(0)
    if np.linalg.matrix_rank(x, tol=1e-8) < 2:
        raise ValueError('trajectory geometry cannot constrain a rigid rotation')
    u, _, vt = np.linalg.svd(x.T @ y)
    d = np.eye(3)
    d[2, 2] = np.linalg.det(u @ vt)
    rotation = u @ d @ vt
    factor = np.sum((x @ rotation) * y) / np.sum(x*x) if scale else 1.
    aligned = factor * x @ rotation + target.mean(0)
    return aligned, float(factor)


def quaternion_rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or abs(norm-1) > .01:
        raise ValueError('invalid ground-truth quaternion')
    w, x, y, z = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def ground_truth(path, calibration):
    # Convert body ground truth to camera centres before association.
    data = yaml.safe_load(Path(calibration).read_text())
    transform = np.array(data['T_BS']['data'], dtype=float).reshape(4, 4)
    times, centres = [], []
    with open(path) as stream:
        for row in csv.reader(line for line in stream if line.strip() and not line.startswith('#')):
            if len(row) < 8:
                raise ValueError('ground-truth CSV needs timestamp, body position and wxyz quaternion')
            stamp = int(row[0])
            centre = np.array(row[1:4], dtype=float) + quaternion_rotation(row[4:8]) @ transform[:3, 3]
            if not np.isfinite(centre).all():
                raise ValueError('nonfinite ground truth')
            times.append(stamp)
            centres.append(centre)
    order = np.argsort(times)
    times = np.asarray(times, dtype=np.int64)[order]
    centres = np.asarray(centres)[order]
    if len(times) < 2 or np.any(np.diff(times) <= 0):
        raise ValueError('ground truth needs unique increasing timestamps')
    return times, centres


def associate(stamps, times, centres, max_gap_ns=20000000):
    indices, target = [], []
    for i, stamp in enumerate(stamps):
        j = int(np.searchsorted(times, stamp))
        if j < len(times) and times[j] == stamp:
            indices.append(i)
            target.append(centres[j])
        elif 0 < j < len(times) and times[j]-times[j-1] <= max_gap_ns:
            u = float(stamp-times[j-1]) / float(times[j]-times[j-1])
            indices.append(i)
            target.append((1-u)*centres[j-1] + u*centres[j])
    return np.array(indices, dtype=int), np.array(target)


def evaluate(run, dataset=None, sim3=False):
    run = Path(run)
    with (run/'trajectory.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError('trajectory needs two frames')
    stamps = np.array([int(r['timestamp_ns']) for r in rows], dtype=np.int64)
    if np.any(np.diff(stamps) <= 0):
        raise ValueError('trajectory timestamps must increase')
    raw = np.array([[float(r['raw_'+axis]) for axis in 'xyz'] for r in rows])
    if not np.isfinite(raw).all():
        raise ValueError('nonfinite raw trajectory')
    with (run/'run_summary.csv').open() as stream:
        summary = next(csv.DictReader(stream))
    result = dict(summary)
    result.update(dataset=str(dataset) if dataset else 'not supplied',
                  metric_evaluation_status='unavailable', metric_ate_rmse_m='', associated_poses=0,
                  sim3_shape_rmse_m='', sim3_fitted_scale='', sim3_status='not_requested')
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].plot(raw[:, 0], raw[:, 1], color='#2166ac', label='Raw monocular VO (arbitrary units)')
    axes[0].set(xlabel='x (arbitrary units)', ylabel='y (arbitrary units)', title='Monocular trajectory: XY projection')
    axes[1].set(xlabel='x (m)', ylabel='y (m)', title='Metric evaluation: rigid alignment only')
    axes[1].text(.5, .5, 'Metric evaluation unavailable', ha='center', transform=axes[1].transAxes)
    if dataset:
        root = Path(dataset)
        gt_path = root/'state_groundtruth_estimate0/data.csv'
        if gt_path.exists():
            times, centres = ground_truth(gt_path, root/'cam0/sensor.yaml')
            idx, target = associate(stamps, times, centres)
            result['associated_poses'] = len(idx)
            if all(r['metric_valid']=='1' for r in rows) and all(r['connected']=='1' for r in rows):
                metric = np.array([[float(r['metric_'+axis]) for axis in 'xyz'] for r in rows])
                if not np.isfinite(metric).all():
                    raise ValueError('metric coordinates must be finite')
                try:
                    aligned, _ = alignment(metric[idx], target)
                    result['metric_ate_rmse_m'] = float(np.sqrt(np.mean(np.sum((aligned-target)**2, axis=1))))
                    result['metric_evaluation_status'] = 'SE3_aligned_no_scale_fit_experimental'
                    axes[1].clear()
                    axes[1].plot(target[:, 0], target[:, 1], label='Ground truth camera centre', color='#555555')
                    axes[1].plot(aligned[:, 0], aligned[:, 1], label='IMU-assisted estimate; SE(3)', color='#1b9e77')
                    axes[1].set(xlabel='x (m)', ylabel='y (m)', title='Metric evaluation: no fitted scale')
                except ValueError as error:
                    result['metric_evaluation_status'] = str(error)
            else:
                result['metric_evaluation_status'] = 'scale_not_established_or_visual_gap'
            if sim3:
                if all(r['connected']=='1' for r in rows):
                    try:
                        aligned, factor = alignment(raw[idx], target, scale=True)
                        result['sim3_shape_rmse_m'] = float(np.sqrt(np.mean(np.sum((aligned-target)**2, axis=1))))
                        result['sim3_fitted_scale'] = factor
                        result['sim3_status'] = 'diagnostic_shape_only_not_recovered_scale'
                        shape_fig = plt.figure(figsize=(6, 5))
                        ax = shape_fig.add_subplot(111, projection='3d')
                        ax.plot(*target.T, label='Ground truth')
                        ax.plot(*aligned.T, label='VO; ground-truth Sim(3) fit')
                        ax.set(title='Shape diagnostic only; scale fitted offline', xlabel='x (m)', ylabel='y (m)', zlabel='z (m)')
                        ax.legend();shape_fig.tight_layout();shape_fig.savefig(run/'sim3_shape.png', dpi=160);plt.close(shape_fig)
                    except ValueError as error:
                        result['sim3_status'] = str(error)
                else:
                    result['sim3_status'] = 'disconnected_trajectory'
        else:
            result['metric_evaluation_status'] = 'ground_truth_not_found'
    for ax in axes:
        ax.grid(alpha=.2);ax.set_aspect('equal', adjustable='datalim')
        if ax.lines:
            ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(run/'trajectory.png', dpi=160);plt.close(fig)
    with (run/'RESULTS.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(result));writer.writeheader();writer.writerow(result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--dataset', help='EuRoC mav0; ground truth is read here only')
    parser.add_argument('--sim3', action='store_true', help='optional, explicitly nonmetric diagnostic')
    args = parser.parse_args()
    print(json.dumps(evaluate(args.run, args.dataset, args.sim3), indent=2))
