"""A fuller SSC demo that builds a richer scene, computes corridor metadata and saves visualizations.

This script imports the helper functions from `ssc_run_demo.py` (same directory) and:
- Builds a larger, denser scene with configurable obstacle patterns
- Calls `build_corridor` to obtain corridor cubes
- Saves corridor metadata (list of cubes as JSON)
- Builds a montage of saved s-d slice images into a single PNG
- Prints a summary and returns a dict with paths for downstream tests

Usage example:
python test/util/ssc_planner/ssc_run_demo_full.py --map-sz 120 40 30 --res 0.5 0.5 0.2 --init-vs 3.0 --obs-density 0.02 --out-dir ./demo_full_out
"""

import os
import json
import argparse
import importlib.util
import shutil
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.image as mpimg
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
SSC_PATH = HERE / 'ssc_run_demo.py'

# dynamic import of ssc_run_demo module
spec = importlib.util.spec_from_file_location('ssc_run_demo', str(SSC_PATH))
ssc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ssc)


def make_montage(image_paths, out_fp):
    imgs = []
    for p in image_paths:
        im = mpimg.imread(p)
        if im is None:
            continue
        a = np.asarray(im)
        if a.ndim == 2:
            a = np.stack([a, a, a], axis=-1)
        if a.dtype == np.float32:
            if a.max() <= 1.0:
                a = (a * 255).astype(np.uint8)
            else:
                a = a.astype(np.uint8)
        imgs.append(a)
    if not imgs:
        return None
    min_h = min(im.shape[0] for im in imgs)
    resized = []
    for im in imgs:
        if im.shape[0] != min_h:
            fig = plt.figure(figsize=(im.shape[1] / 100.0, min_h / 100.0), dpi=100)
            ax = fig.add_axes([0, 0, 1, 1])
            ax.axis('off')
            ax.imshow(im)
            tmp_fp = out_fp + '.tmp.png'
            fig.savefig(tmp_fp, dpi=100)
            plt.close(fig)
            im2 = mpimg.imread(tmp_fp)
            if im2.dtype == np.float32 and im2.max() <= 1.0:
                im2 = (im2 * 255).astype(np.uint8)
            resized.append(im2)
            try:
                os.remove(tmp_fp)
            except Exception:
                pass
        else:
            resized.append(im)
    montage = np.hstack(resized)
    plt.imsave(out_fp, montage)
    return out_fp


def build_rich_scene_and_run(cfg, out_dir):
    # build grid and populate a richer scene
    grid = ssc.GridMap3D(cfg.map_size, cfg.map_resolution)

    # deterministic obstacles: alternating walls and gaps
    d_center = grid.size[1] // 2
    for s in range(30, min(200, grid.size[0]), 8):
        for k in range(grid.size[2]):
            grid.set_occupied((s, max(0, d_center - 6), k))
            grid.set_occupied((s, min(grid.size[1]-1, d_center + 6), k))

    # a winding obstacle lane
    for t in range(0, grid.size[2]):
        s_idx = int(10 + 0.8 * t)
        if 0 <= s_idx < grid.size[0]:
            off = int(3 * np.sin(t * 0.4))
            for d in range(max(0, d_center-2+off), min(grid.size[1], d_center+3+off)):
                grid.set_occupied((s_idx, d, t))

    # random sparse obstacles
    rng = np.random.RandomState(123)
    total = grid.size[0] * grid.size[1] * grid.size[2]
    num = int(total * 0.02)
    for _ in range(num):
        si = rng.randint(0, max(1, grid.size[0]//2))
        di = rng.randint(0, grid.size[1])
        ti = rng.randint(0, grid.size[2])
        grid.set_occupied((si, di, ti))

    # generate trajectory seeds
    initial_fs = {'s': 0.0, 'd': 0.0, 'vs': cfg.init_vs}
    traj_points = []
    for i in range(0, 60):
        s = 1.5 * i + 1.0
        if 40 <= s <= 120:
            d = -3.0 if (i // 3) % 2 == 0 else 3.0
        else:
            d = 0.0
        t = i * cfg.map_resolution[2]
        traj_points.append((s, d, t))

    corridor = ssc.build_corridor(cfg, grid, initial_fs, traj_points, obstacle_density=0.02, max_steps=2000)

    # ensure out_dir exists
    os.makedirs(out_dir, exist_ok=True)

    # run the built-in slice saving (re-implement logic similar to run_demo for visualization)
    saved = []
    for t_idx in range(0, grid.size[2], max(1, grid.size[2] // 8)):
        fig, ax = plt.subplots(1, 1, figsize=(10, 5))
        ax.set_title(f's-d slice at t_idx={t_idx}')
        s_coords = np.arange(0, grid.size[0]) * grid.res[0] + grid.origin[0]
        d_coords = np.arange(0, grid.size[1]) * grid.res[1] + grid.origin[1]
        occ = grid.data[:, :, t_idx]
        ax.imshow(occ.T, origin='lower', extent=[s_coords[0], s_coords[-1], d_coords[0], d_coords[-1]], cmap='Reds', alpha=0.8)
        for c in corridor:
            if c.lower_bound[2] <= t_idx <= c.upper_bound[2]:
                x0 = grid.get_global_metric_on_dim(c.lower_bound[0], 0)
                x1 = grid.get_global_metric_on_dim(c.upper_bound[0], 0) + grid.res[0]
                y0 = grid.get_global_metric_on_dim(c.lower_bound[1], 1)
                y1 = grid.get_global_metric_on_dim(c.upper_bound[1], 1) + grid.res[1]
                rect = plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, edgecolor='blue', linewidth=2)
                ax.add_patch(rect)
        ax.set_xlabel('s (m)')
        ax.set_ylabel('d (m)')
        plt.tight_layout()
        fp = os.path.join(out_dir, f'ssc_full_slice_t{t_idx}.png')
        fig.savefig(fp, dpi=150)
        saved.append(fp)
        plt.close(fig)

    # save corridor metadata (grid indices)
    cubes = []
    for c in corridor:
        cubes.append({'lb': c.lower_bound, 'ub': c.upper_bound, 'volume': (c.volume() if hasattr(c, 'volume') else None)})
    json_fp = os.path.join(out_dir, 'corridor_meta.json')
    with open(json_fp, 'w') as fh:
        json.dump({'corridor_len': len(cubes), 'cubes': cubes}, fh, indent=2)

    montage_fp = os.path.join(out_dir, 'ssc_demo_full_montage.png')
    mont = make_montage(saved, montage_fp)

    # copy montage to repo for inspection
    repo_fp = str(HERE / 'ssc_demo_full_montage.png')
    if mont:
        try:
            shutil.copyfile(mont, repo_fp)
        except Exception:
            pass

    return {'corridor_len': len(corridor), 'saved': saved, 'out_dir': out_dir, 'montage': mont, 'meta': json_fp}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-sz', type=int, nargs=3, default=(120, 40, 30))
    parser.add_argument('--res', type=float, nargs=3, default=(0.5, 0.5, 0.2))
    parser.add_argument('--init-vs', type=float, default=3.0)
    parser.add_argument('--obs-density', type=float, default=0.02)
    parser.add_argument('--out-dir', type=str, default=None)
    args = parser.parse_args()

    out_dir = args.out_dir or (HERE / 'ssc_full_out')
    out_dir = str(out_dir)

    cfg = argparse.Namespace(map_size=tuple(args.map_sz), map_resolution=tuple(args.res), init_vs=args.init_vs)
    res = build_rich_scene_and_run(cfg, out_dir)
    print('Full demo finished:', res)
