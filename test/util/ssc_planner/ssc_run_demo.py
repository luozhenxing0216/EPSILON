#!/usr/bin/env python3
"""ssc_run_demo.py

A configurable, lightweight SSC demo for testing and visualization.

This module provides a minimal 3D grid (s,d,t), fills simple obstacles,
generates seeds from a demo trajectory, inflates axis-aligned cubes
to form a driving corridor, and saves a few s-d slice images.

The public function is `run_demo(...)` which returns a dict with keys
`corridor_len`, `saved` (list of image paths) and `out_dir`.
"""

import argparse
import os
import tempfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class Config:
    def __init__(self, map_size=(200, 80, 40), map_resolution=(0.5, 0.25, 0.2)):
        self.map_size = tuple(map_size)
        self.map_resolution = tuple(map_resolution)
        self.s_back_len = 10.0
        self.kMaxNumOfGridAlongTime = 6
        self.inflate_steps = (10, 3, 6, 6, 2, 1)
        self.kMaxLongitudinalAcc = 3.0
        self.kMaxLongitudinalDecel = -6.0
        self.kMaxLongitudinalVel = 30.0
        self.kMinLongitudinalVel = 0.0


class GridMap3D:
    def __init__(self, size, res, origin=None):
        self.size = tuple(size)
        self.res = tuple(res)
        if origin is None:
            self.origin = (0.0, -((self.size[1] - 1) * self.res[1]) / 2.0, 0.0)
        else:
            self.origin = origin
        self.data = np.zeros(self.size, dtype=np.uint8)

    def world_to_coord(self, p_w):
        return [int(round((p_w[i] - self.origin[i]) / self.res[i])) for i in range(3)]

    def check_in_range(self, coord):
        return all(0 <= coord[i] < self.size[i] for i in range(3))

    def set_occupied(self, coord):
        if self.check_in_range(coord):
            self.data[coord[0], coord[1], coord[2]] = 100

    def is_free(self, coord):
        if not self.check_in_range(coord):
            return False
        return self.data[coord[0], coord[1], coord[2]] == 0

    def get_global_metric_on_dim(self, idx, dim):
        return self.origin[dim] + idx * self.res[dim]


class AxisAlignedCube:
    def __init__(self, lb, ub):
        self.lower_bound = list(lb)
        self.upper_bound = list(ub)

    def contains(self, seed):
        return all(self.lower_bound[i] <= seed[i] <= self.upper_bound[i] for i in range(3))


def get_initial_cube_from_seed(seed0, seed1):
    lb = [min(seed0[i], seed1[i]) for i in range(3)]
    ub = [max(seed0[i], seed1[i]) for i in range(3)]
    return AxisAlignedCube(lb, ub)


def check_cube_free(grid, cube):
    for i in range(cube.lower_bound[0], cube.upper_bound[0] + 1):
        for j in range(cube.lower_bound[1], cube.upper_bound[1] + 1):
            for k in range(cube.lower_bound[2], cube.upper_bound[2] + 1):
                if not grid.is_free((i, j, k)):
                    return False
    return True


def check_plane_free_x(grid, cube, x):
    for j in range(cube.lower_bound[1], cube.upper_bound[1] + 1):
        for k in range(cube.lower_bound[2], cube.upper_bound[2] + 1):
            if not grid.is_free((x, j, k)):
                return False
    return True


def check_plane_free_y(grid, cube, y):
    for i in range(cube.lower_bound[0], cube.upper_bound[0] + 1):
        for k in range(cube.lower_bound[2], cube.upper_bound[2] + 1):
            if not grid.is_free((i, y, k)):
                return False
    return True


def check_plane_free_z(grid, cube, z):
    for i in range(cube.lower_bound[0], cube.upper_bound[0] + 1):
        for j in range(cube.lower_bound[1], cube.upper_bound[1] + 1):
            if not grid.is_free((i, j, z)):
                return False
    return True


def inflate_cube(grid, config, cube, initial_fs):
    x_p_finish = x_n_finish = y_p_finish = y_n_finish = z_p_finish = False
    x_p_step, x_n_step, y_p_step, y_n_step, z_p_step, _ = config.inflate_steps

    t_max_grids = cube.lower_bound[2] + config.kMaxNumOfGridAlongTime
    t = t_max_grids * config.map_resolution[2]
    a_max = config.kMaxLongitudinalAcc
    a_min = config.kMaxLongitudinalDecel
    s0 = initial_fs['s']
    v0 = initial_fs['vs']
    s_u = s0 + v0 * t + 0.5 * a_max * t * t
    s_l = s0 + v0 * t + 0.5 * a_min * t * t
    s_idx_u = int(round((s_u - grid.origin[0]) / grid.res[0]))
    s_idx_l = int(round((s_l - grid.origin[0]) / grid.res[0]))
    s_idx_l = max(s_idx_l, int((config.s_back_len / 2.0) / grid.res[0]))

    while not (x_p_finish and x_n_finish and y_p_finish and y_n_finish):
        if not x_p_finish:
            for _ in range(x_p_step):
                x = cube.upper_bound[0] + 1
                if x >= grid.size[0]:
                    x_p_finish = True
                    break
                if check_plane_free_x(grid, cube, x):
                    cube.upper_bound[0] = x
                else:
                    x_p_finish = True
                    break
        if not x_n_finish:
            for _ in range(x_n_step):
                x = cube.lower_bound[0] - 1
                if x < 0:
                    x_n_finish = True
                    break
                if check_plane_free_x(grid, cube, x):
                    cube.lower_bound[0] = x
                else:
                    x_n_finish = True
                    break
        if not y_p_finish:
            for _ in range(y_p_step):
                y = cube.upper_bound[1] + 1
                if y >= grid.size[1]:
                    y_p_finish = True
                    break
                if check_plane_free_y(grid, cube, y):
                    cube.upper_bound[1] = y
                else:
                    y_p_finish = True
                    break
        if not y_n_finish:
            for _ in range(y_n_step):
                y = cube.lower_bound[1] - 1
                if y < 0:
                    y_n_finish = True
                    break
                if check_plane_free_y(grid, cube, y):
                    cube.lower_bound[1] = y
                else:
                    y_n_finish = True
                    break
        if cube.upper_bound[0] >= s_idx_u:
            x_p_finish = True
        if cube.lower_bound[0] <= s_idx_l:
            x_n_finish = True

    while not z_p_finish:
        for _ in range(z_p_step):
            z = cube.upper_bound[2] + 1
            if z >= grid.size[2]:
                z_p_finish = True
                break
            if check_plane_free_z(grid, cube, z):
                cube.upper_bound[2] = z
            else:
                z_p_finish = True
                break
        if cube.upper_bound[2] - cube.lower_bound[2] >= config.kMaxNumOfGridAlongTime:
            z_p_finish = True

    return cube


def generate_example_scene(grid, obstacle_density=0.02):
    s_start = int((40 - grid.origin[0]) / grid.res[0])
    s_end = int((80 - grid.origin[0]) / grid.res[0])
    d_center = grid.size[1] // 2
    for s in range(s_start, min(s_end, grid.size[0])):
        for k in range(grid.size[2]):
            if d_center - 4 >= 0:
                grid.set_occupied((s, d_center - 4, k))
            if d_center + 4 < grid.size[1]:
                grid.set_occupied((s, d_center + 4, k))

    for t in range(5, min(25, grid.size[2])):
        s_idx = int((20 + (t - 5) * 1.2 - grid.origin[0]) / grid.res[0])
        if 0 <= s_idx < grid.size[0]:
            for d in range(max(0, d_center - 1), min(grid.size[1], d_center + 2)):
                grid.set_occupied((s_idx, d, t))

    if obstacle_density is not None and obstacle_density > 0:
        rng = np.random.RandomState(42)
        total = grid.size[0] * grid.size[1] * grid.size[2]
        num = int(total * float(obstacle_density))
        for _ in range(num):
            si = rng.randint(0, max(1, grid.size[0] // 2))
            di = rng.randint(0, grid.size[1])
            ti = rng.randint(0, grid.size[2])
            grid.set_occupied((si, di, ti))


def build_corridor(cfg, grid, initial_fs, traj_points, obstacle_density=0.02, max_steps=1000):
    traj_seeds = []
    if len(traj_points) >= 1:
        coord0 = grid.world_to_coord((initial_fs['s'], initial_fs['d'], initial_fs.get('t', 0.0)))
        traj_seeds.append(coord0)
        for p in traj_points:
            traj_seeds.append(grid.world_to_coord((p[0], p[1], p[2])))

    driving_corridor = []
    if len(traj_seeds) < 2:
        return driving_corridor

    i = 0
    steps = 0
    while i < len(traj_seeds) - 1 and steps < max_steps:
        steps += 1
        cube = get_initial_cube_from_seed(traj_seeds[i], traj_seeds[i + 1])
        if not check_cube_free(grid, cube):
            break
        cube = inflate_cube(grid, cfg, cube, initial_fs)
        driving_corridor.append(cube)
        j = i + 1
        while j < len(traj_seeds) and cube.contains(traj_seeds[j]):
            j += 1
        if j == i + 1:
            i += 1
        else:
            i = j

    return driving_corridor


def run_demo(map_sz=(200, 80, 40), res=(0.5, 0.25, 0.2), init_vs=5.0, obs_density=0.02, max_steps=1000, out_dir=None):
    cfg = Config(map_sz, res)
    cfg.map_size = tuple(map_sz)
    cfg.map_resolution = tuple(res)
    grid = GridMap3D(cfg.map_size, cfg.map_resolution)

    initial_fs = {'s': 0.0, 'd': 0.0, 'vs': float(init_vs)}

    generate_example_scene(grid, obstacle_density=obs_density)

    traj_points = []
    for i in range(0, 30):
        s = 2.0 * i + 1.0
        if 40 <= s <= 80:
            d = -2.0 if i % 2 == 0 else 2.0
        else:
            d = 0.0
        t = i * cfg.map_resolution[2]
        traj_points.append((s, d, t))

    corridor = build_corridor(cfg, grid, initial_fs, traj_points, obstacle_density=obs_density, max_steps=max_steps)

    if out_dir is None:
        out_dir = tempfile.mkdtemp(prefix='ssc_demo_')
    os.makedirs(out_dir, exist_ok=True)

    saved = []
    for t_idx in [0, 5, 10, 15, 20, 25]:
        if t_idx >= grid.size[2]:
            continue
        fig, ax = plt.subplots(1, 1, figsize=(8, 4))
        ax.set_title(f's-d slice at t_idx={t_idx} (t={t_idx * grid.res[2]:.2f}s)')
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
        fp = os.path.join(out_dir, f'ssc_demo_slice_t{t_idx}.png')
        fig.savefig(fp, dpi=150)
        saved.append(fp)
        plt.close(fig)

    return {'corridor_len': len(corridor), 'saved': saved, 'out_dir': out_dir}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-sz', type=int, nargs=3, default=(200, 80, 40))
    parser.add_argument('--res', type=float, nargs=3, default=(0.5, 0.25, 0.2))
    parser.add_argument('--init-vs', type=float, default=5.0)
    parser.add_argument('--obs-density', type=float, default=0.02)
    parser.add_argument('--max-steps', type=int, default=1000)
    parser.add_argument('--out-dir', type=str, default=None)
    args = parser.parse_args()

    res = run_demo(map_sz=tuple(args.map_sz), res=tuple(args.res), init_vs=args.init_vs, obs_density=args.obs_density, max_steps=args.max_steps, out_dir=args.out_dir)
    print(res)
