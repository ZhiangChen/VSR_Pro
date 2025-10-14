#!/usr/bin/env python3
"""
visualize_npz_simple.py — Load and visualize pose recordings saved by main_gui.py
Edit the `path` variable below to point to your NPZ file.
"""

import os
import re
import numpy as np
import matplotlib.pyplot as plt


# ====== USER SETTINGS ======
path = "../data/run_20251013_200721.npz"   # <-- set your .npz path here
SAVE_PLOTS = False                          # True to save PNGs next to the npz
OUTDIR = None                               # Or set a folder string, e.g. "plots"
# ===========================

try:
    import scienceplots  # pip install SciencePlots
    # Choose one (uncomment what you prefer):
    plt.style.use(['science', 'grid', 'no-latex'])     # general "Science" look
    # plt.style.use(['science', 'nature', 'grid', 'no-latex'])  # Nature-like
    # plt.style.use(['science', 'ieee', 'grid', 'no-latex'])    # IEEE-like

    # If you want LaTeX rendering (slower, requires LaTeX):
    # plt.style.use(['science', 'grid'])  # (omit 'no-latex')
    # mpl.rcParams.update({
    #     "text.usetex": True,
    #     "font.family": "serif",
    #     "font.serif": ["Times New Roman"],  # or "Computer Modern"
    # })
except Exception:
    # Fallback if SciencePlots isn't installed
    plt.style.use('seaborn-v0_8-paper')
    mpl.rcParams.update({
        "font.family": "serif",
        "font.size": 9,
        "axes.titlesize": 10,
        "axes.labelsize": 9,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.fontsize": 8,
        "lines.linewidth": 1.5,
        "axes.grid": True,
        "grid.linestyle": "--",
        "grid.alpha": 0.3,
        "figure.dpi": 150,
    })

def _deg(arr_rad):
    return np.rad2deg(arr_rad)

def _scan_pbr_series(npz):
    """Find all keys for PBR poses and return {id:int -> ndarray}."""
    p = re.compile(r"^pbr_object_(\d+)_poses$")
    out = {}
    for k in npz.files:
        m = p.match(k)
        if m:
            out[int(m.group(1))] = npz[k]
    return out

def _summarize(npz):
    n = int(npz.get('num_samples', 0))
    times = npz.get('times', None)
    dur = float(times[-1]) if (times is not None and times.size) else float('nan')
    num_pbr = int(npz.get('num_pbr_objects', 0))
    pbr_ids = npz.get('pbr_object_ids', np.array([]))
    print("=== Recording Summary ===")
    print(f"samples: {n}")
    print(f"duration: {dur:.3f} s" if np.isfinite(dur) else "duration: (not available)")
    print(f"PBR objects: {num_pbr}  IDs: {list(map(int, pbr_ids)) if pbr_ids.size else '[]'}")
    if 'pedestal_pose_description' in npz:
        print(f"pedestal_pose_description: {str(npz['pedestal_pose_description'])}")
    print("=========================\n")

def _plot_pose_grid(name, times, poses, save_dir=None, filename_stub=None):
    """
    One figure per entity (pedestal or PBR), 3x2 subplots:
      Left column:   x, y, z (m)
      Right column:  roll, pitch, yaw (deg)
    poses shape: [N, 6] -> [x,y,z,roll,pitch,yaw]
    """
    if poses is None or poses.size == 0:
        print(f"No poses for {name} — skipping.")
        return

    fig, axes = plt.subplots(3, 2, figsize=(11, 8), sharex=True)
    fig.suptitle(f"{name} pose")

    # Positions (m): left column
    labels_pos = ['x (m)', 'y (m)', 'z (m)']
    for i in range(3):
        ax = axes[i, 0]
        ax.plot(times, poses[:, i])
        ax.set_ylabel(labels_pos[i])
        ax.grid(True, linestyle='--', alpha=0.4)

    # Orientations (deg): right column
    labels_ang = ['roll (deg)', 'pitch (deg)', 'yaw (deg)']
    ang = _deg(poses[:, 3:6])
    for i in range(3):
        ax = axes[i, 1]
        ax.plot(times, ang[:, i])
        ax.set_ylabel(labels_ang[i])
        ax.grid(True, linestyle='--', alpha=0.4)

    # Bottom x-labels
    axes[2, 0].set_xlabel("time (s)")
    axes[2, 1].set_xlabel("time (s)")

    fig.tight_layout(rect=[0, 0, 1, 0.96])

    if save_dir:
        stub = filename_stub or name.lower().replace(" ", "_")
        outpath = os.path.join(save_dir, f"{stub}_pose_grid.png")
        fig.savefig(outpath, dpi=150, bbox_inches="tight")

def plot_pedestal(times, poses, save_dir=None):
    _plot_pose_grid("Pedestal", times, poses, save_dir=save_dir, filename_stub="pedestal")

def plot_pbr(times, pbr_dict, save_dir=None):
    if not pbr_dict:
        print("No PBR pose series found — skipping PBR plots.")
        return
    for obj_id, poses in pbr_dict.items():
        _plot_pose_grid(f"PBR {obj_id-1}", times, poses, save_dir=save_dir, filename_stub=f"pbr_{obj_id-1}")

def main(npz_path, save=False, outdir=None):
    if not os.path.isfile(npz_path):
        raise FileNotFoundError(f"File not found: {npz_path}")

    npz = np.load(npz_path, allow_pickle=True)
    _summarize(npz)

    times = npz.get('times', None)
    pedestal = npz.get('pedestal_poses', None)
    if times is None or times.size == 0:
        if pedestal is not None:
            # Fallback: assume dt=1.0 if times missing
            times = np.arange(pedestal.shape[0], dtype=float)
            print("Warning: 'times' missing; using index as seconds (dt=1.0).")
        else:
            raise ValueError("Neither 'times' nor 'pedestal_poses' present to infer length.")

    save_dir = None
    if save:
        save_dir = outdir or os.path.join(
            os.path.dirname(npz_path),
            f"plots_{os.path.splitext(os.path.basename(npz_path))[0]}"
        )
        os.makedirs(save_dir, exist_ok=True)
        print(f"Saving figures to: {save_dir}")

    plot_pedestal(times, pedestal, save_dir=save_dir)
    pbr_series = _scan_pbr_series(npz)
    plot_pbr(times, pbr_series, save_dir=save_dir)

    plt.show()

if __name__ == "__main__":
    main(path, save=SAVE_PLOTS, outdir=OUTDIR)
