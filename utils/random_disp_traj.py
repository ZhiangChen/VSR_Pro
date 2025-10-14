#!/usr/bin/env python3
"""
random_displacement_trajectory.py

Generates a random-looking displacement trajectory (6-DOF) that meets
target PGV and PGA values for each enabled degree of freedom within 
an upper frequency limit.

Outputs:
  - .npz file with displacement, velocity, acceleration, and metadata
  - .csv file with time vs displacement for all DOFs (for external use)
"""

import os
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import csv

# ========= USER VARIABLES =========
FS       = 500.0          # sampling frequency (Hz)
DURATION = 20.0           # total duration (s)

# DOF Mask: Enable/disable each degree of freedom
# Order: [X, Y, Z, Roll, Pitch, Yaw]
DOF_MASK = [False, False, False, False, False, True]  # Enable only X axis by default

# Linear motion targets (X, Y, Z)
LINEAR_PGV = 0.20         # target peak ground velocity (m/s)
LINEAR_PGA = 2.50         # target peak ground acceleration (m/s^2)
LINEAR_F_MAX = 5.0        # upper bound frequency (Hz)

# Rotational motion targets (Roll, Pitch, Yaw)
ROTATIONAL_PGV = 0.10     # target peak ground velocity (rad/s)
ROTATIONAL_PGA = 1.50     # target peak ground acceleration (rad/s^2)
ROTATIONAL_F_MAX = 3.0    # upper bound frequency (Hz)

# Common parameters
N_EXTRA_HARMONICS = 6     # number of additional random harmonics
EXTRA_SCALE = 0.15        # relative amplitude of added harmonics
CSV_PATH = "../data/displacement_trajectory.csv"  # path for CSV output
SEED = None               # set to int for reproducibility, or None
# ==================================

# DOF labels
DOF_LABELS = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']

if SEED is not None:
    np.random.seed(SEED)

def build_primary(fs, duration, pgv, pga, f_max):
    """Build a primary sine motion satisfying PGV and PGA."""
    dt = 1.0 / fs
    t = np.arange(0.0, duration, dt)

    if pgv <= 0 or pga <= 0:
        raise ValueError("TARGET_PGV and TARGET_PGA must be positive.")

    omega_star = pga / pgv
    f_star = omega_star / (2.0 * math.pi)
    A_star = (pgv * pgv) / pga
    phi = 2.0 * math.pi * np.random.rand()

    # Frequency clamp if needed
    if f_star <= 0:
        f_star = min(0.1, f_max)
        omega_star = 2.0 * math.pi * f_star
        A_star = pgv / omega_star

    if f_star > f_max:
        f = f_max
        omega = 2.0 * math.pi * f
        A = pgv / omega
        x = A * np.sin(omega * t + phi)
        v = A * omega * np.cos(omega * t + phi)
        a = -A * (omega**2) * np.sin(omega * t + phi)
        print(f"[INFO] Clamped frequency to {f_max:.2f} Hz.")
    else:
        omega = omega_star
        A = A_star
        x = A * np.sin(omega * t + phi)
        v = A * omega * np.cos(omega * t + phi)
        a = -A * (omega**2) * np.sin(omega * t + phi)
        f = f_star

    return t, x, v, a, f

def add_random_harmonics(t, f_max, base_amp, n_terms=6, rel_scale=0.15):
    """Add random low-amplitude harmonics."""
    if n_terms <= 0 or rel_scale <= 0:
        return np.zeros_like(t)

    x_extra = np.zeros_like(t)
    for _ in range(n_terms):
        f = np.random.uniform(0.2, f_max)
        omega = 2.0 * math.pi * f
        A = rel_scale * base_amp * np.random.uniform(0.5, 1.0)
        phi = 2.0 * math.pi * np.random.rand()
        x_extra += A * np.sin(omega * t + phi)
    return x_extra

def derive_vel_acc(x, fs):
    """Compute velocity and acceleration from displacement."""
    dt = 1.0 / fs
    v = np.gradient(x, dt, edge_order=2)
    a = np.gradient(v, dt, edge_order=2)
    return v, a

def renormalize(x, fs, target_pgv, target_pga):
    """Re-normalize displacement to match PGV/PGA targets."""
    v, a = derive_vel_acc(x, fs)
    current_pgv = np.max(np.abs(v))
    if current_pgv > 0:
        x *= (target_pgv / current_pgv)
        v, a = derive_vel_acc(x, fs)

    current_pga = np.max(np.abs(a))
    if current_pga > 1e-12:
        ratio = target_pga / current_pga
        if abs(ratio - 1.0) > 0.05:
            X = np.fft.rfft(x)
            freqs = np.fft.rfftfreq(x.size, d=1.0/fs)
            tilt = 1.0 + (ratio - 1.0) * (freqs / (freqs.max() or 1))**2
            x = np.fft.irfft(X * tilt, n=x.size)
            v, a = derive_vel_acc(x, fs)
            x *= (target_pgv / np.max(np.abs(v)))
    return x, v, a

def save_csv(time_arr, disp_dict, dof_mask, path):
    """Save time and displacement arrays for all DOFs to CSV."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Header row
        header = ["time_s"]
        for i, label in enumerate(DOF_LABELS):
            if dof_mask[i]:
                unit = "m" if i < 3 else "rad"
                header.append(f"{label}_{unit}")
        writer.writerow(header)
        
        # Data rows
        for i, t in enumerate(time_arr):
            row = [t]
            for dof_idx in range(6):
                if dof_mask[dof_idx]:
                    row.append(disp_dict[dof_idx][i])
            writer.writerow(row)
    
    print(f"[OK] Saved displacement CSV: {path}")



def main():
    print("="*60)
    print("6-DOF Random Displacement Trajectory Generator")
    print("="*60)
    
    # Check DOF mask
    if not any(DOF_MASK):
        raise ValueError("At least one DOF must be enabled in DOF_MASK")
    
    print(f"Enabled DOFs: {[DOF_LABELS[i] for i, enabled in enumerate(DOF_MASK) if enabled]}")
    print(f"Duration: {DURATION} s, Sampling frequency: {FS} Hz")
    print()
    
    # Generate time array
    dt = 1.0 / FS
    t = np.arange(0.0, DURATION, dt)
    
    # Storage for all DOFs
    disp_dict = {}
    vel_dict = {}
    acc_dict = {}
    
    # Generate trajectory for each enabled DOF
    for dof_idx, enabled in enumerate(DOF_MASK):
        label = DOF_LABELS[dof_idx]
        
        if not enabled:
            # Disabled DOF: all zeros
            disp_dict[dof_idx] = np.zeros_like(t)
            vel_dict[dof_idx] = np.zeros_like(t)
            acc_dict[dof_idx] = np.zeros_like(t)
            continue
        
        # Determine parameters based on DOF type
        is_linear = dof_idx < 3  # X, Y, Z are linear
        
        if is_linear:
            pgv = LINEAR_PGV
            pga = LINEAR_PGA
            f_max = LINEAR_F_MAX
            unit_disp = "m"
            unit_vel = "m/s"
            unit_acc = "m/s²"
        else:
            pgv = ROTATIONAL_PGV
            pga = ROTATIONAL_PGA
            f_max = ROTATIONAL_F_MAX
            unit_disp = "rad"
            unit_vel = "rad/s"
            unit_acc = "rad/s²"
        
        print(f"--- {label} axis ({unit_disp}) ---")
        
        # Build primary motion
        _, x0, v0, a0, f_used = build_primary(FS, DURATION, pgv, pga, f_max)
        
        # Add random harmonics
        x_extra = add_random_harmonics(t, f_max, np.max(np.abs(x0)), N_EXTRA_HARMONICS, EXTRA_SCALE)
        x = x0 + x_extra
        
        # Re-normalize to match targets
        x, v, a = renormalize(x, FS, pgv, pga)
        
        # Store results
        disp_dict[dof_idx] = x
        vel_dict[dof_idx] = v
        acc_dict[dof_idx] = a
        
        # Report results
        print(f"  Used frequency ≈ {f_used:.3f} Hz (max {f_max:.3f})")
        print(f"  Achieved PGV ≈ {np.max(np.abs(v)):.4f} {unit_vel} (target {pgv:.4f})")
        print(f"  Achieved PGA ≈ {np.max(np.abs(a)):.4f} {unit_acc} (target {pga:.4f})")
        print()
    
    # Save data
    save_csv(t, disp_dict, DOF_MASK, CSV_PATH)
    
    # Plot trajectories in two separate figures
    # Figure 1: Linear motion (X, Y, Z) - 3x3 grid
    linear_enabled = any(DOF_MASK[0:3])
    if linear_enabled:
        fig_linear = plt.figure(figsize=(15, 12))
        fig_linear.suptitle("Linear Motion Trajectories (X, Y, Z)", fontsize=16, fontweight='bold')
        
        for dof_idx in range(3):  # X, Y, Z
            label = DOF_LABELS[dof_idx]
            enabled = DOF_MASK[dof_idx]
            
            # Displacement (column 0)
            ax = plt.subplot(3, 3, dof_idx * 3 + 1)
            if enabled:
                ax.plot(t, disp_dict[dof_idx], lw=1, color='C0')
                ax.set_ylabel(f"{label} disp (m)", fontsize=11, fontweight='bold')
            else:
                ax.plot(t, disp_dict[dof_idx], lw=1, color='gray', linestyle='--', alpha=0.5)
                ax.set_ylabel(f"{label} disp (m)\n[DISABLED]", fontsize=11, color='gray')
            ax.grid(True, ls="--", alpha=0.4)
            if dof_idx == 2:
                ax.set_xlabel("time (s)", fontsize=11)
            
            # Velocity (column 1)
            ax = plt.subplot(3, 3, dof_idx * 3 + 2)
            if enabled:
                ax.plot(t, vel_dict[dof_idx], lw=1, color='C1')
                ax.set_ylabel(f"{label} vel (m/s)", fontsize=11, fontweight='bold')
            else:
                ax.plot(t, vel_dict[dof_idx], lw=1, color='gray', linestyle='--', alpha=0.5)
                ax.set_ylabel(f"{label} vel (m/s)\n[DISABLED]", fontsize=11, color='gray')
            ax.grid(True, ls="--", alpha=0.4)
            if dof_idx == 2:
                ax.set_xlabel("time (s)", fontsize=11)
            
            # Acceleration (column 2)
            ax = plt.subplot(3, 3, dof_idx * 3 + 3)
            if enabled:
                ax.plot(t, acc_dict[dof_idx], lw=1, color='C2')
                ax.set_ylabel(f"{label} acc (m/s²)", fontsize=11, fontweight='bold')
            else:
                ax.plot(t, acc_dict[dof_idx], lw=1, color='gray', linestyle='--', alpha=0.5)
                ax.set_ylabel(f"{label} acc (m/s²)\n[DISABLED]", fontsize=11, color='gray')
            ax.grid(True, ls="--", alpha=0.4)
            if dof_idx == 2:
                ax.set_xlabel("time (s)", fontsize=11)
        
        plt.tight_layout()
    
    # Figure 2: Rotational motion (Roll, Pitch, Yaw) - 3x3 grid
    rotational_enabled = any(DOF_MASK[3:6])
    if rotational_enabled:
        fig_rotational = plt.figure(figsize=(15, 12))
        fig_rotational.suptitle("Rotational Motion Trajectories (Roll, Pitch, Yaw)", fontsize=16, fontweight='bold')
        
        for i, dof_idx in enumerate(range(3, 6)):  # Roll, Pitch, Yaw
            label = DOF_LABELS[dof_idx]
            enabled = DOF_MASK[dof_idx]
            
            # Displacement (column 0)
            ax = plt.subplot(3, 3, i * 3 + 1)
            if enabled:
                ax.plot(t, disp_dict[dof_idx], lw=1, color='C3')
                ax.set_ylabel(f"{label} disp (rad)", fontsize=11, fontweight='bold')
            else:
                ax.plot(t, disp_dict[dof_idx], lw=1, color='gray', linestyle='--', alpha=0.5)
                ax.set_ylabel(f"{label} disp (rad)\n[DISABLED]", fontsize=11, color='gray')
            ax.grid(True, ls="--", alpha=0.4)
            if i == 2:
                ax.set_xlabel("time (s)", fontsize=11)
            
            # Velocity (column 1)
            ax = plt.subplot(3, 3, i * 3 + 2)
            if enabled:
                ax.plot(t, vel_dict[dof_idx], lw=1, color='C4')
                ax.set_ylabel(f"{label} vel (rad/s)", fontsize=11, fontweight='bold')
            else:
                ax.plot(t, vel_dict[dof_idx], lw=1, color='gray', linestyle='--', alpha=0.5)
                ax.set_ylabel(f"{label} vel (rad/s)\n[DISABLED]", fontsize=11, color='gray')
            ax.grid(True, ls="--", alpha=0.4)
            if i == 2:
                ax.set_xlabel("time (s)", fontsize=11)
            
            # Acceleration (column 2)
            ax = plt.subplot(3, 3, i * 3 + 3)
            if enabled:
                ax.plot(t, acc_dict[dof_idx], lw=1, color='C5')
                ax.set_ylabel(f"{label} acc (rad/s²)", fontsize=11, fontweight='bold')
            else:
                ax.plot(t, acc_dict[dof_idx], lw=1, color='gray', linestyle='--', alpha=0.5)
                ax.set_ylabel(f"{label} acc (rad/s²)\n[DISABLED]", fontsize=11, color='gray')
            ax.grid(True, ls="--", alpha=0.4)
            if i == 2:
                ax.set_xlabel("time (s)", fontsize=11)
        
        plt.tight_layout()
    
    # Show all figures
    if linear_enabled or rotational_enabled:
        plt.show()
    else:
        print("[WARNING] No DOFs enabled - no plots generated.")

if __name__ == "__main__":
    main()
