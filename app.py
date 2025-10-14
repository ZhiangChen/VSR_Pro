"""
app.py

VSR Pro Application API Layer
Provides a high-level API for controlling the simulation programmatically.
Can be used by both GUI applications and pure Python scripts.

This module separates the business logic from the GUI presentation layer,
enabling automation and batch processing of simulations.
"""

import time
import math
import threading
import yaml
import logging
import os
import csv
from datetime import datetime
import numpy as np
import pybullet as p

from simulation_core import SimulationCore


class VSRProApp:
    """
    High-level API for VSR Pro shake table simulation.
    
    This class provides methods for:
    - Starting/stopping/pausing simulation
    - Loading and spawning objects
    - Executing pulse and displacement trajectories
    - Recording and saving trajectory data
    - Managing simulation state
    
    Can be used by GUI applications or pure Python scripts.
    """
    
    def __init__(self, config_path="config.yaml"):
        """
        Initialize the VSR Pro application.
        
        Args:
            config_path (str): Path to the YAML configuration file
        """
        self.config_path = config_path
        self.simulation_core = None
        self.simulation_thread = None
        self.simulation_started = False
        self.running = False
        self.paused = False
        self.trajectory_running = False
        self.disp_trajectory_running = False
        self.stop_simulation_flag = False
        
        # Trajectory data
        self.disp_path = None
        self.disp_trajectory_data = None
        self.trajectory_params = {
            "cycle_number": 1,
            "amp_x": 0.0, "freq_x": 0.0,
            "amp_y": 0.0, "freq_y": 0.0,
            "amp_z": 0.0, "freq_z": 0.0,
            "amp_roll": 0.0, "freq_roll": 0.0,
            "amp_pitch": 0.0, "freq_pitch": 0.0,
            "amp_yaw": 0.0, "freq_yaw": 0.0
        }
        
        # Last known pedestal position for position control
        self.last_pedestal_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Data recording
        self.recording_active = False
        self.recorded_times = []
        self.recorded_pedestal_poses = []
        self.recorded_pbr_poses = {}
        self.pbr_object_ids = set()
        self.recording_start_time = None
        
        # Load configuration
        with open(config_path, "r") as file:
            self.config = yaml.safe_load(file)
        
        self.use_real_time = self.config["simulation_settings"]["use_real_time"]
        self.simulation_frequency = self.config["simulation_settings"].get("simulation_frequency", 500)
        self.simulation_dt = 1.0 / self.simulation_frequency
        
        # Get pedestal configuration
        pedestal_config = self.config["structure"]["pedestal"]
        pedestal_type = pedestal_config.get("type", "box")
        if pedestal_type == "box":
            self.pedestal_half_z = pedestal_config["box"]["dimensions"][2] / 2.0
        else:
            self.pedestal_half_z = 0.25  # Default estimate for mesh height
        
        # Setup logging
        log_file_path = self.config["simulation_settings"].get("log_file", "data/simulation.log")
        self._setup_logging(log_file_path)
        
        self.logger.info("VSRProApp initialized with config: {}".format(config_path))
    
    def _setup_logging(self, log_file_path):
        """Setup logging to file with timestamp."""
        log_dir = os.path.dirname(log_file_path)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        self.logger = logging.getLogger('VSRProApp')
        self.logger.setLevel(logging.INFO)
        self.logger.handlers = []
        
        file_handler = logging.FileHandler(log_file_path, mode='a', encoding='utf-8')
        file_handler.setLevel(logging.INFO)
        
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)
        
        self.logger.info("=" * 80)
        self.logger.info("VSRProApp Application started")
    
    # ========================================================================
    # Simulation Control API
    # ========================================================================
    
    def start_simulation(self):
        """
        Start the physics simulation and create the robot.
        
        Returns:
            bool: True if successful, False otherwise
        """
        if self.simulation_started:
            self.logger.warning("Simulation already started")
            return False
        
        try:
            self.logger.info("Starting simulation...")
            self.simulation_core = SimulationCore(self.config_path)
            self.simulation_core.create_robot()
            
            self.simulation_started = True
            self.running = True
            self.stop_simulation_flag = False
            
            # Start simulation thread
            self.simulation_thread = threading.Thread(target=self._run_simulation_loop, daemon=True)
            self.simulation_thread.start()
            
            self.logger.info("Simulation started successfully")
            
            # Load PBR objects from config if enabled
            self._load_pbr_from_config()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start simulation: {e}")
            return False
    
    def _load_pbr_from_config(self):
        """
        Load PBR object from configuration if enabled.
        This is called automatically during start_simulation() and reset_simulation().
        """
        pbr_config = self.config.get("structure", {}).get("pbr", {})
        
        if not pbr_config.get("enable", False):
            self.logger.info("PBR loading from config is disabled")
            return
        
        try:
            obj_path = pbr_config.get("path")
            if not obj_path:
                self.logger.warning("PBR enabled but no path specified in config")
                return
            
            # Get parameters from config
            position_offset = pbr_config.get("position_offset", [0.0, 0.0, 0.5])
            orientation_offset = pbr_config.get("orientation_offset", [0.0, 0.0, 0.0])
            scale = pbr_config.get("scale", [1.0, 1.0, 1.0])
            mass = pbr_config.get("mass", 10.0)
            inertia = pbr_config.get("inertia", [0.1, 0.1, 0.1])
            restitution = pbr_config.get("restitution", 0.3)
            lateral_friction = pbr_config.get("lateralFriction", 0.5)
            spinning_friction = pbr_config.get("spinningFriction", 0.3)
            contact_damping = pbr_config.get("contactDamping", 100000000.0)
            contact_stiffness = pbr_config.get("contactStiffness", 10000000000.0)
            
            # Load the object
            obj_id = self.load_pbr_object(
                obj_path=obj_path,
                offset=position_offset,
                orientation_euler=orientation_offset,
                mesh_scale=scale,
                mass=mass,
                inertia=inertia,
                restitution=restitution,
                lateral_friction=lateral_friction,
                spinning_friction=spinning_friction,
                contact_damping=contact_damping,
                contact_stiffness=contact_stiffness
            )
            
            if obj_id is not None:
                self.logger.info(f"Loaded PBR object from config: {obj_path} (ID={obj_id})")
            else:
                self.logger.error(f"Failed to load PBR object from config: {obj_path}")
                
        except Exception as e:
            self.logger.error(f"Error loading PBR from config: {e}")
    
    def stop_simulation(self):
        """
        Stop the physics simulation and cleanup resources.
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.simulation_started:
            self.logger.warning("Simulation not started")
            return False
        
        try:
            self.logger.info("Stopping simulation...")
            
            # Stop any running trajectory
            if self.trajectory_running or self.disp_trajectory_running:
                self.stop_trajectory()
            
            # Stop recording
            if self.recording_active:
                self.stop_recording()
            
            # Signal thread to stop
            self.stop_simulation_flag = True
            self.running = False
            
            # Wait for thread to finish
            if self.simulation_thread and self.simulation_thread.is_alive():
                self.simulation_thread.join(timeout=2.0)
            
            # Disconnect from PyBullet
            if p.isConnected():
                p.disconnect()
            
            self.simulation_core = None
            self.simulation_started = False
            self.simulation_thread = None
            
            self.logger.info("Simulation stopped successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Error stopping simulation: {e}")
            return False
    
    def pause_simulation(self):
        """
        Pause the simulation (stop stepping physics).
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.running or self.paused:
            return False
        
        self.paused = True
        self.logger.info("Simulation paused")
        return True
    
    def resume_simulation(self):
        """
        Resume the simulation (continue stepping physics).
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.running or not self.paused:
            return False
        
        self.paused = False
        self.logger.info("Simulation resumed")
        return True
    
    def reset_simulation(self):
        """
        Reset the simulation (stop, then start fresh).
        
        Returns:
            bool: True if successful, False otherwise
        """
        self.logger.info("Resetting simulation...")
        
        # Stop current simulation
        self.stop_simulation()
        
        # Wait a moment for cleanup
        time.sleep(0.5)
        
        # Start fresh
        success = self.start_simulation()
        
        if success:
            self.logger.info("Simulation reset successfully")
        
        return success
    
    # ========================================================================
    # Object Loading API
    # ========================================================================
    
    def load_pbr_object(self, obj_path, offset=[0.0, 0.0, 0.0], 
                       orientation_euler=[0.0, 0.0, 0.0], 
                       mesh_scale=[1.0, 1.0, 1.0],
                       mass=10.0, inertia=[0.1, 0.1, 0.1],
                       restitution=0.2, 
                       lateral_friction=0.5, spinning_friction=0.1,
                       contact_damping=10000.0, contact_stiffness=1000000.0):
        """
        Load and spawn an OBJ file on the pedestal with PBR properties.
        
        Args:
            obj_path (str): Path to the .obj file
            offset (list): [dx, dy, dz] offset from pedestal center
            orientation_euler (list): [roll, pitch, yaw] in radians
            mesh_scale (list): [sx, sy, sz] scaling factors
            mass (float): Mass of the object (kg)
            inertia (list): [Ixx, Iyy, Izz] inertia on principal axes (kg·m²)
            restitution (float): Coefficient of restitution (bounciness)
            lateral_friction (float): Lateral friction coefficient
            spinning_friction (float): Spinning friction coefficient
            contact_damping (float): Contact damping
            contact_stiffness (float): Contact stiffness
        
        Returns:
            int: Object ID if successful, None otherwise
        """
        if not self.simulation_started or self.simulation_core is None:
            self.logger.error("Cannot load object: simulation not started")
            return None
        
        try:
            pbr_props = {
                "mass": mass,
                "localInertiaDiagonal": inertia,
                "restitution": restitution,
                "lateralFriction": lateral_friction,
                "spinningFriction": spinning_friction,
                "contactDamping": contact_damping,
                "contactStiffness": contact_stiffness
            }
            
            obj_id = self.simulation_core.spawn_obj_on_pedestal(
                obj_path=obj_path,
                mesh_scale=mesh_scale,
                offset=offset,
                orientation_euler=orientation_euler,
                pbr_props=pbr_props
            )
            
            if obj_id is not None:
                self.register_pbr_object(obj_id)
                self.logger.info(f"Loaded PBR object: {obj_path}, ID={obj_id}")
                return obj_id
            else:
                self.logger.error(f"Failed to spawn object: {obj_path}")
                return None
                
        except Exception as e:
            self.logger.error(f"Error loading PBR object: {e}")
            return None
    
    def register_pbr_object(self, object_id):
        """
        Register a PBR object for tracking and recording.
        
        Args:
            object_id (int): PyBullet object ID
        """
        self.pbr_object_ids.add(object_id)
        
        # If recording is active, initialize recording list for this object
        if self.recording_active:
            if object_id not in self.recorded_pbr_poses:
                self.recorded_pbr_poses[object_id] = []
                # Backfill with empty entries to match timeline
                num_existing_samples = len(self.recorded_times)
                for _ in range(num_existing_samples):
                    self.recorded_pbr_poses[object_id].append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.logger.info(f"Registered PBR object ID: {object_id}")
    
    # ========================================================================
    # Trajectory Execution API
    # ========================================================================
    
    def set_pulse_trajectory(self, cycle_number=1,
                           amp_x=0.0, freq_x=0.0,
                           amp_y=0.0, freq_y=0.0,
                           amp_z=0.0, freq_z=0.0,
                           amp_roll=0.0, freq_roll=0.0,
                           amp_pitch=0.0, freq_pitch=0.0,
                           amp_yaw=0.0, freq_yaw=0.0):
        """
        Set parameters for pulse (sinusoidal) trajectory.
        
        Args:
            cycle_number (int): Number of oscillation cycles
            amp_* (float): Amplitude for each DOF (m or rad)
            freq_* (float): Frequency for each DOF (Hz)
        """
        self.trajectory_params = {
            "cycle_number": cycle_number,
            "amp_x": amp_x, "freq_x": freq_x,
            "amp_y": amp_y, "freq_y": freq_y,
            "amp_z": amp_z, "freq_z": freq_z,
            "amp_roll": amp_roll, "freq_roll": freq_roll,
            "amp_pitch": amp_pitch, "freq_pitch": freq_pitch,
            "amp_yaw": amp_yaw, "freq_yaw": freq_yaw
        }
        self.logger.info(f"Set pulse trajectory: {cycle_number} cycles")
    
    def start_pulse_trajectory(self):
        """
        Start executing the pulse trajectory.
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.simulation_started:
            self.logger.error("Cannot start trajectory: simulation not started")
            return False
        
        if self.trajectory_running or self.disp_trajectory_running:
            self.logger.warning("A trajectory is already running")
            return False
        
        self.trajectory_running = True
        self.start_recording()
        self.logger.info("Started pulse trajectory execution")
        return True
    
    def load_displacement_trajectory(self, csv_path):
        """
        Load a displacement trajectory from a CSV file.
        
        Args:
            csv_path (str): Path to the CSV file
        
        Returns:
            bool: True if successful, False otherwise
        """
        success, data = self._validate_and_load_displacement_trajectory(csv_path)
        
        if success:
            self.disp_path = csv_path
            self.disp_trajectory_data = data
            self.logger.info(f"Loaded displacement trajectory: {csv_path}")
            return True
        else:
            self.logger.error(f"Failed to load displacement trajectory: {csv_path}")
            return False
    
    def start_displacement_trajectory(self):
        """
        Start executing the displacement trajectory.
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.simulation_started:
            self.logger.error("Cannot start trajectory: simulation not started")
            return False
        
        if self.trajectory_running or self.disp_trajectory_running:
            self.logger.warning("A trajectory is already running")
            return False
        
        if self.disp_trajectory_data is None:
            self.logger.error("No displacement trajectory loaded")
            return False
        
        self.disp_trajectory_running = True
        self.start_recording()
        self.logger.info("Started displacement trajectory execution")
        return True
    
    def stop_trajectory(self):
        """
        Stop the currently running trajectory.
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.trajectory_running and not self.disp_trajectory_running:
            return False
        
        self.trajectory_running = False
        self.disp_trajectory_running = False
        self.stop_recording()
        
        self.logger.info("Stopped trajectory execution")
        return True
    
    # ========================================================================
    # Data Recording API
    # ========================================================================
    
    def start_recording(self):
        """
        Start recording trajectory data.
        """
        if self.recording_active:
            return
        
        self.recording_active = True
        self.recorded_times = []
        self.recorded_pedestal_poses = []
        self.recorded_pbr_poses = {obj_id: [] for obj_id in self.pbr_object_ids}
        self.recording_start_time = time.time()
        
        self.logger.info("Started recording trajectory data")
    
    def stop_recording(self):
        """
        Stop recording and save data to file.
        """
        if not self.recording_active:
            return
        
        self.recording_active = False
        self.save_recording_data()
        
        self.logger.info("Stopped recording trajectory data")
    
    def save_recording_data(self):
        """
        Save recorded trajectory data to NPZ file.
        """
        if not self.recorded_times:
            self.logger.warning("No data to save")
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        data_dir = "data/logs"
        os.makedirs(data_dir, exist_ok=True)
        filename = os.path.join(data_dir, f"run_{timestamp}.npz")
        
        # Convert lists to numpy arrays
        times_array = np.array(self.recorded_times)
        pedestal_poses_array = np.array(self.recorded_pedestal_poses)
        
        # Build dictionary for saving
        save_dict = {
            'times': times_array,
            'pedestal_poses': pedestal_poses_array
        }
        
        # Add PBR object poses
        for obj_id, poses in self.recorded_pbr_poses.items():
            if poses:  # Only save if we have data
                save_dict[f'pbr_object_{obj_id}_poses'] = np.array(poses)
        
        # Save metadata
        metadata = {
            'simulation_frequency': self.simulation_frequency,
            'use_real_time': self.use_real_time,
            'num_samples': len(self.recorded_times),
            'duration': times_array[-1] if len(times_array) > 0 else 0.0
        }
        save_dict['metadata'] = metadata
        
        np.savez(filename, **save_dict)
        self.logger.info(f"Saved trajectory data: {filename}")
        self.logger.info(f"  Samples: {len(self.recorded_times)}, Duration: {metadata['duration']:.2f}s")
    
    def record_current_poses(self, simulation_time):
        """
        Record current poses of pedestal and all PBR objects.
        
        Args:
            simulation_time (float): Current simulation time in seconds
        """
        if not self.recording_active or self.simulation_core is None:
            return
        
        try:
            # Get pedestal pose
            pedestal_state = p.getLinkState(self.simulation_core.robot_id, 3)
            pedestal_pos = pedestal_state[0]
            pedestal_orn_quat = pedestal_state[1]
            pedestal_orn_euler = p.getEulerFromQuaternion(pedestal_orn_quat)
            
            # Store: [time, x, y, z, roll, pitch, yaw]
            self.recorded_times.append(simulation_time)
            self.recorded_pedestal_poses.append([
                pedestal_pos[0], pedestal_pos[1], pedestal_pos[2],
                pedestal_orn_euler[0], pedestal_orn_euler[1], pedestal_orn_euler[2]
            ])
            
            # Record all PBR objects
            for obj_id in self.pbr_object_ids:
                try:
                    obj_pos, obj_orn_quat = p.getBasePositionAndOrientation(obj_id)
                    obj_orn_euler = p.getEulerFromQuaternion(obj_orn_quat)
                    
                    self.recorded_pbr_poses[obj_id].append([
                        obj_pos[0], obj_pos[1], obj_pos[2],
                        obj_orn_euler[0], obj_orn_euler[1], obj_orn_euler[2]
                    ])
                except:
                    pass  # Object may have been removed
                    
        except Exception as e:
            self.logger.error(f"Error recording poses: {e}")
    
    # ========================================================================
    # Status Query API
    # ========================================================================
    
    def get_pedestal_pose(self):
        """
        Get current pedestal position and orientation.
        
        Returns:
            dict: {'position': [x, y, z], 'orientation': [roll, pitch, yaw]} or None
        """
        if not self.simulation_started or self.simulation_core is None:
            return None
        
        try:
            state = p.getLinkState(self.simulation_core.robot_id, 3)
            pos = state[0]
            orn_quat = state[1]
            orn_euler = p.getEulerFromQuaternion(orn_quat)
            
            return {
                'position': [pos[0], pos[1], pos[2]],
                'orientation': [orn_euler[0], orn_euler[1], orn_euler[2]]
            }
        except:
            return None
    
    def is_simulation_running(self):
        """Check if simulation is running."""
        return self.running and not self.paused
    
    def is_trajectory_running(self):
        """Check if any trajectory is running."""
        return self.trajectory_running or self.disp_trajectory_running
    
    # ========================================================================
    # Internal Methods
    # ========================================================================
    
    def _run_simulation_loop(self):
        """
        Main simulation loop (runs in separate thread).
        This handles physics stepping and trajectory execution.
        """
        trajectory_start_time = None
        trajectory_sim_time = 0.0
        disp_traj_index = 0
        
        while self.running and not self.stop_simulation_flag:
            if self.paused:
                time.sleep(0.01)
                continue
            
            # Execute pulse trajectory
            if self.trajectory_running:
                if trajectory_start_time is None:
                    trajectory_start_time = time.time()
                    trajectory_sim_time = 0.0
                
                elapsed = time.time() - trajectory_start_time
                trajectory_sim_time = elapsed
                
                # Check if trajectory completed
                max_freq = max([
                    self.trajectory_params.get("freq_x", 0.0),
                    self.trajectory_params.get("freq_y", 0.0),
                    self.trajectory_params.get("freq_z", 0.0),
                    self.trajectory_params.get("freq_roll", 0.0),
                    self.trajectory_params.get("freq_pitch", 0.0),
                    self.trajectory_params.get("freq_yaw", 0.0)
                ])
                
                if max_freq > 0:
                    cycle_number = self.trajectory_params.get("cycle_number", 1)
                    total_duration = cycle_number / max_freq
                    
                    if trajectory_sim_time >= total_duration:
                        self.trajectory_running = False
                        trajectory_start_time = None
                        self.stop_recording()
                        self.logger.info("Pulse trajectory completed")
                    else:
                        self.simulation_core.step_trajectory(self.trajectory_params, trajectory_sim_time)
                        self.record_current_poses(trajectory_sim_time)
            
            # Execute displacement trajectory
            elif self.disp_trajectory_running:
                if trajectory_start_time is None:
                    trajectory_start_time = time.time()
                    disp_traj_index = 0
                
                if disp_traj_index < len(self.disp_trajectory_data):
                    displacement = self.disp_trajectory_data[disp_traj_index]
                    self.simulation_core.step_displacement_trajectory(displacement)
                    
                    elapsed = time.time() - trajectory_start_time
                    self.record_current_poses(elapsed)
                    
                    disp_traj_index += 1
                else:
                    self.disp_trajectory_running = False
                    trajectory_start_time = None
                    self.stop_recording()
                    self.logger.info("Displacement trajectory completed")
            
            # Hold pedestal position when no trajectory is running
            else:
                if trajectory_start_time is not None:
                    trajectory_start_time = None
                self.simulation_core.hold_pedestal_position()
            
            # Step physics
            p.stepSimulation()
            
            # Real-time synchronization
            if self.use_real_time:
                time.sleep(self.simulation_dt)
    
    def _validate_and_load_displacement_trajectory(self, csv_path):
        """
        Validate and load displacement trajectory from CSV.
        
        Args:
            csv_path (str): Path to CSV file
        
        Returns:
            tuple: (success: bool, data: list or None)
        """
        try:
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                headers = reader.fieldnames
                
                # Validate headers
                if 'time_s' not in headers:
                    self.logger.error("CSV must contain 'time_s' column")
                    return False, None
                
                # Check sampling frequency
                rows = list(reader)
                if len(rows) < 2:
                    self.logger.error("CSV must contain at least 2 data points")
                    return False, None
                
                time_0 = float(rows[0]['time_s'])
                time_1 = float(rows[1]['time_s'])
                dt = time_1 - time_0
                
                if abs(dt - self.simulation_dt) > 1e-6:
                    self.logger.error(f"CSV sampling rate ({1/dt:.1f} Hz) must match simulation ({self.simulation_frequency} Hz)")
                    return False, None
                
                # Extract displacement data
                dof_names = ['X_m', 'Y_m', 'Z_m', 'Roll_rad', 'Pitch_rad', 'Yaw_rad']
                trajectory_data = []
                
                for row in rows:
                    displacement = []
                    for dof in dof_names:
                        value = float(row.get(dof, 0.0))
                        displacement.append(value)
                    trajectory_data.append(displacement)
                
                self.logger.info(f"Loaded {len(trajectory_data)} trajectory points from {csv_path}")
                return True, trajectory_data
                
        except Exception as e:
            self.logger.error(f"Error loading displacement trajectory: {e}")
            return False, None
