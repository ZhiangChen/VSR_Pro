# main_gui.py
import sys
import time
import math
import threading
import yaml
import logging
import os
from datetime import datetime
import numpy as np

import pybullet as p

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QCheckBox, QFileDialog, QDialog, QGroupBox,
    QPushButton, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer

from simulation_core import SimulationCore

###############################################################################
# Enhanced Dialog for PBR/Collision + Live Preview
###############################################################################
class PBRLoadDialog(QDialog):
    """
    A dialog for live preview spawning an OBJ with user-specified
    offset/orientation + PBR and collision settings.

    Buttons:
      - Apply: spawn or update the object in real time
      - Confirm: finalize (keep the object)
      - Cancel: remove the object
    """
    def __init__(self, simulation_core, obj_path=None, parent=None, pedestal_half_z=0.0):
        super().__init__(parent)
        self.obj_path = obj_path  # Can be None initially
        self.sim_core = simulation_core
        self.pedestal_half_z = pedestal_half_z

        self.setWindowTitle("Spawn OBJ with PBR & Collision Properties")

        # Default offsets/orientation
        self.default_x = 0.0
        self.default_y = 0.0
        self.default_z = 0.0
        self.default_roll = 0.0
        self.default_pitch = 0.0
        self.default_yaw = 0.0

        # Keep track of the previewed object's ID
        self.preview_obj_id = None

        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout()

        # ---------------------------------------------------------------------
        # 0) OBJ File Selection
        # ---------------------------------------------------------------------
        file_group = QGroupBox("OBJ File")
        file_layout = QHBoxLayout()
        
        self.obj_path_label = QLabel("No file selected" if not self.obj_path else self.obj_path)
        self.obj_path_label.setWordWrap(True)
        file_layout.addWidget(self.obj_path_label, stretch=1)
        
        self.browse_button = QPushButton("Browse...")
        self.browse_button.clicked.connect(self.browse_obj_file)
        file_layout.addWidget(self.browse_button)
        
        file_group.setLayout(file_layout)
        main_layout.addWidget(file_group)

        # ---------------------------------------------------------------------
        # 1) Position & Orientation
        # ---------------------------------------------------------------------
        pose_group = QGroupBox("Pose (to pedestal top center)")
        pose_layout = QGridLayout()

        # First row: Position (X, Y, Z)
        pose_layout.addWidget(QLabel("X(m):"), 0, 0)
        self.x_input = QLineEdit(str(self.default_x))
        self.x_input.setFixedWidth(80)
        pose_layout.addWidget(self.x_input, 0, 1)

        pose_layout.addWidget(QLabel("Y(m):"), 0, 2)
        self.y_input = QLineEdit(str(self.default_y))
        self.y_input.setFixedWidth(80)
        pose_layout.addWidget(self.y_input, 0, 3)

        pose_layout.addWidget(QLabel("Z(m):"), 0, 4)
        self.z_input = QLineEdit(str(self.default_z))
        self.z_input.setFixedWidth(80)
        pose_layout.addWidget(self.z_input, 0, 5)

        # Second row: Orientation (Roll, Pitch, Yaw in degrees)
        pose_layout.addWidget(QLabel("Roll(deg):"), 1, 0)
        self.roll_input = QLineEdit(str(math.degrees(self.default_roll)))
        self.roll_input.setFixedWidth(80)
        pose_layout.addWidget(self.roll_input, 1, 1)

        pose_layout.addWidget(QLabel("Pitch(deg):"), 1, 2)
        self.pitch_input = QLineEdit(str(math.degrees(self.default_pitch)))
        self.pitch_input.setFixedWidth(80)
        pose_layout.addWidget(self.pitch_input, 1, 3)

        pose_layout.addWidget(QLabel("Yaw(deg):"), 1, 4)
        self.yaw_input = QLineEdit(str(math.degrees(self.default_yaw)))
        self.yaw_input.setFixedWidth(80)
        pose_layout.addWidget(self.yaw_input, 1, 5)

        pose_group.setLayout(pose_layout)
        main_layout.addWidget(pose_group)

        # ---------------------------------------------------------------------
        # 2) Collision & PBR Properties
        # ---------------------------------------------------------------------
        pbr_group = QGroupBox("Collision & PBR Properties")
        pbr_layout = QGridLayout()


        # mass
        pbr_layout.addWidget(QLabel("Mass(kg):"), 1, 0)
        self.mass_input = QLineEdit("0.0")
        #self.mass_input.setFixedWidth(120)
        pbr_layout.addWidget(self.mass_input, 1, 1)

        # restitution
        pbr_layout.addWidget(QLabel("Restitution:"), 2, 0)
        self.restitution_input = QLineEdit("0.2")
        pbr_layout.addWidget(self.restitution_input, 2, 1)

        # friction
        pbr_layout.addWidget(QLabel("Lateral Friction:"), 3, 0)
        self.lateral_friction_input = QLineEdit("0.5")
        pbr_layout.addWidget(self.lateral_friction_input, 3, 1)

        # spinning friction
        pbr_layout.addWidget(QLabel("Spinning Friction:"), 4, 0)
        self.spinning_input = QLineEdit("0.1")
        pbr_layout.addWidget(self.spinning_input, 4, 1)

        # contact damping
        pbr_layout.addWidget(QLabel("Contact Damp:"), 5, 0)
        self.contact_damp_input = QLineEdit("10000.0")
        pbr_layout.addWidget(self.contact_damp_input, 5, 1)

        # contact stiffness
        pbr_layout.addWidget(QLabel("Contact Stiff:"), 6, 0)
        self.contact_stiff_input = QLineEdit("1000000.0")
        pbr_layout.addWidget(self.contact_stiff_input, 6, 1)

        pbr_group.setLayout(pbr_layout)
        main_layout.addWidget(pbr_group)

        # ---------------------------------------------------------------------
        # 3) Buttons row: [Apply] [Confirm] [Cancel]
        # ---------------------------------------------------------------------
        button_layout = QHBoxLayout()
        self.apply_button = QPushButton("Apply")
        self.apply_button.clicked.connect(self.on_apply)
        button_layout.addWidget(self.apply_button)

        self.confirm_button = QPushButton("Confirm")
        self.confirm_button.clicked.connect(self.on_confirm)
        button_layout.addWidget(self.confirm_button)

        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.clicked.connect(self.on_cancel)
        button_layout.addWidget(self.cancel_button)

        main_layout.addLayout(button_layout)
        self.setLayout(main_layout)

    # -------------------------------------------------------------------------
    # Parse user fields
    # -------------------------------------------------------------------------
    def parse_transform(self):
        """Returns (offset, orientation_euler) from user inputs."""
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())
        except ValueError:
            x, y, z = 0, 0, 0

        try:
            roll_deg  = float(self.roll_input.text())
            pitch_deg = float(self.pitch_input.text())
            yaw_deg   = float(self.yaw_input.text())
        except ValueError:
            roll_deg, pitch_deg, yaw_deg = 0, 0, 0

        roll_rad  = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad   = math.radians(yaw_deg)
        return [x, y, z], [roll_rad, pitch_rad, yaw_rad]

    def parse_pbr(self):
        """
        Returns (enable_collision: bool, pbr_props: dict)
        """

        pbr_props = {}
        # mass
        try:
            pbr_props["mass"] = float(self.mass_input.text())
        except ValueError:
            pbr_props["mass"] = 0.0

        # restitution
        try:
            pbr_props["restitution"] = float(self.restitution_input.text())
        except ValueError:
            pbr_props["restitution"] = 0.0

        # friction
        try:
            pbr_props["lateralFriction"] = float(self.lateral_friction_input.text())
        except ValueError:
            pbr_props["lateralFriction"] = 0.5

        # spinning friction
        try:
            pbr_props["spinningFriction"] = float(self.spinning_input.text())
        except ValueError:
            pbr_props["spinningFriction"] = 0.0

        # contact damping
        try:
            pbr_props["contactDamping"] = float(self.contact_damp_input.text())
        except ValueError:
            pbr_props["contactDamping"] = 0.0

        # contact stiffness
        try:
            pbr_props["contactStiffness"] = float(self.contact_stiff_input.text())
        except ValueError:
            pbr_props["contactStiffness"] = 1e5

        return pbr_props

    # -------------------------------------------------------------------------
    # Browse for OBJ file
    # -------------------------------------------------------------------------
    def browse_obj_file(self):
        """Open file dialog to select an OBJ file."""
        obj_path, _ = QFileDialog.getOpenFileName(self, "Select OBJ File", "", "OBJ Files (*.obj)")
        if obj_path:
            self.obj_path = obj_path
            self.obj_path_label.setText(obj_path)
            # Note: parent is PyBulletGUI which has logger
            if hasattr(self.parent(), 'logger'):
                self.parent().logger.info(f"Selected OBJ file: {obj_path}")
            
            # If there's an existing preview object, remove it since we're changing files
            if self.preview_obj_id is not None:
                p.removeBody(self.preview_obj_id)
                self.preview_obj_id = None
                if hasattr(self.parent(), 'logger'):
                    self.parent().logger.info("Removed previous preview object due to file change.")

    # -------------------------------------------------------------------------
    # "Apply" => spawn or update live
    # -------------------------------------------------------------------------
    def on_apply(self):
        # Check if OBJ file is selected
        if not self.obj_path:
            QMessageBox.warning(
                self,
                "No File Selected",
                "Please select an OBJ file first by clicking the 'Browse...' button."
            )
            return
        
        offset, orientation_euler = self.parse_transform()
        pbr_props = self.parse_pbr()

        # If we haven't spawned it yet, do so; otherwise update
        if self.preview_obj_id is None:
            self.preview_obj_id = self.sim_core.spawn_obj_on_pedestal(
                obj_path=self.obj_path,
                mesh_scale=[1,1,1],
                offset=offset,
                orientation_euler=orientation_euler,
                pbr_props=pbr_props
            )
            if hasattr(self.parent(), 'logger'):
                self.parent().logger.info(f"[GUI] Created new object with ID={self.preview_obj_id}")
            
            # Register object for data recording
            if hasattr(self.parent(), 'register_pbr_object'):
                self.parent().register_pbr_object(self.preview_obj_id)
        else:
            # Already spawned => update transform & dynamics
            # 1) compute final position based on pedestal
            pedestal_state = p.getLinkState(self.sim_core.robot_id, 3, physicsClientId=self.sim_core.client_id)
            ped_pos = pedestal_state[0]
            final_pos = [
                ped_pos[0] + offset[0],
                ped_pos[1] + offset[1],
                ped_pos[2] + offset[2] + self.pedestal_half_z
            ]
            final_orn = p.getQuaternionFromEuler(orientation_euler)

            p.resetBasePositionAndOrientation(
                self.preview_obj_id,
                final_pos,
                final_orn,
                physicsClientId=self.sim_core.client_id
            )
            # 2) reapply friction, restitution, etc. 
            # (PyBullet can't change mass after creation)
            p.changeDynamics(
                self.preview_obj_id,
                -1,
                restitution=pbr_props.get("restitution", 0.0),
                lateralFriction=pbr_props.get("lateralFriction", 0.5),
                spinningFriction=pbr_props.get("spinningFriction", 0.0),
                contactDamping=pbr_props.get("contactDamping", 0.0),
                contactStiffness=pbr_props.get("contactStiffness", 1e5),
                physicsClientId=self.sim_core.client_id
            )
            if hasattr(self.parent(), 'logger'):
                self.parent().logger.info(f"[GUI] Updated object ID={self.preview_obj_id} to new transform & properties")

    # -------------------------------------------------------------------------
    # "Confirm" => keep the object
    # -------------------------------------------------------------------------
    def on_confirm(self):
        self.accept()

    # -------------------------------------------------------------------------
    # "Cancel" => remove the object
    # -------------------------------------------------------------------------
    def on_cancel(self):
        if self.preview_obj_id is not None:
            p.removeBody(self.preview_obj_id)
            self.preview_obj_id = None
        self.reject()

###############################################################################
# Main GUI
###############################################################################
class PyBulletGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.simulation_core = None
        self.simulation_thread = None
        self.simulation_started = False  # Flag to track if simulation has been started
        self.running = False
        self.paused = False  # New state for pause functionality
        self.trajectory_running = False
        self.stop_simulation_flag = False
        
        # Last known pedestal position for position control
        self.last_pedestal_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]

        # load config
        yaml_path = "config.yaml"
        with open(yaml_path, "r") as file:
            self.config = yaml.safe_load(file)

        self.use_real_time = self.config["simulation_settings"]["use_real_time"]

        self.pedestal_half_z = self.config["structure"]["pedestal"]["dimensions"][2] / 2.0
        self.simulation_frequency = self.config["simulation_settings"].get("simulation_frequency", 500)
        self.simulation_dt = 1.0 / self.simulation_frequency

        self.GUI_update_hz = self.config["simulation_settings"].get("GUI_update_frequency", 60)

        # default trajectory params
        self.trajectory_params = {
            "cycle_number": 1,  # Number of oscillation cycles
            "amp_x": 0.0,   "freq_x": 0.0,
            "amp_y": 0.0,   "freq_y": 0.0,
            "amp_z": 0.0,   "freq_z": 0.0,
            "amp_roll": 0.0,"freq_roll": 0.0,
            "amp_pitch":0.0,"freq_pitch":0.0,
            "amp_yaw": 0.0, "freq_yaw": 0.0
        }

        self.param_inputs = {}
        self.pgv_labels = {}  # PGV display labels
        self.pga_labels = {}  # PGA display labels

        # GUI update timer (separate from simulation stepping)
        self.gui_timer = QTimer(self)
        self.gui_timer.setTimerType(Qt.PreciseTimer)
        self.gui_timer.timeout.connect(self.update_gui_display)
        
        # Data recording attributes
        self.recording_active = False
        self.recorded_times = []
        self.recorded_pedestal_poses = []
        self.recorded_pbr_poses = {}  # Dictionary: object_id -> list of poses
        self.pbr_object_ids = set()  # Track all PBR objects spawned
        self.recording_start_time = None
        
        # Setup logging
        log_file_path = self.config["simulation_settings"].get("log_file", "data/simulation.log")
        self._setup_logging(log_file_path)
        
        self.init_ui()

    def _setup_logging(self, log_file_path):
        """Setup logging to file with timestamp - shared with SimulationCore."""
        # Create directory if it doesn't exist
        log_dir = os.path.dirname(log_file_path)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # Configure logging
        self.logger = logging.getLogger('GUI')
        self.logger.setLevel(logging.INFO)
        
        # Remove existing handlers to avoid duplicates
        self.logger.handlers = []
        
        # File handler with UTF-8 encoding to support Unicode characters
        file_handler = logging.FileHandler(log_file_path, mode='a', encoding='utf-8')
        file_handler.setLevel(logging.INFO)
        
        # Formatter with timestamp
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler.setFormatter(formatter)
        
        self.logger.addHandler(file_handler)
        
        # Log GUI initialization
        self.logger.info("="*80)
        self.logger.info("GUI Application started")

    def init_ui(self):
        self.setWindowTitle("VSR Pro")
        self.setGeometry(100, 100, 650, 550)

        main_layout = QVBoxLayout(self)

        # ---------------- Simulation Controls ----------------
        sim_group = QGroupBox("Simulation Controls")
        sim_layout = QVBoxLayout()

        # Button row layout: Start | Stop | Pause | Reset
        button_layout = QHBoxLayout()
        
        self.start_button = QPushButton("Start Simulation")
        self.start_button.clicked.connect(self.start_simulation)
        button_layout.addWidget(self.start_button)
        
        self.stop_button = QPushButton("Stop Simulation")
        self.stop_button.clicked.connect(self.stop_simulation)
        self.stop_button.setEnabled(False)  # Initially disabled
        button_layout.addWidget(self.stop_button)
        
        self.pause_button = QPushButton("Pause Simulation")
        self.pause_button.clicked.connect(self.toggle_pause_simulation)
        self.pause_button.setEnabled(False)  # Initially disabled
        button_layout.addWidget(self.pause_button)

        self.reset_button = QPushButton("Reset Simulation")
        self.reset_button.clicked.connect(self.reset_simulation)
        self.reset_button.setEnabled(False)  # Initially disabled until simulation is created
        button_layout.addWidget(self.reset_button)
        
        sim_layout.addLayout(button_layout)

        self.position_label = QLabel("Pedestal Position: (0.0, 0.0, 0.0)")
        sim_layout.addWidget(self.position_label)
        
        self.orientation_label = QLabel("Pedestal Orientation: Roll=0.0°, Pitch=0.0°, Yaw=0.0°")
        sim_layout.addWidget(self.orientation_label)

        sim_group.setLayout(sim_layout)
        main_layout.addWidget(sim_group)

        # ---------------- Trajectory Settings ----------------
        traj_group = QGroupBox("Trajectory Settings")
        traj_layout = QVBoxLayout()

        # Cycle Number input
        cycle_layout = QHBoxLayout()
        cycle_layout.addWidget(QLabel("Cycle Number:"))
        self.param_inputs["cycle_number"] = QLineEdit(str(self.trajectory_params["cycle_number"]))
        cycle_layout.addWidget(self.param_inputs["cycle_number"])
        cycle_layout.addStretch()  # Add stretch to keep it compact
        traj_layout.addLayout(cycle_layout)

        # Linear DOFs
        linear_box = QGroupBox("Linear Motion (X, Y, Z)")
        linear_layout = QGridLayout()

        # Header row
        linear_layout.addWidget(QLabel("Axis"), 0, 0)
        linear_layout.addWidget(QLabel("Amp (m)"), 0, 1)
        linear_layout.addWidget(QLabel("Freq (Hz)"), 0, 2)
        linear_layout.addWidget(QLabel("PGV (m/s)"), 0, 3)
        linear_layout.addWidget(QLabel("PGA (m/s²)"), 0, 4)

        # X axis
        linear_layout.addWidget(QLabel("X:"), 1, 0)
        self.param_inputs["amp_x"] = QLineEdit(str(self.trajectory_params["amp_x"]))
        self.param_inputs["amp_x"].textChanged.connect(lambda: self.update_pgv_pga('x'))
        linear_layout.addWidget(self.param_inputs["amp_x"], 1, 1)

        self.param_inputs["freq_x"] = QLineEdit(str(self.trajectory_params["freq_x"]))
        self.param_inputs["freq_x"].textChanged.connect(lambda: self.update_pgv_pga('x'))
        linear_layout.addWidget(self.param_inputs["freq_x"], 1, 2)

        self.pgv_labels["x"] = QLabel("0.00")
        linear_layout.addWidget(self.pgv_labels["x"], 1, 3)

        self.pga_labels["x"] = QLabel("0.00")
        linear_layout.addWidget(self.pga_labels["x"], 1, 4)

        # Y axis
        linear_layout.addWidget(QLabel("Y:"), 2, 0)
        self.param_inputs["amp_y"] = QLineEdit(str(self.trajectory_params["amp_y"]))
        self.param_inputs["amp_y"].textChanged.connect(lambda: self.update_pgv_pga('y'))
        linear_layout.addWidget(self.param_inputs["amp_y"], 2, 1)

        self.param_inputs["freq_y"] = QLineEdit(str(self.trajectory_params["freq_y"]))
        self.param_inputs["freq_y"].textChanged.connect(lambda: self.update_pgv_pga('y'))
        linear_layout.addWidget(self.param_inputs["freq_y"], 2, 2)

        self.pgv_labels["y"] = QLabel("0.00")
        linear_layout.addWidget(self.pgv_labels["y"], 2, 3)

        self.pga_labels["y"] = QLabel("0.00")
        linear_layout.addWidget(self.pga_labels["y"], 2, 4)

        # Z axis
        linear_layout.addWidget(QLabel("Z:"), 3, 0)
        self.param_inputs["amp_z"] = QLineEdit(str(self.trajectory_params["amp_z"]))
        self.param_inputs["amp_z"].textChanged.connect(lambda: self.update_pgv_pga('z'))
        linear_layout.addWidget(self.param_inputs["amp_z"], 3, 1)

        self.param_inputs["freq_z"] = QLineEdit(str(self.trajectory_params["freq_z"]))
        self.param_inputs["freq_z"].textChanged.connect(lambda: self.update_pgv_pga('z'))
        linear_layout.addWidget(self.param_inputs["freq_z"], 3, 2)

        self.pgv_labels["z"] = QLabel("0.00")
        linear_layout.addWidget(self.pgv_labels["z"], 3, 3)

        self.pga_labels["z"] = QLabel("0.00")
        linear_layout.addWidget(self.pga_labels["z"], 3, 4)

        linear_box.setLayout(linear_layout)
        traj_layout.addWidget(linear_box)

        # Rotational DOFs
        rot_box = QGroupBox("Rotational Motion (Roll, Pitch, Yaw)")
        rot_layout = QGridLayout()

        # Header row
        rot_layout.addWidget(QLabel("Axis"), 0, 0)
        rot_layout.addWidget(QLabel("Amp (rad)"), 0, 1)
        rot_layout.addWidget(QLabel("Freq (Hz)"), 0, 2)
        rot_layout.addWidget(QLabel("PGV (rad/s)"), 0, 3)
        rot_layout.addWidget(QLabel("PGA (rad/s²)"), 0, 4)

        # Roll axis
        rot_layout.addWidget(QLabel("Roll:"), 1, 0)
        self.param_inputs["amp_roll"] = QLineEdit(str(self.trajectory_params["amp_roll"]))
        self.param_inputs["amp_roll"].textChanged.connect(lambda: self.update_pgv_pga('roll'))
        rot_layout.addWidget(self.param_inputs["amp_roll"], 1, 1)

        self.param_inputs["freq_roll"] = QLineEdit(str(self.trajectory_params["freq_roll"]))
        self.param_inputs["freq_roll"].textChanged.connect(lambda: self.update_pgv_pga('roll'))
        rot_layout.addWidget(self.param_inputs["freq_roll"], 1, 2)

        self.pgv_labels["roll"] = QLabel("0.00")
        rot_layout.addWidget(self.pgv_labels["roll"], 1, 3)

        self.pga_labels["roll"] = QLabel("0.00")
        rot_layout.addWidget(self.pga_labels["roll"], 1, 4)

        # Pitch axis
        rot_layout.addWidget(QLabel("Pitch:"), 2, 0)
        self.param_inputs["amp_pitch"] = QLineEdit(str(self.trajectory_params["amp_pitch"]))
        self.param_inputs["amp_pitch"].textChanged.connect(lambda: self.update_pgv_pga('pitch'))
        rot_layout.addWidget(self.param_inputs["amp_pitch"], 2, 1)

        self.param_inputs["freq_pitch"] = QLineEdit(str(self.trajectory_params["freq_pitch"]))
        self.param_inputs["freq_pitch"].textChanged.connect(lambda: self.update_pgv_pga('pitch'))
        rot_layout.addWidget(self.param_inputs["freq_pitch"], 2, 2)

        self.pgv_labels["pitch"] = QLabel("0.00")
        rot_layout.addWidget(self.pgv_labels["pitch"], 2, 3)

        self.pga_labels["pitch"] = QLabel("0.00")
        rot_layout.addWidget(self.pga_labels["pitch"], 2, 4)

        # Yaw axis
        rot_layout.addWidget(QLabel("Yaw:"), 3, 0)
        self.param_inputs["amp_yaw"] = QLineEdit(str(self.trajectory_params["amp_yaw"]))
        self.param_inputs["amp_yaw"].textChanged.connect(lambda: self.update_pgv_pga('yaw'))
        rot_layout.addWidget(self.param_inputs["amp_yaw"], 3, 1)

        self.param_inputs["freq_yaw"] = QLineEdit(str(self.trajectory_params["freq_yaw"]))
        self.param_inputs["freq_yaw"].textChanged.connect(lambda: self.update_pgv_pga('yaw'))
        rot_layout.addWidget(self.param_inputs["freq_yaw"], 3, 2)

        self.pgv_labels["yaw"] = QLabel("0.00")
        rot_layout.addWidget(self.pgv_labels["yaw"], 3, 3)

        self.pga_labels["yaw"] = QLabel("0.00")
        rot_layout.addWidget(self.pga_labels["yaw"], 3, 4)

        rot_box.setLayout(rot_layout)
        traj_layout.addWidget(rot_box)

        self.trajectory_button = QPushButton("Execute Trajectory")
        self.trajectory_button.clicked.connect(self.toggle_trajectory)
        traj_layout.addWidget(self.trajectory_button)

        traj_group.setLayout(traj_layout)
        main_layout.addWidget(traj_group)

        # --------------- Object Upload ---------------
        obj_group = QGroupBox("Object Upload")
        obj_layout = QVBoxLayout()

        self.upload_obj_button = QPushButton("Upload PBR OBJ")
        self.upload_obj_button.clicked.connect(self.upload_obj)
        obj_layout.addWidget(self.upload_obj_button)

        obj_group.setLayout(obj_layout)
        main_layout.addWidget(obj_group)
        
        # Initialize PGV/PGA calculations
        self.initialize_pgv_pga()
        
        self.setLayout(main_layout)

    ###########################################################################
    # Position Monitor
    ###########################################################################
    def initialize_pgv_pga(self):
        """Initialize PGV and PGA calculations for all axes."""
        axes = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for axis in axes:
            self.update_pgv_pga(axis)

    def update_pgv_pga(self, axis):
        """
        Update PGV and PGA values for a given axis based on amplitude and frequency.
        PGV = 2π × f × Amp
        PGA = (2π × f)² × Amp
        """
        try:
            # Get amplitude and frequency values
            amp_text = self.param_inputs[f"amp_{axis}"].text()
            freq_text = self.param_inputs[f"freq_{axis}"].text()
            
            amplitude = float(amp_text) if amp_text else 0.0
            frequency = float(freq_text) if freq_text else 0.0
            
            # Calculate PGV and PGA
            omega = 2 * math.pi * frequency  # Angular frequency
            pgv = omega * amplitude          # Peak Ground Velocity
            pga = omega * omega * amplitude  # Peak Ground Acceleration
            
            # Update the display labels
            self.pgv_labels[axis].setText(f"{pgv:.3f}")
            self.pga_labels[axis].setText(f"{pga:.3f}")
            
        except (ValueError, KeyError):
            # Handle invalid input or missing widgets
            if axis in self.pgv_labels:
                self.pgv_labels[axis].setText("0.000")
            if axis in self.pga_labels:
                self.pga_labels[axis].setText("0.000")

    def update_gui_display(self):
        """Update the GUI display with current pedestal position and orientation."""
        if not self.simulation_started or not self.simulation_core or self.simulation_core.robot_id is None:
            return
            
        try:
            link_state = p.getLinkState(self.simulation_core.robot_id, 3, physicsClientId=self.simulation_core.client_id)
            if link_state:
                pos = link_state[0]
                orn_quat = link_state[1]  # Quaternion orientation
                
                # Convert quaternion to Euler angles (roll, pitch, yaw) in radians
                orn_euler = p.getEulerFromQuaternion(orn_quat)
                
                # Store current position for position control when trajectory is not running
                self.last_pedestal_position = [pos[0], pos[1], pos[2], orn_euler[0], orn_euler[1], orn_euler[2]]
                
                # Convert to degrees
                roll_deg = math.degrees(orn_euler[0])
                pitch_deg = math.degrees(orn_euler[1])
                yaw_deg = math.degrees(orn_euler[2])
                
                self.position_label.setText(f"Pedestal Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
                self.orientation_label.setText(f"Pedestal Orientation: Roll={roll_deg:.2f}°, Pitch={pitch_deg:.2f}°, Yaw={yaw_deg:.2f}°")
        except Exception as e:
            self.logger.error(f"update_gui_display error: {e}")

    ###########################################################################
    # Data Recording
    ###########################################################################
    def start_recording(self):
        """Start recording trajectory data."""
        self.recording_active = True
        self.recorded_times = []
        self.recorded_pedestal_poses = []
        self.recorded_pbr_poses = {}
        
        # Initialize arrays for each tracked PBR object
        for obj_id in self.pbr_object_ids:
            self.recorded_pbr_poses[obj_id] = []
        
        self.recording_start_time = None
        self.logger.info("Data recording started.")

    def stop_recording(self):
        """Stop recording and save data to file."""
        if not self.recording_active:
            return
            
        self.recording_active = False
        self.logger.info("Data recording stopped.")
        
        # Save recorded data
        if len(self.recorded_times) > 0:
            self.save_recording_data()
        else:
            self.logger.warning("No data recorded during trajectory run.")

    def record_current_poses(self, simulation_time):
        """
        Record current poses of pedestal and all PBR objects.
        
        :param simulation_time: Current simulation time in seconds
        """
        if not self.recording_active or not self.simulation_core:
            return
        
        try:
            # Record timestamp
            self.recorded_times.append(simulation_time)
            
            # Record pedestal pose (link 3)
            link_state = p.getLinkState(
                self.simulation_core.robot_id, 3,
                physicsClientId=self.simulation_core.client_id
            )
            pos = link_state[0]
            orn_quat = link_state[1]
            orn_euler = p.getEulerFromQuaternion(orn_quat)
            
            # Store as [x, y, z, roll, pitch, yaw]
            pedestal_pose = [pos[0], pos[1], pos[2], orn_euler[0], orn_euler[1], orn_euler[2]]
            self.recorded_pedestal_poses.append(pedestal_pose)
            
            # Record PBR object poses
            for obj_id in self.pbr_object_ids:
                try:
                    obj_pos, obj_orn = p.getBasePositionAndOrientation(
                        obj_id,
                        physicsClientId=self.simulation_core.client_id
                    )
                    obj_euler = p.getEulerFromQuaternion(obj_orn)
                    obj_pose = [obj_pos[0], obj_pos[1], obj_pos[2], 
                               obj_euler[0], obj_euler[1], obj_euler[2]]
                    
                    if obj_id not in self.recorded_pbr_poses:
                        self.recorded_pbr_poses[obj_id] = []
                    self.recorded_pbr_poses[obj_id].append(obj_pose)
                except Exception as e:
                    self.logger.error(f"Error recording pose for object {obj_id}: {e}")
                    
        except Exception as e:
            self.logger.error(f"record_current_poses error: {e}")

    def save_recording_data(self):
        """Save recorded data to NPZ file with timestamp."""
        try:
            # Create data directory if it doesn't exist
            data_dir = "data"
            if not os.path.exists(data_dir):
                os.makedirs(data_dir)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(data_dir, f"run_{timestamp}.npz")
            
            # Convert lists to numpy arrays
            times_array = np.array(self.recorded_times)
            pedestal_poses_array = np.array(self.recorded_pedestal_poses)
            
            # Prepare data dictionary
            data_dict = {
                'times': times_array,
                'pedestal_poses': pedestal_poses_array,
                'pedestal_pose_description': 'Each row: [x, y, z, roll, pitch, yaw]'
            }
            
            # Add PBR object data
            for obj_id, poses_list in self.recorded_pbr_poses.items():
                if len(poses_list) > 0:
                    poses_array = np.array(poses_list)
                    data_dict[f'pbr_object_{obj_id}_poses'] = poses_array
            
            # Add metadata
            data_dict['num_samples'] = len(times_array)
            data_dict['num_pbr_objects'] = len(self.recorded_pbr_poses)
            data_dict['pbr_object_ids'] = np.array(list(self.pbr_object_ids))
            
            # Save to file
            np.savez(filename, **data_dict)
            
            self.logger.info(f"Recorded data saved to {filename}")
            self.logger.info(f"Total samples: {len(times_array)}, Duration: {times_array[-1]:.2f}s")
            self.logger.info(f"PBR objects tracked: {len(self.recorded_pbr_poses)}")
            
        except Exception as e:
            self.logger.error(f"Error saving recording data: {e}")

    def register_pbr_object(self, object_id):
        """
        Register a PBR object for tracking during recording.
        
        :param object_id: PyBullet body ID of the object
        """
        if object_id is not None:
            self.pbr_object_ids.add(object_id)
            self.logger.info(f"Registered PBR object {object_id} for tracking.")

    def run_simulation_loop(self):
        """
        Main simulation loop that runs continuously in a separate thread.
        Steps the simulation and either executes trajectory or maintains position.
        """
        if not self.simulation_core or self.simulation_core.robot_id is None:
            self.logger.error("run_simulation_loop: Simulation core not initialized.")
            return
        
        self.logger.info("run_simulation_loop: Simulation loop started.")
        
        # Read initial pedestal position to avoid forcing it to [0,0,0]
        try:
            link_state = p.getLinkState(self.simulation_core.robot_id, 3, physicsClientId=self.simulation_core.client_id)
            pos = link_state[0]
            orn_quat = link_state[1]
            orn_euler = p.getEulerFromQuaternion(orn_quat)
            self.last_pedestal_position = [pos[0], pos[1], pos[2], orn_euler[0], orn_euler[1], orn_euler[2]]
            self.logger.info(f"run_simulation_loop: Initial pedestal position: {self.last_pedestal_position}")
        except Exception as e:
            self.logger.error(f"run_simulation_loop: Error reading initial position: {e}")
        
        # Get real_time flag from config
        use_real_time = self.config["simulation_settings"].get("use_real_time", False)
        
        # Initialize timing
        start_time = time.time()
        step_count = 0
        trajectory_start_time = None
        trajectory_start_step = 0
        
        while not self.stop_simulation_flag:
            # Skip simulation stepping if paused
            if self.paused:
                time.sleep(0.01)  # Small sleep to prevent busy waiting
                continue
            
            # Calculate current time based on mode:
            # - Real-time mode: use wall-clock time
            # - Non-real-time mode: use simulation time (step_count * dt)
            if use_real_time:
                current_time = time.time() - start_time
            else:
                current_time = step_count * self.simulation_dt
            
            if self.trajectory_running:
                # Calculate trajectory duration if just started
                if trajectory_start_time is None:
                    trajectory_start_time = current_time
                    trajectory_start_step = step_count
                    
                    # Calculate duration based on cycle number and smallest non-zero frequency
                    cycle_number = float(self.trajectory_params.get("cycle_number", 1))
                    freqs = [
                        self.trajectory_params.get("freq_x", 0.0),
                        self.trajectory_params.get("freq_y", 0.0),
                        self.trajectory_params.get("freq_z", 0.0),
                        self.trajectory_params.get("freq_roll", 0.0),
                        self.trajectory_params.get("freq_pitch", 0.0),
                        self.trajectory_params.get("freq_yaw", 0.0),
                    ]
                    nonzero_freqs = [f for f in freqs if f > 0.0]
                    if nonzero_freqs:
                        min_freq = min(nonzero_freqs)
                        self.trajectory_duration = cycle_number / min_freq
                    else:
                        self.logger.warning("run_simulation_loop: All frequencies are zero, using default duration 20 seconds.")
                        self.trajectory_duration = 20.0
                    
                    total_steps = int(self.trajectory_duration * self.simulation_frequency)
                    self.logger.info(f"run_simulation_loop: Starting trajectory: {cycle_number} cycles, {self.trajectory_duration:.2f}s, {total_steps} steps")
                    
                    # Start data recording
                    self.start_recording()
                
                # Calculate time since trajectory started (in simulation time)
                t = current_time - trajectory_start_time
                
                # Check if trajectory duration is exceeded
                if t >= self.trajectory_duration:
                    elapsed_steps = step_count - trajectory_start_step
                    self.logger.info(f"run_simulation_loop: Trajectory completed in {t:.2f}s ({elapsed_steps} steps)")
                    
                    # Get final position
                    try:
                        link_state = p.getLinkState(self.simulation_core.robot_id, 3, physicsClientId=self.simulation_core.client_id)
                        pos = link_state[0]
                        self.logger.info(f"run_simulation_loop: Final position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
                    except Exception as e:
                        self.logger.error(f"run_simulation_loop: Error getting final position: {e}")
                    
                    # Stop trajectory and recording
                    self.trajectory_running = False
                    trajectory_start_time = None
                    self.trajectory_button.setText("Execute Trajectory")
                    
                    # Stop data recording and save
                    self.stop_recording()
                    continue
                
                # Execute trajectory step
                try:
                    self.simulation_core.step_trajectory(
                        self.trajectory_params,
                        t=t
                    )
                except Exception as e:
                    self.logger.error(f"run_simulation_loop: trajectory step error at t={t:.3f}: {e}")
                
                # Record current poses during trajectory execution
                self.record_current_poses(t)
            else:
                # No trajectory running - hold current position with zero velocity control
                try:
                    self.simulation_core.hold_pedestal_position()
                except Exception as e:
                    self.logger.error(f"run_simulation_loop: hold position error: {e}")
                
                # Reset trajectory timing
                trajectory_start_time = None
            
            # Step the physics simulation
            try:
                p.stepSimulation(physicsClientId=self.simulation_core.client_id)
                step_count += 1
            except Exception as e:
                self.logger.error(f"run_simulation_loop: stepSimulation error at step {step_count}: {e}")
            
            # Real-time synchronization if enabled
            if use_real_time:
                elapsed_time = time.time() - start_time
                target_time = (step_count) * self.simulation_dt
                sleep_time = target_time - elapsed_time
                if sleep_time > 0:
                    time.sleep(sleep_time)
        
        self.logger.info("run_simulation_loop: Simulation loop stopped.")

    ###########################################################################
    # Simulation
    ###########################################################################
    def start_simulation(self):
        """Start the simulation."""
        if self.simulation_core is None:
            self.simulation_core = SimulationCore("config.yaml")
            self.simulation_core.create_robot()
            # Update button states after creating simulation core
            self.update_button_states()

        if self.simulation_core.robot_id is None:
            self.logger.error("Failed to create robot.")
            return

        # Set the simulation_started flag
        self.simulation_started = True
        self.running = True
        self.paused = False
        self.stop_simulation_flag = False
        self.update_button_states()

        # Start GUI update timer (for display updates)
        interval_ms = int(1000 / self.GUI_update_hz)
        self.gui_timer.start(interval_ms)
        
        # Start the simulation loop in a separate thread
        self.simulation_thread = threading.Thread(target=self.run_simulation_loop, daemon=True)
        self.simulation_thread.start()
        
        self.logger.info("Simulation started successfully.")

    def stop_simulation(self):
        """Stop the simulation completely."""
        # Stop trajectory if running
        if self.trajectory_running:
            self.trajectory_running = False
            self.trajectory_button.setText("Execute Trajectory")
        
        # Signal simulation thread to stop
        self.stop_simulation_flag = True
        
        # Wait for simulation thread to finish
        if self.simulation_thread and self.simulation_thread.is_alive():
            self.logger.info("Waiting for simulation thread to stop...")
            self.simulation_thread.join(timeout=2.0)
            if self.simulation_thread.is_alive():
                self.logger.warning("Simulation thread did not stop gracefully.")
        
        # Turn off the simulation_started flag
        self.simulation_started = False
        self.running = False
        self.paused = False
        if self.gui_timer.isActive():
            self.gui_timer.stop()
            
        self.update_button_states()
        
        self.logger.info("Simulation stopped.")

    def toggle_pause_simulation(self):
        """Toggle pause/resume for the simulation."""
        if not self.running:
            return  # Can't pause if not running
            
        if self.paused:
            # Resume simulation
            self.paused = False
            if not self.gui_timer.isActive():
                interval_ms = int(1000 / self.GUI_update_hz)
                self.gui_timer.start(interval_ms)
        else:
            # Pause simulation
            self.paused = True
            if self.gui_timer.isActive():
                self.gui_timer.stop()
                
        self.update_button_states()

    def update_button_states(self):
        """Update button enabled/disabled states and text based on current simulation state."""
        if self.simulation_core is None or not self.simulation_started:
            # No simulation created yet or not started
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.pause_button.setEnabled(False)
            self.reset_button.setEnabled(self.simulation_core is not None)
            self.pause_button.setText("Pause Simulation")
        elif self.simulation_started and self.running and not self.paused:
            # Simulation is running
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.pause_button.setEnabled(True)
            self.reset_button.setEnabled(False)
            self.pause_button.setText("Pause Simulation")
        elif self.simulation_started and self.running and self.paused:
            # Simulation is paused
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.pause_button.setEnabled(True)
            self.reset_button.setEnabled(False)
            self.pause_button.setText("Resume Simulation")
        elif self.simulation_started and not self.running:
            # Simulation was started but now stopped
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.pause_button.setEnabled(False)
            self.reset_button.setEnabled(True)
            self.pause_button.setText("Pause Simulation")

    def reset_simulation(self):
        """Resets PyBullet simulation and re-applies all settings."""
        # Check if simulation is currently running or paused
        if self.running or self.simulation_started:
            # Show error dialog prompting user to stop simulation first
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle("Cannot Reset Simulation")
            msg.setText("Simulation is currently running or has been started.")
            msg.setInformativeText("Please stop the simulation before resetting.")
            msg.setDetailedText(
                "To reset the simulation:\n"
                "1. Click 'Stop Simulation' button\n"
                "2. Then click 'Reset Simulation' button\n\n"
                "This ensures a clean and safe reset of the physics simulation."
            )
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            return
        
        # Check if trajectory is running
        if getattr(self, 'trajectory_running', False):
            # Show error dialog prompting user to stop trajectory first
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle("Cannot Reset Simulation")
            msg.setText("Trajectory execution is currently running.")
            msg.setInformativeText("Please stop the trajectory before resetting.")
            msg.setDetailedText(
                "To reset the simulation:\n"
                "1. Click 'Stop Trajectory' button\n"
                "2. Click 'Stop Simulation' button (if running)\n"
                "3. Then click 'Reset Simulation' button\n\n"
                "This ensures a clean and safe reset of the physics simulation."
            )
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            return

        # Proceed with reset only if simulation is stopped
        if self.simulation_core:
            # Reset everything on the correct client
            p.resetSimulation(physicsClientId=self.simulation_core.client_id)

            # Re-apply physics settings
            p.setTimeStep(self.simulation_core.time_step, self.simulation_core.client_id)
            p.setGravity(*self.simulation_core.gravity, physicsClientId=self.simulation_core.client_id)
            #p.setRealTimeSimulation(1 if self.simulation_core.use_real_time else 0, physicsClientId=self.simulation_core.client_id)

            # Re-create your robot (and environment if needed)
            self.simulation_core.create_robot()
            
            # Reset the simulation_started flag since we're creating a fresh simulation
            self.simulation_started = False
            
            # Update button states after reset
            self.update_button_states()
            
            self.logger.info("Simulation reset completed successfully.")


    ###########################################################################
    # Trajectory
    ###########################################################################
    def toggle_trajectory(self):
        if not self.trajectory_running:
            self.start_trajectory()
        else:
            self.stop_trajectory()

    def start_trajectory(self):
        # Check if simulation has been started using the flag
        if not self.simulation_started:
            QMessageBox.warning(
                self,
                "Simulation Not Started",
                "Please start the simulation first before executing a trajectory.\n\n"
                "Click the 'Start Simulation' button to initialize the simulation."
            )
            self.logger.error("Simulation not started.")
            return
        
        # Additional check for simulation core and robot
        if not self.simulation_core or not self.simulation_core.robot_id:
            QMessageBox.warning(
                self,
                "Simulation Error",
                "Simulation core is not properly initialized.\n\n"
                "Please restart the application and try again."
            )
            self.logger.error("Simulation core not properly initialized.")
            return

        # Check if already running
        if self.trajectory_running:
            self.logger.warning("Trajectory is already running.")
            return

        self.update_trajectory_params()
        self.trajectory_running = True
        self.trajectory_button.setText("Stop Trajectory")
        self.logger.info("Trajectory execution started.")

    def stop_trajectory(self):
        """Stop the trajectory execution."""
        self.trajectory_running = False
        self.trajectory_button.setText("Execute Trajectory")
        self.logger.info("Trajectory stopped.")

    def update_trajectory_params(self):
        """Update trajectory parameters from GUI inputs."""
        for key in self.trajectory_params:
            if key in self.param_inputs:
                try:
                    self.trajectory_params[key] = float(self.param_inputs[key].text())
                except ValueError:
                    self.logger.warning(f"Invalid input for '{key}' -> using default {self.trajectory_params[key]}")
        
        # Also update PGV/PGA calculations when parameters change
        axes = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for axis in axes:
            self.update_pgv_pga(axis)

    ###########################################################################
    # Upload OBJ with Live Preview
    ###########################################################################
    def upload_obj(self):
        if not self.simulation_started or not self.simulation_core or self.simulation_core.robot_id is None:
            QMessageBox.warning(
                self,
                "Simulation Not Started",
                "Please start the simulation first before uploading objects.\n\n"
                "Click the 'Start Simulation' button to initialize the simulation."
            )
            self.logger.error("Simulation must be started to upload an OBJ.")
            return

        # Check if simulation is paused
        if self.running and not self.paused:
            reply = QMessageBox.question(
                self,
                "Pause Simulation?",
                "It is recommended to pause the simulation before uploading objects for better stability and control.\n\n"
                "Would you like to pause the simulation now?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes
            )
            
            if reply == QMessageBox.Yes:
                # Pause the simulation
                self.toggle_pause_simulation()
                self.logger.info("Simulation paused before uploading OBJ.")
            else:
                self.logger.info("User chose to upload OBJ without pausing simulation.")

        # Open the PBR dialog directly without pre-selecting a file
        dialog = PBRLoadDialog(self.simulation_core, obj_path=None, parent=self, pedestal_half_z=self.pedestal_half_z)
        result = dialog.exec_()

        if result == QDialog.Accepted:
            self.logger.info("User confirmed final placement; the object remains in the scene.")
        else:
            self.logger.info("User canceled, so the object was removed.")


###############################################################################
# Main Entry
###############################################################################
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = PyBulletGUI()
    gui.show()
    sys.exit(app.exec_())
