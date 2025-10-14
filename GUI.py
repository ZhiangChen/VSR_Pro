# GUI_test.py
"""
Refactored VSR Pro GUI using app.py API layer.
This version delegates all business logic to VSRProApp,
keeping the GUI focused on presentation and user interaction.
"""

import sys
import math
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QCheckBox, QFileDialog, QDialog, QGroupBox,
    QPushButton, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer
import pybullet as p

from app import VSRProApp

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
    def __init__(self, app, obj_path=None, parent=None, pedestal_half_z=0.0):
        super().__init__(parent)
        self.obj_path = obj_path  # Can be None initially
        self.app = app  # VSRProApp instance
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
        pose_layout.addWidget(QLabel("Roll(°):"), 1, 0)
        self.roll_input = QLineEdit(str(math.degrees(self.default_roll)))
        self.roll_input.setFixedWidth(80)
        pose_layout.addWidget(self.roll_input, 1, 1)

        pose_layout.addWidget(QLabel("Pitch(°):"), 1, 2)
        self.pitch_input = QLineEdit(str(math.degrees(self.default_pitch)))
        self.pitch_input.setFixedWidth(80)
        pose_layout.addWidget(self.pitch_input, 1, 3)

        pose_layout.addWidget(QLabel("Yaw(°):"), 1, 4)
        self.yaw_input = QLineEdit(str(math.degrees(self.default_yaw)))
        self.yaw_input.setFixedWidth(80)
        pose_layout.addWidget(self.yaw_input, 1, 5)

        pose_group.setLayout(pose_layout)
        main_layout.addWidget(pose_group)

        # ---------------------------------------------------------------------
        # 2) Mesh Scale
        # ---------------------------------------------------------------------
        scale_group = QGroupBox("Mesh Scale")
        scale_layout = QHBoxLayout()
        
        scale_layout.addWidget(QLabel("SX:"))
        self.sx_input = QLineEdit("1.0")
        self.sx_input.setFixedWidth(60)
        scale_layout.addWidget(self.sx_input)
        
        scale_layout.addWidget(QLabel("SY:"))
        self.sy_input = QLineEdit("1.0")
        self.sy_input.setFixedWidth(60)
        scale_layout.addWidget(self.sy_input)
        
        scale_layout.addWidget(QLabel("SZ:"))
        self.sz_input = QLineEdit("1.0")
        self.sz_input.setFixedWidth(60)
        scale_layout.addWidget(self.sz_input)
        
        scale_layout.addStretch()
        scale_group.setLayout(scale_layout)
        main_layout.addWidget(scale_group)

        # ---------------------------------------------------------------------
        # 3) PBR Properties
        # ---------------------------------------------------------------------
        pbr_group = QGroupBox("PBR & Collision Properties")
        pbr_layout = QGridLayout()

        # Mass
        pbr_layout.addWidget(QLabel("Mass (kg):"), 0, 0)
        self.mass_input = QLineEdit("10.0")
        pbr_layout.addWidget(self.mass_input, 0, 1)

        # Inertia - Principal Axes
        pbr_layout.addWidget(QLabel("Inertia Ixx (kg·m²):"), 1, 0)
        self.inertia_x_input = QLineEdit("0.1")
        pbr_layout.addWidget(self.inertia_x_input, 1, 1)

        pbr_layout.addWidget(QLabel("Inertia Iyy (kg·m²):"), 2, 0)
        self.inertia_y_input = QLineEdit("0.1")
        pbr_layout.addWidget(self.inertia_y_input, 2, 1)

        pbr_layout.addWidget(QLabel("Inertia Izz (kg·m²):"), 3, 0)
        self.inertia_z_input = QLineEdit("0.1")
        pbr_layout.addWidget(self.inertia_z_input, 3, 1)

        # Restitution
        pbr_layout.addWidget(QLabel("Restitution:"), 4, 0)
        self.restitution_input = QLineEdit("0.2")
        pbr_layout.addWidget(self.restitution_input, 4, 1)

        # Lateral Friction
        pbr_layout.addWidget(QLabel("Lateral Friction:"), 5, 0)
        self.lateral_friction_input = QLineEdit("0.5")
        pbr_layout.addWidget(self.lateral_friction_input, 5, 1)

        # Spinning Friction
        pbr_layout.addWidget(QLabel("Spinning Friction:"), 6, 0)
        self.spinning_friction_input = QLineEdit("0.1")
        pbr_layout.addWidget(self.spinning_friction_input, 6, 1)

        # Contact Damping
        pbr_layout.addWidget(QLabel("Contact Damping:"), 7, 0)
        self.contact_damping_input = QLineEdit("10000.0")
        pbr_layout.addWidget(self.contact_damping_input, 7, 1)

        # Contact Stiffness
        pbr_layout.addWidget(QLabel("Contact Stiffness:"), 8, 0)
        self.contact_stiffness_input = QLineEdit("1000000.0")
        pbr_layout.addWidget(self.contact_stiffness_input, 8, 1)

        pbr_group.setLayout(pbr_layout)
        main_layout.addWidget(pbr_group)

        # ---------------------------------------------------------------------
        # 4) Action Buttons
        # ---------------------------------------------------------------------
        button_layout = QHBoxLayout()

        self.apply_button = QPushButton("Apply")
        self.apply_button.clicked.connect(self.apply_spawn)
        button_layout.addWidget(self.apply_button)

        self.confirm_button = QPushButton("Confirm")
        self.confirm_button.clicked.connect(self.confirm_spawn)
        button_layout.addWidget(self.confirm_button)

        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.clicked.connect(self.cancel_spawn)
        button_layout.addWidget(self.cancel_button)

        main_layout.addLayout(button_layout)
        self.setLayout(main_layout)

    def browse_obj_file(self):
        """Open file dialog to select OBJ file."""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Select OBJ File", "", "OBJ Files (*.obj);;All Files (*)"
        )
        if file_path:
            self.obj_path = file_path
            self.obj_path_label.setText(file_path)

    def apply_spawn(self):
        """Spawn or update the preview object."""
        if not self.obj_path:
            QMessageBox.warning(self, "No File", "Please select an OBJ file first.")
            return

        # Remove previous preview if exists
        if self.preview_obj_id is not None:
            try:
                p.removeBody(self.preview_obj_id)
            except:
                pass
            self.preview_obj_id = None

        # Get parameters
        try:
            offset = [
                float(self.x_input.text()),
                float(self.y_input.text()),
                float(self.z_input.text())
            ]
            orientation_euler = [
                math.radians(float(self.roll_input.text())),
                math.radians(float(self.pitch_input.text())),
                math.radians(float(self.yaw_input.text()))
            ]
            mesh_scale = [
                float(self.sx_input.text()),
                float(self.sy_input.text()),
                float(self.sz_input.text())
            ]
            mass = float(self.mass_input.text())
            inertia = [
                float(self.inertia_x_input.text()),
                float(self.inertia_y_input.text()),
                float(self.inertia_z_input.text())
            ]
            restitution = float(self.restitution_input.text())
            lateral_friction = float(self.lateral_friction_input.text())
            spinning_friction = float(self.spinning_friction_input.text())
            contact_damping = float(self.contact_damping_input.text())
            contact_stiffness = float(self.contact_stiffness_input.text())
        except ValueError as e:
            QMessageBox.warning(self, "Invalid Input", f"Error parsing parameters: {e}")
            return

        # Use app.py API to load the object
        self.preview_obj_id = self.app.load_pbr_object(
            obj_path=self.obj_path,
            offset=offset,
            orientation_euler=orientation_euler,
            mesh_scale=mesh_scale,
            mass=mass,
            inertia=inertia,
            restitution=restitution,
            lateral_friction=lateral_friction,
            spinning_friction=spinning_friction,
            contact_damping=contact_damping,
            contact_stiffness=contact_stiffness
        )

        if self.preview_obj_id is None:
            QMessageBox.critical(self, "Error", "Failed to spawn object. Check console/log.")

    def confirm_spawn(self):
        """Finalize the spawn and close dialog."""
        if self.preview_obj_id is None:
            QMessageBox.information(self, "No Object", "No object to confirm. Apply first.")
            return
        # Object is already registered in app.py, just close
        self.accept()

    def cancel_spawn(self):
        """Cancel and remove preview object."""
        if self.preview_obj_id is not None:
            try:
                p.removeBody(self.preview_obj_id)
                # Also remove from tracking
                self.app.pbr_object_ids.discard(self.preview_obj_id)
            except:
                pass
            self.preview_obj_id = None
        self.reject()


###############################################################################
# Main GUI using VSRProApp
###############################################################################
class PyBulletGUI(QWidget):
    """
    Refactored GUI that uses VSRProApp for all business logic.
    This class focuses on UI presentation and delegates simulation control to app.py.
    """
    def __init__(self):
        super().__init__()
        
        # Use VSRProApp instead of SimulationCore directly
        self.app = VSRProApp("config.yaml")
        
        # UI state
        self.param_inputs = {}
        self.pgv_labels = {}  # PGV display labels
        self.pga_labels = {}  # PGA display labels
        
        # GUI update timer
        self.gui_timer = QTimer(self)
        self.gui_timer.setTimerType(Qt.PreciseTimer)
        self.gui_timer.timeout.connect(self.update_gui_display)
        
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("VSR Pro (API Version)")
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
        self.stop_button.setEnabled(False)
        button_layout.addWidget(self.stop_button)
        
        self.pause_button = QPushButton("Pause Simulation")
        self.pause_button.clicked.connect(self.toggle_pause_simulation)
        self.pause_button.setEnabled(False)
        button_layout.addWidget(self.pause_button)

        self.reset_button = QPushButton("Reset Simulation")
        self.reset_button.clicked.connect(self.reset_simulation)
        self.reset_button.setEnabled(False)
        button_layout.addWidget(self.reset_button)
        
        sim_layout.addLayout(button_layout)

        self.position_label = QLabel("Pedestal Position: (0.0, 0.0, 0.0)")
        sim_layout.addWidget(self.position_label)
        
        self.orientation_label = QLabel("Pedestal Orientation: Roll=0.0°, Pitch=0.0°, Yaw=0.0°")
        sim_layout.addWidget(self.orientation_label)

        sim_group.setLayout(sim_layout)
        main_layout.addWidget(sim_group)

        # --------------- Object Upload ---------------
        obj_group = QGroupBox("Object Upload")
        obj_layout = QVBoxLayout()

        self.upload_obj_button = QPushButton("Upload PBR OBJ")
        self.upload_obj_button.clicked.connect(self.upload_obj)
        obj_layout.addWidget(self.upload_obj_button)

        obj_group.setLayout(obj_layout)
        main_layout.addWidget(obj_group)

        # ---------------- Trajectory Settings ----------------
        traj_group = QGroupBox("Pulse Trajectory Settings")
        traj_layout = QVBoxLayout()

        # Cycle Number input
        cycle_layout = QHBoxLayout()
        cycle_layout.addWidget(QLabel("Cycle Number:"))
        self.param_inputs["cycle_number"] = QLineEdit(str(self.app.trajectory_params["cycle_number"]))
        cycle_layout.addWidget(self.param_inputs["cycle_number"])
        cycle_layout.addStretch()
        traj_layout.addLayout(cycle_layout)

        # Linear DOFs
        linear_box = QGroupBox("Linear Motion")
        linear_layout = QGridLayout()

        # Header row
        linear_layout.addWidget(QLabel("Axis"), 0, 0)
        linear_layout.addWidget(QLabel("Amp (m)"), 0, 1)
        linear_layout.addWidget(QLabel("Freq (Hz)"), 0, 2)
        linear_layout.addWidget(QLabel("PGV (m/s)"), 0, 3)
        linear_layout.addWidget(QLabel("PGA (m/s²)"), 0, 4)

        # X axis
        linear_layout.addWidget(QLabel("X:"), 1, 0)
        self.param_inputs["amp_x"] = QLineEdit(str(self.app.trajectory_params["amp_x"]))
        self.param_inputs["amp_x"].textChanged.connect(lambda: self.update_pgv_pga('x'))
        linear_layout.addWidget(self.param_inputs["amp_x"], 1, 1)

        self.param_inputs["freq_x"] = QLineEdit(str(self.app.trajectory_params["freq_x"]))
        self.param_inputs["freq_x"].textChanged.connect(lambda: self.update_pgv_pga('x'))
        linear_layout.addWidget(self.param_inputs["freq_x"], 1, 2)

        self.pgv_labels["x"] = QLabel("0.00")
        linear_layout.addWidget(self.pgv_labels["x"], 1, 3)

        self.pga_labels["x"] = QLabel("0.00")
        linear_layout.addWidget(self.pga_labels["x"], 1, 4)

        # Y axis
        linear_layout.addWidget(QLabel("Y:"), 2, 0)
        self.param_inputs["amp_y"] = QLineEdit(str(self.app.trajectory_params["amp_y"]))
        self.param_inputs["amp_y"].textChanged.connect(lambda: self.update_pgv_pga('y'))
        linear_layout.addWidget(self.param_inputs["amp_y"], 2, 1)

        self.param_inputs["freq_y"] = QLineEdit(str(self.app.trajectory_params["freq_y"]))
        self.param_inputs["freq_y"].textChanged.connect(lambda: self.update_pgv_pga('y'))
        linear_layout.addWidget(self.param_inputs["freq_y"], 2, 2)

        self.pgv_labels["y"] = QLabel("0.00")
        linear_layout.addWidget(self.pgv_labels["y"], 2, 3)

        self.pga_labels["y"] = QLabel("0.00")
        linear_layout.addWidget(self.pga_labels["y"], 2, 4)

        # Z axis
        linear_layout.addWidget(QLabel("Z:"), 3, 0)
        self.param_inputs["amp_z"] = QLineEdit(str(self.app.trajectory_params["amp_z"]))
        self.param_inputs["amp_z"].textChanged.connect(lambda: self.update_pgv_pga('z'))
        linear_layout.addWidget(self.param_inputs["amp_z"], 3, 1)

        self.param_inputs["freq_z"] = QLineEdit(str(self.app.trajectory_params["freq_z"]))
        self.param_inputs["freq_z"].textChanged.connect(lambda: self.update_pgv_pga('z'))
        linear_layout.addWidget(self.param_inputs["freq_z"], 3, 2)

        self.pgv_labels["z"] = QLabel("0.00")
        linear_layout.addWidget(self.pgv_labels["z"], 3, 3)

        self.pga_labels["z"] = QLabel("0.00")
        linear_layout.addWidget(self.pga_labels["z"], 3, 4)

        linear_box.setLayout(linear_layout)
        traj_layout.addWidget(linear_box)

        # Rotational DOFs
        rot_box = QGroupBox("Rotational Motion")
        rot_layout = QGridLayout()

        # Header row
        rot_layout.addWidget(QLabel("Axis"), 0, 0)
        rot_layout.addWidget(QLabel("Amp (rad)"), 0, 1)
        rot_layout.addWidget(QLabel("Freq (Hz)"), 0, 2)
        rot_layout.addWidget(QLabel("PGV (rad/s)"), 0, 3)
        rot_layout.addWidget(QLabel("PGA (rad/s²)"), 0, 4)

        # Roll axis
        rot_layout.addWidget(QLabel("Roll:"), 1, 0)
        self.param_inputs["amp_roll"] = QLineEdit(str(self.app.trajectory_params["amp_roll"]))
        self.param_inputs["amp_roll"].textChanged.connect(lambda: self.update_pgv_pga('roll'))
        rot_layout.addWidget(self.param_inputs["amp_roll"], 1, 1)

        self.param_inputs["freq_roll"] = QLineEdit(str(self.app.trajectory_params["freq_roll"]))
        self.param_inputs["freq_roll"].textChanged.connect(lambda: self.update_pgv_pga('roll'))
        rot_layout.addWidget(self.param_inputs["freq_roll"], 1, 2)

        self.pgv_labels["roll"] = QLabel("0.00")
        rot_layout.addWidget(self.pgv_labels["roll"], 1, 3)

        self.pga_labels["roll"] = QLabel("0.00")
        rot_layout.addWidget(self.pga_labels["roll"], 1, 4)

        # Pitch axis
        rot_layout.addWidget(QLabel("Pitch:"), 2, 0)
        self.param_inputs["amp_pitch"] = QLineEdit(str(self.app.trajectory_params["amp_pitch"]))
        self.param_inputs["amp_pitch"].textChanged.connect(lambda: self.update_pgv_pga('pitch'))
        rot_layout.addWidget(self.param_inputs["amp_pitch"], 2, 1)

        self.param_inputs["freq_pitch"] = QLineEdit(str(self.app.trajectory_params["freq_pitch"]))
        self.param_inputs["freq_pitch"].textChanged.connect(lambda: self.update_pgv_pga('pitch'))
        rot_layout.addWidget(self.param_inputs["freq_pitch"], 2, 2)

        self.pgv_labels["pitch"] = QLabel("0.00")
        rot_layout.addWidget(self.pgv_labels["pitch"], 2, 3)

        self.pga_labels["pitch"] = QLabel("0.00")
        rot_layout.addWidget(self.pga_labels["pitch"], 2, 4)

        # Yaw axis
        rot_layout.addWidget(QLabel("Yaw:"), 3, 0)
        self.param_inputs["amp_yaw"] = QLineEdit(str(self.app.trajectory_params["amp_yaw"]))
        self.param_inputs["amp_yaw"].textChanged.connect(lambda: self.update_pgv_pga('yaw'))
        rot_layout.addWidget(self.param_inputs["amp_yaw"], 3, 1)

        self.param_inputs["freq_yaw"] = QLineEdit(str(self.app.trajectory_params["freq_yaw"]))
        self.param_inputs["freq_yaw"].textChanged.connect(lambda: self.update_pgv_pga('yaw'))
        rot_layout.addWidget(self.param_inputs["freq_yaw"], 3, 2)

        self.pgv_labels["yaw"] = QLabel("0.00")
        rot_layout.addWidget(self.pgv_labels["yaw"], 3, 3)

        self.pga_labels["yaw"] = QLabel("0.00")
        rot_layout.addWidget(self.pga_labels["yaw"], 3, 4)

        rot_box.setLayout(rot_layout)
        traj_layout.addWidget(rot_box)

        self.trajectory_button = QPushButton("Execute Pulse Trajectory")
        self.trajectory_button.clicked.connect(self.toggle_trajectory)
        traj_layout.addWidget(self.trajectory_button)

        traj_group.setLayout(traj_layout)
        main_layout.addWidget(traj_group)

        # ---------------- Displacement Trajectory ----------------
        disp_group = QGroupBox("Displacement Trajectory")
        disp_layout = QVBoxLayout()

        # Row 1: Label + Browse button
        top_row = QHBoxLayout()
        self.disp_path_label = QLabel("No file selected")
        self.disp_path_label.setWordWrap(True)
        top_row.addWidget(self.disp_path_label, stretch=1)

        self.disp_browse_button = QPushButton("Browse...")
        self.disp_browse_button.clicked.connect(self.browse_disp_file)
        top_row.addWidget(self.disp_browse_button)

        disp_layout.addLayout(top_row)

        # Row 2: Execute button
        self.disp_trajectory_button = QPushButton("Execute Displacement Trajectory")
        self.disp_trajectory_button.clicked.connect(self.toggle_disp_trajectory)
        disp_layout.addWidget(self.disp_trajectory_button)

        disp_group.setLayout(disp_layout)
        main_layout.addWidget(disp_group)

        # Initialize PGV/PGA calculations
        self.initialize_pgv_pga()
        
        self.setLayout(main_layout)

    ###########################################################################
    # PGV/PGA Calculations
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
            amp_text = self.param_inputs[f"amp_{axis}"].text()
            freq_text = self.param_inputs[f"freq_{axis}"].text()
            
            amp = float(amp_text) if amp_text else 0.0
            freq = float(freq_text) if freq_text else 0.0
            
            # Calculate PGV and PGA
            omega = 2 * math.pi * freq
            pgv = omega * amp
            pga = omega * omega * amp
            
            # Update labels
            self.pgv_labels[axis].setText(f"{pgv:.4f}")
            self.pga_labels[axis].setText(f"{pga:.4f}")
            
        except ValueError:
            # If invalid input, show zeros
            self.pgv_labels[axis].setText("0.00")
            self.pga_labels[axis].setText("0.00")

    ###########################################################################
    # Simulation Control (Delegates to app.py)
    ###########################################################################
    def start_simulation(self):
        """Start the simulation using app.py API."""
        if self.app.start_simulation():
            # Update UI state
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.pause_button.setEnabled(True)
            self.reset_button.setEnabled(True)
            
            # Start GUI update timer
            gui_update_hz = self.app.config["simulation_settings"].get("GUI_update_frequency", 60)
            self.gui_timer.start(int(1000 / gui_update_hz))
        else:
            QMessageBox.critical(self, "Error", "Failed to start simulation. Check logs.")

    def stop_simulation(self):
        """Stop the simulation using app.py API."""
        self.gui_timer.stop()
        
        if self.app.stop_simulation():
            # Update UI state
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.pause_button.setEnabled(False)
            self.reset_button.setEnabled(False)
            self.pause_button.setText("Pause Simulation")
        else:
            QMessageBox.warning(self, "Warning", "Failed to stop simulation properly.")

    def toggle_pause_simulation(self):
        """Toggle pause/resume using app.py API."""
        if self.app.paused:
            # Currently paused, resume it
            if self.app.resume_simulation():
                self.pause_button.setText("Pause Simulation")
        else:
            # Currently running, pause it
            if self.app.pause_simulation():
                self.pause_button.setText("Resume Simulation")

    def reset_simulation(self):
        """Reset the simulation using app.py API."""
        # Stop GUI timer temporarily
        self.gui_timer.stop()
        
        if self.app.reset_simulation():
            # Restart GUI timer
            gui_update_hz = self.app.config["simulation_settings"].get("GUI_update_frequency", 60)
            self.gui_timer.start(int(1000 / gui_update_hz))
        else:
            QMessageBox.critical(self, "Error", "Failed to reset simulation.")

    ###########################################################################
    # Trajectory Control (Delegates to app.py)
    ###########################################################################
    def toggle_trajectory(self):
        """Toggle pulse trajectory execution."""
        if self.app.is_trajectory_running():
            # Stop trajectory
            self.app.stop_trajectory()
            self.trajectory_button.setText("Execute Pulse Trajectory")
        else:
            # Update parameters from UI
            self.update_trajectory_params()
            
            # Start trajectory
            if self.app.start_pulse_trajectory():
                self.trajectory_button.setText("Stop Trajectory")
            else:
                QMessageBox.warning(self, "Warning", "Failed to start trajectory. Is simulation running?")

    def update_trajectory_params(self):
        """Read trajectory parameters from UI and update app.py."""
        try:
            cycle_number = int(self.param_inputs["cycle_number"].text())
            amp_x = float(self.param_inputs["amp_x"].text())
            freq_x = float(self.param_inputs["freq_x"].text())
            amp_y = float(self.param_inputs["amp_y"].text())
            freq_y = float(self.param_inputs["freq_y"].text())
            amp_z = float(self.param_inputs["amp_z"].text())
            freq_z = float(self.param_inputs["freq_z"].text())
            amp_roll = float(self.param_inputs["amp_roll"].text())
            freq_roll = float(self.param_inputs["freq_roll"].text())
            amp_pitch = float(self.param_inputs["amp_pitch"].text())
            freq_pitch = float(self.param_inputs["freq_pitch"].text())
            amp_yaw = float(self.param_inputs["amp_yaw"].text())
            freq_yaw = float(self.param_inputs["freq_yaw"].text())
            
            # Update app.py trajectory params
            self.app.set_pulse_trajectory(
                cycle_number=cycle_number,
                amp_x=amp_x, freq_x=freq_x,
                amp_y=amp_y, freq_y=freq_y,
                amp_z=amp_z, freq_z=freq_z,
                amp_roll=amp_roll, freq_roll=freq_roll,
                amp_pitch=amp_pitch, freq_pitch=freq_pitch,
                amp_yaw=amp_yaw, freq_yaw=freq_yaw
            )
            
        except ValueError as e:
            QMessageBox.warning(self, "Invalid Input", f"Error parsing trajectory parameters: {e}")

    ###########################################################################
    # Displacement Trajectory (Delegates to app.py)
    ###########################################################################
    def browse_disp_file(self):
        """Browse for displacement trajectory CSV file."""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Select Displacement Trajectory CSV", "", "CSV Files (*.csv);;All Files (*)"
        )
        if file_path:
            if self.app.load_displacement_trajectory(file_path):
                self.disp_path_label.setText(file_path)
            else:
                QMessageBox.critical(self, "Error", "Failed to load displacement trajectory. Check format and sampling rate.")

    def toggle_disp_trajectory(self):
        """Toggle displacement trajectory execution."""
        if self.app.is_trajectory_running():
            # Stop trajectory
            self.app.stop_trajectory()
            self.disp_trajectory_button.setText("Execute Displacement Trajectory")
        else:
            # Start displacement trajectory
            if self.app.start_displacement_trajectory():
                self.disp_trajectory_button.setText("Stop Displacement Trajectory")
            else:
                QMessageBox.warning(self, "Warning", "Failed to start displacement trajectory. Load a CSV file first.")

    ###########################################################################
    # Object Upload (Delegates to app.py)
    ###########################################################################
    def upload_obj(self):
        """Open dialog to upload PBR object."""
        if not self.app.simulation_started:
            QMessageBox.warning(self, "Simulation Not Started", "Please start the simulation first.")
            return
        
        # Check if simulation is running and pause it for object placement
        was_paused = self.app.paused
        if not was_paused and self.app.running:
            self.app.pause_simulation()
            self.pause_button.setText("Resume Simulation")
        
        # Open the dialog
        dialog = PBRLoadDialog(self.app, parent=self, pedestal_half_z=self.app.pedestal_half_z)
        result = dialog.exec_()
        
        # Resume simulation if it was running before
        if not was_paused and self.app.running:
            self.app.resume_simulation()
            self.pause_button.setText("Pause Simulation")

    ###########################################################################
    # GUI Update
    ###########################################################################
    def update_gui_display(self):
        """Update GUI display with current simulation state."""
        # Get pedestal pose from app.py
        pose = self.app.get_pedestal_pose()
        
        if pose:
            pos = pose['position']
            orn = pose['orientation']
            
            # Update position label
            self.position_label.setText(
                f"Pedestal Position: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})"
            )
            
            # Update orientation label (convert to degrees)
            roll_deg = math.degrees(orn[0])
            pitch_deg = math.degrees(orn[1])
            yaw_deg = math.degrees(orn[2])
            self.orientation_label.setText(
                f"Pedestal Orientation: Roll={roll_deg:.2f}°, Pitch={pitch_deg:.2f}°, Yaw={yaw_deg:.2f}°"
            )
        
        # Update trajectory button state
        if not self.app.is_trajectory_running():
            if self.trajectory_button.text() == "Stop Trajectory":
                self.trajectory_button.setText("Execute Pulse Trajectory")
            if self.disp_trajectory_button.text() == "Stop Displacement Trajectory":
                self.disp_trajectory_button.setText("Execute Displacement Trajectory")


###############################################################################
# Main Entry Point
###############################################################################
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = PyBulletGUI()
    gui.show()
    sys.exit(app.exec_())
