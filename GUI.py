# main_gui.py
import sys
import time
import math
import threading
import yaml

import pybullet as p

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QCheckBox, QFileDialog, QDialog, QGroupBox,
    QPushButton
)
from PyQt5.QtCore import Qt, QTimer

from simulation_core import SimulationCore

###############################################################################
# Enhanced Dialog for PBR/Collision + Live Preview
###############################################################################
class EnhancedPBRLoadDialog(QDialog):
    """
    A dialog for live preview spawning an OBJ with user-specified
    offset/orientation + PBR and collision settings.

    Buttons:
      - Apply: spawn or update the object in real time
      - Confirm: finalize (keep the object)
      - Cancel: remove the object
    """
    def __init__(self, obj_path, simulation_core, parent=None):
        super().__init__(parent)
        self.obj_path = obj_path
        self.sim_core = simulation_core

        self.setWindowTitle("Spawn OBJ with PBR & Collision - Live Preview")

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
        # 1) Position & Orientation
        # ---------------------------------------------------------------------
        pose_group = QGroupBox("Position & Orientation")
        pose_layout = QHBoxLayout()

        # Position
        pose_layout.addWidget(QLabel("X:"))
        self.x_input = QLineEdit(str(self.default_x))
        pose_layout.addWidget(self.x_input)

        pose_layout.addWidget(QLabel("Y:"))
        self.y_input = QLineEdit(str(self.default_y))
        pose_layout.addWidget(self.y_input)

        pose_layout.addWidget(QLabel("Z:"))
        self.z_input = QLineEdit(str(self.default_z))
        pose_layout.addWidget(self.z_input)

        # Orientation (deg)
        pose_layout.addWidget(QLabel("Roll(deg):"))
        self.roll_input = QLineEdit(str(math.degrees(self.default_roll)))
        pose_layout.addWidget(self.roll_input)

        pose_layout.addWidget(QLabel("Pitch(deg):"))
        self.pitch_input = QLineEdit(str(math.degrees(self.default_pitch)))
        pose_layout.addWidget(self.pitch_input)

        pose_layout.addWidget(QLabel("Yaw(deg):"))
        self.yaw_input = QLineEdit(str(math.degrees(self.default_yaw)))
        pose_layout.addWidget(self.yaw_input)

        pose_group.setLayout(pose_layout)
        main_layout.addWidget(pose_group)

        # ---------------------------------------------------------------------
        # 2) Collision & PBR Properties
        # ---------------------------------------------------------------------
        pbr_group = QGroupBox("Collision & PBR Properties")
        pbr_layout = QGridLayout()


        # mass
        pbr_layout.addWidget(QLabel("Mass:"), 1, 0)
        self.mass_input = QLineEdit("0.0")
        pbr_layout.addWidget(self.mass_input, 1, 1)

        # restitution
        pbr_layout.addWidget(QLabel("Restitution:"), 2, 0)
        self.restitution_input = QLineEdit("0.2")
        pbr_layout.addWidget(self.restitution_input, 2, 1)

        # friction
        pbr_layout.addWidget(QLabel("Friction:"), 3, 0)
        self.friction_input = QLineEdit("0.5")
        pbr_layout.addWidget(self.friction_input, 3, 1)

        # spinning friction
        pbr_layout.addWidget(QLabel("Spinning Fric:"), 4, 0)
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
            pbr_props["lateralFriction"] = float(self.friction_input.text())
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
    # "Apply" => spawn or update live
    # -------------------------------------------------------------------------
    def on_apply(self):
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
            print(f"[Apply] Created new object with ID={self.preview_obj_id}")
        else:
            # Already spawned => update transform & dynamics
            # 1) compute final position based on pedestal
            pedestal_state = p.getLinkState(self.sim_core.robot_id, 3, physicsClientId=self.sim_core.client_id)
            ped_pos = pedestal_state[0]
            final_pos = [
                ped_pos[0] + offset[0],
                ped_pos[1] + offset[1],
                ped_pos[2] + offset[2]
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
            print(f"[Apply] Updated object ID={self.preview_obj_id} to new transform & properties")

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
        self.running = False
        self.trajectory_running = False
        self.trajectory_thread = None

        # load config
        yaml_path = "config.yaml"
        with open(yaml_path, "r") as file:
            self.config = yaml.safe_load(file)

        self.simulation_frequency = self.config["simulation_settings"].get("simulation_frequency", 240)

        # default trajectory params
        self.trajectory_params = {
            "duration": 20.0,
            "timestep": 0.01,
            "amp_x": 0.5,   "freq_x": 0.5,
            "amp_y": 0.5,   "freq_y": 0.5,
            "amp_z": 0.5,   "freq_z": 0.5,
            "amp_roll": 0.1,"freq_roll": 0.5,
            "amp_pitch":0.1,"freq_pitch":0.5,
            "amp_yaw": 0.1, "freq_yaw": 0.5
        }

        self.param_inputs = {}

        # Simulation ticking via QTimer (avoid threading segfaults on macOS GUI)
        self.timer = QTimer(self)
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.timeout.connect(self.on_tick)
        self.tick_hz =  self.simulation_frequency  # Hz
        self._tick_counter = 0
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("VSR Pro")
        self.setGeometry(100, 100, 650, 550)

        main_layout = QVBoxLayout(self)

        # ---------------- Simulation Controls ----------------
        sim_group = QGroupBox("Simulation Controls")
        sim_layout = QVBoxLayout()

        self.start_button = QPushButton("Start Simulation")
        self.start_button.clicked.connect(self.toggle_simulation)
        sim_layout.addWidget(self.start_button)

        self.realtime_checkbox = QCheckBox("Real-Time Trajectory")
        self.realtime_checkbox.setChecked(False)
        sim_layout.addWidget(self.realtime_checkbox)

        self.reset_button = QPushButton("Reset Simulation")
        self.reset_button.clicked.connect(self.reset_simulation)
        sim_layout.addWidget(self.reset_button)

        self.position_label = QLabel("Pedestal Position: (0.0, 0.0, 0.0)")
        sim_layout.addWidget(self.position_label)

        sim_group.setLayout(sim_layout)
        main_layout.addWidget(sim_group)

        # ---------------- Trajectory Settings ----------------
        traj_group = QGroupBox("Trajectory Settings")
        traj_layout = QVBoxLayout()

        dt_layout = QHBoxLayout()
        dt_layout.addWidget(QLabel("Duration (s):"))
        self.param_inputs["duration"] = QLineEdit(str(self.trajectory_params["duration"]))
        dt_layout.addWidget(self.param_inputs["duration"])

        dt_layout.addWidget(QLabel("Timestep (s):"))
        self.param_inputs["timestep"] = QLineEdit(str(self.trajectory_params["timestep"]))
        dt_layout.addWidget(self.param_inputs["timestep"])
        traj_layout.addLayout(dt_layout)

        # Linear DOFs
        linear_box = QGroupBox("Linear DOFs (X, Y, Z)")
        linear_layout = QGridLayout()

        linear_layout.addWidget(QLabel("X Amp:"), 0,0)
        self.param_inputs["amp_x"] = QLineEdit(str(self.trajectory_params["amp_x"]))
        linear_layout.addWidget(self.param_inputs["amp_x"], 0,1)

        linear_layout.addWidget(QLabel("X Freq:"), 0,2)
        self.param_inputs["freq_x"] = QLineEdit(str(self.trajectory_params["freq_x"]))
        linear_layout.addWidget(self.param_inputs["freq_x"], 0,3)

        linear_layout.addWidget(QLabel("Y Amp:"), 1,0)
        self.param_inputs["amp_y"] = QLineEdit(str(self.trajectory_params["amp_y"]))
        linear_layout.addWidget(self.param_inputs["amp_y"], 1,1)

        linear_layout.addWidget(QLabel("Y Freq:"), 1,2)
        self.param_inputs["freq_y"] = QLineEdit(str(self.trajectory_params["freq_y"]))
        linear_layout.addWidget(self.param_inputs["freq_y"], 1,3)

        linear_layout.addWidget(QLabel("Z Amp:"), 2,0)
        self.param_inputs["amp_z"] = QLineEdit(str(self.trajectory_params["amp_z"]))
        linear_layout.addWidget(self.param_inputs["amp_z"], 2,1)

        linear_layout.addWidget(QLabel("Z Freq:"), 2,2)
        self.param_inputs["freq_z"] = QLineEdit(str(self.trajectory_params["freq_z"]))
        linear_layout.addWidget(self.param_inputs["freq_z"], 2,3)

        linear_box.setLayout(linear_layout)
        traj_layout.addWidget(linear_box)

        # Rotational DOFs
        rot_box = QGroupBox("Rotational DOFs (Roll, Pitch, Yaw)")
        rot_layout = QGridLayout()

        rot_layout.addWidget(QLabel("Roll Amp:"), 0,0)
        self.param_inputs["amp_roll"] = QLineEdit(str(self.trajectory_params["amp_roll"]))
        rot_layout.addWidget(self.param_inputs["amp_roll"], 0,1)

        rot_layout.addWidget(QLabel("Roll Freq:"), 0,2)
        self.param_inputs["freq_roll"] = QLineEdit(str(self.trajectory_params["freq_roll"]))
        rot_layout.addWidget(self.param_inputs["freq_roll"], 0,3)

        rot_layout.addWidget(QLabel("Pitch Amp:"),1,0)
        self.param_inputs["amp_pitch"] = QLineEdit(str(self.trajectory_params["amp_pitch"]))
        rot_layout.addWidget(self.param_inputs["amp_pitch"],1,1)

        rot_layout.addWidget(QLabel("Pitch Freq:"),1,2)
        self.param_inputs["freq_pitch"] = QLineEdit(str(self.trajectory_params["freq_pitch"]))
        rot_layout.addWidget(self.param_inputs["freq_pitch"],1,3)

        rot_layout.addWidget(QLabel("Yaw Amp:"),2,0)
        self.param_inputs["amp_yaw"] = QLineEdit(str(self.trajectory_params["amp_yaw"]))
        rot_layout.addWidget(self.param_inputs["amp_yaw"],2,1)

        rot_layout.addWidget(QLabel("Yaw Freq:"),2,2)
        self.param_inputs["freq_yaw"] = QLineEdit(str(self.trajectory_params["freq_yaw"]))
        rot_layout.addWidget(self.param_inputs["freq_yaw"],2,3)

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

        self.upload_obj_button = QPushButton("Upload OBJ (Live Preview)")
        self.upload_obj_button.clicked.connect(self.upload_obj)
        obj_layout.addWidget(self.upload_obj_button)

        obj_group.setLayout(obj_layout)
        main_layout.addWidget(obj_group)

        self.start_position_monitor()
        self.setLayout(main_layout)

    ###########################################################################
    # Position Monitor
    ###########################################################################
    def start_position_monitor(self):
        """Position updates are handled inside on_tick via QTimer."""
        pass

    def on_tick(self):
        """Main simulation tick on the Qt main thread. on_tick is active when 
        the simulation is running, and it steps the physics simulation, updates
        the trajectory if active, and refreshes the pedestal position label at
        a reduced rate to avoid UI lag.
        """
        if not self.simulation_core or self.simulation_core.robot_id is None:
            return
        try:
            # Step physics on the correct client; running on main thread prevents GUI/Metal crashes
            p.stepSimulation(physicsClientId=self.simulation_core.client_id)
        except Exception as e:
            print(f"[on_tick] stepSimulation error: {e}")
            return
        
        # Drive trajectory (main-thread) to avoid threading crashes on macOS
        if getattr(self, "trajectory_running", False):
            try:
                self.simulation_core.execute_trajectory_tick(
                    self.trajectory_params,
                    t=getattr(self, "_traj_t", 0.0),
                    dt=getattr(self, "_traj_dt", 0.01),
                    real_time=getattr(self, "_traj_realtime", False),
                )
                # Advance local trajectory time
                self._traj_t = getattr(self, "_traj_t", 0.0) + getattr(self, "_traj_dt", 0.01)
                if self._traj_t >= getattr(self, "_traj_duration", 20.0):
                    self.stop_trajectory()
            except Exception as e:
                print(f"[on_tick] trajectory tick error: {e}")

        # Update pedestal position label at a reduced rate
        try:
            if (self._tick_counter % 24) == 0:  # ~10 Hz if tick_hz=240
                link_state = p.getLinkState(self.simulation_core.robot_id, 3, physicsClientId=self.simulation_core.client_id)
                if link_state:
                    pos = link_state[0]
                    self.position_label.setText(f"Pedestal Position: {pos}")
        except Exception:
            pass
        finally:
            self._tick_counter += 1

    ###########################################################################
    # Simulation
    ###########################################################################
    def toggle_simulation(self):
        if not self.running:
            self.start_simulation()
        else:
            self.stop_simulation()

    def start_simulation(self):
        if self.simulation_core is None:
            self.simulation_core = SimulationCore("config.yaml")
            self.simulation_core.create_robot()

        if self.simulation_core.robot_id is None:
            print("Error: Failed to create robot.")
            return

        self.running = True
        self.start_button.setText("Stop Simulation")

        # Start Qt timer
        interval_ms = int(1000 / self.tick_hz)
        self.timer.start(interval_ms)

    def stop_simulation(self):
        self.running = False
        self.start_button.setText("Start Simulation")
        if self.timer.isActive():
            self.timer.stop()

    def run_simulation(self):
        """Unused: simulation is advanced by on_tick() via QTimer on the main thread."""
        pass

    def reset_simulation(self):
        """Resets PyBullet simulation and re-applies all settings."""
        if self.simulation_core:
            # Stop ticking during reset
            if self.timer.isActive():
                self.timer.stop()
            self.running = False
            self.start_button.setText("Start Simulation")

            # Reset everything on the correct client
            p.resetSimulation(physicsClientId=self.simulation_core.client_id)

            # Re-apply physics settings
            p.setTimeStep(self.simulation_core.time_step, self.simulation_core.client_id)
            p.setGravity(*self.simulation_core.gravity, physicsClientId=self.simulation_core.client_id)
            p.setRealTimeSimulation(0, physicsClientId=self.simulation_core.client_id)

            # Re-create your robot (and environment if needed)
            self.simulation_core.create_robot()


    ###########################################################################
    # Trajectory
    ###########################################################################
    def toggle_trajectory(self):
        if not self.trajectory_running:
            self.start_trajectory()
        else:
            self.stop_trajectory()

    def start_trajectory(self):
        if not self.simulation_core or not self.simulation_core.robot_id:
            print("Error: Simulation not running.")
            return

        self.update_trajectory_params()
        self.trajectory_running = True
        self.trajectory_button.setText("Stop Trajectory")

        # Per-trajectory state used by on_tick()
        self._traj_t = 0.0
        try:
            self._traj_dt = float(self.trajectory_params.get("timestep", 0.01))
        except Exception:
            self._traj_dt = 0.01
        self._traj_duration = float(self.trajectory_params.get("duration", 20.0))
        self._traj_realtime = bool(self.realtime_checkbox.isChecked())

    def stop_trajectory(self):
        self.trajectory_running = False
        self.trajectory_button.setText("Execute Trajectory")

    def run_trajectory(self):
        if not self.simulation_core or not self.simulation_core.robot_id:
            print("Error: SimulationCore is not initialized.")
            return

        real_time = self.realtime_checkbox.isChecked()
        print(f"Executing trajectory in {'real-time' if real_time else 'offline'} mode.")
        # Note: Trajectory execution is now handled by execute_trajectory_tick in on_tick()
        # This method is kept for compatibility but actual execution happens via QTimer

        self.trajectory_running = False
        self.trajectory_button.setText("Execute Trajectory")

    def update_trajectory_params(self):
        for key in self.trajectory_params:
            try:
                self.trajectory_params[key] = float(self.param_inputs[key].text())
            except ValueError:
                print(f"Warning: invalid input for '{key}' -> using default {self.trajectory_params[key]}")

    ###########################################################################
    # Upload OBJ with Live Preview
    ###########################################################################
    def upload_obj(self):
        if not self.simulation_core or self.simulation_core.robot_id is None:
            print("Error: Simulation must be running to upload an OBJ.")
            return

        obj_path, _ = QFileDialog.getOpenFileName(self, "Select OBJ File", "", "OBJ Files (*.obj)")
        if obj_path:
            print(f"Selected OBJ file: {obj_path}")
            dialog = EnhancedPBRLoadDialog(obj_path, self.simulation_core, parent=self)
            result = dialog.exec_()

            if result == QDialog.Accepted:
                print("User confirmed final placement; the object remains in the scene.")
            else:
                print("User canceled, so the object was removed.")


###############################################################################
# Main Entry
###############################################################################
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = PyBulletGUI()
    gui.show()
    sys.exit(app.exec_())
