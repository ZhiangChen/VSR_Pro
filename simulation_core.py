# simulation_core.py

import yaml
import pybullet as p
import pybullet_data
import time
import math
import logging
import os
from datetime import datetime

class SimulationCore:
    def __init__(self, yaml_path="config.yaml"):
        """Initialize simulation from YAML configuration."""
        with open(yaml_path, "r") as file:
            self.config = yaml.safe_load(file)

        # Extract Simulation Settings
        self.gravity = tuple(self.config["simulation_settings"]["gravity"])
        self.simulation_frequency = self.config["simulation_settings"]["simulation_frequency"]
        self.time_step = 1.0 / self.simulation_frequency
        self.enable_graphics = self.config["simulation_settings"]["enable_graphics"]
        self.use_real_time = self.config["simulation_settings"]["use_real_time"]
        
        # Setup logging
        log_file_path = self.config["simulation_settings"].get("log_file", "data/simulation.log")
        self._setup_logging(log_file_path)

        if p.isConnected():  # If already connected, disconnect first
            p.disconnect()

        self.client_id = p.connect(p.GUI if self.enable_graphics else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set physics
        p.setTimeStep(self.time_step, self.client_id)
        p.setGravity(*self.gravity, self.client_id)


        # Load structure config
        self.world_box_config = self.config["structure"]["world_box"]
        self.pedestal_config  = self.config["structure"]["pedestal"]
        self.dynamics_config  = self.config["dynamics"]
        self.joint_config     = self.config["joints"]
        self.robot_visual_config = self.config["robot_visual"]

        self.robot_id = None

    def _setup_logging(self, log_file_path):
        """Setup logging to file with timestamp."""
        # Create directory if it doesn't exist
        log_dir = os.path.dirname(log_file_path)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # Configure logging
        self.logger = logging.getLogger('SimulationCore')
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
        
        # Log initialization
        self.logger.info("="*80)
        self.logger.info("SimulationCore initialized")
        self.logger.info(f"Gravity: {self.gravity}")
        self.logger.info(f"Simulation frequency: {self.simulation_frequency} Hz")
        self.logger.info(f"Time step: {self.time_step} s")
        self.logger.info(f"Graphics enabled: {self.enable_graphics}")
        self.logger.info(f"Real-time mode: {self.use_real_time}")

    def create_robot(self):
        self.logger.info("Creating 6-DOF Robot...")

        # Load the plane
        plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)
        self.logger.info(f"Plane loaded with ID: {plane_id}")

        # 1) World box
        world_box_half_extents = [dim / 2 for dim in self.world_box_config["dimensions"]]
        world_box_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=world_box_half_extents)
        world_box_visual_shape    = p.createVisualShape(
            p.GEOM_BOX, halfExtents=world_box_half_extents, rgbaColor=self.robot_visual_config["world_box_color"]
        )
        world_box_position = [0, 0, world_box_half_extents[2]]

        # 2) Pedestal
        pedestal_half_extents = [dim / 2 for dim in self.pedestal_config["dimensions"]]
        pedestal_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=pedestal_half_extents)
        pedestal_visual_shape    = p.createVisualShape(
            p.GEOM_BOX, halfExtents=pedestal_half_extents, rgbaColor=self.robot_visual_config["pedestal_color"]
        )

        link3_position_z = self.pedestal_config["absolute_height"] - world_box_half_extents[2]
        # check if link3_position_z is valid, if not, raise an error
        if link3_position_z < pedestal_half_extents[2]:
            raise ValueError("Pedestal position is too low and would intersect with the world box.")

        num_links = 4
        link_masses = [0.0, 0.0, 0.0, self.pedestal_config["mass"]]
        link_collision_shapes = [-1, -1, -1, pedestal_collision_shape]
        link_visual_shapes    = [-1, -1, -1, pedestal_visual_shape]

        link_positions = [
            [0, 0, 0],   # Link0: Base (non-mass) to first prismatic joint (prismatic_x), to world box
            [0, 0, 0],   # Link1: First (non-mass) to second prismatic joint (prismatic_y), to Link0
            [0, 0, 0],   # Link2: Second (non-mass) to third prismatic joint (prismatic_z), to Link1
            [0, 0, link3_position_z]  # Link3: Pedestal to spherical joint (pedestal top), to Link2
        ]

        link_orientations = [[0, 0, 0, 1]] * num_links
        link_inertial_positions    = [[0, 0, 0]] * num_links
        link_inertial_orientations = [[0, 0, 0, 1]] * num_links
        link_parent_indices = [0, 1, 2, 3]
        link_joint_types = [p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_SPHERICAL]
        link_joint_axes  = [
            [1, 0, 0],  # Corrected axis for prismatic_x
            [0, 1, 0],  # Corrected axis for prismatic_y
            [0, 0, 1],  # Corrected axis for prismatic_z
            [0, 0, 0]   # No fixed axis for spherical_joint
        ]

        self.robot_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=world_box_collision_shape,
            baseVisualShapeIndex=world_box_visual_shape,
            basePosition=world_box_position,
            linkMasses=link_masses,
            linkCollisionShapeIndices=link_collision_shapes,
            linkVisualShapeIndices=link_visual_shapes,
            linkPositions=link_positions,
            linkOrientations=link_orientations,
            linkInertialFramePositions=link_inertial_positions,
            linkInertialFrameOrientations=link_inertial_orientations,
            linkParentIndices=link_parent_indices,
            linkJointTypes=link_joint_types,
            linkJointAxis=link_joint_axes
        )

        if self.robot_id < 0:
            raise RuntimeError("Failed to create robot!")
        
        # todo: set range limits and max efforts from YAML

        self.logger.info(f"Robot created with ID: {self.robot_id}")

        # Set camera view
        p.resetDebugVisualizerCamera(
            cameraDistance=20,
            cameraYaw=0,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )

        # Enable collisions
        p.setCollisionFilterPair(self.robot_id, self.robot_id, -1, 3, enableCollision=1)

        # Apply custom dynamics from YAML
        self.apply_physics_properties()
        
        # Set joint limits and maximum efforts
        self.set_joint_limits_and_efforts()
        
        self.logger.info("Pedestal and dynamics applied.")

    def apply_physics_properties(self):
        self.logger.info("Applying Physics Properties...")
        
        # World Box
        p.changeDynamics(
            self.robot_id,
            -1,
            restitution=self.dynamics_config["world_box"]["restitution"],
            lateralFriction=self.dynamics_config["world_box"]["lateralFriction"],
            spinningFriction=self.dynamics_config["world_box"]["spinningFriction"],
            contactDamping=self.dynamics_config["world_box"]["contactDamping"],
            contactStiffness=self.dynamics_config["world_box"]["contactStiffness"],
            physicsClientId=self.client_id
        )
        
        # Get and log world box dynamics
        wb_dynamics = p.getDynamicsInfo(self.robot_id, -1, physicsClientId=self.client_id)
        self.logger.info("World Box Dynamics:")
        self.logger.info(f"  Mass: {wb_dynamics[0]}")
        self.logger.info(f"  Lateral Friction: {wb_dynamics[1]}")
        self.logger.info(f"  Local Inertia Diagonal: {wb_dynamics[2]}")
        self.logger.info(f"  Restitution: {wb_dynamics[5]}")
        self.logger.info(f"  Rolling Friction: {wb_dynamics[6]}")
        self.logger.info(f"  Spinning Friction: {wb_dynamics[7]}")
        self.logger.info(f"  Contact Damping: {wb_dynamics[8]}")
        self.logger.info(f"  Contact Stiffness: {wb_dynamics[9]}")

        # Pedestal links
        for i in range(4):
            p.changeDynamics(
                self.robot_id,
                i,
                restitution=self.dynamics_config["pedestal"]["restitution"],
                lateralFriction=self.dynamics_config["pedestal"]["lateralFriction"],
                spinningFriction=self.dynamics_config["pedestal"]["spinningFriction"],
                contactDamping=self.dynamics_config["pedestal"]["contactDamping"],
                contactStiffness=self.dynamics_config["pedestal"]["contactStiffness"],
                localInertiaDiagonal=self.pedestal_config["inertia"],
                physicsClientId=self.client_id
            )
            
            # Get and log pedestal dynamics
            ped_dynamics = p.getDynamicsInfo(self.robot_id, i, physicsClientId=self.client_id)
            self.logger.info(f"Pedestal Link {i} Dynamics:")
            self.logger.info(f"  Mass: {ped_dynamics[0]}")
            self.logger.info(f"  Lateral Friction: {ped_dynamics[1]}")
            self.logger.info(f"  Local Inertia Diagonal: {ped_dynamics[2]}")
            self.logger.info(f"  Restitution: {ped_dynamics[5]}")
            self.logger.info(f"  Rolling Friction: {ped_dynamics[6]}")
            self.logger.info(f"  Spinning Friction: {ped_dynamics[7]}")
            self.logger.info(f"  Contact Damping: {ped_dynamics[8]}")
            self.logger.info(f"  Contact Stiffness: {ped_dynamics[9]}")

    def calculate_maximum_efforts(self):
        """
        Calculate maximum force and torque based on pedestal mass, inertia, and maximum acceleration.
        
        For prismatic joints: F = m * a_max + m * g (to overcome gravity)
        For spherical joint: τ = I * α_max (rotational inertia * angular acceleration)
        
        Returns:
            dict: Maximum efforts for each joint type
        """
        pedestal_mass = self.pedestal_config["mass"]
        pedestal_inertia = self.pedestal_config["inertia"]  # [Ixx, Iyy, Izz]
        gravity_magnitude = abs(self.gravity[2])  # |g|
        
        # Get maximum accelerations from config
        max_acc_x = self.joint_config["prismatic_x"]["max_acceleration"]
        max_acc_y = self.joint_config["prismatic_y"]["max_acceleration"]
        max_acc_z = self.joint_config["prismatic_z"]["max_acceleration"]
        max_ang_acc = self.joint_config["spherical_joint"]["max_acceleration"]
        
        # Calculate maximum forces for prismatic joints
        # F = m * a_max + safety_factor * m * g (to overcome gravity and provide margin)
        safety_factor = 3  # 200% safety margin
        
        max_force_x = pedestal_mass * (max_acc_x + safety_factor * gravity_magnitude)
        max_force_y = pedestal_mass * (max_acc_y + safety_factor * gravity_magnitude)
        max_force_z = pedestal_mass * (max_acc_z + safety_factor * gravity_magnitude)
        
        # Calculate maximum torques for spherical joint (use maximum inertia for worst case)
        max_inertia = max(pedestal_inertia)  # Use largest moment of inertia
        max_torque = max_inertia * max_ang_acc * safety_factor/2.
        
        efforts = {
            "prismatic_x": max_force_x,
            "prismatic_y": max_force_y,
            "prismatic_z": max_force_z,
            "spherical": max_torque
        }
        
        self.logger.info("Calculated maximum efforts:")
        self.logger.info(f"  Prismatic X: {max_force_x:.2e} N")
        self.logger.info(f"  Prismatic Y: {max_force_y:.2e} N")
        self.logger.info(f"  Prismatic Z: {max_force_z:.2e} N")
        self.logger.info(f"  Spherical: {max_torque:.2e} N*m")
        
        return efforts

    def set_joint_limits_and_efforts(self):
        """
        Set joint position limits, velocity limits, and maximum efforts based on configuration.
        """
        self.logger.info("Setting joint limits and maximum efforts...")
        
        # Calculate maximum efforts
        self.max_efforts = self.calculate_maximum_efforts()
        
        # Joint configurations
        joint_configs = [
            ("prismatic_x", 0, self.max_efforts["prismatic_x"]),
            ("prismatic_y", 1, self.max_efforts["prismatic_y"]),
            ("prismatic_z", 2, self.max_efforts["prismatic_z"])
        ]
        
        # Set limits for prismatic joints
        for joint_name, joint_idx, max_force in joint_configs:
            joint_config = self.joint_config[joint_name]
            lower_limit = joint_config["limit"][0]
            upper_limit = joint_config["limit"][1]
            max_acceleration = joint_config["max_acceleration"]
            
            # Calculate reasonable velocity limit based on acceleration
            # v_max = sqrt(2 * a_max * range) or a more conservative approach
            joint_range = upper_limit - lower_limit
            max_velocity = min(10.0, max_acceleration)  # Conservative velocity limit
            
            p.changeDynamics(
                self.robot_id,
                joint_idx,
                jointLowerLimit=lower_limit,
                jointUpperLimit=upper_limit,
                maxJointVelocity=max_velocity,
                physicsClientId=self.client_id
            )
            
            self.logger.info(f"Joint {joint_idx} ({joint_name}):")
            self.logger.info(f"  Position limits: [{lower_limit}, {upper_limit}]")
            self.logger.info(f"  Max velocity: {max_velocity} m/s")
            self.logger.info(f"  Max force: {max_force:.2e} N")
        
        # For spherical joint (joint 3), set position limits and torque limits
        spherical_config = self.joint_config["spherical_joint"]
        lower_limit = spherical_config["limit"][0]
        upper_limit = spherical_config["limit"][1]
        max_angular_acceleration = spherical_config["max_acceleration"]
        max_angular_velocity = max_angular_acceleration   
        max_torque = self.max_efforts["spherical"]
        
        # Set spherical joint limits - PyBullet spherical joints support position limits
        try:
            p.changeDynamics(
                self.robot_id,
                3,  # Spherical joint index
                jointLowerLimit=lower_limit,
                jointUpperLimit=upper_limit,
                maxJointVelocity=max_angular_velocity,
                physicsClientId=self.client_id
            )
            self.logger.info(f"Joint 3 (spherical):")
            self.logger.info(f"  Position limits: [{lower_limit}, {upper_limit}] rad")
            self.logger.info(f"  Max angular velocity: {max_angular_velocity} rad/s")
            self.logger.info(f"  Max torque: {max_torque:.2e} N*m")
        except Exception as e:
            self.logger.warning(f"Could not set all spherical joint limits: {e}")
            # Fallback: try setting only velocity limit
            try:
                p.changeDynamics(
                    self.robot_id,
                    3,
                    maxJointVelocity=max_angular_velocity,
                    physicsClientId=self.client_id
                )
                self.logger.info(f"Joint 3 (spherical) - fallback:")
                self.logger.info(f"  Max angular velocity: {max_angular_velocity} rad/s")
                self.logger.info(f"  Max torque: {max_torque:.2e} N*m")
                self.logger.info(f"  Note: Position limits [{lower_limit}, {upper_limit}] rad may not be enforced")
            except Exception as e2:
                self.logger.error(f"Could not set spherical joint properties: {e2}")

    def step_trajectory(self, params, t: float):
        """
        Apply one incremental trajectory command at time t using VELOCITY CONTROL.
        Joints 0-2 are prismatic (x,y,z), joint 3 is spherical (roll, pitch, yaw).
        
        Position: x(t) = amp * (1 - cos(2πft))
        Velocity: v(t) = dx/dt = amp * 2πf * sin(2πft)
        
        Note: For spherical joints, PyBullet doesn't support VELOCITY_CONTROL with 
        setJointMotorControlMultiDof, so we use TORQUE_CONTROL with computed torques.
        """
        # Read params
        amp_x = params.get("amp_x", 0.0);     freq_x = params.get("freq_x", 0.0)
        amp_y = params.get("amp_y", 0.0);     freq_y = params.get("freq_y", 0.0)
        amp_z = params.get("amp_z", 0.0);     freq_z = params.get("freq_z", 0.0)
        amp_roll  = params.get("amp_roll", 0.0);  freq_roll  = params.get("freq_roll", 0.0)
        amp_pitch = params.get("amp_pitch", 0.0); freq_pitch = params.get("freq_pitch", 0.0)
        amp_yaw   = params.get("amp_yaw", 0.0);   freq_yaw   = params.get("freq_yaw", 0.0)

        # Calculate velocities at time t
        # Ground motion: x(t) = amp * (1 - cos(2πft))
        # Derivative: v(t) = amp * 2πf * sin(2πft)
        two_pi = 2 * math.pi
        
        vx = amp_x * two_pi * freq_x * math.sin(two_pi * freq_x * t)
        vy = amp_y * two_pi * freq_y * math.sin(two_pi * freq_y * t)
        vz = amp_z * two_pi * freq_z * math.sin(two_pi * freq_z * t)
        vroll  = amp_roll  * two_pi * freq_roll  * math.sin(two_pi * freq_roll  * t)
        vpitch = amp_pitch * two_pi * freq_pitch * math.sin(two_pi * freq_pitch * t)
        vyaw   = amp_yaw   * two_pi * freq_yaw   * math.sin(two_pi * freq_yaw   * t)

        try:
            # Prismatic joints (indices 0,1,2): X, Y, Z using VELOCITY_CONTROL
            p.setJointMotorControl2(self.robot_id, 0, p.VELOCITY_CONTROL,
                                    targetVelocity=float(vx), 
                                    force=self.max_efforts["prismatic_x"],
                                    physicsClientId=self.client_id)
            p.setJointMotorControl2(self.robot_id, 1, p.VELOCITY_CONTROL,
                                    targetVelocity=float(vy),
                                    force=self.max_efforts["prismatic_y"],
                                    physicsClientId=self.client_id)
            p.setJointMotorControl2(self.robot_id, 2, p.VELOCITY_CONTROL,
                                    targetVelocity=float(vz),
                                    force=self.max_efforts["prismatic_z"],
                                    physicsClientId=self.client_id)

            # Spherical joint (index 3): Use TORQUE_CONTROL with P controller
            # Get current angular velocity
            joint_state = p.getJointStateMultiDof(self.robot_id, 3, physicsClientId=self.client_id)
            current_ang_vel = joint_state[1]  # [wx, wy, wz]
            
            # Proportional controller for velocity tracking
            # PyBullet's internal damping provides sufficient stabilization
            Kp = self.max_efforts["spherical"] * 10.0   # Proportional gain
            target_ang_vel = [vroll, vpitch, vyaw]
            
            # P control: torque = Kp * (target - current)
            torques = [
                Kp * (target_ang_vel[i] - current_ang_vel[i])
                for i in range(3)
            ]
            
            # Clamp torques to max effort
            max_torque = self.max_efforts["spherical"]
            torques = [max(-max_torque, min(max_torque, t)) for t in torques]
            
            p.setJointMotorControlMultiDof(
                self.robot_id, 3, 
                p.TORQUE_CONTROL,
                force=torques,
                physicsClientId=self.client_id
            )
        except Exception as e:
            self.logger.error(f"step_trajectory error: {e}")

    def set_pedestal_position(self, position, orientation_euler):
        """
        Maintain pedestal at a specific position and orientation using POSITION_CONTROL.
        Used when trajectory is not running to hold the last known position.
        
        :param position: [x, y, z] target position
        :param orientation_euler: [roll, pitch, yaw] target orientation in radians
        """
        try:
            # Prismatic joints (indices 0,1,2): X, Y, Z using POSITION_CONTROL
            p.setJointMotorControl2(self.robot_id, 0, p.POSITION_CONTROL,
                                    targetPosition=float(position[0]),
                                    force=self.max_efforts["prismatic_x"],
                                    physicsClientId=self.client_id)
            p.setJointMotorControl2(self.robot_id, 1, p.POSITION_CONTROL,
                                    targetPosition=float(position[1]),
                                    force=self.max_efforts["prismatic_y"],
                                    physicsClientId=self.client_id)
            p.setJointMotorControl2(self.robot_id, 2, p.POSITION_CONTROL,
                                    targetPosition=float(position[2]),
                                    force=self.max_efforts["prismatic_z"],
                                    physicsClientId=self.client_id)

            # Spherical joint (index 3): Use POSITION_CONTROL
            # Convert Euler angles to quaternion
            target_quat = p.getQuaternionFromEuler(orientation_euler)
            
            p.setJointMotorControlMultiDof(
                self.robot_id, 3,
                p.POSITION_CONTROL,
                targetPosition=target_quat,
                force=[self.max_efforts["spherical"]] * 3,
                physicsClientId=self.client_id
            )
        except Exception as e:
            self.logger.error(f"set_pedestal_position error: {e}")

    def hold_pedestal_position(self):
        """
        Hold the pedestal at its current position using zero velocity control.
        Used when trajectory is not running to maintain the current position without drift.
        
        This applies zero velocity targets to all joints, which maintains their current
        positions while allowing the physics engine to handle external disturbances naturally.
        """
        try:
            # Prismatic joints (indices 0,1,2): X, Y, Z using zero velocity
            joint_names = ['prismatic_x', 'prismatic_y', 'prismatic_z']
            for joint_idx in range(3):
                p.setJointMotorControl2(
                    self.robot_id, 
                    joint_idx, 
                    p.VELOCITY_CONTROL,
                    targetVelocity=0.0,
                    force=self.max_efforts[joint_names[joint_idx]],
                    physicsClientId=self.client_id
                )
            
            # Spherical joint (index 3): Use P controller to stop rotation
            # Get current angular velocity
            joint_state = p.getJointStateMultiDof(self.robot_id, 3, physicsClientId=self.client_id)
            current_ang_vel = joint_state[1]  # [wx, wy, wz]
            
            # Proportional controller for stopping rotation
            # PyBullet's internal damping provides sufficient stabilization
            Kp = self.max_efforts["spherical"] * 10.0   # Proportional gain
            target_ang_vel = [0.0, 0.0, 0.0]  # Zero velocity to stop rotation
            
            # P control: torque = Kp * (target - current)
            torques = [
                Kp * (target_ang_vel[i] - current_ang_vel[i])
                for i in range(3)
            ]
            
            # Clamp torques to max effort
            max_torque = self.max_efforts["spherical"]
            torques = [max(-max_torque, min(max_torque, t)) for t in torques]
            
            p.setJointMotorControlMultiDof(
                self.robot_id, 3,
                p.TORQUE_CONTROL,
                force=torques,
                physicsClientId=self.client_id
            )
        except Exception as e:
            self.logger.error(f"hold_pedestal_position error: {e}")

    def step_displacement_trajectory(self, displacement):
        """
        Apply displacement trajectory using POSITION_CONTROL.
        Used for pre-recorded displacement trajectories from CSV files.
        
        :param displacement: List or array [x, y, z, roll, pitch, yaw]
                            Position in meters, orientation in radians
        """
        try:
            # Validate input
            if len(displacement) != 6:
                self.logger.error(f"step_displacement_trajectory: Expected 6 DOFs, got {len(displacement)}")
                return
            
            x, y, z, roll, pitch, yaw = displacement
            
            # Prismatic joints (indices 0,1,2): X, Y, Z using POSITION_CONTROL
            p.setJointMotorControl2(self.robot_id, 0, p.POSITION_CONTROL,
                                    targetPosition=float(x),
                                    force=self.max_efforts["prismatic_x"],
                                    physicsClientId=self.client_id)
            p.setJointMotorControl2(self.robot_id, 1, p.POSITION_CONTROL,
                                    targetPosition=float(y),
                                    force=self.max_efforts["prismatic_y"],
                                    physicsClientId=self.client_id)
            p.setJointMotorControl2(self.robot_id, 2, p.POSITION_CONTROL,
                                    targetPosition=float(z),
                                    force=self.max_efforts["prismatic_z"],
                                    physicsClientId=self.client_id)

            # Spherical joint (index 3): Use POSITION_CONTROL
            # Convert Euler angles to quaternion
            target_quat = p.getQuaternionFromEuler([roll, pitch, yaw])
            
            p.setJointMotorControlMultiDof(
                self.robot_id, 3,
                p.POSITION_CONTROL,
                targetPosition=target_quat,
                force=[self.max_efforts["spherical"]] * 3,
                physicsClientId=self.client_id
            )
        except Exception as e:
            self.logger.error(f"step_displacement_trajectory error: {e}")

    def spawn_obj_on_pedestal(
        self,
        obj_path: str,
        mesh_scale=[1, 1, 1],
        offset=[0.0, 0.0, 0.0],
        orientation_euler=[0.0, 0.0, 0.0],
        pbr_props: dict = None
    ):
        """
        Spawns an OBJ on top of the pedestal (link index=3) with collision always enabled.
        Also applies optional PBR/physics properties.

        :param obj_path: Path to the .obj file
        :param mesh_scale: [sx, sy, sz] scaling of the OBJ
        :param offset: [dx, dy, dz] relative offset from pedestal link 3
        :param orientation_euler: [roll, pitch, yaw] in radians
        :param pbr_props: optional dict for mass, friction, etc.
                        Example:
                        {
                            "mass": 10.0,
                            "restitution": 0.2,
                            "lateralFriction": 0.5,
                            "spinningFriction": 0.1,
                            "contactDamping": 10000.0,
                            "contactStiffness": 1000000.0
                        }
        :return: The newly created object ID (int), or None on error.
        """
        if self.robot_id is None:
            self.logger.error("Robot has not been created yet.")
            return None

        # 1) Get pedestal link (index=3) state
        pedestal_state = p.getLinkState(self.robot_id, 3)
        if not pedestal_state:
            self.logger.error("Could not get pedestal link state.")
            return None

        pedestal_pos = pedestal_state[0]  # (x, y, z)
        final_position = [
            pedestal_pos[0] + offset[0],
            pedestal_pos[1] + offset[1],
            pedestal_pos[2] + offset[2]
        ]
        final_orientation = p.getQuaternionFromEuler(orientation_euler)

        # 2) Always create a collision shape for the OBJ
        obj_collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=obj_path,
            meshScale=mesh_scale
        )

        # 3) Create the visual shape
        obj_visual_shape = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=obj_path,
            meshScale=mesh_scale
        )

        # 4) Determine mass (default=0 => static)
        mass_val = 0.0
        if pbr_props and "mass" in pbr_props:
            mass_val = pbr_props["mass"]

        # 5) Create the multi-body
        obj_id = p.createMultiBody(
            baseMass=mass_val,
            baseCollisionShapeIndex=obj_collision_shape,
            baseVisualShapeIndex=obj_visual_shape,
            basePosition=final_position,
            baseOrientation=final_orientation
        )
        if obj_id < 0:
            self.logger.error("Failed to create OBJ multiBody.")
            return None

        self.logger.info(f"Spawned OBJ '{obj_path}' at {final_position} (Euler={orientation_euler}), ID={obj_id}")

        # 6) Always enable collision between this new object (linkIndex = -1)
        #    and the pedestal link (index=3) in the robot multibody
        p.setCollisionFilterPair(obj_id, self.robot_id, -1, 3, enableCollision=1)
        self.logger.info("Collision enabled with pedestal link=3.")

        # 7) If we have PBR properties, apply them
        if pbr_props:
            p.changeDynamics(
                obj_id,
                -1,
                restitution=pbr_props.get("restitution", 0.0),
                lateralFriction=pbr_props.get("lateralFriction", 0.5),
                spinningFriction=pbr_props.get("spinningFriction", 0.0),
                contactDamping=pbr_props.get("contactDamping", 0.0),
                contactStiffness=pbr_props.get("contactStiffness", 1e5),
                physicsClientId=self.client_id
            )
            self.logger.info(f"Applied PBR properties: {pbr_props}")
            
            # Get and log the actual dynamics info
            obj_dynamics = p.getDynamicsInfo(obj_id, -1, physicsClientId=self.client_id)
            self.logger.info("Object Physics Properties:")
            self.logger.info(f"  Mass: {obj_dynamics[0]}")
            self.logger.info(f"  Lateral Friction: {obj_dynamics[1]}")
            self.logger.info(f"  Local Inertia Diagonal: {obj_dynamics[2]}")
            self.logger.info(f"  Restitution: {obj_dynamics[5]}")
            self.logger.info(f"  Rolling Friction: {obj_dynamics[6]}")
            self.logger.info(f"  Spinning Friction: {obj_dynamics[7]}")
            self.logger.info(f"  Contact Damping: {obj_dynamics[8]}")
            self.logger.info(f"  Contact Stiffness: {obj_dynamics[9]}")

        return obj_id