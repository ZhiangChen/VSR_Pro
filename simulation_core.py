# simulation_core.py

import yaml
import pybullet as p
import pybullet_data
import time
import math

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

        if p.isConnected():  # If already connected, disconnect first
            p.disconnect()

        self.client_id = p.connect(p.GUI if self.enable_graphics else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set physics
        p.setTimeStep(self.time_step, self.client_id)
        p.setGravity(*self.gravity, self.client_id)
        #p.setRealTimeSimulation(1 if self.use_real_time else 0)


        # Load structure config
        self.world_box_config = self.config["structure"]["world_box"]
        self.pedestal_config  = self.config["structure"]["pedestal"]
        self.dynamics_config  = self.config["dynamics"]
        self.joint_config     = self.config["joints"]
        self.robot_visual_config = self.config["robot_visual"]

        self.robot_id = None

    def create_robot(self):
        print("Creating 6-DOF Robot...")

        # Load the plane
        plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)
        print(f"Plane loaded with ID: {plane_id}")

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

        print(f"Robot created with ID: {self.robot_id}")

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
        
        print("Pedestal and dynamics applied.")

    def apply_physics_properties(self):
        print("Applying Physics Properties...")
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
        print(f"World Box Dynamics: {p.getDynamicsInfo(self.robot_id, -1)}")

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
            print(f"Pedestal Link {i} Dynamics: {p.getDynamicsInfo(self.robot_id, i)}")

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
        max_torque = max_inertia * max_ang_acc
        
        efforts = {
            "prismatic_x": max_force_x,
            "prismatic_y": max_force_y,
            "prismatic_z": max_force_z,
            "spherical": max_torque
        }
        
        print(f"Calculated maximum efforts:")
        print(f"  Prismatic X: {max_force_x:.2e} N")
        print(f"  Prismatic Y: {max_force_y:.2e} N")
        print(f"  Prismatic Z: {max_force_z:.2e} N")
        print(f"  Spherical: {max_torque:.2e} N⋅m")
        
        return efforts

    def set_joint_limits_and_efforts(self):
        """
        Set joint position limits, velocity limits, and maximum efforts based on configuration.
        """
        print("Setting joint limits and maximum efforts...")
        
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
            
            print(f"Joint {joint_idx} ({joint_name}):")
            print(f"  Position limits: [{lower_limit}, {upper_limit}]")
            print(f"  Max velocity: {max_velocity} m/s")
            print(f"  Max force: {max_force:.2e} N")
        
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
            print(f"Joint 3 (spherical):")
            print(f"  Position limits: [{lower_limit}, {upper_limit}] rad")
            print(f"  Max angular velocity: {max_angular_velocity} rad/s")
            print(f"  Max torque: {max_torque:.2e} N⋅m")
        except Exception as e:
            print(f"Warning: Could not set all spherical joint limits: {e}")
            # Fallback: try setting only velocity limit
            try:
                p.changeDynamics(
                    self.robot_id,
                    3,
                    maxJointVelocity=max_angular_velocity,
                    physicsClientId=self.client_id
                )
                print(f"Joint 3 (spherical) - fallback:")
                print(f"  Max angular velocity: {max_angular_velocity} rad/s")
                print(f"  Max torque: {max_torque:.2e} N⋅m")
                print(f"  Note: Position limits [{lower_limit}, {upper_limit}] rad may not be enforced")
            except Exception as e2:
                print(f"Error: Could not set spherical joint properties: {e2}")

    def execute_trajectory(self, params, t: float, real_time: bool = False):
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

            # Spherical joint (index 3): Use TORQUE_CONTROL with PD controller
            # Get current angular velocity
            joint_state = p.getJointStateMultiDof(self.robot_id, 3, physicsClientId=self.client_id)
            current_ang_vel = joint_state[1]  # [wx, wy, wz]
            
            # PD controller: torque = Kp * (target_vel - current_vel) + Kd * 0
            # Simple proportional control for velocity tracking
            Kp = self.max_efforts["spherical"] * 10.0  # Proportional gain
            target_ang_vel = [vroll, vpitch, vyaw]
            
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
            print(f"[SimulationCore.execute_trajectory] error: {e}")


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
            print("Error: Robot has not been created yet.")
            return None

        # 1) Get pedestal link (index=3) state
        pedestal_state = p.getLinkState(self.robot_id, 3)
        if not pedestal_state:
            print("Error: Could not get pedestal link state.")
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
            print("Error: Failed to create OBJ multiBody.")
            return None

        print(
            f"Spawned OBJ '{obj_path}' at {final_position} "
            f"(Euler={orientation_euler}), ID={obj_id}"
        )

        # 6) Always enable collision between this new object (linkIndex = -1)
        #    and the pedestal link (index=3) in the robot multibody
        p.setCollisionFilterPair(obj_id, self.robot_id, -1, 3, enableCollision=1)
        print("Collision is always enabled with pedestal link=3.")

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
            print(f"Applied PBR properties: {pbr_props}")
            print(f"Dynamics Info: {p.getDynamicsInfo(obj_id, -1)}")

        return obj_id