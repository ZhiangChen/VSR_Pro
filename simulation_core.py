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
        p.setRealTimeSimulation(1 if self.use_real_time else 0)


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
        pedestal_gap = 0.0

        link3_position_z = self.pedestal_config["absolute_height"] - world_box_half_extents[2]
        # check if link3_position_z is valid, if not, raise an error
        if link3_position_z < pedestal_half_extents[2]:
            raise ValueError("Pedestal position is too low and would intersect with the world box.")

        num_links = 4
        link_masses = [0.0, 0.0, 0.0, self.pedestal_config["mass"]]
        link_collision_shapes = [-1, -1, -1, pedestal_collision_shape]
        link_visual_shapes    = [-1, -1, -1, pedestal_visual_shape]

        link_positions = [
            [0, 0, 0],  
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, link3_position_z]
        ]

        link_orientations = [[0, 0, 0, 1]] * num_links
        link_inertial_positions    = [[0, 0, 0]] * num_links
        link_inertial_orientations = [[0, 0, 0, 1]] * num_links
        link_parent_indices = [0, 1, 2, 3]
        link_joint_types = [p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_SPHERICAL]
        link_joint_axes  = [
            self.joint_config["prismatic_x"]["axis"],
            self.joint_config["prismatic_y"]["axis"],
            self.joint_config["prismatic_z"]["axis"],
            self.joint_config["spherical_joint"]["axis"]
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

        print(f"Robot created with ID: {self.robot_id}")

        # Enable collisions
        p.setCollisionFilterPair(self.robot_id, self.robot_id, -1, 3, enableCollision=1)
        p.changeDynamics(
            self.robot_id, 2,
            jointLowerLimit=pedestal_gap,
            jointUpperLimit=3.0,
            maxJointVelocity=0.1,
            physicsClientId=self.client_id
        )

        # Apply custom dynamics from YAML
        self.apply_dynamic_physics_properties()
        print("Pedestal and dynamics applied.")

    def apply_dynamic_physics_properties(self):
        print("Applying Dynamics Properties...")
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

    def execute_trajectory_tick(self, params, t: float, dt: float, real_time: bool = False):
        """
        Apply one incremental trajectory command at time t using JOINT CONTROL,
        consistent with execute_trajectory(): joints 0-2 are prismatic (x,y,z),
        joint 3 is spherical (roll, pitch, yaw via quaternion).
        This MUST NOT call stepSimulation() or sleep; the GUI (QTimer) advances the world.
        """
        # Read params
        amp_x = params.get("amp_x", 0.0);     freq_x = params.get("freq_x", 0.0)
        amp_y = params.get("amp_y", 0.0);     freq_y = params.get("freq_y", 0.0)
        amp_z = params.get("amp_z", 0.0);     freq_z = params.get("freq_z", 0.0)
        amp_roll  = params.get("amp_roll", 0.0);  freq_roll  = params.get("freq_roll", 0.0)
        amp_pitch = params.get("amp_pitch", 0.0); freq_pitch = params.get("freq_pitch", 0.0)
        amp_yaw   = params.get("amp_yaw", 0.0);   freq_yaw   = params.get("freq_yaw", 0.0)

        # Desired joint positions at time t
        x = amp_x * math.sin(2 * math.pi * freq_x * t)
        y = amp_y * math.sin(2 * math.pi * freq_y * t)
        z = amp_z * math.sin(2 * math.pi * freq_z * t)
        roll  = amp_roll  * math.sin(2 * math.pi * freq_roll  * t)
        pitch = amp_pitch * math.sin(2 * math.pi * freq_pitch * t)
        yaw   = amp_yaw   * math.sin(2 * math.pi * freq_yaw   * t)

        try:
            # Prismatic joints (indices 0,1,2): X, Y, Z
            p.setJointMotorControl2(self.robot_id, 0, p.POSITION_CONTROL,
                                    targetPosition=float(x), physicsClientId=self.client_id)
            p.setJointMotorControl2(self.robot_id, 1, p.POSITION_CONTROL,
                                    targetPosition=float(y), physicsClientId=self.client_id)
            p.setJointMotorControl2(self.robot_id, 2, p.POSITION_CONTROL,
                                    targetPosition=float(z), physicsClientId=self.client_id)

            # Spherical joint (index 3): orientation from Euler -> quaternion
            target_orientation = p.getQuaternionFromEuler([roll, pitch, yaw])
            p.setJointMotorControlMultiDof(self.robot_id, 3, p.POSITION_CONTROL,
                                           targetPosition=target_orientation,
                                           physicsClientId=self.client_id)
        except Exception as e:
            print(f"[SimulationCore.execute_trajectory_tick] error: {e}")


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