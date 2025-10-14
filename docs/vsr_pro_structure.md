# VSR Pro Robot Structure

The VSR Pro robot represents a **6-DOF shake platform**. The first three joints (X, Y, Z) define linear motion axes, and the last spherical joint defines rotation. The pedestal serves as the physical platform for testing, while the base world box anchors the structure. Intermediate links are massless, ensuring numerical stability and efficiency for precise actuation-based simulations.

VSR Pro supports **two pedestal types**:
- **Box Pedestal** (default): Simple rectangular geometry for standard shake table simulations
- **Mesh Pedestal**: Custom 3D mesh (OBJ format) for terrain simulation and complex surface interactions

## Links and Joints

| Index | Element | Parent | Joint Type | Axis | Geometry / Mass | Role |
|------:|---------|--------|-------------|------|------------------|------|
| Base (-1) | **World Box** | — | — | — | Box (50×14×2), mass=0 | Static base that anchors the robot |
| 0 | **Link 0** | Base | **Prismatic** | [1, 0, 0] | None (virtual), mass=0 | X-axis translation DOF |
| 1 | **Link 1** | Link 0 | **Prismatic** | [0, 1, 0] | None (virtual), mass=0 | Y-axis translation DOF |
| 2 | **Link 2** | Link 1 | **Prismatic** | [0, 0, 1] | None (virtual), mass=0 | Z-axis translation DOF |
| 3 | **Pedestal** | Link 2 | **Spherical (Multi-DOF)** | (no fixed axis) | Box (10×10×0.5) OR Mesh, mass=2000000 | Roll–Pitch–Yaw rotational DOFs; physical top platform |

**Total Degrees of Freedom:** 6 (3 translational + 3 rotational)

---

## Geometry and Placement

- **World Box:** Serves as the fixed base. Centered at `(0, 0, world_box_half_height)`.
- **Links 0–2:** Represent virtual translational joints (X, Y, Z). Their `linkPositions` should all be `[0, 0, 0]` to align all joint pivots at the same location.
- **Link 3 (Pedestal):** The top platform, positioned at `[0, 0, absolute_height]` above the Z-joint. It's the only physical link with mass and geometry.
  - **Box Pedestal**: Created with `p.GEOM_BOX` using dimensions from `config.yaml`
  - **Mesh Pedestal**: Created with `p.GEOM_MESH` loading OBJ file from path specified in `config.yaml`

---

## Robot Creation Functions

The simulation provides three functions for creating the robot:

### `create_robot()`
Main entry point that selects the appropriate robot creation method based on the `type` field in `config.yaml`:
- If `pedestal.type == "mesh"`: calls `create_mesh_robot()`
- Otherwise (default): calls `create_box_robot()`
- If both are enabled (should not happen), defaults to box pedestal

### `create_box_robot()`
Creates the shake table with a **box pedestal**:
- Uses `pedestal.box.dimensions` for geometry
- Uses `pedestal.box.mass` and `pedestal.box.inertia` for dynamics
- Creates collision and visual shapes using `p.GEOM_BOX`
- Suitable for standard shake table simulations

### `create_mesh_robot()`
Creates the shake table with a **mesh pedestal**:
- Loads OBJ mesh from `pedestal.mesh.path`
- Applies scaling from `pedestal.mesh.scaling`
- Uses `pedestal.mesh.mass` and `pedestal.mesh.inertia` for dynamics
- Creates collision and visual shapes using `p.GEOM_MESH`
- Suitable for terrain simulation and complex surface interactions


