# VSR Pro API Architecture

## Overview

The VSR Pro application has been refactored into a modular architecture separating business logic from presentation:

```
┌─────────────────────────────────────────┐
│          User Interfaces                │
├─────────────────┬───────────────────────┤
│    GUI.py       │  example_automation.py│
│  (PyQt5 GUI)    │  (Python Scripts)     │
└────────┬────────┴──────────┬────────────┘
         │                   │
         └─────────┬─────────┘
                   │
         ┌─────────▼──────────┐
         │      app.py        │
         │  (VSRProApp API)   │
         └─────────┬──────────┘
                   │
         ┌─────────▼──────────┐
         │ simulation_core.py │
         │  (Physics Engine)  │
         └─────────┬──────────┘
                   │
         ┌─────────▼──────────┐
         │     PyBullet       │
         └────────────────────┘
```

## Files

### 1. `app.py` - Application API Layer
**Purpose:** Provides high-level API for controlling the simulation programmatically.

**Key Class:** `VSRProApp`

**Responsibilities:**
- Simulation lifecycle management (start/stop/pause/resume/reset)
- Object loading and registration
- Trajectory execution (pulse and displacement)
- Data recording and saving
- Status queries

**Can be used by:**
- GUI applications (GUI.py)
- Automation scripts (example_automation.py)
- Batch processing scripts
- Testing frameworks
- Any Python code that needs simulation control

---

### 2. `GUI.py` - User Interface (To Be Refactored)
**Purpose:** PyQt5 graphical user interface

**Will be refactored to:**
- Use `VSRProApp` instead of directly managing `SimulationCore`
- Focus on UI concerns (buttons, sliders, displays)
- Delegate business logic to `app.py`

---

### 3. `example_automation.py` - Automation Example
**Purpose:** Demonstrates headless/scriptable usage of VSRProApp

**Shows how to:**
- Start/stop simulation programmatically
- Load objects with PBR properties
- Configure and execute trajectories
- Query simulation state
- Batch process multiple configurations

---

## VSRProApp API Reference

### Initialization

```python
from app import VSRProApp

app = VSRProApp("config.yaml")
```

---

### Simulation Control

#### `start_simulation() -> bool`
Start the physics simulation and create the robot.

```python
app.start_simulation()
```

#### `stop_simulation() -> bool`
Stop the physics simulation and cleanup resources.

```python
app.stop_simulation()
```

#### `pause_simulation() -> bool`
Pause the simulation (stop stepping physics).

```python
app.pause_simulation()
```

#### `resume_simulation() -> bool`
Resume the simulation (continue stepping physics).

```python
app.resume_simulation()
```

#### `reset_simulation() -> bool`
Reset the simulation (stop, then start fresh).

```python
app.reset_simulation()
```

---

### Object Loading

#### `load_pbr_object(...) -> int`
Load and spawn an OBJ file on the pedestal with PBR properties.

```python
obj_id = app.load_pbr_object(
    obj_path="data/cube.obj",
    offset=[0.0, 0.0, 0.5],              # [dx, dy, dz] from pedestal center
    orientation_euler=[0.0, 0.0, 0.0],   # [roll, pitch, yaw] in radians
    mesh_scale=[1.0, 1.0, 1.0],          # [sx, sy, sz] scaling
    mass=10.0,                            # kg
    restitution=0.2,                      # bounciness (0-1)
    lateral_friction=0.5,
    spinning_friction=0.1,
    contact_damping=10000.0,
    contact_stiffness=1000000.0
)
```

**Returns:** PyBullet object ID (int) if successful, `None` otherwise

---

### Pulse Trajectory

#### `set_pulse_trajectory(...)`
Configure parameters for sinusoidal (pulse) trajectory.

```python
app.set_pulse_trajectory(
    cycle_number=2,      # Number of oscillation cycles
    amp_x=0.1,          # X amplitude (meters)
    freq_x=1.0,         # X frequency (Hz)
    amp_y=0.05,         # Y amplitude (meters)
    freq_y=1.0,         # Y frequency (Hz)
    amp_z=0.0,          # Z amplitude (meters)
    freq_z=0.0,         # Z frequency (Hz)
    amp_roll=0.0,       # Roll amplitude (radians)
    freq_roll=0.0,      # Roll frequency (Hz)
    amp_pitch=0.0,      # Pitch amplitude (radians)
    freq_pitch=0.0,     # Pitch frequency (Hz)
    amp_yaw=0.05,       # Yaw amplitude (radians)
    freq_yaw=1.0        # Yaw frequency (Hz)
)
```

#### `start_pulse_trajectory() -> bool`
Start executing the configured pulse trajectory.

```python
app.start_pulse_trajectory()
```

**Note:** Automatically starts recording. Trajectory stops after specified cycles complete.

---

### Displacement Trajectory

#### `load_displacement_trajectory(csv_path) -> bool`
Load a displacement trajectory from CSV file.

```python
success = app.load_displacement_trajectory("data/trajectory.csv")
```

**CSV Format Requirements:**
- Must contain column: `time_s`
- Must contain DOF columns: `X_m`, `Y_m`, `Z_m`, `Roll_rad`, `Pitch_rad`, `Yaw_rad`
- Sampling rate must match simulation frequency (default: 500 Hz)
- Minimum 2 data points

#### `start_displacement_trajectory() -> bool`
Start executing the loaded displacement trajectory.

```python
app.start_displacement_trajectory()
```

**Note:** Must call `load_displacement_trajectory()` first.

#### `stop_trajectory() -> bool`
Stop the currently running trajectory (pulse or displacement).

```python
app.stop_trajectory()
```

---

### Data Recording

#### `start_recording()`
Start recording trajectory data.

```python
app.start_recording()
```

**Note:** Called automatically when starting trajectories.

#### `stop_recording()`
Stop recording and save data to NPZ file.

```python
app.stop_recording()
```

**Output:**
- Saves to `data/logs/run_YYYYMMDD_HHMMSS.npz`
- Contains: times, pedestal poses, PBR object poses, metadata

---

### Status Queries

#### `get_pedestal_pose() -> dict`
Get current pedestal position and orientation.

```python
pose = app.get_pedestal_pose()
# Returns:
# {
#     'position': [x, y, z],
#     'orientation': [roll, pitch, yaw]
# }
```

#### `is_simulation_running() -> bool`
Check if simulation is running (and not paused).

```python
if app.is_simulation_running():
    print("Simulation is active")
```

#### `is_trajectory_running() -> bool`
Check if any trajectory is currently executing.

```python
if app.is_trajectory_running():
    print("Trajectory in progress")
```

---

## Usage Examples

### Example 1: Basic Automation

```python
from app import VSRProApp
import time

# Initialize and start
app = VSRProApp("config.yaml")
app.start_simulation()
time.sleep(1)

# Load object
obj_id = app.load_pbr_object(
    obj_path="data/cube.obj",
    offset=[0.0, 0.0, 0.5],
    mass=5.0
)

# Execute trajectory
app.set_pulse_trajectory(
    cycle_number=3,
    amp_x=0.1,
    freq_x=1.0
)
app.start_pulse_trajectory()

# Wait for completion
time.sleep(4.0)

# Cleanup
app.stop_simulation()
```

---

### Example 2: Batch Processing

```python
from app import VSRProApp
import time

test_frequencies = [0.5, 1.0, 2.0, 4.0]

for freq in test_frequencies:
    print(f"Testing frequency: {freq} Hz")
    
    app = VSRProApp("config.yaml")
    app.start_simulation()
    time.sleep(1)
    
    # Run test
    app.set_pulse_trajectory(
        cycle_number=5,
        amp_x=0.1,
        freq_x=freq
    )
    app.start_pulse_trajectory()
    
    # Wait for completion
    duration = 5.0 / freq  # cycles / frequency
    time.sleep(duration + 1.0)
    
    # Data automatically saved to data/logs/
    app.stop_simulation()
    time.sleep(2)

print("Batch processing complete!")
```

---

### Example 3: Displacement Trajectory

```python
from app import VSRProApp
import time

app = VSRProApp("config.yaml")
app.start_simulation()
time.sleep(1)

# Load object
app.load_pbr_object("data/object.obj", offset=[0.0, 0.0, 0.5])

# Load and execute displacement trajectory
if app.load_displacement_trajectory("data/trajectory.csv"):
    app.start_displacement_trajectory()
    
    # Wait for completion (adjust time based on trajectory length)
    time.sleep(10.0)

app.stop_simulation()
```

---

### Example 4: Real-time Monitoring

```python
from app import VSRProApp
import time

app = VSRProApp("config.yaml")
app.start_simulation()
time.sleep(1)

# Start trajectory
app.set_pulse_trajectory(cycle_number=2, amp_x=0.1, freq_x=1.0)
app.start_pulse_trajectory()

# Monitor while running
while app.is_trajectory_running():
    pose = app.get_pedestal_pose()
    if pose:
        print(f"X: {pose['position'][0]:.4f} m")
    time.sleep(0.1)

app.stop_simulation()
```

---

## Benefits of This Architecture

### 1. **Separation of Concerns**
- Business logic in `app.py` (simulation control, trajectories, recording)
- Presentation logic in `GUI.py` (buttons, displays, user interaction)
- Backend physics in `simulation_core.py`

### 2. **Reusability**
- `app.py` can be used by multiple interfaces (GUI, CLI, web API, etc.)
- Same API for interactive GUI and batch scripts
- Easy to write test scripts

### 3. **Automation**
- No GUI required for automated testing
- Batch processing of multiple configurations
- Integration with CI/CD pipelines
- Scriptable experiments

### 4. **Maintainability**
- Clear API surface (documented methods)
- Easier to test business logic independently
- Changes to UI don't affect automation scripts
- Changes to business logic update all interfaces

### 5. **Extensibility**
- Easy to add new trajectory types
- Can build other interfaces (web, CLI, ROS node)
- Plugin architecture possible
- External tool integration

---

## Next Steps

### To Complete the Refactoring:

1. **Refactor GUI.py:**
   - Replace direct `SimulationCore` usage with `VSRProApp` instance
   - Remove duplicate business logic (move to `app.py`)
   - Keep only PyQt5 UI code in `GUI.py`
   - GUI methods become thin wrappers calling `app.py`

2. **Example GUI.py Structure After Refactoring:**
   ```python
   class PyBulletGUI(QMainWindow):
       def __init__(self):
           super().__init__()
           self.app = VSRProApp("config.yaml")  # Use API
           self.setup_ui()
       
       def on_start_button_clicked(self):
           self.app.start_simulation()  # Delegate to API
           self.update_ui_state()
       
       def on_trajectory_button_clicked(self):
           # Get parameters from UI
           params = self.get_trajectory_params_from_ui()
           self.app.set_pulse_trajectory(**params)
           self.app.start_pulse_trajectory()
   ```

3. **Testing:**
   - Test automation scripts work correctly
   - Verify GUI still has all functionality
   - Ensure no regressions in existing features

4. **Documentation:**
   - Add docstrings to all public API methods
   - Create tutorial notebooks
   - Add more usage examples

---

## Migration Guide for Existing Code

If you have existing code using the old `GUI.py` directly, here's how to migrate:

### Old Way (Direct SimulationCore):
```python
from simulation_core import SimulationCore

core = SimulationCore("config.yaml")
core.create_robot()
# ... manual thread management
# ... manual recording
```

### New Way (VSRProApp API):
```python
from app import VSRProApp

app = VSRProApp("config.yaml")
app.start_simulation()  # Handles thread management automatically
# ... trajectories automatically record
```

The new API handles all the complexity internally while providing a clean, intuitive interface.
