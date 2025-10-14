# VSR_Pro

Virtual Shake Robot Pro (VSR Pro) is an advanced software platform for simulating and controlling a 6-DOF shake table using PyBullet physics simulation and PyQt5 GUI.

## Overview

VSR Pro provides a complete simulation environment for a 6-degree-of-freedom (6-DOF) shake table system. The platform includes real-time physics simulation, trajectory generation, data recording, and an intuitive graphical user interface for interactive control and visualization.

### Key Features

- **6-DOF Shake Table Simulation**: Simulates translational (X, Y, Z) and rotational (Roll, Pitch, Yaw) movements
- **Dual Pedestal Options**: Choose between box pedestal (default) or mesh pedestal for terrain simulation
- **Real-time Physics**: Powered by PyBullet physics engine with customizable simulation frequency
- **Modular API Architecture**: Clean separation between GUI, API layer, and simulation core enabling both interactive and programmatic control
- **Interactive GUI**: PyQt5-based interface for real-time control and visualization with pause/resume capability
- **Python API**: `app.py` provides programmatic access for automation, batch processing, and scripting
- **Dual Trajectory Modes**: 
  - Pulse trajectory: Customizable sine/cosine wave patterns with independent amplitude and frequency control
  - Displacement trajectory: CSV-based pre-recorded trajectories with position control
- **Object Loading**: Support for loading external 3D objects (OBJ format) with PBR materials and collision detection
  - **Auto-load from Config**: Configure PBR objects in `config.yaml` to auto-load on simulation start/reset
  - **Custom Inertia**: Specify inertia on principal axes (Ixx, Iyy, Izz) for accurate rotational dynamics
  - **GUI Upload**: Manually upload additional objects via "Upload PBR OBJ" button with live preview
  - **Multi-object Tracking**: Track and record all objects from both config and GUI uploads
- **Data Recording**: Automatic recording of pedestal and object poses during trajectory execution saved as NPZ files
- **Comprehensive Logging**: UTF-8 encoded log files with timestamps for all operations
- **YAML Configuration**: Flexible configuration system for simulation parameters, dynamics, and visual settings
- **Utility Scripts**: Tools for trajectory generation, data visualization, and automation examples

## Project Structure

```
VSR_Pro/
├── LICENSE                     # Apache License 2.0
├── README.md                   # This file
├── GUI.py                      # Original GUI application (legacy)
├── GUI_test.py                 # Refactored GUI using app.py API
├── app.py                      # Application API layer for programmatic control
├── simulation_core.py          # Core simulation engine
├── example_automation.py       # Example automation script using app.py
├── config.yaml                 # Simulation configuration
├── data/                       # Data storage directory
│   ├── logs/                   # Recorded trajectory data
│   │   └── run_*.npz          # NPZ files with trajectory recordings
│   ├── simulation.log         # Simulation log file
│   └── displacement_trajectory.csv  # Example displacement trajectory
├── docs/                       # Documentation files
│   ├── api_architecture.md    # API architecture documentation
│   ├── segmentation_fault.md  # Troubleshooting guide
│   └── vsr_pro_structure.md   # Architecture documentation
├── utils/                      # Utility scripts
│   ├── random_disp_traj.py    # Random trajectory generator
│   └── plot_npz.py            # Data visualization tool
└── .gitignore                  # Git ignore file
```

### File Descriptions

- **`GUI.py`**: Original GUI application (legacy version, will be deprecated)
- **`GUI_test.py`**: Refactored GUI using the new API architecture - cleaner, more maintainable code
- **`app.py`**: Application API layer (`VSRProApp` class) providing programmatic access to simulation control, trajectory execution, object loading, and data recording. Can be used by GUI or Python scripts
- **`example_automation.py`**: Example script demonstrating how to use `app.py` for automation, batch processing, and headless simulation
- **`simulation_core.py`**: Core simulation engine managing PyBullet physics, robot creation (box or mesh pedestal), joint control (velocity and position), and object spawning
- **`config.yaml`**: Configuration file defining simulation settings (frequency, gravity, real-time mode), structural parameters (including pedestal type selection), dynamics properties, joint limits, and visual appearance
- **`data/`**: Directory for storing recorded trajectory data (NPZ format), simulation logs (UTF-8 encoded), displacement trajectory CSV files, and mesh files (e.g., rough_terrain.obj)
- **`docs/`**: Documentation and troubleshooting guides including detailed API architecture documentation
- **`utils/`**: Utility scripts for trajectory generation and data analysis

## Installation & Setup

### Prerequisites

- Python 3.7 or higher
- pip package manager

### Environment Setup

1. **Clone the repository:**
   ```bash
   git clone https://github.com/ZhiangChen/VSR_Pro.git
   cd VSR_Pro
   ```

2. **Create a virtual environment (recommended):**
   ```bash
   python3 -m venv vsr_env
   vsr_env\Scripts\activate     # On Windows
   ```

   For the PowerShell error, run before activating the virtual environment `Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned`

3. **Install required libraries:**
   ```bash
   pip install pybullet PyQt5 PyYAML numpy matplotlib
   ```

### Required Dependencies

- **PyBullet** (`pybullet`): Physics simulation engine
- **PyQt5** (`PyQt5`): GUI framework for the user interface
- **PyYAML** (`PyYAML`): YAML configuration file parsing
- **NumPy** (`numpy`): Numerical computing and data handling
- **Matplotlib** (`matplotlib`): Data visualization (for utility scripts)
- **Standard Libraries**: `sys`, `time`, `math`, `threading`, `csv`, `logging`, `os`, `datetime`

## Usage

### Running the Application

#### Option 1: GUI Interface (Recommended for Interactive Use)

Start the new API-based GUI:
```bash
python GUI_test.py
```

Or use the legacy GUI:
```bash
python GUI.py
```

#### Option 2: Python API (Recommended for Automation)

Use the `app.py` API for programmatic control:

```python
from app import VSRProApp
import time

# Initialize application
app = VSRProApp("config.yaml")

# Start simulation
app.start_simulation()
time.sleep(1)

# Load an object
obj_id = app.load_pbr_object(
    obj_path="data/cube.obj",
    offset=[0.0, 0.0, 0.5],
    mass=10.0
)

# Execute a pulse trajectory
app.set_pulse_trajectory(
    cycle_number=2,
    amp_x=0.1,
    freq_x=1.0
)
app.start_pulse_trajectory()

# Wait for completion
time.sleep(3)

# Stop simulation
app.stop_simulation()
```

See `example_automation.py` for more examples including batch processing.

#### Option 3: Quick Start with Example Automation

Run the example automation script:
```bash
python example_automation.py
```

This demonstrates:
- Starting and stopping simulation
- Configuring and executing trajectories
- Querying simulation state
- Batch processing multiple configurations

### GUI Controls and Features

#### Simulation Controls
- **Start Simulation**: Initialize the physics simulation and create the shake table robot
- **Stop Simulation**: Terminate the simulation and stop all trajectories
- **Pause/Resume Simulation**: Pause physics stepping while keeping simulation state (useful for loading objects)
- **Reset Simulation**: Clear simulation and reinitialize (only available when stopped)

#### Pulse Trajectory Control
Configure sinusoidal motion for each DOF independently:
- **Cycle Number**: Number of oscillation cycles to complete
- **Amplitude and Frequency**: Set for each axis (X, Y, Z, Roll, Pitch, Yaw)
- **PGV/PGA Display**: Real-time calculation of Peak Ground Velocity and Acceleration
- **Execute Pulse Trajectory**: Start/stop trajectory execution using velocity control

#### Displacement Trajectory Control
Execute pre-recorded displacement trajectories from CSV files:
- **Browse**: Select CSV file containing displacement trajectory
- **File Validation**: Automatic validation of sampling frequency (must be the same as simulation frequency) and DOF columns
- **Execute Displacement Trajectory**: Start/stop trajectory execution using position control

CSV Format Requirements:
```csv
time_s,X_m,Y_m,Z_m,Roll_rad,Pitch_rad,Yaw_rad
0.0,0.0001,-0.0095,-0.0014,0.0044,-0.0056,0.0000
0.002,0.0001,-0.0097,-0.0017,0.0045,-0.0057,0.0000
...
```
- `time_s` column is mandatory
- Sampling must be constant (e.g., 500 Hz, 0.002s intervals)
- Missing DOF columns are filled with zeros
- At least one DOF column required

#### Object Loading
Upload 3D objects to the pedestal with live preview:
- **Browse**: Select OBJ file to load
- **Pose Configuration**: Set position offset (X, Y, Z) and orientation (Roll, Pitch, Yaw)
- **Physics Properties**: Configure mass, restitution, friction, contact damping/stiffness
- **Apply**: Preview changes in real-time
- **Confirm**: Finalize object placement
- Objects are automatically tracked during trajectory recording

#### Real-time Display
- **Pedestal Position**: Current X, Y, Z position in meters
- **Pedestal Orientation**: Current Roll, Pitch, Yaw in degrees
- Updates at 60 Hz during simulation

#### Data Recording
Automatic recording during trajectory execution:
- Records pedestal pose (6-DOF)
- Records all loaded object poses
- Saves to `data/run_YYYYMMDD_HHMMSS.npz`
- NPZ contains: times, pedestal_poses, pbr_object_*_poses, metadata
- Log file: `data/simulation.log` (UTF-8 encoded with timestamps)

### Utility Scripts

#### 1. Random Displacement Trajectory Generator (`utils/random_disp_traj.py`)

Generate random 6-DOF displacement trajectories meeting specified PGV and PGA targets.

**Usage:**
```bash
cd utils
python random_disp_traj.py
```

**Configuration** (edit variables in script):
```python
FS = 500.0                # Sampling frequency (Hz)
DURATION = 20.0           # Total duration (s)
DOF_MASK = [True, True, True, True, True, False]  # Enable/disable each DOF

# Linear motion targets (X, Y, Z)
LINEAR_PGV = 0.20         # Peak ground velocity (m/s)
LINEAR_PGA = 2.50         # Peak ground acceleration (m/s^2)
LINEAR_F_MAX = 5.0        # Upper frequency limit (Hz)

# Rotational motion targets (Roll, Pitch, Yaw)
ROTATIONAL_PGV = 0.10     # Peak ground velocity (rad/s)
ROTATIONAL_PGA = 1.50     # Peak ground acceleration (rad/s^2)
ROTATIONAL_F_MAX = 3.0    # Upper frequency limit (Hz)

CSV_PATH = "../data/displacement_trajectory.csv"
```

**Outputs:**
- CSV file: `displacement_trajectory.csv` (for GUI execution)
- Plots: Two 3x3 figures showing position and orientation trajectories

**Features:**
- Generates trajectories with primary harmonic plus random harmonics
- Meets target PGV/PGA values for each enabled DOF
- Separate limits for linear and rotational motion
- Visualizes displacement, velocity, and acceleration
- Compatible with GUI displacement trajectory execution

#### 2. Data Visualization Tool (`utils/plot_npz.py`)

Visualize recorded trajectory data from NPZ files.

**Usage:**
```bash
cd utils
python plot_npz.py
```

**Configuration** (edit variables in script):
```python
path = "../data/run_20251013_200721.npz"   # Set your NPZ file path
SAVE_PLOTS = False                          # True to save PNG files
OUTDIR = None                               # Output directory for plots
```

**Features:**
- Plots pedestal position (X, Y, Z) vs time
- Plots pedestal orientation (Roll, Pitch, Yaw) vs time
- Plots PBR object poses if available
- Optional SciencePlots styling (install with `pip install SciencePlots`)
- Saves plots as PNG files if enabled

**Plot Output:**
- Position subplot: X, Y, Z translations
- Orientation subplot: Roll, Pitch, Yaw rotations
- Object tracking: Additional plots for each loaded object

### Configuration

Modify `config.yaml` to customize:

- **Simulation Settings**: Frequency (500 Hz), gravity, GUI enable/disable, real-time mode, log file path
- **Structural Parameters**: Dimensions and mass properties of shake table components (world box, pedestal)
  - **Pedestal Type Selection**: Choose between "box" (simple geometry) or "mesh" (terrain model)
  - **Box Pedestal**: Configurable dimensions, mass, and inertia
  - **Mesh Pedestal**: OBJ file path, scaling factors, mass, and inertia
- **Joint Configuration**: Movement limits, maximum accelerations, maximum efforts (force/torque)
- **Dynamics Properties**: Friction (lateral, rolling, spinning), restitution, contact damping/stiffness
- **Visual Settings**: Colors (RGBA) for world box and pedestal
- **Robot Visual**: Color scheme for the shake table structure

### Simulation Modes

VSR Pro supports two execution modes controlled by the `use_real_time` flag in `config.yaml`:

#### Non-Real-Time Mode (Fast Execution)
```yaml
simulation_settings:
  use_real_time: False  # Run as fast as possible
  simulation_frequency: 500  # Hz
```
- **Performance**: Runs 50-60x faster than real-time
- **Use case**: Rapid testing, batch simulations, data generation
- **Example**: 5-second trajectory completes in ~0.08 seconds

#### Real-Time Mode (Synchronized Execution)
```yaml
simulation_settings:
  use_real_time: True  # Synchronize with wall clock
  simulation_frequency: 500  # Hz
```
- **Performance**: 1:1 synchronization with wall time
- **Use case**: Visualization, hardware-in-the-loop testing, real-time control
- **Example**: 5-second trajectory takes exactly 5 seconds

### Example Configuration

```yaml
simulation_settings:
  gravity: [0, 0, -9.81]
  simulation_frequency: 500           # Simulation frequency in Hz
  GUI_update_frequency: 60            # GUI display update rate in Hz
  enable_graphics: True
  use_real_time: False                # false = fast mode, true = real-time mode

structure:
  world_box:
    dimensions: [50.0, 14.0, 2.0]     # Width, Length, Height
    mass: 0.0                          # Static base
  
  pedestal:
    type: "box"                        # Options: "box" or "mesh"
    absolute_height: 6.0               # Height from ground to center
    
    # Box pedestal configuration
    box:
      dimensions: [10.0, 10.0, 0.5]   # Width, Length, Height
      mass: 2000000.0
      inertia: [1668333333.3, 1668333333.3, 3333333333.3]
    
    # Mesh pedestal configuration (used when type="mesh")
    mesh:
      path: "data/rough_terrain.obj"  # Path to OBJ mesh file
      scaling: [1.0, 1.0, 1.0]        # Scale factors [x, y, z]
      mass: 2000000.0
      inertia: [1668333333.3, 1668333333.3, 3333333333.3]
````
  log_file: "data/simulation.log"    # Log file path

structure:
  world_box:
    dimensions: [10.0, 10.0, 1.0]     # X, Y, Z in meters
    mass: 0                            # 0 = static
  pedestal:
    dimensions: [2.0, 2.0, 0.6]       # X, Y, Z in meters
    mass: 500.0                        # kg

joints:
  prismatic_x:
    limit: [-5.0, 5.0]                # Movement range in meters
    max_acceleration: 20.0            # m/s^2
    max_effort: 100000.0              # Maximum force in N
  spherical:
    max_effort: 50000.0               # Maximum torque in Nm

dynamics:
  lateral_friction: 0.9
  restitution: 0.1
  contact_damping: 10000.0
  contact_stiffness: 1000000.0
```

## Workflow Examples

### Example 1: Interactive GUI - Execute Pulse Trajectory
```bash
1. Run: python GUI_test.py
2. Click "Start Simulation"
3. Configure trajectory parameters:
   - Cycle Number: 3
   - X: Amp=0.01m, Freq=2Hz
   - Y: Amp=0.01m, Freq=2Hz
4. Click "Execute Pulse Trajectory"
5. Monitor real-time position display
6. Data automatically saved to data/logs/run_YYYYMMDD_HHMMSS.npz
```

### Example 2: Interactive GUI - Execute Displacement Trajectory
```bash
1. Generate trajectory:
   cd utils
   python random_disp_traj.py
   
2. Run GUI:
   cd ..
   python GUI_test.py
   
3. Click "Start Simulation"
4. Click "Browse..." in Displacement Trajectory section
5. Select "data/displacement_trajectory.csv"
6. Click "Execute Displacement Trajectory"
7. Review recorded data in data/logs/run_YYYYMMDD_HHMMSS.npz
```

### Example 3: Interactive GUI - Load Object and Visualize
```bash
1. python GUI_test.py
2. Click "Start Simulation"
3. Click "Upload PBR OBJ" (simulation automatically pauses)
4. Browse and select your OBJ file
5. Configure pose and physics properties
6. Click "Apply" to preview
7. Click "Confirm" to finalize (simulation resumes)
8. Execute trajectory - object motion is recorded
```

### Example 4: Python API - Automated Batch Processing
```python
from app import VSRProApp
import time

test_frequencies = [0.5, 1.0, 2.0, 4.0]

for freq in test_frequencies:
    print(f"Testing frequency: {freq} Hz")
    
    # Create and start simulation
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

### Example 5: Python API - Load Object and Execute Trajectory
```python
from app import VSRProApp
import time

app = VSRProApp("config.yaml")
app.start_simulation()
time.sleep(1)

# Load object on pedestal
obj_id = app.load_pbr_object(
    obj_path="data/cube.obj",
    offset=[0.0, 0.0, 0.5],
    mass=10.0,
    restitution=0.3,
    lateral_friction=0.6
)

# Execute displacement trajectory
if app.load_displacement_trajectory("data/trajectory.csv"):
    app.start_displacement_trajectory()
    time.sleep(10.0)  # Wait for completion

app.stop_simulation()
```

### Example 6: Visualize Recorded Data
```bash
cd utils
# Edit plot_npz.py to set path to your NPZ file
python plot_npz.py
```

## Control Modes

### Pulse Trajectory (Velocity Control)
- Uses velocity control for prismatic joints (X, Y, Z)
- Uses PD control with computed torques for spherical joint (Roll, Pitch, Yaw)
- Real-time calculation: v(t) = amp * 2πf * sin(2πft)
- Smooth continuous motion with acceleration control

### Displacement Trajectory (Position Control)
- Uses position control for all 6 DOFs
- Direct array indexing (CSV sampled at 500 Hz)
- Precise trajectory tracking from pre-recorded data
- Suitable for replaying recorded motions or standard earthquake signals

### Hold Position (Zero Velocity Control)
- Active when no trajectory is running
- Maintains current position without drift
- Zero velocity target for prismatic joints
- Zero torque for spherical joint

## Development

### System Architecture

The system follows a modular three-layer architecture with clear separation of concerns:

```
┌─────────────────────────────────────────┐
│       Presentation Layer                │
├─────────────────┬───────────────────────┤
│  GUI_test.py    │  example_automation.py│
│  (PyQt5 UI)     │  (Python Scripts)     │
└────────┬────────┴──────────┬────────────┘
         │                   │
         └─────────┬─────────┘
                   │
         ┌─────────▼──────────┐
         │      app.py        │
         │  (VSRProApp API)   │  ← Business Logic Layer
         └─────────┬──────────┘
                   │
         ┌─────────▼──────────┐
         │ simulation_core.py │  ← Physics Engine Layer
         │  (PyBullet Core)   │
         └────────────────────┘
```

#### 1. **Presentation Layer** (`GUI_test.py`, `example_automation.py`): 
   - **GUI_test.py**: PyQt5 user interface with real-time controls and displays
     - Delegates all business logic to `app.py`
     - Focuses purely on UI concerns (buttons, inputs, labels)
     - Automatically pauses simulation when loading objects
   - **example_automation.py**: Python scripts for headless automation
     - Batch processing capabilities
     - Programmatic control without GUI
     - CI/CD integration ready

#### 2. **Business Logic Layer** (`app.py`): 
   - **VSRProApp API class**: High-level application programming interface
   - **Simulation Control**: start, stop, pause, resume, reset
   - **Trajectory Management**: 
     - Pulse trajectories with sinusoidal motion
     - Displacement trajectories from CSV files
   - **Object Management**: Load and track PBR objects
   - **Data Recording**: Automatic recording to NPZ files
   - **Thread Management**: Handles simulation loop in separate thread
   - **State Management**: Running, paused, trajectory flags
   - **GUI-agnostic**: Can be used by any Python code

#### 3. **Physics Engine Layer** (`simulation_core.py`): 
   - PyBullet physics simulation (500 Hz default)
   - Robot creation with box or mesh pedestal
   - Three control methods:
     - `step_trajectory()`: Velocity control for pulse trajectories
     - `step_displacement_trajectory()`: Position control for displacement trajectories
     - `hold_pedestal_position()`: Zero velocity control when idle
   - Object spawning with PBR properties
   - UTF-8 logging

#### 4. **Configuration Layer** (`config.yaml`): 
   - YAML-based parameter management
   - Simulation settings, structure, joints, dynamics, visuals
   - Shared across all layers

#### 5. **Utility Layer** (`utils/`):
   - Trajectory generation tools
   - Data visualization tools
   - Extensible for custom analysis

### API Reference

See `docs/api_architecture.md` for complete API documentation including:
- Full method reference with parameters
- Usage examples for each API method
- Architecture diagrams
- Migration guide from legacy code
- Best practices for automation

### Key Design Features

- **Separation of Concerns**: UI, business logic, and physics engine are cleanly separated
- **Reusability**: Same API works for GUI and automation scripts
- **Thread Safety**: Daemon thread for simulation loop, main thread for GUI
- **Mutual Exclusion**: Only one trajectory type active at a time
- **Data Integrity**: Automatic recording with timestamped filenames
- **Error Handling**: Comprehensive validation with user-friendly messages
- **Logging**: All operations logged with timestamps to file
- **Extensibility**: Modular design allows easy addition of new features

### Extending the System

- **New GUI Interfaces**: Create new UIs using the `VSRProApp` API
- **Web Interface**: Build REST API or web frontend using `app.py`
- **ROS Integration**: Create ROS nodes that use `VSRProApp`
- **New Trajectory Types**: Add control methods in `SimulationCore` (e.g., `step_custom_trajectory()`)
- **Additional Sensors**: Implement sensor simulation in `SimulationCore.step_simulation()`
- **Custom Objects**: Extend `spawn_obj_on_pedestal()` with new object types
- **Analysis Tools**: Create new scripts in `utils/` for specialized analysis
- **Control Strategies**: Implement new control algorithms (e.g., adaptive control, MPC)



### Performance Notes

- **Non-real-time mode**: Runs much faster than real-time, suitable for batch processing
- **Real-time mode**: 1:1 wall clock synchronization, suitable for visualization and hardware-in-loop
- **Recording overhead**: Minimal performance impact, data recorded at trajectory frequency
- **GUI updates**: 60 Hz default, can be reduced in config for slower systems

### Data Files

**NPZ Format** (`data/logs/run_*.npz`):
```python
data = np.load('data/logs/run_20251013_200721.npz')
times = data['times']                    # Time array (s)
pedestal = data['pedestal_poses']        # Shape: (N, 6) - [x,y,z,roll,pitch,yaw]
obj_1 = data['pbr_object_1_poses']      # Shape: (N, 6) if object loaded
metadata = data['metadata'].item()       # Simulation metadata
```

**Log Format** (`data/simulation.log`):
```
2025-10-13 20:00:00 - GUI - INFO - Simulation started successfully.
2025-10-13 20:00:01 - SimulationCore - INFO - Robot created with ID: 1
2025-10-13 20:00:05 - GUI - INFO - Pulse trajectory execution started.
```

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Documentation

- **API Architecture**: See `docs/api_architecture.md` for detailed API documentation, usage examples, and architecture diagrams
- **PBR Configuration**: See `docs/pbr_configuration.md` for guide on auto-loading PBR objects from config and multi-object tracking
- **Troubleshooting**: See `docs/segmentation_fault.md` for common issues and solutions
- **System Structure**: See `docs/vsr_pro_structure.md` for detailed system architecture

## Migration Guide

If you're using the legacy `GUI.py`, consider migrating to the new API-based architecture:

### For Interactive Users:
- Switch from `python GUI.py` to `python GUI_test.py`
- All functionality is preserved with improved stability
- Simulation automatically pauses when loading objects

### For Developers/Automation:
- Use `app.py` API instead of directly managing `SimulationCore`
- See `example_automation.py` for reference implementations
- Cleaner code with better separation of concerns

## Todo
- grid cosine pipeline
- ground motion batch pipeline