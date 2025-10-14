# VSR_Pro

Virtual Shake Robot Pro (VSR Pro) is an advanced software platform for simulating and controlling a 6-DOF shake table using PyBullet physics simulation and PyQt5 GUI.

## Overview

VSR Pro provides a complete simulation environment for a 6-degree-of-freedom (6-DOF) shake table system. The platform includes real-time physics simulation, trajectory generation, data recording, and an intuitive graphical user interface for interactive control and visualization.

### Key Features

- **6-DOF Shake Table Simulation**: Simulates translational (X, Y, Z) and rotational (Roll, Pitch, Yaw) movements
- **Real-time Physics**: Powered by PyBullet physics engine with 500 Hz simulation frequency
- **Interactive GUI**: PyQt5-based interface for real-time control and visualization with pause/resume capability
- **Dual Trajectory Modes**: 
  - Pulse trajectory: Customizable sine/cosine wave patterns with independent amplitude and frequency control
  - Displacement trajectory: CSV-based pre-recorded trajectories with position control
- **Object Loading**: Support for loading external 3D objects (OBJ format) with PBR materials and collision detection
- **Data Recording**: Automatic recording of pedestal and object poses during trajectory execution saved as NPZ files
- **Comprehensive Logging**: UTF-8 encoded log files with timestamps for all operations
- **YAML Configuration**: Flexible configuration system for simulation parameters, dynamics, and visual settings
- **Utility Scripts**: Tools for trajectory generation and data visualization

## Project Structure

```
VSR_Pro/
├── LICENSE                     # Apache License 2.0
├── README.md                   # This file
├── GUI.py                      # Main GUI application
├── simulation_core.py          # Core simulation engine
├── config.yaml                 # Simulation configuration
├── data/                       # Data storage directory
│   ├── run_*.npz              # Recorded trajectory data
│   ├── simulation.log         # Simulation log file
│   └── displacement_trajectory.csv  # Example displacement trajectory
├── docs/                       # Documentation files
│   ├── segmentation_fault.md  # Troubleshooting guide
│   └── vsr_pro_structure.md   # Architecture documentation
├── utils/                      # Utility scripts
│   ├── random_disp_traj.py    # Random trajectory generator
│   └── plot_npz.py            # Data visualization tool
└── .gitignore                  # Git ignore file
```

### File Descriptions

- **`GUI.py`**: Main application entry point with PyQt5 interface including trajectory controls, object loading dialogs, and real-time visualization
- **`simulation_core.py`**: Core simulation engine managing PyBullet physics, robot creation, joint control (velocity and position), and object spawning
- **`config.yaml`**: Configuration file defining simulation settings (frequency, gravity, real-time mode), structural parameters, dynamics properties, joint limits, and visual appearance
- **`data/`**: Directory for storing recorded trajectory data (NPZ format), simulation logs (UTF-8 encoded), and displacement trajectory CSV files
- **`docs/`**: Documentation and troubleshooting guides
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
   source vsr_env/bin/activate  # On macOS/Linux
   # or
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

Start the GUI application:
```bash
python GUI.py
```

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
- **File Validation**: Automatic validation of sampling frequency (must be 500 Hz) and DOF columns
- **Execute Displacement Trajectory**: Start/stop trajectory execution using position control

CSV Format Requirements:
```csv
time_s,X_m,Y_m,Z_m,Roll_rad,Pitch_rad,Yaw_rad
0.0,0.0001,-0.0095,-0.0014,0.0044,-0.0056,0.0000
0.002,0.0001,-0.0097,-0.0017,0.0045,-0.0057,0.0000
...
```
- `time_s` column is mandatory
- Sampling must be 500 Hz (0.002s intervals)
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
- NPZ file: `displacement_trajectory.npz` (full data with velocity and acceleration)
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

### Example 1: Execute Pulse Trajectory
```bash
1. Run: python GUI.py
2. Click "Start Simulation"
3. Configure trajectory parameters:
   - Cycle Number: 3
   - X: Amp=0.01m, Freq=2Hz
   - Y: Amp=0.01m, Freq=2Hz
4. Click "Execute Pulse Trajectory"
5. Monitor real-time position display
6. Data automatically saved to data/run_YYYYMMDD_HHMMSS.npz
```

### Example 2: Execute Displacement Trajectory
```bash
1. Generate trajectory:
   cd utils
   python random_disp_traj.py
   
2. Run GUI:
   cd ..
   python GUI.py
   
3. Click "Start Simulation"
4. Click "Browse..." in Displacement Trajectory section
5. Select "data/displacement_trajectory.csv"
6. Click "Execute Displacement Trajectory"
7. Review recorded data in data/run_YYYYMMDD_HHMMSS.npz
```

### Example 3: Load Object and Visualize
```bash
1. python GUI.py
2. Click "Start Simulation"
3. Click "Pause Simulation" (recommended for stability)
4. Click "Upload PBR OBJ"
5. Browse and select your OBJ file
6. Configure pose and physics properties
7. Click "Apply" to preview
8. Click "Confirm" to finalize
9. Click "Resume Simulation"
10. Execute trajectory - object motion is recorded
```

### Example 4: Visualize Recorded Data
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

The system follows a modular architecture with clear separation of concerns:

1. **GUI Layer** (`GUI.py`): 
   - PyQt5 user interface with real-time controls
   - State management (running, paused, trajectory flags)
   - CSV validation and loading
   - Data recording coordination
   - Threaded simulation loop

2. **Simulation Layer** (`simulation_core.py`): 
   - PyBullet physics simulation (500 Hz)
   - Robot creation and joint control
   - Three control methods:
     - `step_trajectory()`: Velocity control for pulse trajectories
     - `step_displacement_trajectory()`: Position control for displacement trajectories
     - `hold_pedestal_position()`: Zero velocity control when idle
   - Object spawning with PBR properties
   - UTF-8 logging

3. **Configuration Layer** (`config.yaml`): 
   - YAML-based parameter management
   - Simulation settings, structure, joints, dynamics, visuals
   - Shared between GUI and simulation core

4. **Utility Layer** (`utils/`):
   - Trajectory generation tools
   - Data visualization tools
   - Extensible for custom analysis

### Key Design Features

- **Thread Safety**: Daemon thread for simulation loop, main thread for GUI
- **Mutual Exclusion**: Only one trajectory type active at a time
- **Data Integrity**: Automatic recording with timestamped filenames
- **Error Handling**: Comprehensive validation with user-friendly messages
- **Logging**: All operations logged with timestamps to file
- **Extensibility**: Modular design allows easy addition of new features

### Extending the System

- **New Trajectory Types**: Add control methods in `SimulationCore` (e.g., `step_custom_trajectory()`)
- **Additional Sensors**: Implement sensor simulation in `SimulationCore.step_simulation()`
- **Custom Objects**: Extend `spawn_obj_on_pedestal()` with new object types
- **Analysis Tools**: Create new scripts in `utils/` for specialized analysis
- **Control Strategies**: Implement new control algorithms (e.g., adaptive control, MPC)


## Troubleshooting

### Common Issues

**Issue**: Displacement trajectory CSV file rejected with frequency mismatch error
**Solution**: Ensure CSV is sampled at exactly 500 Hz (0.002s time steps). Use `random_disp_traj.py` to generate compatible files or resample your data to 500 Hz.

**Issue**: Cannot execute displacement trajectory while pulse trajectory is running
**Solution**: This is by design (mutual exclusion). Stop the pulse trajectory first, then execute displacement trajectory.

**Issue**: Pedestal position jumps to origin when simulation starts
**Solution**: System now reads initial position on startup. If issue persists, check that `hold_pedestal_position()` is being called correctly when trajectories are not active.

**Issue**: GUI freezes during trajectory execution
**Solution**: GUI updates are on main thread, simulation on separate thread. If freeze occurs, check for blocking operations in GUI thread. Reduce `GUI_update_frequency` in config if needed.

**Issue**: Object falls through pedestal
**Solution**: Collision is explicitly enabled between objects and pedestal (link 3). Check mass and contact properties in object physics settings.

**Issue**: Log file has encoding errors
**Solution**: All logging uses UTF-8 encoding. Ensure your text editor supports UTF-8 when viewing `data/simulation.log`.

### Performance Notes

- **Non-real-time mode**: Runs 50-60x faster than real-time, suitable for batch processing
- **Real-time mode**: 1:1 wall clock synchronization, suitable for visualization and hardware-in-loop
- **Recording overhead**: Minimal performance impact, data recorded at trajectory frequency
- **GUI updates**: 60 Hz default, can be reduced in config for slower systems

### Data Files

**NPZ Format** (`data/run_*.npz`):
```python
data = np.load('data/run_20251013_200721.npz')
times = data['times']                    # Time array (s)
pedestal = data['pedestal_poses']        # Shape: (N, 6) - [x,y,z,roll,pitch,yaw]
obj_1 = data['pbr_object_1_poses']      # Shape: (N, 6) if object loaded
```

**Log Format** (`data/simulation.log`):
```
2025-10-13 20:00:00 - GUI - INFO - Simulation started successfully.
2025-10-13 20:00:01 - SimulationCore - INFO - Robot created with ID: 1
2025-10-13 20:00:05 - GUI - INFO - Pulse trajectory execution started.
```

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.



