# VSR_Pro

Virtual Shake Robot Pro (VSR Pro) is an advanced software platform for simulating and controlling a 6-DOF shake table using PyBullet physics simulation and PyQt5 GUI.

## Overview

VSR Pro provides a complete simulation environment for a 6-degree-of-freedom (6-DOF) shake table system. The platform includes real-time physics simulation, trajectory generation, data logging capabilities, and an intuitive graphical user interface for interactive control and visualization.

### Key Features

- **6-DOF Shake Table Simulation**: Simulates translational (X, Y, Z) and rotational (Roll, Pitch, Yaw) movements
- **Real-time Physics**: Powered by PyBullet physics engine with configurable parameters
- **Interactive GUI**: PyQt5-based interface for real-time control and visualization
- **Trajectory Generation**: Customizable sine/cosine wave patterns with independent amplitude and frequency control for each DOF
- **Object Loading**: Support for loading external 3D objects (OBJ format) with PBR materials and collision detection
- **Data Logging**: Comprehensive logging of simulation data including joint positions, velocities, and object states
- **YAML Configuration**: Flexible configuration system for simulation parameters, dynamics, and visual settings

## Project Structure

```
VSR_Pro/
├── LICENSE                 # Apache License 2.0
├── README.md               # This file
├── GUI.py                  # Main GUI application
├── simulation_core.py      # Core simulation engine
├── config.yaml             # Simulation configuration
├── data/                   # Data storage directory
├── docs/                   # Documentation files
└── .gitignore              # Git ignore file
```

### File Descriptions

- **`GUI.py`**: Main application entry point containing the PyQt5 GUI implementation with real-time controls, object loading dialogs, and visualization interface
- **`simulation_core.py`**: Core simulation engine that manages PyBullet physics, robot creation, joint control, and object spawning
- **`config.yaml`**: Configuration file defining simulation settings, structural parameters, dynamics properties, and visual appearance
- **`data/`**: Directory for storing simulation data and logs
- **`docs/`**: Documentation and additional reference materials

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
   pip install pybullet
   pip install PyQt5
   pip install PyYAML
   pip install numpy
   ```

   Or install all dependencies at once:
   ```bash
   pip install pybullet PyQt5 PyYAML numpy
   ```

### Required Dependencies

- **PyBullet** (`pybullet`): Physics simulation engine
- **PyQt5** (`PyQt5`): GUI framework for the user interface
- **PyYAML** (`PyYAML`): YAML configuration file parsing
- **Standard Libraries**: `sys`, `time`, `math`, `threading`, `csv`, `typing`

## Usage

### Running the Application

   ```bash
   python GUI.py
   ```

### GUI Controls

The main interface provides the following functionality:

- **Trajectory Control**: Configure amplitude and frequency for each DOF (X, Y, Z, Roll, Pitch, Yaw)
- **Simulation Control**: Start/stop simulation, real-time parameter adjustment
- **Object Loading**: Load external 3D objects (.obj files) with material and collision properties
- **Live Preview**: Real-time visualization of trajectory execution
- **Data Export**: Save simulation data for analysis

### Configuration

Modify `config.yaml` to customize:

- **Simulation Settings**: Time step, gravity, GUI enable/disable, real-time mode
- **Structural Parameters**: Dimensions and mass properties of shake table components
- **Joint Configuration**: Movement limits, force constraints, and joint types
- **Dynamics Properties**: Friction, restitution, contact parameters
- **Visual Settings**: Colors and appearance of simulation elements

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
  simulation_frequency: 500  # Simulation frequency in Hz
  enable_graphics: True
  use_real_time: False  # false = fast mode, true = real-time mode

joints:
  prismatic_x:
    limit: [-5.0, 5.0]
    max_acceleration: 20.0
```

## Development

### System Architecture

The system follows a modular architecture:

1. **GUI Layer** (`GUI.py`): User interface and interaction handling
2. **Simulation Layer** (`simulation_core.py`): Physics simulation and robot control
3. **Configuration Layer** (`config.yaml`): Parameter management

### Extending the System

- **New Trajectory Types**: Extend trajectory functionality within `SimulationCore` class with custom motion patterns
- **Additional Sensors**: Add sensor simulation in `SimulationCore`
- **Custom Objects**: Implement new object types with specific physics properties
- **Analysis Tools**: Extend data analysis capabilities for specialized analysis


## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.



