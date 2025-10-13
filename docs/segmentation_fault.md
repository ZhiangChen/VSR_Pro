# Segmentation Fault 

Segmentation faults encountered in the VSR_Pro GUI were caused by **multi-threaded access to the PyBullet C++ engine** while the Metal-based GUI renderer was active. On macOS, PyBullet's GUI mode is not thread-safe, any simultaneous calls to `p.stepSimulation()` or joint control APIs from different threads can cause **race conditions** in the physics and rendering backends, leading to native segmentation faults.

## Examples of the Issue

### Example 1 — Start Simulation Crash
When clicking **"Start Simulation,"** the program created a background thread (`run_simulation`) that repeatedly called `p.stepSimulation()`, while the PyBullet GUI rendering ran on the Qt main thread. Because Metal rendering requires all GPU calls to occur on the same thread, these simultaneous PyBullet calls led to a segmentation fault.

### Example 2 — Execute Trajectory Crash
After fixing the first crash, clicking **"Execute Trajectory"** caused another segmentation fault. The trajectory logic launched a separate worker thread that used PyBullet APIs such as `setJointMotorControl` and `setJointMotorControlMultiDof` while the GUI's QTimer continued calling `p.stepSimulation()`. This again caused concurrent access to the PyBullet engine, producing the same thread-safety violation.

## Solution
I have tested the following solutions. The segmentation fault could be solved, but new issues are created. Therefore, I have decided to not support MacOS. 

- Replaced all background simulation and trajectory threads with a **Qt `QTimer`** that executes at 240 Hz.
- The timer's callback (`on_tick()`) now performs both `p.stepSimulation()` and incremental trajectory updates via a new **`execute_trajectory_tick()`** method in `SimulationCore`.
- The new method mirrors the original joint-driven trajectory behavior (joints 0–2: prismatic; joint 3: spherical) but is non-blocking and does not call `stepSimulation()` or sleep.
- All PyBullet API calls now include the correct `physicsClientId` and execute on a single thread managed by the Qt event loop.
