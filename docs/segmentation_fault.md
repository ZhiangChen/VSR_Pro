# Segmentation Fault 

Segmentation faults encountered in the VSR_Pro GUI were caused by **multi-threaded access to the PyBullet C++ engine** while the Metal-based GUI renderer was active. On macOS, PyBullet's GUI mode is not thread-safe, any simultaneous calls to `p.stepSimulation()` or joint control APIs from different threads can cause **race conditions** in the physics and rendering backends, leading to native segmentation faults.

## Examples of the Issue

### Example 1 — Start Simulation Crash
When clicking **"Start Simulation,"** the program created a background thread (`run_simulation`) that repeatedly called `p.stepSimulation()`, while the PyBullet GUI rendering ran on the Qt main thread. Because Metal rendering requires all GPU calls to occur on the same thread, these simultaneous PyBullet calls led to a segmentation fault.

### Example 2 — Execute Trajectory Crash
After fixing the first crash, clicking **"Execute Trajectory"** caused another segmentation fault. The trajectory logic launched a separate worker thread that used PyBullet APIs such as `setJointMotorControl` and `setJointMotorControlMultiDof` while the GUI's QTimer continued calling `p.stepSimulation()`. This again caused concurrent access to the PyBullet engine, producing the same thread-safety violation.

## Solution
The root cause—concurrent multi-threaded access—was resolved by consolidating all PyBullet calls on the **Qt main thread**:

- Replaced all background simulation and trajectory threads with a **Qt `QTimer`** that executes at 240 Hz.
- The timer's callback (`on_tick()`) now performs both `p.stepSimulation()` and incremental trajectory updates via a new **`execute_trajectory_tick()`** method in `SimulationCore`.
- The new method mirrors the original joint-driven trajectory behavior (joints 0–2: prismatic; joint 3: spherical) but is non-blocking and does not call `stepSimulation()` or sleep.
- All PyBullet API calls now include the correct `physicsClientId` and execute on a single thread managed by the Qt event loop.

## Result
These changes unified the simulation and control logic into a single-threaded model, eliminating race conditions and restoring full stability on macOS. Both the simulation and trajectory execution now behave identically to their previous implementations but are **thread-safe and crash-free**.

## Summary of Fixes
| Component | Root Cause | Fix | Status |
|------------|-------------|-----|--------|
| Start Simulation | Background thread calling `p.stepSimulation()` | Replaced with QTimer-based main-thread stepping | Fixed |
| Execute Trajectory | Separate thread issuing PyBullet control calls | Integrated control into main-thread `on_tick()` | Fixed |

---

These modifications ensure that PyBullet, Qt, and Metal rendering operate harmoniously under a **single-threaded execution model**, providing reliable and stable GUI-based simulation across all macOS environments.