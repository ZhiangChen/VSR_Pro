"""
example_automation.py

Example script demonstrating automated VSR Pro simulation workflow.
Shows how to use app.py API without GUI for batch processing and automation.

Usage:
    python example_automation.py
"""

from app import VSRProApp
import time


def main():
    print("=" * 80)
    print("VSR Pro Automation Example")
    print("=" * 80)
    
    # Initialize the application
    print("\n1. Initializing application...")
    app = VSRProApp("config.yaml")
    
    # Start simulation
    print("\n2. Starting simulation...")
    if not app.start_simulation():
        print("ERROR: Failed to start simulation")
        return
    
    time.sleep(5)  # Wait for simulation to stabilize
    print("   Simulation started successfully")
    
    # Load a PBR object (example - adjust path and parameters as needed)
    print("\n3. Loading PBR object...")
    # Note: Update this path to point to an actual .obj file
    # obj_id = app.load_pbr_object(
    #     obj_path="data/objects/test_cube.obj",
    #     offset=[0.0, 0.0, 0.5],  # 0.5m above pedestal
    #     orientation_euler=[0.0, 0.0, 0.0],
    #     mesh_scale=[1.0, 1.0, 1.0],
    #     mass=10.0,
    #     inertia=[0.1, 0.1, 0.1],  # [Ixx, Iyy, Izz] in kg·m²
    #     restitution=0.3,
    #     lateral_friction=0.6
    # )
    # 
    # if obj_id is not None:
    #     print(f"   Object loaded successfully (ID: {obj_id})")
    # else:
    #     print("   WARNING: Could not load object (check path)")
    
    # Execute a pulse trajectory
    print("\n4. Configuring pulse trajectory...")
    app.set_pulse_trajectory(
        cycle_number=2,
        amp_x=1.0,      # 1 m amplitude in X
        freq_x=1.0,     # 1 Hz frequency
        amp_yaw=0.05,   # 0.05 rad (~2.9°) yaw amplitude
        freq_yaw=1.0    # 1 Hz frequency
    )
    print("   Trajectory configured: 2 cycles, X translation + Yaw rotation")
    
    print("\n5. Starting trajectory execution...")
    if app.start_pulse_trajectory():
        print("   Trajectory started, recording data...")
        
        # Wait for trajectory to complete (2 cycles at 1 Hz = 2 seconds + margin)
        duration = 2.0 / 1.0  # cycles / frequency
        time.sleep(duration + 1.0)
        
        print("   Trajectory completed")
    else:
        print("   ERROR: Failed to start trajectory")
    
    # Check simulation status
    print("\n6. Querying simulation status...")
    pose = app.get_pedestal_pose()
    if pose:
        print(f"   Pedestal Position: X={pose['position'][0]:.4f}, Y={pose['position'][1]:.4f}, Z={pose['position'][2]:.4f}")
        print(f"   Pedestal Orientation: Roll={pose['orientation'][0]:.4f}, Pitch={pose['orientation'][1]:.4f}, Yaw={pose['orientation'][2]:.4f}")
    
    # Reset simulation
    print("\n7. Resetting simulation...")
    if app.reset_simulation():
        print("   Simulation reset successfully")
        time.sleep(1)
    
    # Example: Execute displacement trajectory from CSV
    print("\n8. Testing displacement trajectory...")
    # Note: This requires a CSV file with proper format
    # csv_path = "data/trajectories/test_displacement.csv"
    # if app.load_displacement_trajectory(csv_path):
    #     print(f"   Loaded displacement trajectory from {csv_path}")
    #     
    #     if app.start_displacement_trajectory():
    #         print("   Displacement trajectory started...")
    #         # Wait for completion (adjust based on trajectory length)
    #         time.sleep(5.0)
    #         print("   Displacement trajectory completed")
    # else:
    #     print("   Could not load displacement trajectory (check path)")
    
    # Stop simulation
    print("\n9. Stopping simulation...")
    if app.stop_simulation():
        print("   Simulation stopped successfully")
    
    print("\n" + "=" * 80)
    print("Automation complete!")
    print("Check data/logs/ for recorded trajectory data (.npz files)")
    print("=" * 80)


def example_batch_processing():
    """
    Example of batch processing multiple configurations.
    """
    print("\n" + "=" * 80)
    print("Batch Processing Example")
    print("=" * 80)
    
    # Define test cases
    test_cases = [
        {"name": "Low Frequency", "freq": 0.5, "amp": 0.05},
        {"name": "Medium Frequency", "freq": 1.0, "amp": 0.10},
        {"name": "High Frequency", "freq": 2.0, "amp": 0.15},
    ]
    
    for i, test in enumerate(test_cases):
        print(f"\nTest {i+1}/{len(test_cases)}: {test['name']}")
        print(f"  Frequency: {test['freq']} Hz, Amplitude: {test['amp']} m")
        
        # Initialize
        app = VSRProApp("config.yaml")
        
        # Start simulation
        if not app.start_simulation():
            print("  ERROR: Failed to start simulation")
            continue
        
        time.sleep(1)
        
        # Configure and run trajectory
        app.set_pulse_trajectory(
            cycle_number=3,
            amp_x=test['amp'],
            freq_x=test['freq']
        )
        
        app.start_pulse_trajectory()
        
        # Wait for completion
        duration = 3.0 / test['freq']
        time.sleep(duration + 1.0)
        
        # Stop simulation
        app.stop_simulation()
        
        print(f"  Test {i+1} completed, data saved to data/logs/")
        
        time.sleep(2)  # Brief pause between tests
    
    print("\n" + "=" * 80)
    print("Batch processing complete!")
    print("=" * 80)


if __name__ == "__main__":
    # Run basic automation example
    main()
    
    # Uncomment to run batch processing example
    # example_batch_processing()
