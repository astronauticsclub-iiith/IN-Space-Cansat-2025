import mujoco
import mujoco.viewer
import numpy as np
import time
from scipy.spatial.transform import Rotation

# --- Simulation & Physical Parameters ---
INITIAL_ALTITUDE = 1000.0
AIR_DENSITY_SEA_LEVEL = 1.225
SCALE_HEIGHT = 8500.0
WIND_FORCE = np.array([0.2, 0.1, 0.0])
DISTURBANCE_TORQUE = np.array([0.02, -0.01, 0.015])

# --- FIX: Realistic Drag Parameters for a Hexagonal Parachute ---
# Parachute area corresponding to a diameter of ~0.5m
PARACHUTE_AREA = 0.2 
# Realistic drag coefficient for a standard CanSat parachute
CD_PARACHUTE = 1.4  

# --- Orientation Control Parameters ---
KP_ori = 0.5
KD_vel = 0.1

def apply_forces(model, data):
    """Calculates and applies realistic drag and wind forces."""
    # --- 1. Calculate Drag Force ---
    air_density = AIR_DENSITY_SEA_LEVEL * np.exp(-data.qpos[2] / SCALE_HEIGHT)
    world_vel = data.qvel[:3]
    velocity_mag = np.linalg.norm(world_vel)

    if velocity_mag < 1e-6:
        world_drag_force = np.zeros(3)
    else:
        # Drag is always opposite to the direction of velocity
        drag_direction = -world_vel / velocity_mag
        # Use the standard drag equation with realistic parameters
        drag_force_magnitude = 0.5 * air_density * (velocity_mag**2) * CD_PARACHUTE * PARACHUTE_AREA
        world_drag_force = drag_direction * drag_force_magnitude

    # --- 2. Apply Forces ---
    # Gravity is handled by the MuJoCo integrator. We apply the external forces.
    data.xfrc_applied[1, :3] = world_drag_force + WIND_FORCE
    data.xfrc_applied[1, 3:] = DISTURBANCE_TORQUE

def orientation_hold_control(model, data):
    """PID Controller to hold a target orientation (0,0,0)."""
    quat_mujoco = data.qpos[3:7]
    quat_scipy = [quat_mujoco[1], quat_mujoco[2], quat_mujoco[3], quat_mujoco[0]]
    r = Rotation.from_quat(quat_scipy)
    yaw, pitch, roll = r.as_euler('zyx', degrees=False)
    
    orientation_error = -np.array([roll, pitch, yaw])
    p_term = KP_ori * orientation_error
    
    world_ang_vel = data.qvel[3:6]
    R_mat = data.xmat[1].reshape(3,3)
    local_ang_vel = R_mat.T @ world_ang_vel
    d_term = -KD_vel * local_ang_vel

    control_torque_local = p_term + d_term
    data.ctrl[0] = -control_torque_local[0] # Roll motor
    data.ctrl[1] = -control_torque_local[1] # Pitch motor
    data.ctrl[2] = -control_torque_local[2] # Yaw motor

# --- Main Simulation Execution ---
if __name__ == "__main__":
    try:
        model = mujoco.MjModel.from_xml_path('cansat_model.xml')
    except Exception as e:
        print(f"Error loading 'cansat_model.xml': {e}")
        exit()

    data = mujoco.MjData(model)
    data.qpos[2] = INITIAL_ALTITUDE

    print("--- CanSat Final Descent Simulation (Realistic Parameters) ---")
    print(f"--- Parachute Cd = {CD_PARACHUTE}, Area = {PARACHUTE_AREA} m^2 ---")
    print("-" * 85)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        last_print_time = -1.0
        
        while viewer.is_running() and data.qpos[2] > 0.2:
            if data.time >= last_print_time + 1.0:
                speed = np.linalg.norm(data.qvel[:3])
                vertical_speed = data.qvel[2]
                quat = data.qpos[3:7]
                r = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])
                yaw, pitch, roll = r.as_euler('zyx', degrees=True)
                
                print(f"Time: {data.time:5.1f}s | Alt: {data.qpos[2]:6.1f}m | Speed: {speed:5.1f}m/s | Vz: {vertical_speed:5.1f}m/s | "
                      f"Yaw:{yaw:5.1f}° Pitch:{pitch:5.1f}° Roll:{roll:5.1f}°")
                last_print_time = data.time

            apply_forces(model, data)
            orientation_hold_control(model, data)
            
            mujoco.mj_step(model, data)
            viewer.cam.lookat[:3] = data.qpos[:3]
            viewer.sync()
            
            time.sleep(0.01)

        print("-" * 85)
        print("--- Simulation Finished ---")