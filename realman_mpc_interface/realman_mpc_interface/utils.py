import numpy as np

def generate_traj(start_pose, goal_pose, start_time, goal_time, timestep):
    # ... existing code ...
    
    time_trajectory = np.arange(start_time, goal_time, timestep)
    num_steps = len(time_trajectory)
    
    # Separate translation and quaternion from start and goal poses
    start_translation = start_pose[:3]
    start_quaternion = start_pose[3:]
    goal_translation = goal_pose[:3]
    goal_quaternion = goal_pose[3:]
    
    # Interpolate translation
    translation_trajectory = np.zeros((num_steps, 3))
    for i in range(3):
        translation_trajectory[:, i] = np.linspace(start_translation[i], goal_translation[i], num_steps)
    
    # Interpolate quaternion
    quaternion_trajectory = np.zeros((num_steps, 4))
    for t in range(num_steps):
        frac = t / (num_steps - 1)
        quaternion_trajectory[t] = np.array(start_quaternion) * (1 - frac) + np.array(goal_quaternion) * frac
        quaternion_trajectory[t] /= np.linalg.norm(quaternion_trajectory[t])
    
    state_trajectory = np.hstack((translation_trajectory, quaternion_trajectory))
    
    return time_trajectory, state_trajectory