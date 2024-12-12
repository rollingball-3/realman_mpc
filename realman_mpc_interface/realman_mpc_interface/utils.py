import numpy as np
import bisect
from typing import List

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


def interpolate_trajectory(timestamps, values, query_time):
    """
    在轨迹点之间进行线性插值
    
    Args:
        timestamps: 时间序列
        values: 对应的值序列
        query_time: 查询时间点
    
    Returns:
        插值后的值
    """
    if query_time <= timestamps[0]:
        return values[0]
    if query_time >= timestamps[-1]:
        return values[-1]
    
    # 找到查询时间所在的区间
    idx = bisect.bisect_right(timestamps, query_time) - 1
    
    # 计算插值
    t0, t1 = timestamps[idx], timestamps[idx + 1]
    v0, v1 = values[idx], values[idx + 1]
    alpha = (query_time - t0) / (t1 - t0)
    return [v0[i] + alpha * (v1[i] - v0[i]) for i in range(len(v0))]

def find_nearest_timestamp_index(timestamps: List[int], current_time: int) -> int:
    """
    Find the index of the timestamp in the list that is closest to the given current time.
    
    Args:
        timestamps (List[int]): A list of timestamps in increasing order.
        current_time (int): The current time for which the nearest timestamp is to be found.
        
    Returns:
        int: The index of the timestamp in the list that is closest to the current time.
    """
    # Use bisect_left to find the index of the first timestamp greater than or equal to current_time
    insertion_point = bisect.bisect_left(timestamps, current_time)
    
    # If the insertion point is 0, return 0 as there are no timestamps smaller than current_time
    if insertion_point == 0:
        return 0
    
    # If the insertion point is len(timestamps), return len(timestamps) - 1 as there are no timestamps greater than current_time
    if insertion_point == len(timestamps):
        return len(timestamps) - 1
    
    # Otherwise, check which timestamp is closer to current_time and return the corresponding index
    if current_time - timestamps[insertion_point - 1] < timestamps[insertion_point] - current_time:
        return insertion_point - 1
    else:
        return insertion_point
