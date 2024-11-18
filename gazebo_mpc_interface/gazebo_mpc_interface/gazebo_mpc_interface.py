import rclpy
from rclpy.node import Node
from ocs2_msgs.msg import MpcFlattenedController, MpcObservation, MpcTargetTrajectories, MpcState, MpcInput
from ocs2_msgs.srv import Reset
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.clock import Clock 
import threading
import numpy as np

import bisect
from typing import List

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


def send_mpc_reset_request(arm_name):
    node = rclpy.create_node('mpc_reset_client')
    cli = node.create_client(Reset, f'{arm_name}_mpc_reset')
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f'Service {arm_name}_mpc_reset not available, waiting again...')

    req = Reset.Request()
    req.reset = True
    mpcstate = MpcState(value=[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    mpcinput = MpcInput(value=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    target_traj = MpcTargetTrajectories(time_trajectory=[0.0], state_trajectory=[mpcstate], input_trajectory = [mpcinput])
    req.target_trajectories = target_traj

    try:
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        if response.done:
            node.get_logger().info(f'MPC Reset successful for {arm_name}')
        else:
            node.get_logger().error(f'MPC Reset failed for {arm_name}')
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')
    finally:
        node.destroy_node()

class GazeboMpcInterface(Node):
    def __init__(self, arm_name, mpc_freq = 100, control_freq = 400):
        super().__init__(f'{arm_name}_gazebo_mpc_message_convert')

        self.mpc_freq = mpc_freq
        self.control_freq = control_freq

        # MPC Observation Publisher
        self.mpc_observation_pub = self.create_publisher(MpcObservation, f'{arm_name}_mpc_observation', 10)
        self.mpc_observation = MpcObservation()
        self.mpc_observation_lock = threading.Lock()
        self.mpc_timer = self.create_timer(1.0/self.mpc_freq, self.mpc_timer_callback)

        # Joint States Subscriber
        self.joint_states_sub = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)

        # MPC Policy Subscriber
        self.mpc_policy_sub = self.create_subscription(MpcFlattenedController, f'{arm_name}_mpc_policy', self.mpc_policy_callback, 10)
        self.mpc_policy = MpcFlattenedController()

        # Velocity Control Command Publisher
        self.velocity_command_pub = self.create_publisher(Float64MultiArray, '/joint_velocity_controller/commands', 10)
        self.control_timer = self.create_timer(1.0/self.control_freq, self.control_timer_callback)

        self.mpc_init()
        self.get_logger().info('Gazebo MPC Interface Node Started')

    def mpc_init(self):
        self.current_input = np.zeros(7)
        self.mpc_observation.input.value = list(self.current_input)
        self.start_time = self.get_clock().now()

    def get_elapsed_time(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds * 1e-9
        return float(elapsed_time)

    def mpc_timer_callback(self):
        with self.mpc_observation_lock:
            if len(self.mpc_observation.state.value) > 0:
                self.mpc_observation_pub.publish(self.mpc_observation)

    def joint_states_callback(self, msg:JointState):
        with self.mpc_observation_lock:
            self.mpc_observation.time = self.get_elapsed_time()

            # joint_state里面的数据不一定按照顺序的
            if 'joint7' in msg.name:
                joint_names = ['joint1', 'joint2', 'joint3',
                            'joint4', 'joint5', 'joint6', 'joint7']
            else:
                joint_names = ['joint1', 'joint2', 'joint3',
                            'joint4', 'joint5', 'joint6']
            self.mpc_observation.state.value = [msg.position[msg.name.index(joint_name)] for joint_name in joint_names]

    def mpc_policy_callback(self, msg:MpcFlattenedController):
        self.mpc_policy = msg

    def control_timer_callback(self):

        current_time = self.get_elapsed_time()
        if len(self.mpc_policy.input_trajectory) > 0:
            index = find_nearest_timestamp_index(self.mpc_policy.time_trajectory, current_time)
            current_input = self.mpc_policy.input_trajectory[index].value

            velocity_command = Float64MultiArray()
            velocity_command.data = list(current_input)
            with self.mpc_observation_lock:
                self.mpc_observation.input.value = list(current_input)
            self.velocity_command_pub.publish(velocity_command)


def main(args=None):
    rclpy.init(args=args)
    send_mpc_reset_request("mobile_manipulator")
    node = GazeboMpcInterface("mobile_manipulator")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
