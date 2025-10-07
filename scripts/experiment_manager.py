#!/usr/bin/env python3

import math, json
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from simulation_msgs.srv import Reset
from agents_msgs.msg import AgentArray
from tf_transformations import euler_from_quaternion
from robotsnap.utils.config import load_ros2_package_config, get_package_src_directory, RosJsonEncoder

class ExperimentManager(Node):
    def __init__(self):
        super().__init__('experiment_manager')

        self.config_name = "config.yaml"
        self.config = load_ros2_package_config("robotsnap", "config/" + self.config_name)

        self.log_name = self.config.log_name
        if not self.log_name or self.log_name.strip() == "":
            self.log_name = "experiment_log.json"

        src_dir = get_package_src_directory('robotsnap', 'robotsnap_pkg')
        data_dir = src_dir / 'data'
        data_dir.mkdir(parents=True, exist_ok=True)
        self.file_path = data_dir / 'experiment_log.json'
        self.file = open(self.file_path, "w")

        self.prefix = self.config.prefix

        self.num_trials = self.config.num_trials
        self.current_trial = 0
        self.max_trial_time = self.config.max_trial_time
        self.trial_start_time = None
        self.dataset = self.config.dataset
        self.min_human = self.config.min_human
        self.max_human = self.config.max_human
        self.goal_radius = self.config.goal_radius

        self.create_subscription(PoseStamped, self.prefix + '/robot_pose', self._robot_pose_callback, 1)
        self.create_subscription(PoseStamped, self.prefix + '/global_goal', self._final_goal_callback, 1)
        self.create_subscription(Path, self.prefix + '/global_path', self.global_path_callback, 10)

        self.create_subscription(AgentArray, self.prefix + '/agents', self.agents_callback, 10)
        self.create_subscription(AgentArray, self.prefix + '/agents/global', self.agents_global_callback, 10)

        self.reset_client = self.create_client(Reset, self.prefix + self.config.ros.reset_service)
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().fatal('Waiting reset service...')

        self.init()
        self.timer = self.create_timer(0.25, self.timer_callback)

    def init(self):
        self.trial_start_time = self.get_clock().now()
        self.header = Header()
        self.curr_pose = None
        self.final_goal = None
        self.global_path = None
        self.agents = None
        self.agents_global = None
        self.current_trial_logs = []
        
        req = Reset.Request()
        req.dataset = self.dataset
        req.min_human = self.min_human
        req.max_human = self.max_human

        
        future = self.reset_client.call_async(req)
        future.add_done_callback(self._after_reset)

    def _after_reset(self, future):
        self.wait_for_pose_timer = self.create_timer(0.5, self._wait_for_pose)
        
    def _wait_for_pose(self):
        if self.curr_pose is not None and self.final_goal is not None:
            self.wait_for_pose_timer.cancel()

    def timer_callback(self):
        self.header.stamp = self.get_clock().now().to_msg()

        if self.curr_pose is None or self.final_goal is None or self.global_path is None:
            return  # encore en attente d'init

        self.info = {
            "goal": [self.final_goal.x, self.final_goal.y],
            "robot_pose": self.curr_pose,
            "agents": self.agents,
            "agents_global": self.agents_global,
            "secs": self.header.stamp.sec,
            "nsecs": self.header.stamp.nanosec,
            }
        
        self.current_trial_logs.append(self.info)
        
        # --- Condition 1 : goal ---
        goal_reached = self.dist_to_goal() <= self.goal_radius

        # --- Condition 2 : time ---
        if self.trial_start_time is not None:
            elapsed = (self.get_clock().now() - self.trial_start_time).nanoseconds * 1e-9
        else:
            elapsed = 0.0

        time_exceeded = elapsed >= self.max_trial_time

        if goal_reached or time_exceeded:
            reason = "goal" if goal_reached else f"time ({elapsed:.1f}s)"
            self.get_logger().info(f"Trial {self.current_trial + 1} done (reason : {reason})")

            self.write_episode_logs()
            self.current_trial += 1

            if self.current_trial < self.num_trials:
                self.init()
            else:
                self.get_logger().info("All trials done.")
                self.file.close()
                rclpy.shutdown()
        
    def _robot_pose_callback(self, robot_pose_msg):
        self.curr_pose = robot_pose_msg.pose

    def _final_goal_callback(self, final_goal_msg):
        self.final_goal = final_goal_msg.pose.position

    def global_path_callback(self, msg):
        self.global_path = msg

    def agents_callback(self, msg: AgentArray):
        self.agents = []
        for i in range(len(msg.agents)):
            agent = msg.agents[i]
            x = agent.pose.position.x
            y = agent.pose.position.y
            vx = agent.velocity.linear.x
            vy = agent.velocity.linear.y
            dist = math.sqrt(x**2 + y**2)
            self.agents.append(np.array((x, y, vx, vy, dist), dtype=np.float32))

    def agents_global_callback(self, msg: AgentArray):
        self.agents_global = []

        for i in range(len(msg.agents)):
            agent = msg.agents[i]
            x = agent.pose.position.x
            y = agent.pose.position.y
            vx = agent.velocity.linear.x
            vy = agent.velocity.linear.y
            q = agent.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            _, _, yaw = euler_from_quaternion(quat)
            theta = math.degrees(yaw)
            self.agents_global.append([x, y, theta, vx, vy])

    def write_episode_logs(self):
        for entry in self.current_trial_logs:
            self.file.write(json.dumps(entry, cls=RosJsonEncoder) + "\n")
        self.file.flush()

    def dist_to_goal(self, init_pose=None):
        total_distance = 0.0
        poses = self.global_path.poses
        if init_pose is not None:
            poses[0] = init_pose

        for i in range(len(poses) - 1):
            x1 = poses[i].pose.position.x
            y1 = poses[i].pose.position.y
            z1 = poses[i].pose.position.z

            x2 = poses[i+1].pose.position.x
            y2 = poses[i+1].pose.position.y
            z2 = poses[i+1].pose.position.z

            dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
            total_distance += dist

        return total_distance

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentManager()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
