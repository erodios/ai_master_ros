#!/usr/bin/env python3
import rclpy
# Lifecycle imports
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
import tf2_ros
from math import pi, cos, sin
from threading import Thread

from std_srvs.srv import Empty

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
from rclpy.time import Time
from math import atan2, degrees


class TableExplorer(Node):
    def __init__(self):
        super().__init__('tt_table_explorer')      

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')
        t = Thread(target=self.activate_thread)
        t.start()

        return super().on_activate(state)
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        return TransitionCallbackReturn.SUCCESS

    def transform_to_pose(self, transform_stamped, goal):
        self.get_logger().info("Transforming to pose")
        pose_stamped = PoseStamped()
        pose_stamped.header = transform_stamped.header
        pose_stamped.pose.position.z = transform_stamped.transform.translation.z
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        if goal == 1:
            pose_stamped.pose.position.x = transform_stamped.transform.translation.x + 0.1 # +0.1
            pose_stamped.pose.position.y = transform_stamped.transform.translation.y + 0.8 # +0.8 20/20
            angle = atan2(transform_stamped.transform.translation.y - pose_stamped.pose.position.y , transform_stamped.transform.translation.x -  pose_stamped.pose.position.x)
            pose_stamped.pose.orientation.z = sin(angle/2)
            pose_stamped.pose.orientation.w = cos(angle/2)
        elif goal == 2:
            pose_stamped.pose.position.x = transform_stamped.transform.translation.x + 0.1 # +0.1
            pose_stamped.pose.position.y = transform_stamped.transform.translation.y - 0.45 # -0.45 20/20 (μου εβγαλε και 13/20)
            angle = atan2(transform_stamped.transform.translation.y - pose_stamped.pose.position.y , transform_stamped.transform.translation.x -  pose_stamped.pose.position.x)
            pose_stamped.pose.orientation.z = sin(angle/2)
            pose_stamped.pose.orientation.w = cos(angle/2)
        elif goal == 3:
            pose_stamped.pose.position.x = transform_stamped.transform.translation.x + 0.80  # 0.85
            pose_stamped.pose.position.y = transform_stamped.transform.translation.y - 0.15  # - 0.15 20/20
            pose_stamped.pose.orientation.z = sin(2*pi/4)
            pose_stamped.pose.orientation.w = cos(2*pi/4)  
        else:
            pose_stamped.pose.position.x = transform_stamped.transform.translation.x - 0.80 # -0.8
            pose_stamped.pose.position.y = transform_stamped.transform.translation.y - 0.15  #  -0.15  20/20
            angle = atan2(transform_stamped.transform.translation.y - pose_stamped.pose.position.y , transform_stamped.transform.translation.x -  pose_stamped.pose.position.x)
            pose_stamped.pose.orientation.z = sin(angle/2)
            pose_stamped.pose.orientation.w = cos(angle/2)
  
        return pose_stamped

    def activate_thread(self):
        self.request = Empty.Request()
        self.cli = self.create_client(Empty, '/tt_umpire/assignment2/i_feel_confident')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            rclpy.spin_once(self)
        self.get_logger().info('service available, continuing...')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        while (not self.tf_buffer.can_transform("map", "tt_table_boundary_1_link", Time())):
            self.get_logger().info("Waiting for transform...")
            rclpy.spin_once(self, timeout_sec=1.0)

        goal1 = self.tf_buffer.lookup_transform("map", "tt_table_boundary_1_link", Time())
        goal2 = self.tf_buffer.lookup_transform("map", "tt_table_boundary_2_link", Time())
        goal3 = self.tf_buffer.lookup_transform("map", "tt_table_boundary_3_link", Time())
        goal4 = self.tf_buffer.lookup_transform("map", "tt_table_boundary_4_link", Time())

        self.get_logger().info(f"Goal 1: {goal1}")
        self.get_logger().info(f"Goal 2: {goal2}")
        self.get_logger().info(f"Goal 3: {goal3}")
        self.get_logger().info(f"Goal 4: {goal4}")

        goTo1 = self.transform_to_pose(goal1, 1)
        goTo2 = self.transform_to_pose(goal2, 2)
        goTo3 = self.transform_to_pose(goal3, 3)
        goTo4 = self.transform_to_pose(goal4, 4)

        self.move_to(goTo1)
        response = self.send_request()
        time.sleep(1)
        self.move_to(goTo2)
        response = self.send_request()
        time.sleep(1)
        self.move_to(goTo3)
        response = self.send_request()
        time.sleep(1)
        self.move_to(goTo4)
        response = self.send_request()
        time.sleep(1)

    def get_the_angle(self,goal):
        while not self.tf_buffer.can_transform('map', 'base_link', Time()):
            self.get_logger().info("Waiting for transform...")
            rclpy.spin_once(self, timeout_sec=1.0)
        position = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        robot_x = position.transform.translation.x
        robot_y = position.transform.translation.y
        robot_z = position.transform.translation.z
        goal_x = goal.transform.translation.x
        goal_y = goal.transform.translation.y
        angle = atan2(goal_y - robot_y, goal_x - robot_x)
        # x = 0.0
        # y = 0.0
        # z = sin(angle / 2)
        # w = cos(angle / 2)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = robot_x
        pose_stamped.pose.position.y = robot_y
        pose_stamped.pose.position.z = robot_z
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = sin(angle / 2)
        pose_stamped.pose.orientation.w = cos(angle / 2)
        self.navigator.goToPose(pose_stamped)
        while not self.navigator.isTaskComplete():
            time.sleep(1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    def move_to(self, pose_stamped):
        self.navigator.goToPose(pose_stamped)
        while not self.navigator.isTaskComplete():
            time.sleep(1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    def send_request(self):
        self.get_logger().info('Sending service request...')
        self.future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    node = TableExplorer()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()