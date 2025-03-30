#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('kyron_nav_wf')
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.declare_parameter('waypoints', [])
        self.send_waypoints()


    def define_waypoints(self):
        waypoints = [
            (10.15, -0.77),
            (17.86, -0.77),
            (21.58, -3.5)
        ]
        
        #Lista de goals
        goal_poses = []
        
        #Por cada punto en waypoints
        for point in waypoints:
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            
            goal_pose.pose.position.x = point[0]
            
            goal_pose.pose.position.y = point[1]
            
            goal_pose.pose.orientation.w = 1.0
            goal_poses.append(goal_pose)
            
        return goal_poses

    def send_waypoints(self):
        self.client.wait_for_server()
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.define_waypoints()
        self.send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Waypoint following goal was rejected!')
            return
        self.get_logger().info('Waypoint following goal accepted!')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        current_waypoint = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Executing waypoint: {current_waypoint + 1}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation completed successfully!')


def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
