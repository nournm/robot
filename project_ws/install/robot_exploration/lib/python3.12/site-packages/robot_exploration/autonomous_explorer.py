#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Header
import numpy as np
import random
import time
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        # Parameters
        self.declare_parameter('explore_rate', 2.0)
        self.declare_parameter('min_frontier_distance', 0.5)
        self.declare_parameter('max_goal_distance', 5.0)
        self.declare_parameter('robot_radius', 0.3)
        
        self.explore_rate = self.get_parameter('explore_rate').value
        self.min_frontier_distance = self.get_parameter('min_frontier_distance').value
        self.max_goal_distance = self.get_parameter('max_goal_distance').value
        self.robot_radius = self.get_parameter('robot_radius').value
        
        # State variables
        self.map_data = None
        self.map_info = None
        self.current_goal = None
        self.exploring = False
        self.goal_sent = False
        
        # ROS2 components
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF2 for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for exploration
        self.exploration_timer = self.create_timer(
            1.0 / self.explore_rate,
            self.exploration_callback
        )
        
        self.get_logger().info('Autonomous Explorer Node Started')
        self.get_logger().info('Waiting for map data...')
    
    def map_callback(self, msg):
        """Process incoming map data"""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info
        
        if not self.exploring:
            self.get_logger().info('Map received. Starting exploration...')
            self.exploring = True
    
    def find_frontiers(self):
        """Find frontier points (boundaries between known and unknown areas)"""
        if self.map_data is None:
            return []
        
        frontiers = []
        height, width = self.map_data.shape
        
        # Find cells that are free (0) and adjacent to unknown (-1)
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if self.map_data[y, x] == 0:  # Free space
                    # Check 8-connected neighbors for unknown space
                    neighbors = [
                        self.map_data[y-1:y+2, x-1:x+2]
                    ]
                    
                    if np.any(self.map_data[y-1:y+2, x-1:x+2] == -1):  # Unknown neighbor
                        # Convert grid coordinates to world coordinates
                        world_x = x * self.map_info.resolution + self.map_info.origin.position.x
                        world_y = y * self.map_info.resolution + self.map_info.origin.position.y
                        frontiers.append((world_x, world_y))
        
        return frontiers
    
    def filter_frontiers(self, frontiers):
        """Filter frontiers to remove those too close together"""
        if not frontiers:
            return []
        
        filtered = []
        for frontier in frontiers:
            too_close = False
            for existing in filtered:
                distance = np.sqrt((frontier[0] - existing[0])**2 + (frontier[1] - existing[1])**2)
                if distance < self.min_frontier_distance:
                    too_close = True
                    break
            
            if not too_close:
                filtered.append(frontier)
        
        return filtered
    
    def get_robot_position(self):
        """Get current robot position"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
        except Exception as e:
            self.get_logger().warn(f'Could not get robot position: {e}')
            return None
    
    def select_best_frontier(self, frontiers):
        """Select the best frontier to explore"""
        robot_pos = self.get_robot_position()
        if not robot_pos or not frontiers:
            return None
        
        # Filter frontiers by maximum distance
        valid_frontiers = []
        for frontier in frontiers:
            distance = np.sqrt((frontier[0] - robot_pos[0])**2 + (frontier[1] - robot_pos[1])**2)
            if distance <= self.max_goal_distance:
                valid_frontiers.append((frontier, distance))
        
        if not valid_frontiers:
            # If no close frontiers, pick the closest one anyway
            distances = [(f, np.sqrt((f[0] - robot_pos[0])**2 + (f[1] - robot_pos[1])**2)) for f in frontiers]
            if distances:
                return min(distances, key=lambda x: x[1])[0]
            return None
        
        # Select frontier with good balance of distance and exploration value
        # For now, just pick a random close frontier to ensure good coverage
        return random.choice(valid_frontiers)[0]
    
    def send_navigation_goal(self, target_x, target_y):
        """Send navigation goal to Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'Sending goal: ({target_x:.2f}, {target_y:.2f})')
        
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
        self.current_goal = (target_x, target_y)
        self.goal_sent = True
        return True
    
    def goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.goal_sent = False
            return
        
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle navigation goal result"""
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.goal_sent = False
        self.current_goal = None
    
    def exploration_callback(self):
        """Main exploration logic"""
        if not self.exploring or self.map_data is None:
            return
        
        # Don't send new goal if one is already active
        if self.goal_sent:
            return
        
        # Find frontiers
        frontiers = self.find_frontiers()
        
        if not frontiers:
            self.get_logger().info('No frontiers found. Exploration may be complete!')
            return
        
        # Filter and select best frontier
        filtered_frontiers = self.filter_frontiers(frontiers)
        
        if not filtered_frontiers:
            self.get_logger().info('No valid frontiers after filtering')
            return
        
        target_frontier = self.select_best_frontier(filtered_frontiers)
        
        if target_frontier:
            self.send_navigation_goal(target_frontier[0], target_frontier[1])
        else:
            self.get_logger().warn('Could not select target frontier')

def main(args=None):
    rclpy.init(args=args)
    
    explorer = AutonomousExplorer()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    