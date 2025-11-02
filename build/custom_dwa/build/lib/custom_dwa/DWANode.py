#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from .DWAPlanner import DWAPlanner
from .DWAConfig import DWAConfig
from .DWALogging import DWALogging
import logging

class DWANode(Node):
    """
        Custom Dynamic Window Approach (DWA) planner for ROS2 Turtlebot3
    """
    def __init__(self, nodeName = 'custom_dwa_node'):
        super().__init__(nodeName)
        config = DWAConfig()

        useSimTimeParam = rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([useSimTimeParam])

        # Setup Debug Logging #
        self.get_logger().set_level(logging.DEBUG)
        logging.root = DWALogging(self)

        # Setup Planner #
        self.dwaPlanner = DWAPlanner(config, logging.getLogger())
        self.tolGoal = config.tolGoal

        # Empty Init #
        self.currentPose = None
        self.currentTwist = None
        self.laserData = None
        self.goalPose = None
        self.goalQueue = []

        qos = QoSProfile(
                reliability = ReliabilityPolicy.RELIABLE,
                history = HistoryPolicy.KEEP_LAST,
                depth = 10
            )

        # sub to /odom 
        self.odomSub = self.create_subscription(Odometry, '/odom', self.odom_cb, qos)

        # sub to /scan
        self.laserSub = self.create_subscription(LaserScan, '/scan', self.laser_cb, qos)

        # sub to /goal for receiving goal from RViz
        self.goalSub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, qos)
        self.rvizGoalSub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_cb, qos)

        # pub to /cmd_vel 
        self.cmdVelPub = self.create_publisher(Twist, '/cmd_vel', qos)

        # pub trajs for visualization at /dwa_trajs
        self.trajVisPub = self.create_publisher(MarkerArray, '/dwa_trajs', qos)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.timer = self.create_timer(config.dt, self.control_loop)
        self.get_logger().info("Custom DWA Initialization Complete")

    def odom_cb(self, msg):
        """ Callback for /odom updates """
        self.currentPose = msg.pose.pose
        self.currentTwist = msg.twist.twist

    def laser_cb(self, msg):
        """ Callback for /scan updates """
        self.laserData = msg

    def goal_cb(self, msg):
        """ Callback for goal input, can queue multiple """
        self.get_logger().info(f"Recieved goal: x = {msg.pose.position.x:.2f}, y = {msg.pose.position.y:.2f}")
        self.goalQueue.append(msg.pose)

        if self.goalPose is None:
            self.goalPose = self.goalQueue.pop(0)

    def control_loop(self):
        """ Control-Loop called by ROS2 Timer, executes best traj computed with DWA """
        
        # Busy Control Loop #
        if not self.is_ready():
            return
        
        # Goal Reached # 
        if self.is_goal_reached():
            self.stop_robot()
            self.get_logger().info("Goal reached!")

            if self.goalQueue:
                self.goalPose = self.goalQueue.pop(0)
                self.get_logger().info(f"Fetching next goal from queue: x = {self.goalPose.position.x:.2f}, y = {self.goalPose.position.y:.2f}")
            else:
                self.goalPose = None # All goals achieved

            return
        
        self.get_logger().warn(f'laserData: {self.laserData}')
        # Sample DWA Commands #
        bestCmd, trajArray, bestCmdId = self.dwaPlanner.dwa_plan(
                    self.currentTwist, self.currentPose,
                    self.goalPose, self.laserData
                )
        
        self.vis_trajs(trajArray, bestCmd, bestCmdId)
        
        if bestCmd is not None:
            cmdMsg = Twist()
            cmdMsg.linear.x, cmdMsg.angular.z = bestCmd
            self.cmdVelPub.publish(cmdMsg)
        else:
            self.stop_robot()
            self.get_logger().warn("No safe traj found - stopping robot")

    def is_ready(self):
        """ Check if all required sensor inputs and goals are available """
        return all([
            self.currentPose is not None,
            self.currentTwist is not None,
            self.goalPose is not None,
            self.laserData is not None,
        ])

    def is_goal_reached(self):
        """ Check if the goal input is within tolerance """ 
        if self.goalPose is None or self.currentPose is None:
            return False
        dx = self.currentPose.position.x - self.goalPose.position.x
        dy = self.currentPose.position.y - self.goalPose.position.y
        return math.sqrt( dx*dx + dy*dy ) < self.tolGoal

    def stop_robot(self):
        """ Stop Robot Immediately """
        self.cmdVelPub.publish(Twist())

    def vis_trajs(self, trajs, bestCmd, bestCmdId):
        """ Publish sampled trajectories for visualization in RVIZ """
        markerArray = MarkerArray()
        for i, (traj, cost) in enumerate(trajs):
            marker = Marker()
            marker.header.frame_id = "base_link" 
            marker.header.stamp = self.get_clock().now().to_msg() 
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.scale.x = 0.02
            marker.lifetime = rclpy.time.Duration(seconds=1.0).to_msg() 

            if bestCmd is not None and i == bestCmdId:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                if cost == float('inf'):
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                else:
                    norm = min(cost / 10.0, 1.0)
                    marker.color.r = norm
                    marker.color.g = 0.0
                    marker.color.b = 1.0 - norm

            for x, y in traj:
                p = Point()
                p.x, p.y = x, y
                p.z = 0.0
                marker.points.append(p)
            markerArray.markers.append(marker)
        self.trajVisPub.publish(markerArray)



    
 
    

        
