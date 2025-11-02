#!/usr/bin/env python3
"""
DWA Local Planner - 10xConstruction Assignment

The following should implement Dynamic Window Approach (DWA) [local planner] for navigation.
It subscribes to /scan, /odom topics as requested and publish velocity commands to /cmd_vel 
based on cost-optimized constrained trajectory predictons. Trajectories should be visible in RViz.
"""

import sys
import rclpy
from .DWANode import DWANode

def main(args=None):
    "ROS2 EntryPoint"
    
    rclpy.init(args=args)
    plannerNode = DWANode('custom_dwa_node')

    try:
        rclpy.spin(plannerNode)
    except KeyboardInterrupt:
        print("KeyboardInterrupt raised, exiting custom_DWA")
        sys.exit(1)
    finally:
        plannerNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
