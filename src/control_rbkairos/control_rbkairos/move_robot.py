   #!/usr/bin/env python3
"""
RBKairos Robot Movement Controller
This script demonstrates how to control the RBKairos robot through cmd_vel topic
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        
        # Publisher for robot movement commands (ros2_control topic)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for movement control  
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Movement parameters
        self.start_time = time.time()
        self.movement_phase = 0
        
        print('\n🤖 === RBKAIROS CONTROLLER STARTED (3D Mesh) ===')
        print('📡 Publishing to: /cmd_vel')
        print('✅ Using Gazebo Ignition DiffDrive Plugin')
        print('🤖 Robot: Real 3D Mesh Model with Sensors')
        print('🎮 Movement sequence:')
        print('   1️⃣  เดินหน้า 2 เมตร')
        print('   2️⃣  เลี้ยวซ้าย + ถอยหลัง 2 เมตร')
        print('   3️⃣  เลี้ยวขวา + เดินหน้า 2 เมตร')
        print('⏰ Starting NOW...\n')

    def timer_callback(self):
        msg = Twist()
        
        # Get elapsed time
        elapsed_time = time.time() - self.start_time
        
        # Movement sequence: forward 2m → turn left + backward 2m → turn right + forward 2m
        # Assuming speed 0.5 m/s: 2m takes 4 seconds
        if elapsed_time < 4.0:
            # Phase 1: เดินหน้า 2 เมตร (0-4 seconds at 0.5 m/s)
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            if self.movement_phase != 1:
                print('🏃 Phase 1: เดินหน้า 2 เมตร (forward 2m)')
                self.movement_phase = 1
                
        elif elapsed_time < 5.5:
            # Phase 2a: เลี้ยวซ้าย 90 degrees (4-5.5 seconds)
            msg.linear.x = 0.0
            msg.angular.z = 1.0  # Turn left
            if self.movement_phase != 2:
                print('↩️  Phase 2: เลี้ยวซ้าย (turn left)')
                self.movement_phase = 2
                
        elif elapsed_time < 9.5:
            # Phase 2b: ถอยหลัง 2 เมตร (5.5-9.5 seconds at -0.5 m/s)
            msg.linear.x = -0.5  # Backward
            msg.angular.z = 0.0
            if self.movement_phase != 3:
                print('🔙 Phase 2: ถอยหลัง 2 เมตร (backward 2m)')
                self.movement_phase = 3
                
        elif elapsed_time < 11.0:
            # Phase 3a: เลี้ยวขวา 90 degrees (9.5-11.0 seconds)
            msg.linear.x = 0.0
            msg.angular.z = -1.0  # Turn right
            if self.movement_phase != 4:
                print('↪️  Phase 3: เลี้ยวขวา (turn right)')
                self.movement_phase = 4
                
        elif elapsed_time < 15.0:
            # Phase 3b: เดินหน้า 2 เมตร (11.0-15.0 seconds at 0.5 m/s)
            msg.linear.x = 0.5  # Forward
            msg.angular.z = 0.0
            if self.movement_phase != 5:
                print('🏃 Phase 3: เดินหน้า 2 เมตร (forward 2m)')
                self.movement_phase = 5
                
        else:
            # Phase 4: Stop (after 15 seconds)
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if self.movement_phase != 6:
                print('🛑 Phase 4: หยุด (stopped)\n')
                self.movement_phase = 6
        
        # Publish the movement command
        self.pub.publish(msg)
        
        # Debug output every 2 seconds
        if int(elapsed_time * 5) % 10 == 0:  # Every 2 seconds at 10Hz
            print(f'📡 CMD: linear.x={msg.linear.x:.1f}, angular.z={msg.angular.z:.1f} (t={elapsed_time:.1f}s)')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MoveRobot()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down robot controller...")
    finally:
        # Stop the robot before shutting down
        if 'node' in locals():
            stop_msg = Twist()  # All velocities default to 0
            node.pub.publish(stop_msg)
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
