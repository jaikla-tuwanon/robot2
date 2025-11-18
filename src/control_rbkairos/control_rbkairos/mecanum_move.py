#!/usr/bin/env python3
"""
RBKairos Mecanum Drive Controller
Interactive Manual Control: Step-by-step movement with Enter key
Step 1: Forward 2m
Step 2: Backward 2m
Step 3: Left strafe 2m
Step 4: Right strafe 2m
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time
import threading

class MecanumMoveRobot(Node):
    def __init__(self):
        super().__init__('mecanum_move_robot')
        
        # Publisher for mecanum drive controller
        self.pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        
        # Movement parameters
        self.linear_speed = 0.5  # m/s
        self.distance = 2.0  # meters
        self.duration = self.distance / self.linear_speed  # seconds
        
        self.is_moving = False
        self.start_time = 0.0
        self.current_direction = None
        
        print('\n🤖 === RBKAIROS MECANUM DRIVE CONTROLLER ===')
        print('📡 Publishing to: /cmd_vel')
        print('✅ Using ros2_control mecanum_drive_controller')
        print('🎮 Interactive Manual Control Mode')
        print(f'⚙️  Settings: Speed={self.linear_speed} m/s, Distance={self.distance} m')
        print('\n' + '='*50)
        print('🎯 MOVEMENT SEQUENCE:')
        print('  1️⃣  กด Enter → ไปข้างหน้า (Forward) 2m')
        print('  2️⃣  กด Enter → ถอยหลัง (Backward) 2m')
        print('  3️⃣  กด Enter → สไลด์ซ้าย (Left Strafe) 2m')
        print('  4️⃣  กด Enter → สไลด์ขวา (Right Strafe) 2m')
        print('='*50 + '\n')

    def move_robot(self, direction_name, linear_x, linear_y):
        """Move robot in specified direction for the set duration"""
        self.current_direction = direction_name
        self.is_moving = True
        self.start_time = time.time()
        
        print(f'\n🚀 {direction_name}...')
        
        # Keep publishing commands while moving
        while time.time() - self.start_time < self.duration:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_footprint"
            msg.twist.linear.x = linear_x
            msg.twist.linear.y = linear_y
            msg.twist.angular.z = 0.0
            self.pub.publish(msg)
            time.sleep(0.05)  # 20 Hz update rate
        
        # Stop the robot
        self.stop_robot()
        elapsed = time.time() - self.start_time
        print(f'✅ {direction_name} เสร็จแล้ว! (ใช้เวลา {elapsed:.1f} วินาที)')
        self.is_moving = False

    def stop_robot(self):
        """Send stop command to robot"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.angular.z = 0.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumMoveRobot()
    
    # Run ROS2 spin in separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        # Step 1: Forward
        input('📍 ขั้นตอนที่ 1/4: กด Enter เพื่อไปข้างหน้า 2m... ')
        node.move_robot('⬆️  ไปข้างหน้า (Forward)', 0.5, 0.0)
        
        # Step 2: Backward
        input('\n📍 ขั้นตอนที่ 2/4: กด Enter เพื่อถอยหลัง 2m... ')
        node.move_robot('⬇️  ถอยหลัง (Backward)', -0.5, 0.0)
        
        # Step 3: Left strafe
        input('\n📍 ขั้นตอนที่ 3/4: กด Enter เพื่อสไลด์ซ้าย 2m... ')
        node.move_robot('⬅️  สไลด์ซ้าย (Left Strafe)', 0.0, -0.5)
        
        # Step 4: Right strafe
        input('\n📍 ขั้นตอนที่ 4/4: กด Enter เพื่อสไลด์ขวา 2m... ')
        node.move_robot('➡️  สไลด์ขวา (Right Strafe)', 0.0, 0.5)
        
        print('\n' + '='*50)
        print('🎉 ทำภารกิจครบทั้ง 4 ขั้นตอนแล้ว!')
        print('='*50 + '\n')
        
    except KeyboardInterrupt:
        print("\n\n🛑 ยกเลิกการทำงาน...")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
