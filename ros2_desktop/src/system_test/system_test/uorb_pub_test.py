#####
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleAttitude,VehicleLocalPosition,ActuatorMotors,VehicleAngularVelocity,BatteryStatus

import random
import numpy as np

class UORBPubTestNode(Node):
    def __init__(self):
        super().__init__('uorb_pub_test')# 传递节点名称到父类构造函数
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 创建所有发布器
        self.attitude_pub = self.create_publisher(VehicleAttitude, '/fmu/out/vehicle_attitude', qos_profile)
        self.position_pub = self.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position', qos_profile)
        self.actuator_pub = self.create_publisher(ActuatorMotors, '/fmu/out/actuator_motors', qos_profile)
        self.angular_vel_pub = self.create_publisher(VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity', qos_profile)
        self.battery_pub = self.create_publisher(BatteryStatus, '/fmu/out/battery_status', qos_profile)

        # 创建10ms定时器
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        # 初始化随机数生成器
        np.random.seed(42)

    def timer_callback(self):
        timestamp = self.get_clock().now().nanoseconds // 1000  # 转换为微秒
        
        # 生成VehicleAttitude测试数据
        attitude = VehicleAttitude()
        attitude.timestamp = timestamp
        # 生成归一化的有效四元数
        q = np.random.normal(size=4)
        q /= np.linalg.norm(q)
        attitude.q = q.tolist()  # 格式为[w, x, y, z]

        # 生成VehicleLocalPosition测试数据
        position = VehicleLocalPosition()
        position.timestamp = timestamp
        position.x = random.uniform(-10, 10)
        position.y = random.uniform(-10, 10)
        position.z = random.uniform(-10, 10)
        position.vx = random.uniform(-5, 5)
        position.vy = random.uniform(-5, 5)
        position.vz = random.uniform(-5, 5)
        
        # 生成ActuatorMotors测试数据
        actuator = ActuatorMotors()
        actuator.timestamp = timestamp
        # 前6通道随机数，后6通道NaN
        actuator.control = np.concatenate([
            np.random.uniform(-1, 1, 6),
            np.full(6, np.nan)
        ]).tolist()  # 12通道电机控制信号
        
        # 生成VehicleAngularVelocity测试数据
        angular_vel = VehicleAngularVelocity()
        angular_vel.timestamp = timestamp
        angular_vel.xyz = np.random.uniform(-2, 2, 3).tolist()
        
        # 生成BatteryStatus测试数据
        battery = BatteryStatus()
        battery.timestamp = timestamp
        battery.voltage_v = random.uniform(0.0, 1.0)  # 剩余电量百分比
        
        # 发布所有消息
        self.attitude_pub.publish(attitude)
        self.position_pub.publish(position)
        self.actuator_pub.publish(actuator)
        self.angular_vel_pub.publish(angular_vel)
        self.battery_pub.publish(battery)

def main(args=None):
    rclpy.init(args=args)
    node = UORBPubTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)