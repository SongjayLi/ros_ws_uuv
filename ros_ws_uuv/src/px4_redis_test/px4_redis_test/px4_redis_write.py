import rclpy
import redis
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleAttitude,VehicleLocalPosition


class RedisWrite(Node):

    def __init__(self) -> None:
        super().__init__('redis_write')# 传递节点名称到父类构造函数

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)

        self.vehicle_Local_Position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_Local_Position = VehicleLocalPosition()
        self.r = redis.Redis(host='localhost', port=6379, decode_responses=True)

    def vehicle_attitude_callback(self,vehicle_attitude):
        self.vehicle_attitude = vehicle_attitude
        #self.r.hset()
        self.r.zadd("vehicle_attitude",{','.join(str(i) for i in vehicle_attitude.q)+','+str(vehicle_attitude.timestamp):vehicle_attitude.timestamp})
        #self.r.lpush("list1",float(vehicle_attitude.q[0]))
        #self.r.lpush("list2",float(vehicle_attitude.q[1]))
        #self.r.lpush("list3",float(vehicle_attitude.q[2]))
        #self.r.lpush("list4",float(vehicle_attitude.q[3]))

    def vehicle_local_position_callback(self,vehicle_local_position):
        self.vehicle_Local_Position = vehicle_local_position
        #self.r.hset()
        self.r.zadd("vehicle_Local_Position",{str(vehicle_local_position.x)+str(vehicle_local_position.y)+str(vehicle_local_position.z)+str(vehicle_local_position.vx)+str(vehicle_local_position.vy)+str(vehicle_local_position.vz)+str(vehicle_local_position.timestamp):vehicle_local_position.timestamp})
        #self.r.lpush("list1",float(vehicle_attitude.q[0]))
        #self.r.lpush("list2",float(vehicle_attitude.q[1]))
        #self.r.lpush("list3",float(vehicle_attitude.q[2]))
        #self.r.lpush("list4",float(vehicle_attitude.q[3]))

def main(args=None) -> None:
    print('Starting redis_write_test node...')
    rclpy.init(args=args)
    rediswrite = RedisWrite()
    rclpy.spin(rediswrite)
    rediswrite.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)