from launch import LaunchDescription #引入launch描述信息接口
from launch_ros.actions import Node #引入节点功能

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = '', #功能包名
            executable= '',#可执行文件名
        ),
    ])