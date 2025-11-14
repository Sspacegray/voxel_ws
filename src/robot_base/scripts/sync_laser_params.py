#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import time

class LaserParamSyncer(Node):
    """
    激光雷达参数同步器
    监听laser_calibration_node的参数变化，同步到dynamic_laser_tf_publisher
    """
    
    def __init__(self):
        super().__init__('laser_param_syncer')
        
        # 创建服务客户端
        self.tf_publisher_client = self.create_client(
            SetParameters, 
            '/dynamic_laser_tf_publisher/set_parameters'
        )
        
        # 等待服务可用
        self.get_logger().info('Waiting for dynamic_laser_tf_publisher service...')
        self.tf_publisher_client.wait_for_service(timeout_sec=10.0)
        
        # 参数列表
        self.param_names = [
            'front_right_laser.position.x',
            'front_right_laser.position.y', 
            'front_right_laser.position.z',
            'front_right_laser.orientation.roll',
            'front_right_laser.orientation.pitch',
            'front_right_laser.orientation.yaw',
            'rear_left_laser.position.x',
            'rear_left_laser.position.y',
            'rear_left_laser.position.z', 
            'rear_left_laser.orientation.roll',
            'rear_left_laser.orientation.pitch',
            'rear_left_laser.orientation.yaw',
        ]
        
        # 创建定时器，定期检查参数变化
        self.timer = self.create_timer(0.5, self.sync_parameters)
        self.last_params = {}
        
        self.get_logger().info('Laser parameter syncer started.')
    
    def sync_parameters(self):
        """同步参数"""
        try:
            # 获取当前参数值
            current_params = {}
            for param_name in self.param_names:
                try:
                    param_value = self.get_parameter(param_name).value
                    current_params[param_name] = param_value
                except:
                    # 参数可能还没有声明
                    continue
            
            # 检查是否有参数变化
            changed_params = []
            for param_name, value in current_params.items():
                if param_name not in self.last_params or self.last_params[param_name] != value:
                    changed_params.append((param_name, value))
            
            # 如果有参数变化，同步到TF发布器
            if changed_params:
                self.sync_to_tf_publisher(changed_params)
                self.last_params = current_params.copy()
                
        except Exception as e:
            # 忽略错误，继续运行
            pass
    
    def sync_to_tf_publisher(self, changed_params):
        """同步参数到TF发布器"""
        if not self.tf_publisher_client.service_is_ready():
            return
        
        # 构建参数列表
        parameters = []
        for param_name, value in changed_params:
            param = Parameter()
            param.name = param_name
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(value)
            parameters.append(param)
        
        # 发送参数更新请求
        request = SetParameters.Request()
        request.parameters = parameters
        
        future = self.tf_publisher_client.call_async(request)
        
        self.get_logger().info(f'Synced {len(changed_params)} parameters to TF publisher')

def main(args=None):
    rclpy.init(args=args)
    node = LaserParamSyncer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
