import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformListener, Buffer
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import time

class CloudTransformer(Node):
    def __init__(self):
        super().__init__('cloud_transformer')
        
        # 定义源话题和目标话题
        # /cloud_registered 通常是 Fast-LIO 的输出，在 camera_init 帧
        self.target_frame = 'map'
        self.source_topic = '/cloud_registered'
        self.target_topic = '/cloud_registered_map'
        
        self.subscription = self.create_subscription(
            PointCloud2,
            self.source_topic,
            self.listener_callback,
            10)
            
        self.publisher = self.create_publisher(PointCloud2, self.target_topic, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(f'🚀 点云变换节点启动: {self.source_topic} -> {self.target_topic} (目标帧: {self.target_frame})')

    def listener_callback(self, msg):
        try:
            # 检查是否有可用的TF变换
            # 使用 rclpy.time.Time() 获取最新变换
            if self.tf_buffer.can_transform(self.target_frame, msg.header.frame_id, rclpy.time.Time()):
                trans = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, rclpy.time.Time())
                
                # 执行变换
                cloud_out = do_transform_cloud(msg, trans)
                
                # 发布变换后的点云
                self.publisher.publish(cloud_out)
            else:
                # 变换不可用时，不打印过多日志，避免刷屏
                pass
                
        except Exception as e:
            self.get_logger().warn(f'变换失败: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CloudTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
