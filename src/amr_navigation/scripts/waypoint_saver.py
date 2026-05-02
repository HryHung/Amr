#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import json
import os

class WaypointSaver(Node):
    def __init__(self):
        super().__init__('waypoint_saver')
        # Lắng nghe dữ liệu mũi tên bạn vẽ trên RViz
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.pose_callback,
            10)
        
        # File cơ sở dữ liệu thu nhỏ
        self.file_path = os.path.expanduser('~/Documents/amr_ws/waypoints.json')
        self.waypoints = self.load_waypoints()
        
        self.get_logger().info('✅ Đã sẵn sàng! Hãy dùng mũi tên "2D Goal Pose" trên RViz để chọn điểm...')

    def load_waypoints(self):
        """Tải dữ liệu cũ nếu file đã tồn tại"""
        if os.path.exists(self.file_path):
            with open(self.file_path, 'r') as f:
                return json.load(f)
        return {}

    def pose_callback(self, msg):
        """Hàm này tự động kích hoạt khi bạn thả chuột trên RViz"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        self.get_logger().info(f'\n🎯 BẮT ĐƯỢC TỌA ĐỘ MỚI: X={x:.2f}, Y={y:.2f}')
        
        # Chờ người dùng nhập tên trên Terminal
        name = input("👉 Bạn muốn đặt tên điểm này là gì? (Ví dụ: Trạm 1). Bỏ trống để hủy: ").strip()

        if name:
            self.waypoints[name] = {'x': x, 'y': y, 'qz': qz, 'qw': qw}
            # Ghi vào file JSON
            with open(self.file_path, 'w', encoding='utf-8') as f:
                json.dump(self.waypoints, f, indent=4, ensure_ascii=False)
            print(f"💾 Đã lưu thành công điểm '{name}' vào Database!")
        else:
            print("❌ Đã hủy điểm vừa chọn.")
        
        print("\n------------------------------------------------")
        print("Tiếp tục chọn điểm khác trên RViz nếu bạn muốn...")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()