#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import json
import os
import sys
import select

def create_pose(navigator, x, y, qz, qw):
    """Hàm tạo tọa độ (Pose) cho Nav2 dùng trực tiếp Quaternion"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = float(qz)
    pose.pose.orientation.w = float(qw)
    return pose

def get_waypoint_name(user_input, point_names):
    """Xử lý việc người dùng nhập số thứ tự HOẶC tên trạm"""
    user_input = user_input.strip()
    if user_input.isdigit():
        idx = int(user_input) - 1
        if 0 <= idx < len(point_names):
            return point_names[idx]
    else:
        for name in point_names:
            if user_input.lower() == name.lower():
                return name
    return None

def check_for_cancel():
    """Hàm đọc bàn phím liên tục (non-blocking) để chờ lệnh 'cancel'"""
    if sys.stdin in select.select([sys.stdin], [], [], 0.0)[0]:
        line = sys.stdin.readline().strip().lower()
        if line == 'cancel':
            return True
    return False

def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("⏳ Đang chờ hệ thống Nav2 khởi động...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 đã sẵn sàng!")

    # 1. ĐỌC CƠ SỞ DỮ LIỆU JSON
    file_path = os.path.expanduser('~/Documents/amr_ws/waypoints.json')
    if not os.path.exists(file_path):
        print("\n❌ LỖI: Không tìm thấy file waypoints.json!")
        rclpy.shutdown()
        return

    with open(file_path, 'r') as f:
        db_points = json.load(f)

    if not db_points:
        print("\n❌ LỖI: File waypoints.json chưa có điểm nào!\n")
        rclpy.shutdown()
        return

    locations = {}
    for name, data in db_points.items():
        locations[name] = create_pose(navigator, data['x'], data['y'], data['qz'], data['qw'])

    point_names = list(locations.keys())

    # 2. XÁC ĐỊNH ĐIỂM "HOME" MẶC ĐỊNH
    home_name = None
    for name in point_names:
        if 'home' in name.lower() or 'o' == name.lower():
            home_name = name
            break
            
    if not home_name:
        print("\n⚠️ Không tìm thấy trạm 'Home'. Vui lòng thiết lập.")
        for i, name in enumerate(point_names, 1):
            print(f"  {i}. {name}")
        while not home_name:
            choice = input("👉 Chọn TRẠM MẶC ĐỊNH để tự động quay về sau nhiệm vụ: ")
            home_name = get_waypoint_name(choice, point_names)

    # =========================================================================
    # VÒNG LẶP CHÍNH CỦA HỆ THỐNG ĐIỀU PHỐI (FMS)
    # =========================================================================
    while rclpy.ok():
        print("\n" + "="*50)
        print("💤 ROBOT ĐANG Ở TRẠNG THÁI CHỜ LỆNH (IDLE)")
        print("==================================================")
        cmd = input("⌨️  Nhập 'start' để tạo hành trình mới, hoặc 'quit' để thoát: ").strip().lower()
        
        if cmd == 'quit':
            break
        elif cmd != 'start':
            continue

        # --------- LÊN LỊCH TRÌNH ---------
        print("\n🗺️ DANH SÁCH TRẠM (AVAILABLE WAYPOINTS):")
        for i, name in enumerate(point_names, 1):
            print(f"  {i}. {name}")
        print("-" * 50)

        mission_sequence = []
        
        # Chọn các trạm (người dùng chỉ cần chọn A, B, C...)
        while True:
            start_input = input("🏁 Start point (Nhập tên/số): ")
            start_name = get_waypoint_name(start_input, point_names)
            if start_name:
                mission_sequence.append(start_name)
                break
            print("❌ Trạm không hợp lệ!")

        while True:
            next_input = input("➡️ Next point (Nhập tên/số, hoặc gõ 'done' để kết thúc): ").strip().lower()
            if next_input == 'done':
                break
            next_name = get_waypoint_name(next_input, point_names)
            if next_name:
                mission_sequence.append(next_name)
            else:
                print("❌ Trạm không hợp lệ!")

        if not mission_sequence:
            continue

        # --------- XÁC NHẬN TỪ NGƯỜI DÙNG ---------
        mission_str = " -> ".join(mission_sequence) + f" -> {home_name} (Auto Return Home)"
        print(f"\n📋 Lịch trình của bạn: {mission_str}")
        confirm = input("❓ Are you sure to create this mission? (y/n): ").strip().lower()

        if confirm != 'y':
            print("🛑 Đã hủy lịch trình.")
            continue

        # --------- THỰC THI NHIỆM VỤ ---------
        print("\n🚀 BẮT ĐẦU CHẠY! (Gõ 'cancel' + Enter bất cứ lúc nào để hủy nhiệm vụ và về nhà lập tức)")
        is_canceled = False

        # Chạy qua các trạm người dùng đã chọn
        for target_name in mission_sequence:
            print(f"\n📍 Đang di chuyển đến: [{target_name}]...")
            navigator.goToPose(locations[target_name])

            # Chờ robot chạy (và lắng nghe lệnh Cancel)
            while not navigator.isTaskComplete():
                if check_for_cancel():
                    print("\n🛑 PHÁT HIỆN LỆNH 'CANCEL'! Đang hủy nhiệm vụ hiện tại...")
                    navigator.cancelTask()
                    is_canceled = True
                    break
            
            if is_canceled:
                break

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f"✅ Đã đến [{target_name}] thành công!")
                
                # Chờ xác nhận công việc tại trạm
                while True:
                    user_input = input(f"📦 Đang ở {target_name}. Hoàn tất (y) hoặc Hủy nhiệm vụ (cancel)? : ").strip().lower()
                    if user_input == 'y':
                        print("👍 Xác nhận hoàn tất! Chuẩn bị di chuyển tiếp...\n")
                        break
                    elif user_input == 'cancel':
                        print("\n🛑 ĐÃ HỦY NHIỆM VỤ TẠI TRẠM!")
                        is_canceled = True
                        break
                    else:
                        print("⏳ Lệnh không hợp lệ. Vui lòng gõ 'y' hoặc 'cancel'.")
            
            elif result == TaskResult.FAILED:
                print(f"❌ Lỗi: Robot không thể tìm đường đến {target_name}!")
                is_canceled = True
                break
            
            if is_canceled:
                break

        # --------- BƯỚC TỰ ĐỘNG THU HỒI (VỀ HOME) ---------
        print(f"\n🏠 ĐANG TỰ ĐỘNG QUAY VỀ TRẠM MẶC ĐỊNH [{home_name}]...")
        navigator.goToPose(locations[home_name])
        
        while not navigator.isTaskComplete():
            if check_for_cancel():
                print("⚠️ Robot hiện vốn dĩ đang trên đường về nhà rồi!")
        
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"🎉 ROBOT ĐÃ QUAY VỀ [{home_name}] AN TOÀN.")
        else:
            print(f"❌ Robot gặp sự cố khi về nhà!")

    # Dọn dẹp khi thoát hẳn chương trình (Lệnh 'quit')
    print("👋 Đã tắt hệ thống quản lý nhiệm vụ.")
    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()