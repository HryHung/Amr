⚠️ 1. Lỗi Logic Đặc Biệt Nghiêm Trọng ở Khâu Tính RPM
Trong hàm compute_control_loop(), bạn đã vô tình phá hỏng biến tính RPM của thuật toán PID.

Đoạn code của bạn:

C++
M1.prevCount = c1; M2.prevCount = c2; // BƯỚC A: Cập nhật prevCount 

// ... (Tính x, y, theta)

auto pid = [](Motor &m) {
  // BƯỚC B: Tính RPM bằng công thức sai
  m.actualRPM = ((float)(m.count - m.prevCount + (m.prevCount)) / PULSES_PER_REV) * (60.0 / Ts); 
  // ...
};
Bản chất vấn đề: Bạn đã gán m.prevCount = c1 ở BƯỚC A. Do đó, khi gọi hàm pid(M1) ở BƯỚC B, phép trừ (m.count - m.prevCount) sẽ cho ra kết quả gần như bằng 0. Tệ hơn nữa, cụm + (m.prevCount) đằng sau khiến biểu thức rút gọn thành m.count.
Hậu quả: actualRPM sẽ đếm tổng số vòng quay từ lúc bật máy, biến thiên lên mức hàng triệu RPM, làm bộ PID tích lũy Error âm khổng lồ và motor sẽ kẹt ở tốc độ tối đa theo chiều ngược lại.

Cách Fix: Tính luôn actualRPM ngay tại thời điểm lấy diff xung, không được để gán đè biến.

⚠️ 2. Lỗi Crash Micro-ROS do C-String chưa cấp phát Size
Trong hàm setup(), bạn gán ID cho frame TF như sau:

C++
tf_stamped.header.frame_id.data = (char*)"odom";
tf_stamped.child_frame_id.data = (char*)"base_link";
Trong môi trường micro_ros, các biến chuỗi (rosidl_runtime_c__String) được quản lý bộ nhớ cực kỳ nghiêm ngặt. Nếu bạn chỉ trỏ con trỏ .data mà không khai báo độ dài .size và .capacity, khi Agent bên máy tính (Raspberry Pi) nhận gói tin, nó sẽ không biết đọc bao nhiêu byte, dẫn đến Segfault và Disconnect Agent ngay lập tức.
