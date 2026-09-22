# AMR Graduation Project — ROS2 Workspace

Autonomous Mobile Robot (AMR) đồ án tốt nghiệp. ROS2 là phần điều khiển trung tâm cho 4 tính năng: teleoperation + video streaming, SLAM/Nav2, object tracking, đo khoảng cách bằng LiDAR `/scan`.

## Phần cứng
- Khung: RB080 chassis
- Máy tính robot: Raspberry Pi 4 (4GB), Ubuntu Server 22.04.5 LTS 64-bit
- LiDAR: YDLidar X4 Pro
- Camera: chưa xác định model

## Stack phần mềm
- ROS2 Humble trên cả Pi (`ros-humble-ros-base`) và PC/WSL2 base station (`ros-humble-desktop`)
- SLAM Toolbox, Nav2, `ydlidar_ros2_driver`, `rf2o_laser_odometry` (LIDAR-only odometry fallback)
- Motor control: `gpiozero` (backend `lgpio`)

## Cấu trúc workspace

```
dev_ws/
  src/
    my_bot/       # package chính, dựa trên template joshnewans/my_bot
```

Package `my_bot` khởi tạo từ [joshnewans/my_bot](https://github.com/joshnewans/my_bot) (Articulated Robotics tutorial template).

**Quy tắc bắt buộc khi theo tutorial Articulated Robotics:**
- Mọi chỗ hướng dẫn dùng `foxy` → đổi thành `humble`
- Mọi chỗ hướng dẫn dùng `rplidar_ros` → đổi thành `ydlidar_ros2_driver`

## Build

```bash
cd dev_ws
colcon build --symlink-install
source install/setup.bash
```

## Chạy robot_state_publisher

```bash
ros2 launch my_bot rsp.launch.py
```

## Ghi chú mạng

Robot đặt cố định tại trường, Wi-Fi "project" cô lập (`192.168.0.x`, không Internet). Pi: `192.168.0.5` (user `project1`), PC base station: `192.168.0.20`. Xem chi tiết trong tài liệu nội bộ nhóm.
