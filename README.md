UR5e MoveIt Demo

Demo điều khiển robot UR5e bằng MoveIt (ROS Noetic) với:

Hiển thị điểm HOME và TARGET

Tạo 2 vật cản ngẫu nhiên

Robot tự lập kế hoạch và né vật cản

Chạy ở chế độ fake execution (RViz)

Yêu cầu

Ubuntu + ROS Noetic

MoveIt

universal_robot

ur5e_moveit_config

Chạy demo

Mở terminal 1

source ~/ur5_ws/devel/setup.bash

roslaunch ur5e_moveit_config demo.launch


Mở terminal 2

source ~/ur5_ws/devel/setup.bash

rosrun ur5e_moveit_config ur5_moveit_control.py

Chức năng chính

Hiển thị HOME / TARGET bằng marker

Sinh 2 vật cản ngẫu nhiên

Robot tự tìm đường tránh vật cản

Điều chỉnh tốc độ & thời gian planning

Chạy thử di chuyển đơn giản

Ghi chú

Robot chỉ mô phỏng (fake controller)

Chuyển động hiển thị trong RViz


