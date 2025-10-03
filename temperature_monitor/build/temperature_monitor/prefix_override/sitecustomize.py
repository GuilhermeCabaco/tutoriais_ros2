import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/guilhermecabaco/Desktop/ros2_ws/tutoriais_ros2/temperature_monitor/install/temperature_monitor'
