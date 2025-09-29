import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/guilhermecabaco/Desktop/TESE/ros2_ws/install/temperature_monitor'
