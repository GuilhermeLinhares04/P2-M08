import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/linhares1111/the_ws/src/the_slam_py/install/the_slam_py'
