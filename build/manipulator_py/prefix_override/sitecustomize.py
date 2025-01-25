import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zahran/Desktop/manipulator_ws/install/manipulator_py'
