import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/movemaster/movemaster/movemaster_ws/install/movemaster_ros'
