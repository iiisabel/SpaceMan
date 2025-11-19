import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lianxin/Projects/SpaceMan_ws/install/remote_control_pkg'
