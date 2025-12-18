import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robotarm/Desktop/SpaceMan-feature-branch/install/remote_control_pkg'
