import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/juptegraph/Workspaces/Robotics/Projects/CORI/cori_ws/install/cori_gui'
