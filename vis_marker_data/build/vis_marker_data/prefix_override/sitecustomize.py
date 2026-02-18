import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/matthew/sim_ws/src/vis_marker_data/install/vis_marker_data'
