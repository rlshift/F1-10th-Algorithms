import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/matthew/sim_ws/src/follow_the_gap/install/follow_the_gap'
