import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/recklurker/assgn_turtlebot/install/custom_dwa'
