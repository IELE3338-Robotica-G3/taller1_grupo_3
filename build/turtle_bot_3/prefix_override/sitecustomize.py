import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robotica/taller1_ws/src/turtle_bot_3/install/turtle_bot_3'
