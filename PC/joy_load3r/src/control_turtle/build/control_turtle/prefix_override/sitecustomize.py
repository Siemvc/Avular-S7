import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/teunjacobs/Documents/Avular/Avular-S7/PC/joy_load3r/src/control_turtle/install/control_turtle'
