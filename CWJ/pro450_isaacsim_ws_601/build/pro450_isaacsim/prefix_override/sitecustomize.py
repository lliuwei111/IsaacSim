import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspace/dev/IsaacSim/CWJ/pro450_isaacsim_ws_601/install/pro450_isaacsim'
