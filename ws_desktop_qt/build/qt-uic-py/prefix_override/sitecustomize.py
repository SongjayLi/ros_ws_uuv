import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/uuv/Development/rov-ros2/ws_desktop_qt/install/qt-uic-py'
