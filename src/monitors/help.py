import socket
from ros_monitoring.msg import MonitoringInfo


def fillMachineInfo(m):
    m.pc.Hostname = socket.gethostname()
