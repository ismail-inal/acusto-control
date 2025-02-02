from pipython import GCSDevice, pitools

# CONNECT DEVICES
def connect_pi(controllername, serialnum, stages, refmodes):
    pidevice = GCSDevice(controllername)
    pidevice.ConnectUSB(serialnum=serialnum)
    pitools.startup(pidevice, stages=stages, refmodes=refmodes)
    return pidevice
