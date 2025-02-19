from typing import List

from pipython import GCSDevice, pitools


def connect_pi(
    controllername: str, serialnum: str, stages: List[str], refmodes: List[str]
) -> GCSDevice:
    pidevice = GCSDevice(controllername)
    pidevice.ConnectUSB(serialnum=serialnum)
    pitools.startup(pidevice, stages=stages, refmodes=refmodes)
    return pidevice
