from ..agent.control.BinaryController import BinaryController
from ..agent.control.HumanController import HumanController
from ..agent.control.StaticController import StaticController
from ..agent.control.NaryController import NaryController

def getBinaryControllerPeaks(bc: BinaryController):
    peakV = max(bc.a[0], bc.b[0])
    peakOmega = max(abs(bc.a[1]), abs(bc.b[1]))
    return peakV, peakOmega

def getHumanControllerPeaks(hc: HumanController):
    peakV = max([abs(v) for v in hc.speed_range])
    peakOmega = max([abs(omega) for omega in hc.turn_range])
    return peakV, peakOmega

def getStaticControllerPeaks(sc: StaticController):
    peakV = 10
    peakOmega = 3
    return peakV, peakOmega

def getNaryControllerPeaks(nc: NaryController):
    peak_v, peak_omega = nc.on_detect[0][0], abs(nc.on_detect[0][1])

    for i in range(1, nc.n_sensors):
        v, omega = nc.on_detect[i]
        peak_v, peak_omega = max(peak_v, v), max(peak_omega, abs(omega))

    return peak_v, peak_omega

def getControllerPeaks(controller):
    if isinstance(controller, BinaryController):
        return getBinaryControllerPeaks(controller)
    elif isinstance(controller, HumanController):
        return getHumanControllerPeaks(controller)
    elif isinstance(controller, StaticController):
        return getStaticControllerPeaks(controller)
    elif isinstance(controller, NaryController):
        return getNaryControllerPeaks(controller)

    raise TypeError(f"ERROR: controller type unanalyzable: {type(controller)}")
