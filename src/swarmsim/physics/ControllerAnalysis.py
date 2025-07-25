from ..agent.control.BinaryController import BinaryController
from ..agent.control.HumanController import HumanController
from ..agent.control.StaticController import StaticController

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

def getControllerPeaks(controller):
    if isinstance(controller, BinaryController):
        return getBinaryControllerPeaks(controller)
    elif isinstance(controller, HumanController):
        return getHumanControllerPeaks(controller)
    elif isinstance(controller, StaticController):
        return getStaticControllerPeaks(controller)
    print("ERROR", "controller type unanalyzable:", controller)
    return None, None