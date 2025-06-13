from ..agent.control.BinaryController import BinaryController

def getBinaryControllerPeaks(bc: BinaryController):
    peakV = max(bc.a[0], bc.b[0])
    peakOmega = max(abs(bc.a[1]), abs(bc.b[1]))
    return peakV, peakOmega

def getControllerPeaks(controller):
    if isinstance(controller, BinaryController):
        return getBinaryControllerPeaks(controller)
    print("ERROR", "controller type unanalyzable:", controller)
    return None, None