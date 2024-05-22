import sys
import time

from icube import ICubeInterface
from icube import ICubeVersion

from icube_movements_classifier.IMCFactory import *
from icube_movements_classifier.imc_matb import MatbMovementEvents
from icube_movements_classifier.imc_grab_pose import GrabPoseEvents
from icube_movements_classifier.imc_swipes import SwipeEvents


def print_data(quaternions, touches, accelerometer):
    print("PRINT", quaternions, touches, accelerometer)


def compute_some_features(quaternions, touches, accelerometer):
    print("COUNT touches - the next callback should start")
    time.sleep(2)
    print("N touches")


def main():

    factory = IMCFactory()
    print(factory.list_available_classifiers())

    device = ICubeInterface(version=ICubeVersion.V3, enable_gui=("--gui" in sys.argv))
    device.init(name="cube_test", serial_port="/dev/ttyUSB0")

    gp_classifier = factory.get_movement_classifier("base")()
    gp_classifier.set_callback(GrabPoseEvents.GRAB, lambda: print("GRAB"))
    gp_classifier.set_callback(GrabPoseEvents.POSE, lambda: print("POSE"))
    device.bind_callback(gp_classifier.handle)

    swipes_classifier = factory.get_movement_classifier("swipes")(detect_on_face=0)
    swipes_classifier.set_callback(SwipeEvents.COVER, lambda x: print(f"COVER {x}"))
    swipes_classifier.set_callback(SwipeEvents.FORWARD, lambda x: print(f"FORWARD {x}"))
    swipes_classifier.set_callback(SwipeEvents.BACKWARD, lambda x: print(f"BACKWARD {x}"))
    device.bind_callback(swipes_classifier.handle)

    matb = factory.get_movement_classifier("matb")()
    matb.set_callback(MatbMovementEvents.TOUCH_RIGHTFACE, lambda: print("TOUCH_RIGHTFACE"))
    matb.set_callback(MatbMovementEvents.TOUCH_LEFTFACE, lambda: print("TOUCH_LEFTFACE"))
    matb.set_callback(MatbMovementEvents.TOUCH_FRONTFACE, lambda: print("TOUCH_FRONTFACE"))
    matb.set_callback(MatbMovementEvents.TOUCH_BACKFACE, lambda: print("TOUCH_BACKFACE"))
    matb.set_callback(MatbMovementEvents.TOUCH_TOPFACE_TOPLEFT, lambda: print("TOUCH_TOPFACE_TOPLEFT"))
    matb.set_callback(MatbMovementEvents.TOUCH_TOPFACE_TOPRIGHT, lambda: print("TOUCH_TOPFACE_TOPRIGHT"))
    matb.set_callback(MatbMovementEvents.TOUCH_TOPFACE_BOTTOMLEFT, lambda: print("TOUCH_TOPFACE_BOTTOMLEFT"))
    matb.set_callback(MatbMovementEvents.TOUCH_TOPFACE_BOTTOMRIGHT, lambda: print("TOUCH_TOPFACE_BOTTOMRIGHT"))
    matb.set_callback(MatbMovementEvents.ROTATE_RIGHT, lambda: print("ROTATE_RIGHT"))
    matb.set_callback(MatbMovementEvents.ROTATE_LEFT, lambda: print("ROTATE_LEFT"))
    matb.set_callback(MatbMovementEvents.ROTATE_FORWARD, lambda: print("ROTATE_FORWARD"))
    matb.set_callback(MatbMovementEvents.ROTATE_BACKWARD, lambda: print("ROTATE_BACKWARD"))
    matb.set_callback(MatbMovementEvents.TURNED_CLOCKWISE, lambda: print("TURNED_CLOCKWISE"))
    matb.set_callback(MatbMovementEvents.TURNED_ANTICLOCKWISE, lambda: print("TURNED_ANTICLOCKWISE"))
    device.bind_callback(matb.handle)

    device.start_streaming()
    time.sleep(100000)  # seconds
    device.stop_streaming()


if __name__ == '__main__':
    main()

