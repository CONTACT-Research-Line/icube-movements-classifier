import sys
import time

from icube import ICubeInterface
from icube import ICubeVersion
from icube_movements_classifier import ButtonEvents

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

    gp_classifier = factory.get_movement_classifier(IMC.GRAB_POSE, grab_tolerance=1.0)
    gp_classifier.set_callback(GrabPoseEvents.GRAB, lambda: print("GRAB"))
    gp_classifier.set_callback(GrabPoseEvents.POSE, lambda: print("POSE"))
    device.bind_callback(gp_classifier.handle)

    button_classifier = factory.get_movement_classifier(IMC.BUTTON_PRESS,
                                                        detect_on_face=None, release_after=2.0, min_cover=10)
    button_classifier.set_callback(ButtonEvents.PRESS, lambda x: print(f"PRESS {x}"))
    button_classifier.set_callback(ButtonEvents.RELEASE, lambda x: print(f"RELEASE {x}"))
    device.bind_callback(button_classifier.handle)

    device.start_streaming()
    time.sleep(100000)  # seconds
    device.stop_streaming()


if __name__ == '__main__':
    main()

