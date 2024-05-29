import enum

from icube_movements_classifier import MatbMovementDetector
from icube_movements_classifier import GrabPoseDetector
from icube_movements_classifier import SwipeDetector
from icube_movements_classifier import DoubleFullTouchDetector
from icube_movements_classifier import ButtonPressDetector


class IMC(enum.Enum):
    GRAB_POSE = 0,
    DOUBLE_TOUCH = 1,
    SWIPE = 2,
    MATB = 3,
    BUTTON_PRESS = 4


class IMCFactory:
    def __init__(self):
        self.available_classifiers = {
            IMC.GRAB_POSE: GrabPoseDetector,
            IMC.DOUBLE_TOUCH: DoubleFullTouchDetector,
            IMC.SWIPE: SwipeDetector,
            IMC.MATB: MatbMovementDetector,
            IMC.BUTTON_PRESS: ButtonPressDetector
        }

    def get_movement_classifier(self, imc, **kwargs):
        if imc in self.available_classifiers:
            return self.available_classifiers[imc](**kwargs)

        raise ValueError("Unavailable classifier")

    def list_available_classifiers(self):
        return list(self.available_classifiers.keys())
