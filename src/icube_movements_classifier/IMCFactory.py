from icube_movements_classifier import MatbMovementDetector
from icube_movements_classifier import GrabPoseDetector
from icube_movements_classifier import SwipeDetector
from icube_movements_classifier import DoubleFullTouchDetector
from icube_movements_classifier import ButtonPressDetector


class IMCFactory:
    def __init__(self):
        self.available_classifiers = {
            "base": GrabPoseDetector,
            "double_touch": DoubleFullTouchDetector,
            "swipes": SwipeDetector,
            "matb": MatbMovementDetector,
            "button_press": ButtonPressDetector
        }

    def get_movement_classifier(self, classifier_type):
        if classifier_type in self.available_classifiers:
            return self.available_classifiers[classifier_type]

        raise ValueError("Unavailable classifier")

    def list_available_classifiers(self):
        return list(self.available_classifiers.keys())
