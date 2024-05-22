from icube_movements_classifier import MovementsDetector
from icube_movements_classifier.imc_double_full_touch import DoubleFullTouchDetector
from icube_movements_classifier.swipes import SwipeDetector


class IMCFactory:
    def __init__(self):
        self.available_classifiers = {
            "base": MovementsDetector,
            "double_touch": DoubleFullTouchDetector,
            "swipes": SwipeDetector,
        }

    def get_movement_classifier(self, classifier_type):
        if classifier_type in self.available_classifiers:
            return self.available_classifiers[classifier_type]

        raise ValueError("Unavailable classifier")

    def list_available_classifiers(self):
        return list(self.available_classifiers.keys())
