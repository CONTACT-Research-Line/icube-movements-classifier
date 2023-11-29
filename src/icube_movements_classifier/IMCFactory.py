from icube_movements_classifier import MovementsDetector
from icube_movements_classifier import DoubleFullTouchDetector


class IMCFactory:
    def __init__(self):
        self.available_classifiers = {
            "base": MovementsDetector,
            "double_touch": DoubleFullTouchDetector
        }

    def get_movement_classifier(self, classifier_type):
        if classifier_type in self.available_classifiers:
            return self.available_classifiers[classifier_type]

        raise ValueError("Unavailable classifier")


