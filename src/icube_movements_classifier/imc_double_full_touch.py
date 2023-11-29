import enum

from icube_movements_classifier import MovementsDetector


class DoubleTouchEvents(enum.Enum):
    DOUBLE_TOUCH = 0


CUBE_POSED_FACE = "1111111111111111"


class DoubleFullTouchDetector(MovementsDetector):
    def __init__(self, grab_tolerance=1):
        super().__init__(grab_tolerance=grab_tolerance)

    def handle(self, quaternions, touches, accelerometer):

        if touches.count(CUBE_POSED_FACE) == 2:
            self.mapping_event_to_callback[DoubleTouchEvents.DOUBLE_TOUCH]()

        super().handle(quaternions, touches, accelerometer)

    def set_callback(self, event, callback):
        if event in DoubleTouchEvents:
            self.mapping_event_to_callback[event] = callback
        else:
            super().set_callback(event, callback)
