import enum

from icube_movements_classifier import BaseEventHandler


class DoubleTouchEvents(enum.Enum):
    DOUBLE_TOUCH = 0


CUBE_POSED_FACE = "1111111111111111"


class DoubleFullTouchDetector(BaseEventHandler):

    def handle(self, quaternions, touches, accelerometer):
        if touches.count(CUBE_POSED_FACE) == 2:
            self.fire(DoubleTouchEvents.DOUBLE_TOUCH)

