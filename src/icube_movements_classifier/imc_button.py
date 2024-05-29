import enum
import threading
import time

import numpy as np
from icube_movements_classifier import BaseEventHandler


class ButtonEvents(enum.Enum):
    PRESS = 0,
    RELEASE = 1,


class ICubeFaceState:
    def __init__(self, face_id, on_press, on_release, release_after=2.0):
        self.face_id = face_id
        self.on_press = on_press
        self.on_release = on_release

        self.is_pressed = False
        self.release_timer = None
        self.release_after = release_after

    def release(self):
        self.is_pressed = False
        self.on_release(self.face_id)

    def press(self):
        if self.is_pressed:
            self.release_timer.cancel()
            self.release_timer = threading.Timer(self.release_after, self.release)
            self.release_timer.start()

        else:
            self.on_press(self.face_id)
            self.is_pressed = True
            self.release_timer = threading.Timer(self.release_after, self.release)
            self.release_timer.start()


N_FACES = 6
N_ROWS = 4
N_COLS = 4


class ButtonPressDetector(BaseEventHandler):
    """
    @package ButtonPressDetector
    @brief
    @author Dario Pasquali
    """

    def __init__(self, detect_on_face=None, release_after=2.0, min_cover=10):
        assert 1 < min_cover < 16

        super().__init__()
        self.detect_on_face = detect_on_face
        self.release_after = release_after
        self.min_cover = min_cover

        # Internal State
        self.faces = [ICubeFaceState(face_id=id,
                                     on_press=self.on_press, on_release=self.on_release,
                                     release_after=release_after) for id in range(6)]

    def on_press(self, face_id):
        self.fire(ButtonEvents.PRESS, face_id)

    def on_release(self, face_id):
        self.fire(ButtonEvents.RELEASE, face_id)

    def handle(self, quaternions, touches, accelerometer):
        if self.detect_on_face is not None:
            if touches[self.detect_on_face].count('1') > self.min_cover:
                self.faces[self.detect_on_face].press()
        else:
            for face_id, touch in enumerate(touches):
                if touch.count('1') >= self.min_cover:
                    self.faces[face_id].press()

