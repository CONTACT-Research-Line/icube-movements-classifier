import enum
import numpy as np
from icube_movements_classifier.base_event_handler import BaseEventHandler

CUBE_POSED_FACE = "1111111111111111"


class GrabPoseEvents(enum.Enum):
    MOVE = 0,
    GRAB = 1,
    POSE = 2


class GrabPoseDetector(BaseEventHandler):
    class State(enum.Enum):
        POSED = 0,
        GRABBED = 1
        MOVED = 2
        UNKNOWN = 3

    """
    @package GraspDetector
    @brief a module to detect when the participant grasps the iCube or place it on a flat surface
    @author Dario Pasquali
    """

    def __init__(self, grab_tolerance=1):
        """
        @param grab_tolerance: how much being tolerant on classifying an acceleration as grasping
        """
        super().__init__()
        self.init_acc = None
        self.delta_movement = 0.0

        self.icube_state = GrabPoseDetector.State.UNKNOWN
        self.grab_tolerance = grab_tolerance
        self.mapping_event_to_callback = {}

    def __icube_posed(self, touches):
        """
        Classify if the iCube is posed based on touches
        If only one face is fully active the cube is posed somewhere
        Otherwise the cube is held
        @param touches: a set of touches form the iCube
        @return: True if touched
        """

        if touches is None:
            return False
        full_covered_faces = touches.count(CUBE_POSED_FACE)
        touched_faces = ["1" in t for t in touches].count(True)
        return full_covered_faces == 1 and touched_faces == 1

    def handle(self, quaternions, touches, accelerometer):
        """
        Classifies participants' behavior
        @param quaternions:
        @param touches:
        @param accelerometer:
        @return:
        """
        if accelerometer is None or accelerometer == []:
            return False

        if self.icube_state == GrabPoseDetector.State.UNKNOWN:
            if self.__icube_posed(touches):
                self.icube_state = GrabPoseDetector.State.POSED
                self.fire(GrabPoseEvents.POSE)
            else:
                self.icube_state = GrabPoseDetector.State.GRABBED
                self.fire(GrabPoseEvents.GRAB)

        np_acc = np.array(accelerometer)
        if self.init_acc is None:
            self.init_acc = np_acc

        self.delta_movement = np.linalg.norm(accelerometer - self.init_acc)
        if self.delta_movement > 0:
            self.fire(GrabPoseEvents.MOVE, args=self.delta_movement)

        if self.icube_state == GrabPoseDetector.State.POSED:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = GrabPoseDetector.State.GRABBED
                self.fire(GrabPoseEvents.GRAB)

        if self.icube_state == GrabPoseDetector.State.GRABBED:
            if self.__icube_posed(touches):
                self.icube_state = GrabPoseDetector.State.POSED
                self.fire(GrabPoseEvents.POSE)

        self.init_acc = np_acc
