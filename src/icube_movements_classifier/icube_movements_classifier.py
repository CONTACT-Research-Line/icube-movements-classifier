import enum

from icube.data_handlers.base import BaseHandler
import numpy as np

CUBE_POSED_FACE = "1111111111111111"


class MovementState(enum.Enum):
    POSED = 0,
    GRABBED = 1
    MOVED = 2
    UNKNOWN = 3


class ICubeBaseEvents(enum.Enum):
    MOVE = 0,
    GRAB = 1,
    POSE = 2


class MovementsDetector(BaseHandler):
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

        self.icube_state = MovementState.UNKNOWN
        self.grab_tolerance = grab_tolerance
        self.mapping_event_to_callback = {}

    def __fire(self, event, args=None):
        if event in self.mapping_event_to_callback:
            if args is not None:
                self.mapping_event_to_callback[event](args)
            else:
                self.mapping_event_to_callback[event]()
            

    def set_callback(self, event, callback):
        self.mapping_event_to_callback[event] = callback

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

        if self.icube_state == MovementState.UNKNOWN:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.__fire(ICubeBaseEvents.POSE)
            else:
                self.icube_state = MovementState.GRABBED
                self.__fire(ICubeBaseEvents.GRAB)

        np_acc = np.array(accelerometer)
        if self.init_acc is None:
            self.init_acc = np_acc

        self.delta_movement = np.linalg.norm(accelerometer - self.init_acc)
        if self.delta_movement > 0:
            self.__fire(ICubeBaseEvents.MOVE, args=self.delta_movement)

        if self.icube_state == MovementState.POSED:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.GRABBED
                self.__fire(ICubeBaseEvents.GRAB)

        if self.icube_state == MovementState.GRABBED:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.__fire(ICubeBaseEvents.POSE)

        self.init_acc = np_acc
