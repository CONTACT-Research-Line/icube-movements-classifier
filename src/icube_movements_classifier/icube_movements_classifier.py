import enum

from icube.data_handlers.base import BaseHandler
import numpy as np

CUBE_POSED_FACE = "1111111111111111"


class MovementState(enum.Enum):
    POSED = 0,
    GRABBED = 1
    MOVED = 2
    UNKNOWN = 3
    TURNED = 4


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
        self.on_grab = None
        self.on_pose = None
        self.on_move = None
        self.on_turn = None

    def set_on_grab_callback(self, on_grab):
        """
        What to do when the cube is grasped
        @param on_grab: function in format event_trigger()
        @return:
        """
        self.on_grab = on_grab

    def set_on_pose_callback(self, on_pose):
        """
        What to do when the cube is posed
        @param on_pose: function in format event_trigger()
        @return:
        """
        self.on_pose = on_pose

    def set_on_move_callback(self, on_move):
        """
        What to do when the cube is moved
        @param on_move: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_move = on_move

    def set_on_turn_callback(self, on_turn):
        """
        What to do when the cube is turned
        @param on_turn: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_turn = on_turn

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
                self.on_pose()
            else:
                self.icube_state = MovementState.GRABBED
                self.on_grab()

        np_acc = np.array(accelerometer)
        if self.init_acc is None:
            self.init_acc = np_acc

        self.delta_movement = np.linalg.norm(accelerometer - self.init_acc)
        if self.delta_movement > 0:
            self.on_move(self.delta_movement)

        if self.icube_state == MovementState.POSED:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.GRABBED
                self.on_grab()


        if self.icube_state == MovementState.GRABBED:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.on_pose()

        if self.icube_state == MovementState.GRABBED:
            if self.delta_movement > self.turn_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TURNED
                self.on_turn()

        if self.icube_state == MovementState.TURNED:
            if self.delta_movement < self.turn_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.GRABBED
                self.on_grab()

        self.init_acc = np_acc
