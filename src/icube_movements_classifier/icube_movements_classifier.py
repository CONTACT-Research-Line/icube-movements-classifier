import enum

from icube.data_handlers.base import BaseHandler
import numpy as np

CUBE_POSED_FACE = "1111111111111111"


class MovementState(enum.Enum):
    POSED = 0,
    GRABBED = 1
    MOVED = 2
    UNKNOWN = 3
    TOUCHED = 4
    TURNED = 5
    TURNED_RIGHT = 6
    TURNED_LEFT = 7
    TURNED_UPWARD = 8
    TURNED_DOWNWARD = 9
    TOUCH_LEFTFACEF5 = 10
    TOUCH_RIGHTFACEF6 = 11
    TOUCH_UPLEFTF1 = 12
    TOUCH_UPRIGHTF2 = 13
    TOUCH_DOWNLEFTF3 = 14
    TOUCH_DOWNRIGHTF4 = 15


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
        self.on_touch = None
        self.on_turn = None
        self.on_turn_right = None
        self.on_turn_left =None
        self.on_turn_upward = None
        self.on_turn_downward = None
        self.on_touch_leftfaceF5 = None
        self.on_touch_rightfaceF6 = None
        self.on_touch_upleftF1 = None
        self.on_touch_uprightF2 = None
        self.on_touch_downleftF3 = None
        self.on_touch_downrightF4 = None


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
        @param on_move: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_turn = on_turn

    def set_on_touch_callback(self, on_touch):
        """
        What to do when the cube is touched
        @param on_touch: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch = on_touch

    def set_on_turn_right_callback(self, on_turn_right):
        """
        What to do when the cube is turned right
        @param on_turn_right: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_turn_right = on_turn_right

    def set_on_turn_left_callback(self, on_turn_left):
        """
        What to do when the cube is turned left
        @param on_turn_left: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_turn_left = on_turn_left

    def set_on_turn_upward_callback(self, on_turn_upward):
        """
        What to do when the cube is turned upward
        @param on_turn_upward: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_turn_upward = on_turn_upward

    def set_on_turn_downward_callback(self, on_turn_downward):
        """
        What to do when the cube is turned upward
        @param on_turn_downward: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_turn_downward = on_turn_downward

    def set_on_touch_leftfaceF5_callback(self, on_touch_leftfaceF5):
        """
        What to do when the cube is touched left face F5
        @param on_touch_leftfaceF5: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_leftfaceF5 = on_touch_leftfaceF5

    def set_on_touch_rightfaceF6_callback(self, on_touch_rightfaceF6):
        """
        What to do when the cube is touched right face F6
        @param on_touch_rightfaceF6: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_rightfaceF6 = on_touch_rightfaceF6

    def set_on_touch_upleftF1_callback(self, on_touch_upleftF1):
        """
        What to do when the cube is touched the up left part of the first face F1
        @param on_touch_upleftF1: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_upleftF1 = on_touch_upleftF1

    def set_on_touch_uprightF2_callback(self, on_touch_uprightF2):
        """
        What to do when the cube is touched the up right part of the first face F2
        @param on_touch_uprightF2: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_uprightF2 = on_touch_uprightF2

    def set_on_touch_downleftF3_callback(self, on_touch_downleftF3):
        """
        What to do when the cube is touched the down left part of the first face F3
        @param on_touch_downleftF3: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_downleftF3 = on_touch_downleftF3

    def set_on_touch_downrightF4_callback(self, on_touch_downrightF4):
        """
        What to do when the cube is touched the down right part of the first face F4
        @param on_touch_downrightF4: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_downrightF4 = on_touch_downrightF4


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
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TURNED_RIGHT
                self.on_turn_right()

        if self.icube_state == MovementState.GRABBED:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TURNED_LEFT
                self.on_turn_left()

        if self.icube_state == MovementState.GRABBED:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TURNED_UPWARD
                self.on_turn_upward()

        if self.icube_state == MovementState.GRABBED:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TURNED_DOWNWARD
                self.on_turn_downward()

        if self.icube_state == MovementState.GRABBED:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TOUCH_LEFTFACEF5
                self.on_touch_leftfaceF5()

        if self.icube_state == MovementState.GRABBED:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TOUCH_RIGHTFACEF6
                self.on_touch_rightfaceF6()

        if self.icube_state == MovementState.GRABBED:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TOUCH_UPLEFTF1
                self.on_touch_upleftF1()

        if self.icube_state == MovementState.GRABBED:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TOUCH_UPRIGHTF2
                self.on_touch_uprightF2()

        if self.icube_state == MovementState.GRABBED:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TOUCH_DOWNLEFTF3
                self.on_touch_downleftF3()

        if self.icube_state == MovementState.GRABBED:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TOUCH_DOWNRIGHTF4
                self.on_touch_downrightF4()

        if self.icube_state in [MovementState.GRABBED, MovementState.TURNED]:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.on_pose()

        self.init_acc = np_acc
