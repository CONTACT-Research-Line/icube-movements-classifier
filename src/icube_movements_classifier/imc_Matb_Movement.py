import enum
import math
import pandas as pd
import pyquaternion as pyp
import time
import numpy as np

from icube_movements_classifier import MovementsDetector
from icube_movements_classifier import ICubeBaseEvents
from icube_movements_classifier import MovementState

CUBE_POSED_FACE = "1111111111111111"

class MovementStateIcube(enum.Enum):
    ROTATED_RIGHT = 0

class MatbMovementEvents(enum.Enum):
    ROTATE_RIGHT = 0
    ROTATE_LEFT = 1
    ROTATE_FORWARD = 2
    ROTATE_BACKWARD = 3
    TOUCH_RIGHTFACE = 4
    TOUCH_LEFTFACE = 5
    TOUCH_FIRSTFACE_TOPLEFT = 6
    TOUCH_FIRSTFACE_TOPRIGHT = 7
    TOUCH_FIRSTFACE_BOTTOMLEFT = 8
    TOUCH_FIRSTFACE_BOTTOMRIGHT = 9


class MatbMovementDetector(MovementsDetector):
    def __init__(self, grab_tolerance=1, threshold=8, threshold_zero=0):
        super().__init__(grab_tolerance=grab_tolerance)
        self.quaternions_old = []
        self.threshold = threshold
        self.threshold_zero = threshold_zero

        print("""
        ======================= MATB MOVEMENT DETECTOR - WARNING =============================
        This ICube Classifier assumes that you are holding the iCub with the charger
        face (Face 0) on top, and with the IIT logo straight in front of you.
        Don't expect this Classifier to handle rotations!
                                -------------------
                                | _ | _  |  _ | _ |
                                | _ | _ IIT _ | _ |
                                | _ | _  0  _ | _ |
                                | _ | _  |  _ | _ |
                                -------------------
        ==============================================================================
        """)


    def compute_angles(self, quaternions_old, quaternions):

        print('Sto calcolando le rotazioni...')

        # definisco assi assoluti
        X = [1, 0, 0]
        Y = [0, 1, 0]
        Z = [0, 0, 1]

        q_upper = pyp.Quaternion(quaternions_old)

        q_lower = pyp.Quaternion(quaternions)

        # Get the 3D difference between these two orientations
        qd = q_upper.conjugate * q_lower
        # print('qd', qd)

        # Calculate Euler angles from this difference quaternion
        phi_qd_rad   = math.atan2( 2 * (qd.w * qd.x + qd.y * qd.z), 1 - 2 * (qd.x**2 + qd.y**2) )
        theta_qd_rad = math.asin ( 2 * (qd.w * qd.y - qd.z * qd.x) )
        psi_qd_rad   = math.atan2( 2 * (qd.w * qd.z + qd.x * qd.y), 1 - 2 * (qd.y**2 + qd.z**2) )

        phi_qd_deg   = math.degrees(phi_qd_rad) # X
        theta_qd_deg = math.degrees(theta_qd_rad) # Y
        psi_qd_deg   = math.degrees(psi_qd_rad) # Z

        print('phi_qd_deg', phi_qd_deg, 'theta_qd_deg', theta_qd_deg, 'psi_qd_deg', psi_qd_deg)

        # ruoto assi

        delta_X = qd.rotate(X)
        delta_Y = qd.rotate(Y)
        delta_Z = qd.rotate(Z)

        print('delta_X', delta_X, 'delta_Y', delta_Y, 'delta_Z', delta_Z)

        """
        #angolo tra due vettori

        angle_X = math.degrees(np.arccos(np.dot(X, delta_X) / (np.linalg.norm(X) * np.linalg.norm(delta_X))))
        angle_Y = math.degrees(np.arccos(np.dot(Y, delta_Y) / (np.linalg.norm(Y) * np.linalg.norm(delta_Y))))
        angle_Z = math.degrees(np.arccos(np.dot(Z, delta_Z) / (np.linalg.norm(Z) * np.linalg.norm(delta_Z))))

        print('angle_X', angle_X, ' angle_Y', angle_Y, ' angle_Z', angle_Z)
        """

        return phi_qd_deg, theta_qd_deg, psi_qd_deg


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

    def __icube_touch_faces(self, touches):
        """
        Touch only one element of the right face, or left face or top face, activates the function.
        Used this function when the iCube is posed or also when the iCube is grabbed.
        @param touches: a set of touches form the iCube
        @return: True if touched the right face or the left face.
        """

        if touches is None:
            return False

        # touch right face
        if touches[1].count('1') > 0:
            return 'touch_rightface'

        # touch left face
        if touches[4].count('1') > 0:
            return 'touch_leftface'

    def __icube_corner_face (self, touches):
        """
        Touching the 4 corners (top left, top right, bottom left, bottom right) of the top face, activates the function.
        Used this function when the iCube is posed or also when the iCube is grabbed.
        This function checks if there is at least a '1' in the specific positions for 'top_left', 'top_right', 'bottom_left' and 'bottom_right'.
        @param touches: a set of touches form the iCube
        @return: True if touched
        """

        if touches is None:
            return False

        if touches[0][:2].count('1') > 0 or touches[0][4:6].count('1') > 0:
            return 'top_left'
        elif touches[0][2:4].count('1') > 0 or touches[0][6:8].count('1') > 0:
            return 'top_right'
        elif touches[0][8:10].count('1') > 0 or touches[0][12:14].count('1') > 0:
            return 'bottom_left'
        elif touches[0][10:12].count('1') > 0 or touches[0][14:].count('1') > 0:
            return 'bottom_right'


    def handle(self, quaternions, touches, accelerometer):
        """
        Classifies participants' behavior
        @param quaternions:
        @param touches:
        @param accelerometer:
        @return:0
        """
        super().handle(quaternions, touches, accelerometer)

        print('quaternions', quaternions)
        print ('touches', touches)

        # handling quaternions
        # convertion from quaternions to euler angles

        phi_qd, theta_qd, psi_qd = self.compute_angles(self.quaternions_old, quaternions)
        "print ('phi_qd', phi_qd, 'theta_qd', theta_qd, 'psi_qd', psi_qd)"

        self.quaternions_old = quaternions

        # from pose to different events
        if self.icube_state == MovementState.POSED:
            # right rotation
            if phi_qd < -self.threshold and theta_qd < self.threshold_zero and psi_qd < self.threshold_zero:
                self.mapping_event_to_callback[MatbMovementEvents.ROTATE_RIGHT]()
            # left rotation
            elif phi_qd > self.threshold and theta_qd > self.threshold_zero and psi_qd > self.threshold_zero:
                self.mapping_event_to_callback[MatbMovementEvents.ROTATE_LEFT]()
            # forward rotation
            elif phi_qd < self.threshold_zero and theta_qd > + self.threshold and psi_qd < self.threshold_zero:
                self.mapping_event_to_callback[MatbMovementEvents.ROTATE_FORWARD]()
            # backward rotation
            elif phi_qd > self.threshold_zero and theta_qd < - self.threshold and psi_qd < self.threshold_zero:
                self.mapping_event_to_callback[MatbMovementEvents.ROTATE_BACKWARD]()


        # touch the top left part of the top face
        if self.icube_state == MovementState.POSED:
            touch_event = self.__icube_corner_face(touches)
            if  touch_event == 'top_left':
                self.mapping_event_to_callback[MatbMovementEvents.TOUCH_FIRSTFACE_TOPLEFT]()
                print ('<<<<<<<<<touch', touches[0])
        # touch the top right part of the top face
            elif touch_event == 'top_right':
                self.mapping_event_to_callback[MatbMovementEvents.TOUCH_FIRSTFACE_TOPRIGHT]()
                print ('<<<<<<<<<touch', touches[0])
        # touch the bottom left part of the top face
            elif touch_event == 'bottom_left':
                self.mapping_event_to_callback[MatbMovementEvents.TOUCH_FIRSTFACE_BOTTOMLEFT]()
                print ('<<<<<<<<<touch', touches[0])
        # touch the bottom right part of the top face
            elif touch_event == 'bottom_right':
                self.mapping_event_to_callback[MatbMovementEvents.TOUCH_FIRSTFACE_BOTTOMRIGHT]()
                print ('<<<<<<<<<touch', touches[0])
            else:
                'other'

        # touch right face
        if self.icube_state == MovementState.POSED:
            if self.__icube_touch_faces(touches) == 'touch_rightface':
                self.mapping_event_to_callback[MatbMovementEvents.TOUCH_RIGHTFACE]()
                print('<<<<<<<<touch', touches[1])
            #touch left face
            elif self.__icube_touch_faces(touches) == 'touch_leftface':
                self.mapping_event_to_callback[MatbMovementEvents.TOUCH_LEFTFACE]()
                print('<<<<<<<touch', touches[4])

    def set_callback(self, event, callback):
        if event in MatbMovementEvents:
            self.mapping_event_to_callback[event] = callback
        else:
            super().set_callback(event, callback)
