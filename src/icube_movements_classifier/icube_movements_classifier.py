import enum
import math
import pandas as pd
import pyquaternion as pyp
import time


from icube.data_handlers.base import BaseHandler
from icube.tactile.common import tactile_logging as log

import numpy as np

CUBE_POSED_FACE = "1111111111111111"

class MovementState(enum.Enum):
    POSED = 0,
    GRABBED = 1
    MOVED = 2
    UNKNOWN = 3
    TOUCHED = 4
    TURNED = 5
    TURNED_CLOCKWISE = 6
    TURNED_ANTICLOCKWISE = 7
    ROTATE_RIGHT = 8
    ROTATE_LEFT = 9
    ROTATE_FORWARD = 10
    ROTATE_BACKWARD = 11
    TOUCH_RIGHTFACE = 12
    TOUCH_LEFTFACE = 13
    TOUCH_FIRSTFACE_TOPLEFT = 14
    TOUCH_FIRSTFACE_TOPRIGHT = 15
    TOUCH_FIRSTFACE_BOTTOMLEFT = 16
    TOUCH_FIRSTFACE_BOTTOMRIGHT = 17
    TOUCH_TOPFACE = 18
    IDLE = 19


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
        self.quaternions_old = 0.0

        self.icube_state = MovementState.UNKNOWN
        self.grab_tolerance = grab_tolerance
        self.on_grab = None
        self.on_pose = None
        self.on_move = None
        self.on_touch = None
        self.on_turn = None
        self.on_turn_clockwise = None
        self.on_turn_anticlockwise = None
        self.on_rotate_right = None
        self.on_rotate_left = None
        self.on_rotate_forward = None
        self.on_rotate_backward = None
        self.on_touch_rightface = None
        self.on_touch_leftface = None
        self.on_touch_firstface_topleft = None
        self.on_touch_firstface_topright = None
        self.on_touch_firstface_bottomleft = None
        self.on_touch_firstface_bottomright = None
        self.on_touch_topface = None
        self.on_idle = None

    def compute_angles(self, quaternions_old, quaternions, quanti_trial):

        print('Sto calcolando le rotazioni...')

        "definisco assi assoluti"
        X = [1, 0, 0]
        Y = [0, 1, 0]
        Z = [0, 0, 1]

        for trial in range(1,quanti_trial+1):

            for i in range(1,2):

                # print('quat_old', pyp.Quaternion(quaternions_old))

                q_upper = pyp.Quaternion(quaternions_old)
                # print('q_upper', q_upper)

                q_lower = pyp.Quaternion(quaternions)
                # print('q_lower', q_lower)

                """
                print('q_upper_w', q_upper.w, 'q_upper_x', q_upper.x, 'q_upper_y', q_upper.y, 'q_upper_z', q_upper.z)
                print('q_lower_w', q_lower.w, 'q_lower_x', q_lower.x, 'q_lower_y', q_lower.y, 'q_lower_z', q_lower.z)
                """

                # Get the 3D difference between these two orientations
                qd = q_upper.conjugate * q_lower
                # print('qd', qd)

                phi_qd_rad   = math.atan2( 2 * (qd.w * qd.x + qd.y * qd.z), 1 - 2 * (qd.x**2 + qd.y**2) )
                theta_qd_rad = math.asin ( 2 * (qd.w * qd.y - qd.z * qd.x) )
                psi_qd_rad   = math.atan2( 2 * (qd.w * qd.z + qd.x * qd.y), 1 - 2 * (qd.y**2 + qd.z**2) )

                phi_qd_deg   = math.degrees(phi_qd_rad)
                theta_qd_deg = math.degrees(theta_qd_rad)
                psi_qd_deg   = math.degrees(psi_qd_rad)

                print('phi_qd_deg', phi_qd_deg, 'theta_qd_deg', theta_qd_deg, 'psi_qd_deg', psi_qd_deg)

                # ruoto assi

                delta_X = qd.rotate(X)
                delta_Y = qd.rotate(Y)
                delta_Z = qd.rotate(Z)

                print('delta_X', delta_X, 'delta_Y', delta_Y, 'delta_Z', delta_Z)

                """""
                "angolo tra due vettori"

                angle_X = math.degrees(np.arccos(np.dot(X, delta_X) / (np.linalg.norm(X) * np.linalg.norm(delta_X))))
                angle_Y = math.degrees(np.arccos(np.dot(Y, delta_Y) / (np.linalg.norm(Y) * np.linalg.norm(delta_Y))))
                angle_Z = math.degrees(np.arccos(np.dot(Z, delta_Z) / (np.linalg.norm(Z) * np.linalg.norm(delta_Z))))

                print('angle_X', angle_X, ' angle_Y', angle_Y, ' angle_Z', angle_Z)

                
                phi_upper_rad   = math.atan2( 2 * (q_upper.w * q_upper.x + q_upper.y * q_upper.z), 1 - 2 * (q_upper.x**2 + q_upper.y**2) )
                theta_upper_rad = math.asin ( 2 * (q_upper.w * q_upper.y - q_upper.z * q_upper.x) )
                psi_upper_rad   = math.atan2( 2 * (q_upper.w * q_upper.z + q_upper.x * q_upper.y), 1 - 2 * (q_upper.y**2 + q_upper.z**2) )

                phi_upper_deg = math.degrees(phi_upper_rad)
                theta_upper_deg = math.degrees(theta_upper_rad)
                psi_upper_deg = math.degrees(psi_upper_rad)

                print('phi_upper_deg', phi_upper_deg, 'theta_upper_deg', theta_upper_deg, 'psi_upper_deg', psi_upper_deg)
                
                phi_lower_rad   = math.atan2( 2 * (q_lower.w * q_lower.x + q_lower.y * q_lower.z), 1 - 2 * (q_lower.x**2 + q_lower.y**2) )
                theta_lower_rad = math.asin ( 2 * (q_lower.w * q_lower.y - q_lower.z * q_lower.x) )
                psi_lower_rad   = math.atan2( 2 * (q_lower.w * q_lower.z + q_lower.x * q_lower.y), 1 - 2 * (q_lower.y**2 + q_lower.z**2) )

                phi_lower_deg = math.degrees(phi_lower_rad)
                theta_lower_deg = math.degrees(theta_lower_rad)
                psi_lower_deg = math.degrees(psi_lower_rad)

                print('phi_lower_deg', phi_lower_deg, 'theta_lower_deg', theta_lower_deg, 'psi_lower_deg', psi_lower_deg)
                """

                # Calculate Euler angles from this difference quaternion
                # phi_rad   = math.atan2( 2 * (qd.w * qd.x + qd.y * qd.z), 1 - 2 * (qd.x**2 + qd.y**2) )
                # theta_rad = math.asin ( 2 * (qd.w * qd.y - qd.z * qd.x) )
                # psi_rad   = math.atan2( 2 * (qd.w * qd.z + qd.x * qd.y), 1 - 2 * (qd.y**2 + qd.z**2) )

                # phi_deg = math.degrees(phi_rad)  #X
                # theta_deg = math.degrees(theta_rad) #Y
                # psi_deg = math.degrees(psi_rad) #Z
                # maxx1 = abs(math.degrees(qd.angle))


        """""
        for trial in range(1,quanti_trial+1):
            df_sub = delta[delta['trial_number'] == trial] #seleziono subset x ogni trial
            df_sub.reset_index(drop=True,inplace=True)
            df_sub = df_sub.loc[:,['X','Y','Z']]   #seleziono colonne con rotazioni
            somma_rotazioni.append(df_sub.sum())

            # print('Il soggetto ha ruotato',somma_rotazioni[trial-1][0],'° lungo asse X in trial', trial)
            # print('Il soggetto ha ruotato',somma_rotazioni[trial-1][1],'° lungo asse Y in trial', trial)
            # print('Il soggetto ha ruotato',somma_rotazioni[trial-1][2],'° lungo asse Z in trial', trial)

        #converto in numpy arrays
        somma_rotazioni = np.asarray(somma_rotazioni)
        """

        """"
        return angle_X, angle_Y, angle_Z
    
        return phi_upper_deg, theta_upper_deg, psi_upper_deg
        
        return phi_lower_deg, theta_lower_deg, psi_lower_deg
        """
        return phi_qd_deg, theta_qd_deg, psi_qd_deg

    def set_on_grab_callback(self, on_grab):
        """""
        What to do when the cube is grasped
        @param on_grab: function in format event_trigger()
        @return:
        """
        self.on_grab = on_grab

    def set_callback(self, event, callback):
        self.mapping_event_to_callback[event] = callback

    def set_on_touch_callback(self, on_touch):
        """
        What to do when the cube is touched
        @param on_touch: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch = on_touch

    def set_on_turn_clockwise_callback(self, on_turn_clockwise):
        """
        What to do when the cube is turned clockwise
        @param on_turn_clockwise: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_turn_clockwise = on_turn_clockwise

    def set_on_turn_anticlockwise_callback(self, on_turn_anticlockwise):
        """
        What to do when the cube is turned anticlockwise
        @param on_turn_anticlockwise: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_turn_anticlockwise = on_turn_anticlockwise

    def set_on_rotate_right_callback(self, on_rotate_right):
        """
        What to do when the cube is rotated right
        @param on_rotate_right: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_rotate_right = on_rotate_right

    def set_on_rotate_left_callback(self, on_rotate_left):
        """
        What to do when the cube is rotated left
        @param on_rotate_left: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_rotate_left = on_rotate_left

    def set_on_rotate_forward_callback(self, on_rotate_forward):
        """
        What to do when the cube is rotated forward
        @param on_rotate_forward: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_rotate_forward = on_rotate_forward

    def set_on_rotate_backward_callback(self, on_rotate_backward):
        """
        What to do when the cube is rotated backward
        @param on_rotate_backward: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_rotate_backward = on_rotate_backward

    def set_on_touch_rightface_callback(self, on_touch_rightface):
        """
        What to do when the cube is touched right face
        @param on_touch_rightface: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_rightface = on_touch_rightface

    def set_on_touch_leftface_callback(self, on_touch_leftface):
        """
        What to do when the cube is touched left face
        @param on_touch_leftface: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_leftface = on_touch_leftface

    def set_on_touch_firstface_topleft_callback(self, on_touch_firstface_topleft):
        """
        What to do when the cube is touched the top left part of the first face
        @param on_touch_firstface_topleft: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_firstface_topleft = on_touch_firstface_topleft

    def set_on_touch_firstface_topright_callback(self, on_touch_firstface_topright):
        """
        What to do when the cube is touched the top right part of the first face
        @param on_touch_firstface_topright: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_firstface_topright = on_touch_firstface_topright

    def set_on_touch_firstface_bottomleft_callback(self, on_touch_firstface_bottomleft):
        """
        What to do when the cube is touched the bottom left part of the first face
        @param on_touch_firstface_bottomleft: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_firstface_bottomleft = on_touch_firstface_bottomleft

    def set_on_touch_firstface_bottomright_callback(self, on_touch_firstface_bottomright):
        """
        What to do when the cube is touched the bottom right part of the first face
        @param on_touch_firstface_bottomright: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_firstface_bottomright = on_touch_firstface_bottomright

    def set_on_touch_topface_callback(self, on_touch_topface):
        """
        What to do when the cube is touched top face
        @param on_touch_topface: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_topface = on_touch_topface

    def set_on_idle_callback (self, on_idle):
        """
        What to do when there is the end of the rotation of the cube and the state became neutral
        @param on_idle: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_idle = on_idle

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

    def __icube_right_face (self, touches):
        """
        Classify if the iCube is posed based on touches
        If only one face is fully active the cube is posed somewhere
        Otherwise the cube is held. Touching only one element of the right face, activates the function.
        @param touches: a set of touches form the iCube
        @return: True if touched
        """
        if touches is None:
            return False
        full_covered_faces = touches.count(touches[1])
        touched_faces = ["1" in t for t in touches].count(True)
        return full_covered_faces == 1 and touched_faces == 2

    def __icube_left_face (self, touches):
        """
        Classify if the iCube is posed based on touches
        If only one face is fully active the cube is posed somewhere
        Otherwise the cube is held. Touching only one element of the left face, activates the function.
        @param touches: a set of touches form the iCube
        @return: True if touched
        """
        if touches is None:
            return False
        full_covered_faces = touches.count(touches[4])
        touched_faces = ["1" in t for t in touches].count(True)
        return full_covered_faces == 1 and touched_faces == 2

    def __icube_top_face (self, touches):
        """
        Classify if the iCube is posed based on touches
        If only one face is fully active the cube is posed somewhere
        Otherwise the cube is held. Touching only one element of the top face, activates the function.
        @param touches: a set of touches form the iCube
        @return: True if touched
        """
        if touches is None:
            return False
        full_covered_faces = touches.count(touches[0])
        touched_faces = ["1" in t for t in touches].count(True)
        return full_covered_faces == 1 and touched_faces == 2

    def __icube_corner_face (self, touches):
        """""
        Classify if the iCube is posed based on touches
        If only one face is fully active the cube is posed somewhere
        Otherwise the cube is held. Touching the 4 corners of the top face, activates the function.
        @param touches: a set of touches form the iCube
        @return: True if touched
        """
        if touches is None:
            return False

        full_covered_faces = touches.count(touches[0])
        
        if touches[0] == '1000000000000000':
            print ('top_left', touches[0])
            return 'top_left'
        if touches[0] == '0001000000000000':
            print ('top_right', touches[0])
            return 'top_right'
        if touches[0] == '0000000000001000':
            print ('bottom_left', touches[0])
            return 'bottom_left'
        if touches[0] == '0000000000000001':
            print ('bottom_right', touches[0])
            return 'bottom_right'

    def handle(self, quaternions, touches, accelerometer):
        """
        Classifies participants' behavior
        @param quaternions:
        @param touches:
        @param accelerometer:
        @return:
        """

        # define the function variables
        delta = 0.8
        threshold_zero = 0
        threshold = 4

        print('quaternions', quaternions)
        print ('touches', touches)

        # handling quaternions
        # convertion from quaternions to angles

        quanti_trial_value = 1
        """""
        print('Il dataframe ha', quanti_trial_value, 'trials')
        """
        "print('quaternions_old', self.quaternions_old)"

        phi_qd, theta_qd, psi_qd = self.compute_angles(self.quaternions_old, quaternions, quanti_trial_value)
        "print ('phi_qd', phi_qd, 'theta_qd', theta_qd, 'psi_qd', psi_qd)"

        self.quaternions_old = quaternions

        # handling accelerometer
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

        """""
        if self.icube_state == MovementState.POSED:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.GRABBED
                self.on_grab()
        """

        "from pose to different rotation"
        if self.icube_state == MovementState.POSED:
            if phi_qd < threshold_zero and theta_qd > threshold_zero and psi_qd < -threshold:
                self.icube_state = MovementState.ROTATE_RIGHT
                self.on_rotate_right()

        if self.icube_state == MovementState.POSED:
            if phi_qd > threshold_zero and theta_qd < threshold_zero and psi_qd > +threshold:
                self.icube_state = MovementState.ROTATE_LEFT
                self.on_rotate_left()

        if self.icube_state == MovementState.POSED:
            if phi_qd < threshold_zero and theta_qd > +threshold and psi_qd < threshold_zero:
                self.icube_state = MovementState.ROTATE_FORWARD
                self.on_rotate_forward()

        if self.icube_state == MovementState.POSED:
            if phi_qd > threshold_zero and theta_qd < -threshold and psi_qd < threshold_zero:
                self.icube_state = MovementState.ROTATE_BACKWARD
                self.on_rotate_backward()

        if self.icube_state == MovementState.POSED:
            if phi_qd < -threshold and theta_qd < threshold_zero and psi_qd < threshold_zero:
                self.icube_state = MovementState.TURNED_CLOCKWISE
                self.on_turn_clockwise()

        if self.icube_state == MovementState.IDLE:
            if phi_qd > +threshold and theta_qd > threshold_zero and psi_qd > threshold_zero:
                 self.icube_state = MovementState.TURNED_ANTICLOCKWISE
                 self.on_turn_anticlockwise()

        # right rotation of the cube"
        if self.icube_state == MovementState.ROTATE_RIGHT:
            if psi_qd < 0 + delta and psi_qd > 0 - delta:
                self.icube_state = MovementState.IDLE
                self.on_idle()

        if self.icube_state == MovementState.IDLE:
            if phi_qd < threshold_zero and theta_qd > threshold_zero and psi_qd < -threshold:
                self.icube_state = MovementState.ROTATE_RIGHT
                self.on_rotate_right()

        # left rotation of the cube
        if self.icube_state == MovementState.IDLE:
            if phi_qd > threshold_zero and theta_qd < threshold_zero and psi_qd > +threshold:
                self.icube_state = MovementState.ROTATE_LEFT
                self.on_rotate_left()

        if self.icube_state == MovementState.ROTATE_LEFT:
            if psi_qd < 0 + delta and psi_qd > 0 - delta:
                self.icube_state = MovementState.IDLE
                self.on_idle()


        # forward rotation of the cube
        if self.icube_state == MovementState.IDLE:
            if phi_qd < threshold_zero and theta_qd > +threshold and psi_qd < threshold_zero:
                self.icube_state = MovementState.ROTATE_FORWARD
                self.on_rotate_forward()

        if self.icube_state == MovementState.ROTATE_FORWARD:
            if theta_qd < 0 + delta and theta_qd > 0 - delta:
                self.icube_state = MovementState.IDLE
                self.on_idle()

        # backward rotation of the cube
        if self.icube_state == MovementState.IDLE:
            if phi_qd > threshold_zero and theta_qd < -threshold and psi_qd < threshold_zero:
                self.icube_state = MovementState.ROTATE_BACKWARD
                self.on_rotate_backward()

        if self.icube_state == MovementState.ROTATE_BACKWARD:
            if theta_qd < 0 + delta and theta_qd > 0 - delta:
                self.icube_state = MovementState.IDLE
                self.on_idle()

        # clockwise turn of the cube"
        if self.icube_state == MovementState.IDLE:
            if phi_qd < -threshold and theta_qd < threshold_zero and psi_qd < threshold_zero:
                self.icube_state = MovementState.TURNED_CLOCKWISE
                self.on_turn_clockwise()

        if self.icube_state == MovementState.TURNED_CLOCKWISE:
            if phi_qd < 0 + delta and phi_qd > 0 - delta:
                self.icube_state = MovementState.IDLE
                self.on_idle()

        # anticlockwise turned of the cube"
        if self.icube_state == MovementState.IDLE:
            if phi_qd > +threshold and theta_qd > threshold_zero and psi_qd > threshold_zero:
                 self.icube_state = MovementState.TURNED_ANTICLOCKWISE
                 self.on_turn_anticlockwise()

        if self.icube_state == MovementState.TURNED_ANTICLOCKWISE:
            if phi_qd < 0 + delta and phi_qd > 0 - delta:
                self.icube_state = MovementState.IDLE
                self.on_idle()

        # stop rotation"
        if self.icube_state == MovementState.IDLE:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.on_pose()

        """""
        "touch top face"
        if self.icube_state == MovementState.POSED:
            if self.__icube_top_face(touches):
                if self.__icube_corner_face(touches) == 'tl':
                    "topleft"
                    self.icube_state = MovementState.TOUCH_FIRSTFACE_UPLEFT
                    self.on_touch_firstface_upleft()
                elif self.__icube_corner_face(touches) == 'tr':
                    "topright"
                    self.icube_state = MovementState.TOUCH_FIRSTFACE_UPRIGHT
                    self.on_touch_firstface_upright()
                elif self.__icube_corner_face(touches) == 'bl':
                    "bottomleft"
                    self.icube_state = MovementState.TOUCH_FIRSTFACE_DOWNLEFT
                    self.on_touch_firstface_downleft()
                elif self.__icube_corner_face(touches) == 'br':
                    "bottomright"
                    self.icube_state = MovementState.TOUCH_FIRSTFACE_DOWNRIGHT
                    self.on_touch_firstface_downright()
                else:
                    self.icube_state = MovementState.TOUCH_TOPFACE
                    self.on_touch_topface()

        if self.icube_state == MovementState.TOUCH_TOPFACE:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.on_pose()
            """

        # touch the top left part of the top face"
        if self.icube_state == MovementState.POSED:
            if self.__icube_corner_face(touches) == 'top_left':
                self.icube_state = MovementState.TOUCH_FIRSTFACE_TOPLEFT
                self.on_touch_firstface_topleft()
                print ('<<<<<<<<<touch', touches[0])

        if self.icube_state == MovementState.TOUCH_FIRSTFACE_TOPLEFT:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.on_pose()

        # touch the top right part of the top face"
        if self.icube_state == MovementState.POSED:
            if self.__icube_corner_face(touches) == 'top_right':
                self.icube_state = MovementState.TOUCH_FIRSTFACE_TOPRIGHT
                self.on_touch_firstface_topright()
                print ('<<<<<<<<<touch', touches[0])

        if self.icube_state == MovementState.TOUCH_FIRSTFACE_TOPRIGHT:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.on_pose()

        # touch the bottom left part of the top face"
        if self.icube_state == MovementState.POSED:
            if self.__icube_corner_face(touches) == 'bottom_left':
                self.icube_state = MovementState.TOUCH_FIRSTFACE_BOTTOMLEFT
                self.on_touch_firstface_bottomleft()
                print ('<<<<<<<<<touch', touches[0])

        if self.icube_state == MovementState.TOUCH_FIRSTFACE_BOTTOMLEFT:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.on_pose()

        # touch the bottom right part of the top face"
        if self.icube_state == MovementState.POSED:
            if self.__icube_corner_face(touches) == 'bottom_right':
                self.icube_state = MovementState.TOUCH_FIRSTFACE_BOTTOMRIGHT
                self.on_touch_firstface_bottomright()
                print ('<<<<<<<<<touch', touches[0])

        if self.icube_state == MovementState.TOUCH_FIRSTFACE_BOTTOMRIGHT:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.on_pose()

        # touch right face"
        if self.icube_state == MovementState.POSED:
            if self.__icube_right_face(touches):
                self.icube_state = MovementState.TOUCH_RIGHTFACE
                self.on_touch_rightface()

        if self.icube_state == MovementState.TOUCH_RIGHTFACE:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.on_pose()

        # touch left face"
        if self.icube_state == MovementState.POSED:
            if self.__icube_left_face(touches):
                self.icube_state = MovementState.TOUCH_LEFTFACE
                self.on_touch_leftface()

        if self.icube_state == MovementState.TOUCH_LEFTFACE:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.on_pose()
        # touch top face
        if self.icube_state == MovementState.POSED:
            if self.__icube_top_face(touches):
                self.icube_state = MovementState.TOUCH_TOPFACE
                self.on_touch_topface()

        if self.icube_state == MovementState.TOUCH_TOPFACE:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.on_pose()


        self.init_acc = np_acc

