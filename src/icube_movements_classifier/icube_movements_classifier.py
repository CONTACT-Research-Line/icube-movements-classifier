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
    TURNED_RIGHT = 6
    TURNED_LEFT = 7
    TURNED_UPWARD = 8
    TURNED_DOWNWARD = 9
    TOUCH_RIGHTFACEF5 = 10
    TOUCH_LEFTFACEF6 = 11
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
        self.quaternions_old = 0.0

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
        self.on_touch_rightfaceF5 = None
        self.on_touch_leftfaceF6 = None
        self.on_touch_upleftF1 = None
        self.on_touch_uprightF2 = None
        self.on_touch_downleftF3 = None
        self.on_touch_downrightF4 = None

    def compute_angles(self, quaternions_old, quaternions, quanti_trial):

        print('Sto calcolando le rotazioni...')

        "definisco assi assoluti"
        X = [1, 0, 0]
        Y = [0, 1, 0]
        Z = [0, 0, 1]

        delta = []

        """""
        df_quat = df.loc[:,['trial_number','quaternion_w','quaternion_x','quaternion_y','quaternion_z']]
        df_quat = df_quat.astype('float')

        #converto trial number in intero
        df_quat['trial_number'] = df_quat['trial_number'].astype('int')
        """

        for trial in range(1,quanti_trial+1):
            """""
            df_sub = df_quat[df_quat['trial_number'] == trial] #seleziono subset x ogni trial

            lista = df_sub.iloc[:,1:].values.tolist()

            for i in range(len(lista)-1):
            """
            for i in range(1,2):
                """""
                "start position of the cube"
                q1x = 0.028300000354647636
                q1y = -0.72009998559951782
                q1z = 0.0057000000961124897
                q1w = 0.69330000877380371
                q2x = 0.028300000354647636
                q2y = -0.72009998559951782
                q2z = 0.0057000000961124897
                q2w = 0.69330000877380371
                """
                """""
                "turn right the cube of 90°"
                q1x = 0.019300000742077827
                q1y = -0.72030001878738403
                q1z = 0.0013000000035390258
                q1w = 0.69340002536773682
                q2x = 0.52310001850128174
                q2y = -0.49180001020431519
                q2z = -0.48879998922348022
                q2w = 0.49559998512268066
                """
                """
                "turn left the cube of -90°"
                q1x = -0.21420000493526459
                q1y = -0.67479997873306274
                q1z = -0.23059999942779541
                q1w = 0.66750001907348633
                q2x = -0.63569998741149902
                q2y = -0.30880001187324524
                q2z = 0.28690001368522644
                q2w = 0.64670002460479736
                """
                """""
                "turn upward the cube"
                q1x = -0.25260001420974731
                q1y = -0.65950000286102295
                q1z = -0.26589998602867126
                q1w = 0.65609997510910034
                q2x = -0.0059000002220273018
                q2y = 0.0020000000949949026
                q2z = -0.30120000243186951
                q2w = 0.95359998941421509
                """
                """""
                "turn downward the cube"
                q1x = -0.18719999492168427
                q1y = -0.68040001392364502
                q1z = -0.2020999938249588
                q1w = 0.67909997701644897
                q2x = -0.33410000801086426
                q2y = -0.94230002164840698
                q2z = -0.0013000000035390258
                q2w = 0.019799999892711639
                """

                q1w = 0.69290000200271606
                q1x = 0.021199999377131462
                q1y = -0.72079998254776001
                q1z = 0.00030000001424923539

                q_calibration = pyp.Quaternion(q1w, q1x, q1y, q1z)

                print('q_calibration', q_calibration)

                "print ('q_calibration_w', q_calibration.w, 'q_calibration_x', q_calibration.x, 'q_calibration_y', q_calibration.y, 'q_calibration_z', q_calibration.z)"

                q_calib = q_calibration * q_calibration.conjugate
                print('q_calib', q_calib)

                "q_upper = pyp.Quaternion(q1w,q1x,q1y,q1z)"
                print('quat_old', pyp.Quaternion(quaternions_old))

                q_upper = pyp.Quaternion(quaternions_old) * q_calibration.conjugate
                print('q_upper', q_upper)

                "q_lower = pyp.Quaternion(q2w,q2x,q2y,q2z)"
                q_lower = pyp.Quaternion(quaternions)
                print('q_lower', q_lower)

                """
                q_upper = pyq.Quaternion(lista[i][0],lista[i][1],lista[i][2],lista[i][3])
                q_lower = pyq.Quaternion(lista[i+1][0],lista[i+1][1],lista[i+1][2],lista[i+1][3])
                
                print('q_upper_w', q_upper.w, 'q_upper_x', q_upper.x, 'q_upper_y', q_upper.y, 'q_upper_z', q_upper.z)
                print('q_lower_w', q_lower.w, 'q_lower_x', q_lower.x, 'q_lower_y', q_lower.y, 'q_lower_z', q_lower.z)
                """

                "Get the 3D difference between these two orientations"
                qd = q_upper.conjugate * q_lower
                print('qd', qd)

                phi_qd_rad  = math.atan2( 2 * (qd.w * qd.x + qd.y * qd.z), 1 - 2 * (qd.x**2 + qd.y**2) )
                theta_qd_rad = math.asin ( 2 * (qd.w * qd.y - qd.z * qd.x) )
                psi_qd_rad   = math.atan2( 2 * (qd.w * qd.z + qd.x * qd.y), 1 - 2 * (qd.y**2 + qd.z**2) )

                phi_qd_deg = math.degrees(phi_qd_rad)
                theta_qd_deg = math.degrees(theta_qd_rad)
                psi_qd_deg = math.degrees(psi_qd_rad)

                print('phi_qd_deg', phi_qd_deg, 'theta_qd_deg', theta_qd_deg, 'psi_qd_deg', psi_qd_deg)

                "ruoto assi"

                delta_X = qd.rotate(X)
                delta_Y = qd.rotate(Y)
                delta_Z = qd.rotate(Z)

                print('delta_X', delta_X, 'delta_Y', delta_Y, 'delta_Z', delta_Z)

                "angolo tra due vettori"
                """""
                angle_X = math.degrees(np.arccos(np.dot(X, delta_X) / (np.linalg.norm(X) * np.linalg.norm(delta_X))))
                angle_Y = math.degrees(np.arccos(np.dot(Y, delta_Y) / (np.linalg.norm(Y) * np.linalg.norm(delta_Y))))
                angle_Z = math.degrees(np.arccos(np.dot(Z, delta_Z) / (np.linalg.norm(Z) * np.linalg.norm(delta_Z))))

                print('angle_X', angle_X, ' angle_Y', angle_Y, ' angle_Z', angle_Z)
                """
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

                # Calculate Euler angles from this difference quaternion
                # phi_rad   = math.atan2( 2 * (qd.w * qd.x + qd.y * qd.z), 1 - 2 * (qd.x**2 + qd.y**2) )
                # theta_rad = math.asin ( 2 * (qd.w * qd.y - qd.z * qd.x) )
                # psi_rad   = math.atan2( 2 * (qd.w * qd.z + qd.x * qd.y), 1 - 2 * (qd.y**2 + qd.z**2) )

                # phi_deg = math.degrees(phi_rad)  #X
                # theta_deg = math.degrees(theta_rad) #Y
                # psi_deg = math.degrees(psi_rad) #Z
                # maxx1 = abs(math.degrees(qd.angle))
                """""
                delta.append([trial, angle_X, angle_Y, angle_Z])
                
                delta.append([angle_X,angle_Y,angle_Z])
                """
        """""
        #converto in dataframe
        delta = pd.DataFrame(delta)
        delta.rename(columns={0: "trial_number", 1: 'X', 2: 'Y', 3: 'Z'},inplace=True)
        delta['trial_number'] = delta['trial_number'].astype('int')
        
        #inizializzo liste rotazioni orarie e antiorarie
        somma_rotazioni = []
        
        
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

        return delta, somma_rotazioni
        
 
        return angle_X, angle_Y, angle_Z
        """
        return phi_upper_deg, theta_upper_deg, psi_upper_deg
        return phi_lower_deg, theta_lower_deg, psi_lower_deg
        return phi_qd_deg, theta_qd_deg, psi_qd_deg

    def set_on_grab_callback(self, on_grab):
        """""
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

    def set_on_touch_rightfaceF5_callback(self, on_touch_rightfaceF5):
        """
        What to do when the cube is touched right face F5
        @param on_touch_rightfaceF5: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_rightfaceF5 = on_touch_rightfaceF5

    def set_on_touch_leftfaceF6_callback(self, on_touch_leftfaceF6):
        """
        What to do when the cube is touched left face F6
        @param on_touch_leftfaceF6: function in format event_trigger(delta_acceleration)
        @return:
        """
        self.on_touch_leftfaceF6 = on_touch_leftfaceF6

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

        print('quaternions', quaternions)


        "handling quaternions"
        "convertion from quaternions to angles"
        quanti_trial_value = 1
        print('Il dataframe ha', quanti_trial_value, 'trials')
        """""
        "turn right the cube of 90°"
        q1x = 0.019300000742077827
        q1y = -0.72030001878738403
        q1z = 0.0013000000035390258
        q1w = 0.69340002536773682
        
        q2x = 0.52310001850128174
        q2y = -0.49180001020431519
        q2z = -0.48879998922348022
        q2w = 0.49559998512268066
        """

        print('quaternions_old', self.quaternions_old)
        """""
        angleX, angleY, angleZ = self.compute_angles(self.quaternions_old, quaternions, quanti_trial_value)
        """
        phi_upper_deg, theta_upper_deg, psi_upper_deg = self.compute_angles(self.quaternions_old, quaternions, quanti_trial_value)
        phi_lower_deg, theta_lower_deg, psi_lower_deg = self.compute_angles(self.quaternions_old, quaternions, quanti_trial_value)
        phi_qd_deg, theta_qd_deg, psi_qd_deg = self.compute_angles(self.quaternions_old, quaternions, quanti_trial_value)

        self.quaternions_old = quaternions

        "handling accelerometer"
        if accelerometer is None or accelerometer == []:
            return False

        if self.icube_state == MovementState.UNKNOWN:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.on_pose()
        """"
            else:
                self.icube_state = MovementState.GRABBED
                self.on_grab()
        """

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

        if self.icube_state == MovementState.TURNED_RIGHT:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TURNED_LEFT
                self.on_turn_left()

        if self.icube_state == MovementState.TURNED_LEFT:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TURNED_UPWARD
                self.on_turn_upward()

        if self.icube_state == MovementState.TURNED_UPWARD:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TURNED_DOWNWARD
                self.on_turn_downward()

        if self.icube_state == MovementState.TURNED_DOWNWARD:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TOUCH_RIGHTFACEF5
                self.on_touch_rightfaceF5()

        if self.icube_state == MovementState.TOUCH_RIGHTFACEF5:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TOUCH_LEFTFACEF6
                self.on_touch_leftfaceF6()

        if self.icube_state == MovementState.TOUCH_LEFTFACEF6:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TOUCH_UPLEFTF1
                self.on_touch_upleftF1()

        if self.icube_state == MovementState.TOUCH_UPLEFTF1:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TOUCH_UPRIGHTF2
                self.on_touch_uprightF2()

        if self.icube_state == MovementState.TOUCH_UPRIGHTF2:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TOUCH_DOWNLEFTF3
                self.on_touch_downleftF3()

        if self.icube_state == MovementState.TOUCH_DOWNLEFTF3:
            if self.delta_movement > self.grab_tolerance and not self.__icube_posed(touches):
                self.icube_state = MovementState.TOUCH_DOWNRIGHTF4
                self.on_touch_downrightF4()

        if self.icube_state in [MovementState.GRABBED, MovementState.TOUCH_DOWNRIGHTF4]:
            if self.__icube_posed(touches):
                self.icube_state = MovementState.POSED
                self.on_pose()

        self.init_acc = np_acc
