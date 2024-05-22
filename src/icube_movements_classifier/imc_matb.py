import enum
import math
import pyquaternion as pyp

from icube_movements_classifier import GrabPoseDetector

CUBE_POSED_FACE = "1111111111111111"


class MatbMovementEvents(enum.Enum):
    TOUCH_RIGHTFACE = 0
    TOUCH_LEFTFACE = 1
    TOUCH_FRONTFACE = 2
    TOUCH_BACKFACE = 3

    TOUCH_TOPFACE_TOPLEFT = 4
    TOUCH_TOPFACE_TOPRIGHT = 5
    TOUCH_TOPFACE_BOTTOMLEFT = 6
    TOUCH_TOPFACE_BOTTOMRIGHT = 7

    ROTATE_RIGHT = 8
    ROTATE_LEFT = 9
    ROTATE_FORWARD = 10
    ROTATE_BACKWARD = 11

    TURNED_CLOCKWISE = 12
    TURNED_ANTICLOCKWISE = 13

    # TOUCHED = 4
    # TURNED = 5

    # TOUCH_TOPFACE = 18
    # IDLE = 19


def compute_angles(quaternions_old, quaternions):
    # print('Compute Rotations...')

    # definisco assi assoluti
    X = [1, 0, 0]
    Y = [0, 1, 0]
    Z = [0, 0, 1]

    q_upper = pyp.Quaternion(quaternions_old)

    q_lower = pyp.Quaternion(quaternions)

    # Get the 3D difference between these two orientations
    qd = q_upper.conjugate * q_lower

    # Calculate Euler angles from this difference quaternion
    phi_qd_rad = math.atan2(2 * (qd.w * qd.x + qd.y * qd.z), 1 - 2 * (qd.x ** 2 + qd.y ** 2))
    theta_qd_rad = math.asin(2 * (qd.w * qd.y - qd.z * qd.x))
    psi_qd_rad = math.atan2(2 * (qd.w * qd.z + qd.x * qd.y), 1 - 2 * (qd.y ** 2 + qd.z ** 2))

    phi_qd_deg = math.degrees(phi_qd_rad)  # X
    theta_qd_deg = math.degrees(theta_qd_rad)  # Y
    psi_qd_deg = math.degrees(psi_qd_rad)  # Z

    # print('phi_qd_deg', phi_qd_deg, 'theta_qd_deg', theta_qd_deg, 'psi_qd_deg', psi_qd_deg)

    # Rotate the Axis
    delta_X = qd.rotate(X)
    delta_Y = qd.rotate(Y)
    delta_Z = qd.rotate(Z)

    # print('delta_X', delta_X, 'delta_Y', delta_Y, 'delta_Z', delta_Z)

    return phi_qd_deg, theta_qd_deg, psi_qd_deg


def detect_touch_on_face(touches):
    """
    Touch only one element of the right face, or left face or top face, activates the function.
    Used this function when the iCube is posed or also when the iCube is grabbed.
    @param touches: a set of touches form the iCube
    @return: True if touched the right face or the left face.
    """

    if touches is None:
        return False

    if touches[1].count('1') > 0:
        return 'right'
    if touches[4].count('1') > 0:
        return 'left'
    if touches[2].count('1') > 0:
        return "front"
    if touches[3].count('1') > 0:
        return "back"

    return None


def detect_corners_touch(touches, face_id=0):
    """
    Touching the 4 corners (top left, top right, bottom left, bottom right) of the top face, activates the function.
    Used this function when the iCube is posed or also when the iCube is grabbed.
    This function checks if there is at least a '1' in the specific positions for 'top_left', 'top_right', 'bottom_left' and 'bottom_right'.
    @param touches: a set of touches form the iCube
    @return: True if touched
    """

    if touches is None:
        return False

    if touches[face_id][:2].count('1') > 0 or touches[face_id][4:6].count('1') > 0:
        return 'top_left'
    elif touches[face_id][2:4].count('1') > 0 or touches[face_id][6:8].count('1') > 0:
        return 'top_right'
    elif touches[face_id][8:10].count('1') > 0 or touches[face_id][12:14].count('1') > 0:
        return 'bottom_left'
    elif touches[face_id][10:12].count('1') > 0 or touches[face_id][14:].count('1') > 0:
        return 'bottom_right'

    return None


class MatbMovementDetector(GrabPoseDetector):
    def __init__(self, grab_tolerance=1, threshold=8, threshold_zero=0):
        super().__init__(grab_tolerance=grab_tolerance)
        self.quaternions_old = []
        self.threshold = threshold
        self.threshold_zero = threshold_zero

        print("""
        ======================= MATB MOVEMENT DETECTOR - WARNING =============================
        This ICube Classifier assumes that you are holding the iCube with the charger
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

        self.mapping_touch_on_face = {
            'right': MatbMovementEvents.TOUCH_RIGHTFACE,
            'left': MatbMovementEvents.TOUCH_LEFTFACE,
            'front': MatbMovementEvents.TOUCH_FRONTFACE,
            'back': MatbMovementEvents.TOUCH_BACKFACE,
        }

        self.mapping_touch_corners = {
            'top_right': MatbMovementEvents.TOUCH_TOPFACE_TOPLEFT,
            'top_left': MatbMovementEvents.TOUCH_TOPFACE_TOPRIGHT,
            'bottom_right': MatbMovementEvents.TOUCH_TOPFACE_BOTTOMLEFT,
            'bottom_left': MatbMovementEvents.TOUCH_TOPFACE_BOTTOMRIGHT,
        }

    def handle(self, quaternions, touches, accelerometer):
        """
        Classifies participants' behavior
        @param quaternions:
        @param touches:
        @param accelerometer:
        @return:0
        """
        # Detect if the iCube is grabbed or posed
        super().handle(quaternions, touches, accelerometer)

        # Compute iCube Rotation
        phi_qd, theta_qd, psi_qd = compute_angles(self.quaternions_old, quaternions)
        self.quaternions_old = quaternions

        # Check touches for when POSED
        side_touch_event = detect_touch_on_face(touches)
        corner_touch_event = detect_corners_touch(touches, face_id=0)

        if self.icube_state != GrabPoseDetector.State.POSED:
            return

        # Check Rotation Events
        # right rotation
        if phi_qd < -self.threshold and theta_qd < self.threshold_zero and psi_qd < self.threshold_zero:
            self.fire(MatbMovementEvents.ROTATE_RIGHT)
        # left rotation
        elif phi_qd > self.threshold and theta_qd > self.threshold_zero and psi_qd > self.threshold_zero:
            self.fire(MatbMovementEvents.ROTATE_LEFT)
        # forward rotation
        elif phi_qd < self.threshold_zero and theta_qd > + self.threshold and psi_qd < self.threshold_zero:
            self.fire(MatbMovementEvents.ROTATE_FORWARD)
        # backward rotation
        elif phi_qd > self.threshold_zero > psi_qd and theta_qd < - self.threshold:
            self.fire(MatbMovementEvents.ROTATE_BACKWARD)

        # Check Corner touch events
        if corner_touch_event is not None:
            self.fire(self.mapping_touch_corners[corner_touch_event])

        # Check Side touch events
        if side_touch_event is not None:
            self.fire(self.mapping_touch_on_face[side_touch_event])
