import enum
import time

import numpy as np
from icube_movements_classifier import MovementState, MovementsDetector


FULLY_COVERED_FACE = "1111111111111111"


class SwipeEvents(enum.Enum):
    COVER = 0,
    FORWARD = 1,
    BACKWARD = 2,
    RIGHT = 3,
    LEFT = 4
    LONG_PRESS = 5

N_FACES = 6
N_ROWS = 4
N_COLS = 4


class SwipeDetector(MovementsDetector):
    """
    @package SwipeDetector
    @brief a detector of the swipes for VOJEXT like we do with the Flex-TS
    @author Dario Pasquali
    """

    def __init__(self, grab_tolerance=1,
                 swipe_min_duration=1.5,
                 threshold_multiplier=0.2,
                 min_sequence_length=2,
                 detect_only_when_grabbed=False,                
                 detect_on_face=None,
                 long_press_duration=2.0):
        """
        @param grab_tolerance: how much being tolerant on classifying an acceleration as grasping
        @param swipe_min_duration: classify a gesture every N seconds
        """
        super().__init__()
        self.init_acc = None
        self.delta_movement = 0.0

        self.icube_state = MovementState.UNKNOWN
        self.grab_tolerance = grab_tolerance

        self.swipe_min_duration = swipe_min_duration
        
        self.threshold_multiplier = threshold_multiplier
        self.min_sequence_length = min_sequence_length
        
        self.detect_only_when_grabbed = detect_only_when_grabbed
        self.face_to_detect_on = -1
        
        self.long_press_duration = long_press_duration
        self.long_press_accumulator = [
            [], [], [], [], [], []
        ]
        self.long_press_last_time = time.time()

        if detect_on_face is not None:
            if self.face_to_detect_on < 0:
                raise ValueError(f"[SWIPE DETECTOR] detect_on_face must be > 0, provided {detect_on_face}")
            if self.face_to_detect_on > 5:
                raise ValueError(f"[SWIPE DETECTOR] detect_on_face must be <= 5, provided {detect_on_face}")
            
            self.face_to_detect_on = detect_on_face

        
        self.mapping_event_to_callback = {}

        self.sequences_by_face = [
            [], [], [], [], [], []
        ]
        self.swipe_last_time = time.time()

        print("""
        ======================= SWIPE DETECTOR - WARNING =============================
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

    def set_callback(self, event, callback):
        if event in SwipeEvents and not event in self.mapping_event_to_callback:
            self.mapping_event_to_callback[event] = callback
        else:
            super().set_callback(event, callback)

    def __touches_to_sequences(self, touches):
        try:
            [self.sequences_by_face[f].extend([(x, y)
                                                for y in range(N_ROWS)
                                                for x in range(N_COLS) if touches[f] is not None and len(touches[f]) > 0 and touches[f][N_ROWS*x + y] == '1' and (x,y) not in self.sequences_by_face[f]
                                                ]) for f in range(N_FACES)]
        except Exception as e:
            print(e)

    def __fire(self, event, face_id):
        if event in self.mapping_event_to_callback:
            self.mapping_event_to_callback[event](face_id)

    def __classify_swipe_single_face(self, sequence):

        # Check if I have enough sequential touches
        if len(sequence) < self.min_sequence_length:
            return None

        threshold = self.threshold_multiplier * len(sequence)
        np_x = np.array([float(x) for (x, _) in sequence], dtype=np.float16)
        np_y = np.array([float(y) for (_, y) in sequence], dtype=np.float16)

        std_x = np.std(np_x)
        std_y = np.std(np_y)
        increasing_x = all(x < y for x, y in zip(np_x, np_x[1:]))
        increasing_y = all(x < y for x, y in zip(np_y, np_y[1:]))
        
        # print(sequence, threshold, std_x, std_y, increasing_x, increasing_y)

        if std_x > threshold:
            if increasing_x:
                print("BACKWARD")
                return SwipeEvents.BACKWARD
            print("FORWARD")
            return SwipeEvents.FORWARD

        if std_y > threshold:
            if increasing_y:
                print("RIGHT")
                return SwipeEvents.RIGHT
            print("LEFT")
            return SwipeEvents.LEFT

        return None


    def handle(self, quaternions, touches, accelerometer):
        super().handle(quaternions, touches, accelerometer)        

        # Update the gestures
        self.__touches_to_sequences(touches)

        if self.detect_only_when_grabbed and self.icube_state == MovementState.POSED:
            return

        try:
            
            if self.face_to_detect_on >= 0:
                if touches[self.face_to_detect_on] == FULLY_COVERED_FACE:
                    self.__fire(SwipeEvents.COVER, self.face_to_detect_on)
                    self.long_press_accumulator[self.face_to_detect_on].append(SwipeEvents.COVER)
                    if (time.time()-self.long_press_last_time) >= self.long_press_duration:
                        if self.long_press_accumulator[self.face_to_detect_on].count(SwipeEvents.COVER) == len(self.long_press_accumulator[self.face_to_detect_on]):
                            self.__fire(SwipeEvents.LONG_PRESS, self.face_to_detect_on)
                        self.long_press_accumulator[self.face_to_detect_on] = []
                        self.long_press_last_time = 0.0
                    
            else:
                for face_id, face_touch in enumerate(touches):
                    if face_touch == FULLY_COVERED_FACE:
                        self.__fire(SwipeEvents.COVER, face_id)
                
        except TypeError as e:
            print(e)

        # If at lease time passed
        if (time.time()-self.swipe_last_time) >= self.swipe_min_duration:
            # Check the gestures and fire the events
            try:
                if self.face_to_detect_on >= 0:
                    event = self.__classify_swipe_single_face(self.sequences_by_face[self.face_to_detect_on])
                    if event is not None and event in self.mapping_event_to_callback:
                        self.__fire(event, self.face_to_detect_on)
                        self.long_press_accumulator[self.face_to_detect_on].append(event)
                else:
                    for face, seq in enumerate(self.sequences_by_face):
                       event = self.__classify_swipe_single_face(seq)
                       if event is not None and event in self.mapping_event_to_callback:
                           self.__fire(event, face)
            except Exception as e:
                print(e)

            # Reset space and time
            self.sequences_by_face = [
                [], [], [], [], [], []
            ]
            self.swipe_last_time = time.time()



