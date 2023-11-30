# Collection of modules to classify the interaction with the iCube

## Available Classifiers

| Name  | Description                                                                                                                  | Events                                                                   | Parameters                                                                                                                                                                                                                                                                                                                                                                         |
|-------|------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| base  | detect the basic interactions with the iCube like grabbing, posing and moving it                                             | ICubeBaseEvents.GRAB / POSE / MOVE                                       | `grab_tolerance=1.5 (sensitivity on detecting a GRAB)`                                                                                                                                                                                                                                                                                                                             |
| swipe | Detect swipe gestures on the iCube faces. **It assumes the iCube to be placed with the 0 face on top and straight IIT logo** | SwipeEvents.RIGHT / LEFT / FORWARD / BACKWARD / COVER=face fully covered | `swipe_min_duration=1.5 (minimum gesture duration in seconds)`, `threshold_multiplier=0.2 (sensitivity on detecting a swipe)`, `min_sequence_lenght=2 (minimum number of sequential touches to try detecting a swipe)`, `detect_only_when_grabbed=False (perform the detection only if the cube is grabbed)`, `detect_on_face=None/0-5 (detect swipes only on the selected face)` |
|       |                                                                                                                              |                                                                          |                                                                                                                                                                                                                                                                                                                                                                                    |


## Installation

To download and install system-wise
```bash
git clone https://gitlab.iit.it/cognitiveInteraction/icube-movements-classifier.git
pip install .
```

To install in an environment
```bash
cd <project/path>
source venv/bin/activate
(venv) pip install icube-movements-classifier@git+https://gitlab.iit.it/cognitiveInteraction/icube-movements-classifier
```

## Usage
```python
from icube_movements_classifier.IMCFactory import IMCFactory
from icube_movements_classifier import ICubeBaseEvents
from icube_movements_classifier.swipes import SwipeEvents

from icube.data_handlers import CallbackAggregator, AggregateMode

# Init the Classifier Factory
imc_factory = IMCFactory()
print(imc_factory.list_available_classifiers())
# ['base', 'double_touch', 'swipes']

# Get the desired classifier, and initialize it with the proper params
classifier = imc_factory.get_movement_classifier("swipes")(grab_tolerance=1.5, detect_on_face=5)

callbacks_aggregator = CallbackAggregator(AggregateMode.SYNC)

# Set the callbacks to Base Events
classifier.set_callback(ICubeBaseEvents.GRAB, (lambda x: print("Event GRAB!")))
classifier.set_callback(ICubeBaseEvents.POSE, (lambda x: print("Event POSE!")))
classifier.set_callback(ICubeBaseEvents.MOVE, (lambda x: print("Event MOVE!")))

# Set the callbacks to Swipe Events
classifier.set_callback(SwipeEvents.RIGHT, (lambda x: print(f"Event RIGHT! on face {x}")))
classifier.set_callback(SwipeEvents.LEFT, (lambda x: print(f"Event LEFT! on face {x}")))
classifier.set_callback(SwipeEvents.FORWARD, (lambda x: print(f"Event FORWARD! on face {x}")))
classifier.set_callback(SwipeEvents.BACKWARD, (lambda x: print(f"Event BACKWARD! on face {x}")))
classifier.set_callback(SwipeEvents.COVER, (lambda x: print(f"Event COVER! on face {x}")))

callbacks_aggregator.add_callback(classifier.handle)

...

```

## Development

To develop your own classifier, take the double full touch as an example, and:
1. Create a class with an **Unique Name** that extends the `MovementsDetector` or another existing classifier. This allows you to reuse the events it already defines
2. Implement the method `handle` with the detections you need to perform
3. Remember to call the `super().handle(quaternions, touches, accelerometer)` to pass the iCube data to the superclass
4. Add your class to the `IMCFactory` dictionary using a meaningful and short name for the classifier (avoid spaces)
5. Update the table in this `README.md` file with your classifier info