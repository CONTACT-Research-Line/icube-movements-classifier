class BaseEventHandler:
    def __init__(self):
        super().__init__()
        self.mapping_event_to_callback = {}

    def handle(self, quaternions, touches, accelerometer):
        pass

    def quit(self):
        pass

    def set_callback(self, event, callback):
        self.mapping_event_to_callback[event] = callback

    def fire(self, event, args=None):
        if event in self.mapping_event_to_callback:
            if args is not None:
                self.mapping_event_to_callback[event](args)
            else:
                self.mapping_event_to_callback[event]()
