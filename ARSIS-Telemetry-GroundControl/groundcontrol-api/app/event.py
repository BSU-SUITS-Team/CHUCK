import time


class Event:
    @staticmethod
    def create_event(event_type, data):
        return {
            "type": event_type,
            "time": time.time_ns(),
            "data": data,
        }
