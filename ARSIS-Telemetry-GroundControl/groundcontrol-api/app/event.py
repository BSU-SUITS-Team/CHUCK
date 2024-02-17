import time


class Event:
    @staticmethod
    def create_event(event_type, data, label=None):
        return {
            "type": event_type,
            "label": label or time.time_ns(),
            "data": data,
        }
