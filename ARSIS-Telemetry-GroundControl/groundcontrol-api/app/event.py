import time


class Event:
    @staticmethod
    def create_event(event_type, data, upsert_key=""):
        return {
            "type": event_type,
            "label": time.time_ns(),
            "data": data,
            "upsert": upsert_key,
        }
