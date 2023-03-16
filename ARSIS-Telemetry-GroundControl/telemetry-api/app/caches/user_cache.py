class UserCache:
    def __init__(self):
        self.users = {}

    def create_new_user_dict(self):
        bpm = 100
        o2 = 100
        battery = 100
        latitude = 10
        longitude = 100
        altitude = 0
        heading = 0
        return {
            "biometrics": {
                "bpm": bpm, 
                "o2": o2, 
                "battery": battery,
            },
            "location": {
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude,
                "heading": heading,
            },
        }

    def get_all(self):
        return self.users

    def get(self, user_id):
        return self.users.get(user_id, None)

    def register(self, user_id):
        if user_id in self.users:
            return None
        self.users[user_id] = self.create_new_user_dict()
        return user_id

    def update_location(self, user_id, new_location):
        if user_id not in self.users:
            return None
        self.users[user_id]["location"] = { "latitude": new_location.latitude, "longitude": new_location.longitude, "altitude": new_location.altitude, "heading": new_location.heading }
        return new_location

    def update_biometrics(self, user_id, new_biometrics):
        if user_id not in self.users:
            return None
        self.users[user_id]["biometrics"] = { "o2": new_biometrics.o2, "battery": new_biometrics.battery, "bpm": new_biometrics.bpm }
        return new_biometrics