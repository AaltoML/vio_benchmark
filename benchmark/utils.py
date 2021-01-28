import math

class GpsToLocalConverter:
    initialized = False
    lat0 = 0
    lon0 = 0
    z0 = 0
    metersPerLonDeg = 0
    prev = None

    def __init__(self, prev={"x": 0, "y": 0, "z": 0}):
        self.prev = prev

    def convert(self, latitude, longitude, altitude=0, accuracy=1.0, minAccuracy=-1.0, **kwargs):
        # Filter out inaccurate measurements to make pose alignment easier.
        if (minAccuracy > 0.0 and (accuracy > minAccuracy or accuracy < 0.0)):
            return self.prev

        EARTH_CIRCUMFERENCE_EQUATORIAL = 40075.017e3
        EARTH_CIRCUMFERENCE_POLAR = 40007.863e3
        METERS_PER_LAT_DEG = EARTH_CIRCUMFERENCE_POLAR / (2 * math.pi)
        if not self.initialized:
            self.lat0 = latitude
            self.lon0 = longitude
            self.z0 = altitude
            self.metersPerLonDeg = math.cos(math.radians(self.lat0)) * EARTH_CIRCUMFERENCE_EQUATORIAL / (2 * math.pi)

        self.initialized = True
        vec = {
            "x": self.metersPerLonDeg * (math.radians(longitude - self.lon0)),
            "y": METERS_PER_LAT_DEG * (math.radians(latitude - self.lat0)),
            "z": altitude - self.z0
        }
        self.prev = vec
        return vec
