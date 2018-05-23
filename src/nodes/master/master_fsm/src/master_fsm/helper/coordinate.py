from math import sqrt, pi, atan2, cos

class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __abs__(self):
        return sqrt(self.x ** 2, self.y ** 2)
    
    def phase(self):
        return atan2(self.x, self.y)
    
    def phaseDeg(self):
        return self.phase() * 180.0 / pi
    
    def unit(self):
        return self / self.abs()
    
    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)
    
    def __mul__(self, scale):
        return Vector(self.x * scale, self.y * scale)

    def __truediv__(self, scale):
        return Vector(self.x / scale, self.y / scale)

class Coordinate:
    # reference point: Rodeheaver
    refLatitude  = 34.874264
    refLongitude = -82.363416

    def __init__(self, latitude, longitude):
        self.latitude  = latitude
        self.longitude = longitude
    
    def fromLocal(x, y):
        lat_const  = 2.0 * pi * (6371000.0 / 360.0)
        long_const = lat_const * cos(Coordinate.refLatitude / 360.0 * 2.0 * pi)
        lat  = Coordinate.refLatitude  + lat_const  * y
        long = Coordinate.refLongitude + long_const * x
        return Coordinate(lat, long)
    
    def localX(self):
        lat_const  = 2.0 * pi * (6371000.0 / 360.0)
        long_const = lat_const * cos(Coordinate.refLatitude / 360.0 * 2.0 * pi)
        return (self.longitude - Coordinate.refLongitude) * long_const
    
    def localY(self):
        lat_const  = 2.0 * pi * (6371000.0 / 360.0)
        return (self.latitude - Coordinate.refLatitude) * lat_const
    
    def getLatitude(self):
        return self.latitude
    
    def getLongitude(self):
        return self.longitude
    
    def __sub__(self, other):
        return Vector(self.localX() - other.localX(), self.localY() - other.localY())
