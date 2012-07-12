from collections import namedtuple

class V3(namedtuple('V3Base', 'x y z')):

    def cross(self, other):
        return V3((self.y * other.z) - (self.z * other.y), (self.z * other.x) - (self.x * other.z), (self.x * other.y) - (self.y * other.x))

    def __add__(self, other):
        return V3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return V3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other):
        if isinstance(other, float) or isinstance(other, int):
            return V3(self.x * other, self.y * other, self.z * other)
        elif isinstance(other, V3):
            return V3(self.x * other.x, self.y * other.y, self.z * other.z)
        else:
            raise ValueError()

    def __div__(self, other):
        return self * (1.0 / other)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __repr__(self):
        return "(%f, %f, %f)" % (self.x, self.y, self.z)


class M3(object):
    pass