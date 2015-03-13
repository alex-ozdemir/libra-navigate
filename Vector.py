# Alex Ozdemir
# aozdemir@hmc.edu

# Vector class
from math import sqrt, atan2
from operator import add
from string import join

class Vector(tuple):
    def __init__(self, xs):
        self.superclass = tuple
        self.superclass.__init__(xs)
        self.size = self.superclass.__len__(self)
        self.length = sqrt(reduce(add, [x ** 2 for x in self], 0))
    def __len__(self):
        return self.length
    def __add__(self, other):
        return Vector([self[i] + other[i] for i in range(self.size)])
    def __sub__(self, other):
        return Vector([self[i] - other[i] for i in range(self.size)])
    def __neg__(self):
        return Vector([-self[i] for i in range(self.size)])
    def dot(self, other):
        assert(self.size == other.size)
        return sum([p[0]*p[1] for p in zip(self, other)])
    def scale(self, scalar):
        return Vector([float(x) * scalar for x in self])
    def norm(self):
        return self.scale( 1. / self.length)
    def proj(self, other):
        return other.norm().scale(float(self.dot(other)) / other.length)
    def reject(self, other):
        return self - self.proj(other)
    def angle(self):
        return atan2(self[1], self[0])
    def __repr__(self):
        return "Vector(%s)" % (self.superclass.__repr__(self))
    def __str__(self):
        return "<" + join([str(x) for x in self],", ") + ">"

x = Vector((1, 0))
x5 = Vector((5, 0))
v = Vector((2, 1))
