# coding=utf-8
"""
Frames...

TODO: add ellipse and pill shapes
"""

__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@crans.org>")

from core import Shape

class Point(Shape):
    """
    A point
    """
    def __init__(self, frame, name=None):
        Shape.__init__(self, frame, name)


class Box(Shape):
    """
    A box
    """
    def __init__(self, frame, lengths=(1.,1.,1.), name=None):
        Shape.__init__(self, frame, name)
        self.lengths = lengths


class Cylinder(Shape):
    """
    A cylinder
    """
    def __init__(self, frame, length=1., radius=1., name=None):
        Shape.__init__(self, frame, name)
        self.radius = radius
        self.length = length


class Sphere(Shape):
    """
    A sphere
    """
    def __init__(self, frame, radius=1., name=None):
        Shape.__init__(self, frame, name)
        self.radius = radius

