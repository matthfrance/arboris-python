# -*- encoding: UTF-8 -*-

import unittest
from arboris.homogeneousmatrix import *
from numpy.linalg import norm
from ArborisTests import BaseTest
from numpy import arange, eye
from arboris.constraints import JointLimits, BallAndSocketConstraint 
from arboris.controllers import WeightController
from arboris.core import simplearm, simulate, Body, World
from arboris.joints import FreeJoint


class HomogeneousMatrixTestCase(BaseTest):
	
	def test(self):
		(az, ay, ax) = (3.14/6, 3.14/4, 3.14/3)
		self.assertTrue(norm(rotzyx(az,ay,ax) - dot(rotz(az),dot(roty(ay),rotx(ax)))) < 1e-10)


ts = unittest.TestSuite()
ts.addTest(HomogeneousMatrixTestCase('test'))