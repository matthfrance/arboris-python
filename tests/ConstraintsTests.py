import unittest
from ArborisTests import BaseTest
from numpy import arange, eye
from arboris.constraints import JointLimits
from arboris.controllers import WeightController
from arboris.controllers import WeightController, BallAndSocketConstraint 
from arboris.core import simplearm, simulate, Body, World
from arboris.joints import FreeJoint

class ConstraintsTestCase(BaseTest):

    def testJoinLimits(self):
        world = simplearm()
        a = WeightController(world)
        world.register(a)
        joints = world.getjoints()
        c = JointLimits(joints['Shoulder'], -3.14/2, 3.14/2)
        world.register(c)
        joints['Shoulder'].gpos[0] = 3.14/2 - 0.1
        time = arange(0., 0.1, 1e-3)
        simulate(world, time)
        self.assertTrue(3.14/2 > joints['Shoulder'].gpos[0])
        joints['Shoulder'].gpos[0] = -3.14/2 + 0.1
        time = arange(0., 0.1, 1e-3)
        simulate(world, time)
        self.assertTrue(-3.14/2 < joints['Shoulder'].gpos[0])


ts = unittest.TestSuite()
ts.addTest(ConstraintsTestCase('testJoinLimits'))
