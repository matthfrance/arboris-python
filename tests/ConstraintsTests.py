import unittest
from ArborisTests import BaseTest
from numpy import arange, eye
from arboris.constraints import JointLimits, BallAndSocketConstraint
from arboris.constraints import SoftFingerContact
from arboris.controllers import WeightController
from arboris.core import simplearm, simulate, Body, World
from arboris.joints import FreeJoint
from arboris.robots.simpleshapes import add_sphere


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

    def testBallAndSocketConstraint(self):
        b0 = Body(mass=eye(6))
        w = World()
        w.add_link(w.ground, FreeJoint(), b0)
        b1 = Body(mass=eye(6))
        w.add_link(b0, FreeJoint(), b1)
        w.init()
        ctrl =  WeightController(w)
        w.register(ctrl)
        c0 = BallAndSocketConstraint(frames=(w.ground, b0))
        w.register(c0)
        w.init()
        w.update_dynamic()
        dt = 0.001
        w.update_controllers(dt)
        w.update_constraints(dt)
        self.assertListsAlmostEqual(c0._force, [ 0.  ,  9.81,  0.  ])
        w.integrate(dt)
        w.update_dynamic()
        self.assertListsAlmostEqual(b0.pose,
            [[1.0, 0.0, 0.0, 0.0],
             [0.0, 1.0, 0.0, -3.09167026e-22],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]])

    def testSoftFingerContact(self):
        world = World()
        add_sphere(world, name='ball0')
        add_sphere(world, name='ball1')
        bodies = world.getbodies()
        bodies['ball0'].parentjoint.gpos[1,3] = 2.5
        bodies['ball0'].parentjoint.gvel[4] = -1.
        c = SoftFingerContact(world._shapes, 0.1)
        world.register(c)
        world.init()
        world.update_dynamic()
        dt = 0.001
        world.update_controllers(dt)
        world.update_constraints(dt)
        world.integrate(dt)
        world.update_dynamic()
        self.assertListsAlmostEqual(c.jacobian,
            [ [ 0., 1.,  0.,  0., 0., 0.,  0.   , -1. , 0.    , 0.,  0.,  0.],
              [-1., 0.,  0.,  0., 0., 1., -1.499,  0. , 0.    , 0.,  0., -1.],
              [ 0., 0., -1., -1., 0., 0.,  0.   ,  0. , -1.499, 1.,  0.,  0.],
              [ 0., 0.,  0.,  0., 1., 0.,  0.   ,  0. , 0.    , 0., -1.,  0.]])
        self.assertListsAlmostEqual(bodies['ball0'].pose,
            [ [ 1.   ,  0.   ,  0.   ,  0.   ],
              [ 0.   ,  1.   ,  0.   ,  2.499],
              [ 0.   ,  0.   ,  1.   ,  0.   ],
              [ 0.   ,  0.   ,  0.   ,  1.   ] ])


ts = unittest.TestSuite()
ts.addTest(ConstraintsTestCase('testJoinLimits'))
ts.addTest(ConstraintsTestCase('testBallAndSocketConstraint'))
ts.addTest(ConstraintsTestCase('testSoftFingerContact'))
