import unittest
from numpy import array, zeros, eye, sin, cos, dot, ndarray
from ArborisTests import BaseTest
from numpy.linalg import norm
from arboris.core import World, Body
from arboris.joints import RxJoint, RyJoint, RzJoint, RzRyRxJoint, FreeJoint, RzRyJoint, RzRyJoint, RzRxJoint, RyRxJoint
import arboris.homogeneousmatrix
from arboris.homogeneousmatrix import rotx
from arboris.core import Joint, LinearConfigurationSpaceJoint

class JointsTestCase(BaseTest):

    def test(self):
        world = World()
        Ba = Body()
        Rzyx = RzRyRxJoint()
        world.add_link(world.ground, Rzyx, Ba)
        Bzy = Body(name='zy')
        Byx = Body(name='yx')
        Bb = Body()
        Rz = RzJoint()
        world.add_link(world.ground, Rz, Bzy)
        Ry = RyJoint()
        world.add_link(Bzy, Ry, Byx)
        Rx = RxJoint()
        world.add_link(Byx, Rx, Bb)
        world.init()
        (az, ay, ax) = (3.14/6, 3.14/4, 3.14/3)
        Rzyx.gpos[:] = (az, ay, ax)
        (Rz.gpos[0], Ry.gpos[0], Rx.gpos[0]) = (az, ay, ax)
        world.update_dynamic()
        Ja = Ba.jacobian[:, 0:3]
        Jb = Bb.jacobian[:, 3:6]
        self.assertTrue(norm(Ja-Jb) < 1e-10)

    def testConstruction(self):
        a = FreeJoint()
        a = RzRyRxJoint()
        a = RzRyJoint()
        a = RzRxJoint()
        a = RyRxJoint()
        a = RzJoint()
        a = RyJoint()
        a = RxJoint()

    def testFreeJoints(self):
        j = FreeJoint()
        self.assertListsAlmostEqual(j.gpos,([[ 1.,  0.,  0.,  0.],
               [ 0.,  1.,  0.,  0.],
               [ 0.,  0.,  1.,  0.],
               [ 0.,  0.,  0.,  1.]]))

        self.assertListsAlmostEqual(j.gvel,([ 0.,  0.,  0.,  0.,  0.,  0.]))

    def testRzRyRxJoint(self):
        r = RzRyRxJoint()
        self.assertListsAlmostEqual(r.jacobian,(
        [[ -sin(r.gpos[1]), 0. , 1. ],
         [  sin(r.gpos[2])*cos(r.gpos[1]) ,cos(r.gpos[2])  , 0. ],
         [  cos(r.gpos[2])*cos(r.gpos[1]) , -sin(r.gpos[2])  , 0. ],
         [  0.    ,   0. , 0. ],
         [  0.    ,   0. , 0. ],
         [  0.    ,   0. , 0. ]]))

        self.assertListsAlmostEqual(r.djacobian, (
        [[-(r.gvel[1])*cos(r.gpos[1]) , 0.    ,  0. ],
         [(r.gvel[2])*cos(r.gpos[2])*cos(r.gpos[1])-
         (r.gvel[1])*sin(r.gpos[2])*sin(r.gpos[1]) ,
         -(r.gvel[2])*sin(r.gpos[2]) ,  0. ],
         [-(r.gvel[2])*sin(r.gpos[2])*cos(r.gpos[1])-
         (r.gvel[1])*cos(r.gpos[2])*sin(r.gpos[1]) ,
         -(r.gvel[2])*cos(r.gpos[2]) ,  0. ],
         [ 0.                ,  0.   ,  0. ],
         [ 0.                ,  0.   ,  0. ],
         [ 0.                ,  0.   ,  0. ]]))

    def testRzRyJoint(self):
        r = RzRyJoint()
        self.assertListsAlmostEqual(r.jacobian,(
            [[ -sin(r.gpos[1]) , 0. ],
             [  0. , 1. ],
             [  cos(r.gpos[1]) , 0. ],
             [  0. , 0. ],
             [  0. , 0. ],
             [  0. , 0. ]]))
        self.assertListsAlmostEqual(r.djacobian,    (
                [[ -(r.gvel[1])*cos(r.gpos[1]) , 0.  ],
                 [  0.    , 0.  ],
                 [ -(r.gvel[1])*sin(r.gpos[1]) , 0.  ],
                 [   0.   , 0.  ],
                 [   0.   , 0.  ],
                 [   0.   , 0.  ]]))

    def testRzRxJoint(self):
        r = RzRxJoint()
        self.assertListsAlmostEqual(r.jacobian,(
            [[ 0. , 1. ],
             [ sin(r.gpos[1]) , 0. ],
             [ cos(r.gpos[1]) , 0. ],
             [ 0. , 0. ],
             [ 0. , 0. ],
             [ 0. , 0. ]]))
        self.assertListsAlmostEqual(r.djacobian,    (
                [[  0.    , 0.  ],
                 [  (r.gvel[1])*cos(r.gpos[1]) , 0.  ],
                 [ -(r.gvel[1])*sin(r.gpos[1]) , 0.  ],
                 [   0.   , 0.  ],
                 [   0.   , 0.  ],
                 [   0.   , 0.  ]]))

    def testRyRxJoint(self):
        r = RyRxJoint()
        self.assertListsAlmostEqual(r.jacobian,(
            [[  0. , 1. ],
             [  cos(r.gpos[1]) , 0. ],
             [ -sin(r.gpos[1]) , 0. ],
             [  0. , 0. ],
             [  0. , 0. ],
             [  0. , 0. ]]))
        self.assertListsAlmostEqual(r.djacobian,(
                [[ 0.    , 0. ],
                 [-(r.gvel[1])*sin(r.gpos[1]) , 0. ],
                 [-(r.gvel[1])*cos(r.gpos[1]) , 0. ],
                 [ 0.    , 0. ],
                 [ 0.    , 0. ],
                 [ 0.    , 0. ]]))

    def testRzJoint(self):
        j = RzJoint(gpos = 3.14/2., gvel = 1.)
        self.assertListsAlmostEqual(j.pose,(
        [[  7.96326711e-04,  -9.99999683e-01,   0.00000000e+00,
                  0.00000000e+00],
         [  9.99999683e-01,   7.96326711e-04,   0.00000000e+00,
                  0.00000000e+00],
         [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,
                  0.00000000e+00],
         [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                  1.00000000e+00]]))
        self.assertListsAlmostEqual(j.ipose,(
        [[  7.96326711e-04,   9.99999683e-01,   0.00000000e+00,
                          0.00000000e+00],
         [ -9.99999683e-01,   7.96326711e-04,   0.00000000e+00,
                          0.00000000e+00],
         [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,
                          0.00000000e+00],
         [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                          1.00000000e+00]]))
        self.assertListsAlmostEqual(j.jacobian,(
        [[ 0.],[ 0.],[ 1.],[ 0.],[ 0.],[ 0.]]))

    def testRyJoint(self):
        j = RyJoint()
        self.assertListsAlmostEqual(j.pose, rotx(j.gpos[0]))
        self.assertListsAlmostEqual(j.ipose, rotx(-j.gpos[0]))
        self.assertListsAlmostEqual(j.jacobian, (
        [[0.], [1.], [0.], [0.], [0.], [0.]]))
        self.assertListsAlmostEqual(j.djacobian, zeros((6,1)))

    def testRxJoint(self):
        j = RxJoint()
        self.assertListsAlmostEqual(j.pose, rotx(j.gpos[0]))
        self.assertListsAlmostEqual(j.ipose, rotx(-j.gpos[0]))
        self.assertListsAlmostEqual(j.jacobian, (
        [[1.], [0.], [0.], [0.], [0.], [0.]]))
        self.assertListsAlmostEqual(j.djacobian, zeros((6,1)))


ts = unittest.TestSuite()
ts.addTest(JointsTestCase('test'))
ts.addTest(JointsTestCase('testConstruction'))
ts.addTest(JointsTestCase('testFreeJoints'))
ts.addTest(JointsTestCase('testRzRyRxJoint'))
ts.addTest(JointsTestCase('testRzRyJoint'))
ts.addTest(JointsTestCase('testRzRxJoint'))
ts.addTest(JointsTestCase('testRyRxJoint'))
ts.addTest(JointsTestCase('testRzJoint'))
ts.addTest(JointsTestCase('testRyJoint'))
ts.addTest(JointsTestCase('testRxJoint'))