import unittest
from ArborisTests import BaseTest
from arboris.core import Body, SubFrame, World
from arboris.homogeneousmatrix import transl
from arboris.joints import RyJoint, RzRxJoint, RyRxJoint, RzRyRxJoint
from arboris.robots.simplearm import add_simplearm
from numpy import eye, zeros


class WorldTestCase(BaseTest):

    def testConstruction(self):
        w = World('ArborisLand')


class TestLinks(WorldTestCase):

    def testLinksCreation(self):
        w = World()
        bulb = RyRxJoint(name='bulb')
        stem = Body(name='stem', mass=.1)
        w.add_link(w.ground, bulb, stem)

        stem_top = SubFrame(stem, bpose=transl(0, 0, 1), name='stem_top')
        flowerR = RyJoint(gpos=.5, name='flowerR')
        leafR = Body('leafR', mass=.01)
        w.add_link(stem_top, flowerR, leafR)

        flowerL = RyJoint(gpos=(-.5), name='flowerL')
        leafL = Body('leafL', mass=.01)
        w.add_link(stem_top, flowerL, leafL)

        frames = [ w.ground, stem, leafR, leafL, stem_top ]
        for frame1, frame2 in zip(w.iterframes(), frames):
            self.assertEqual(frame1, frame2)

        joints = [ bulb, flowerR, flowerL ]
        for joint1, joint2 in zip(w.iterjoints(), joints):
            self.assertEqual(joint1, joint2)

    def testLinksReplacement(self):
        w = World()
        bulb = RyRxJoint(name='bulb')
        stem = Body(name='stem', mass=.1)
        w.add_link(w.ground, bulb, stem)

        new_bulb = RzRxJoint(name='bulb')
        w.replace_joint(bulb, new_bulb) # By another joint
        joints = w.getjoints()
        self.assertEqual(len(joints), 1)
        self.assertEqual(joints[0], new_bulb)

        new_new_bulb = RzRyRxJoint(name='bulb')
        w.replace_joint(new_bulb, w.ground, new_new_bulb, stem) # By a kinematic chain
        joints = w.getjoints()
        self.assertEqual(len(joints), 1)
        self.assertEqual(joints[0], new_new_bulb)


class TestUpdates(WorldTestCase):

    def testGeometricUpdate(self):
        w = World()
        add_simplearm(w)
        bodies = w.getbodies()
        arm = bodies['Arm']
        forearm = bodies['ForeArm']
        hand = bodies['Hand']
        w.update_geometric()
        self.assertListsAlmostEqual(arm.pose, eye(4))
        self.assertListsAlmostEqual(forearm.pose, [ [ 1. ,  0. ,  0. ,  0. ],
                                                    [ 0. ,  1. ,  0. ,  0.5],
                                                    [ 0. ,  0. ,  1. ,  0. ],
                                                    [ 0. ,  0. ,  0. ,  1. ] ])
        self.assertListsAlmostEqual(hand.pose, [ [ 1. ,  0. ,  0. ,  0. ],
                                                 [ 0. ,  1. ,  0. ,  0.9],
                                                 [ 0. ,  0. ,  1. ,  0. ],
                                                 [ 0. ,  0. ,  0. ,  1. ] ])

    def testDynamicUpdate(self):
        w = World()
        add_simplearm(w)
        joints  = w.getjoints()
        joints[0].gpos[0] = 0.5
        joints[0].gvel[0] = 2.5
        joints[1].gpos[0] = 1.0
        joints[1].gvel[0] = -1.0
        joints[2].gpos[0] = 2.0/3.0
        joints[2].gvel[0] = -0.5
        w.update_dynamic()
        bodies = w.getbodies()
        self.assertListsAlmostEqual(bodies['Arm'].pose,
            [[ 0.87758256, -0.47942554,  0.        ,  0.        ],
             [ 0.47942554,  0.87758256,  0.        ,  0.        ],
             [ 0.        ,  0.        ,  1.        ,  0.        ],
             [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.assertListsAlmostEqual(bodies['ForeArm'].pose,
            [[ 0.0707372 , -0.99749499,  0.        , -0.23971277],
             [ 0.99749499,  0.0707372 ,  0.        ,  0.43879128],
             [ 0.        ,  0.        ,  1.        ,  0.        ],
             [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.assertListsAlmostEqual(bodies['Hand'].pose,
            [[-0.56122931, -0.82766035,  0.        , -0.63871076],
             [ 0.82766035, -0.56122931,  0.        ,  0.46708616],
             [ 0.        ,  0.        ,  1.        ,  0.        ],
             [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.assertListsAlmostEqual(bodies['ground'].jacobian,
            [[ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.]])
        self.assertListsAlmostEqual(bodies['Arm'].jacobian,
            [[ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 1.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.]])
        self.assertListsAlmostEqual(bodies['ForeArm'].jacobian,
            [[ 0.        ,  0.        ,  0.        ],
             [ 0.        ,  0.        ,  0.        ],
             [ 1.        ,  1.        ,  0.        ],
             [-0.27015115,  0.        ,  0.        ],
             [ 0.42073549,  0.        ,  0.        ],
             [ 0.        ,  0.        ,  0.        ]])
        self.assertListsAlmostEqual(bodies['Hand'].jacobian,
            [[ 0.        ,  0.        ,  0.        ],
             [ 0.        ,  0.        ,  0.        ],
             [ 1.        ,  1.        ,  1.        ],
             [-0.26649313, -0.3143549 ,  0.        ],
             [ 0.7450519 ,  0.24734792,  0.        ],
             [ 0.        ,  0.        ,  0.        ]])
        self.assertListsAlmostEqual(bodies['ground'].twist,
            [ 0.,  0.,  0.,  0.,  0.,  0.])
        self.assertListsAlmostEqual(bodies['Arm'].twist,
            [ 0. ,  0. ,  2.5,  0. ,  0. ,  0. ])
        self.assertListsAlmostEqual(bodies['ForeArm'].twist,
            [ 0., 0., 1.5, -0.67537788, 1.05183873, 0. ])
        self.assertListsAlmostEqual(bodies['Hand'].twist,
            [ 0., 0., 1., -0.35187792, 1.61528183, 0. ])
        self.assertListsAlmostEqual(w.viscosity,
            [[ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.]])
        self.assertListsAlmostEqual(bodies['Arm'].mass,
            [[  8.35416667e-02,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   2.50000000e-01],
             [  0.00000000e+00,   4.16666667e-04,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,   8.35416667e-02,
               -2.50000000e-01,   0.00000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,  -2.50000000e-01,
                1.00000000e+00,   0.00000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   1.00000000e+00,   0.00000000e+00],
             [  2.50000000e-01,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
        self.assertListsAlmostEqual(bodies['ForeArm'].mass,
            [[  4.27733333e-02,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   1.60000000e-01],
             [  0.00000000e+00,   2.13333333e-04,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,   4.27733333e-02,
               -1.60000000e-01,   0.00000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,  -1.60000000e-01,
                8.00000000e-01,   0.00000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   8.00000000e-01,   0.00000000e+00],
             [  1.60000000e-01,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   8.00000000e-01]])
        self.assertListsAlmostEqual(bodies['Hand'].mass,
            [[  2.67333333e-03,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   2.00000000e-02],
             [  0.00000000e+00,   1.33333333e-05,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,   2.67333333e-03,
               -2.00000000e-02,   0.00000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,  -2.00000000e-02,
                2.00000000e-01,   0.00000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   2.00000000e-01,   0.00000000e+00],
             [  2.00000000e-02,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   2.00000000e-01]])
        self.assertListsAlmostEqual(w.mass,
            [[ 0.55132061,  0.1538999 ,  0.0080032 ],
             [ 0.1538999 ,  0.09002086,  0.00896043],
             [ 0.0080032 ,  0.00896043,  0.00267333]])
        self.assertListsAlmostEqual(bodies['ground'].djacobian,
            [[ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.]])
        self.assertListsAlmostEqual(bodies['Arm'].djacobian,
            [[ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.],
             [ 0.,  0.,  0.]])
        self.assertListsAlmostEqual(bodies['ForeArm'].djacobian,
            [[ 0.        ,  0.        ,  0.        ],
             [ 0.        ,  0.        ,  0.        ],
             [ 0.        ,  0.        ,  0.        ],
             [-0.42073549,  0.        ,  0.        ],
             [-0.27015115,  0.        ,  0.        ],
             [ 0.        ,  0.        ,  0.        ]])
        self.assertListsAlmostEqual(bodies['Hand'].djacobian,
            [[ 0.        ,  0.        ,  0.        ],
             [ 0.        ,  0.        ,  0.        ],
             [ 0.        ,  0.        ,  0.        ],
             [-0.87022993, -0.12367396,  0.        ],
             [-0.08538479, -0.15717745,  0.        ],
             [ 0.        ,  0.        ,  0.        ]])
        self.assertListsAlmostEqual(bodies['ground'].nleffects,
            [[ 0.,  0.,  0.,  0.,  0.,  0.],
             [ 0.,  0.,  0.,  0.,  0.,  0.],
             [ 0.,  0.,  0.,  0.,  0.,  0.],
             [ 0.,  0.,  0.,  0.,  0.,  0.],
             [ 0.,  0.,  0.,  0.,  0.,  0.],
             [ 0.,  0.,  0.,  0.,  0.,  0.]])
        self.assertListsAlmostEqual(bodies['Arm'].nleffects,
            [[  0.00000000e+00,  -1.04166667e-03,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
             [  5.26041667e-02,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   6.25000000e-01,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,  -2.50000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,  -6.25000000e-01,
                2.50000000e+00,   0.00000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   0.00000000e+00]])
        self.assertListsAlmostEqual(bodies['ForeArm'].nleffects,
            [[  0.00000000e+00,  -3.20000000e-04,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
             [  1.61600000e-02,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,  -2.22261445e-18],
             [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   2.40000000e-01,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,  -1.20000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,  -2.40000000e-01,
                1.20000000e+00,   0.00000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   0.00000000e+00]])
        self.assertListsAlmostEqual(bodies['Hand'].nleffects,
            [[  0.00000000e+00,  -1.33333333e-05,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
             [  6.73333333e-04,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,  -1.10961316e-18],
             [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   2.00000000e-02,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,  -2.00000000e-01,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,  -2.00000000e-02,
                2.00000000e-01,   0.00000000e+00,   0.00000000e+00],
             [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                0.00000000e+00,   0.00000000e+00,   0.00000000e+00]])
        self.assertListsAlmostEqual(w.nleffects,
            [[ 0.11838112, -0.15894538, -0.01490104],
             [ 0.27979997,  0.00247348, -0.00494696],
             [ 0.03230564,  0.00742044,  0.        ]])


ts = unittest.TestSuite()
ts.addTest(WorldTestCase('testConstruction'))
ts.addTest(TestLinks('testLinksCreation'))
ts.addTest(TestLinks('testLinksReplacement'))
ts.addTest(TestUpdates('testGeometricUpdate'))
ts.addTest(TestUpdates('testDynamicUpdate'))
