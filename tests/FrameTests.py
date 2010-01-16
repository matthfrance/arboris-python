import unittest
from ArborisTests import BaseTest
from arboris.core import Body, SubFrame, World
from arboris import homogeneousmatrix as Hg
from arboris.robots.simplearm import add_simplearm
from numpy import array, eye


class BodyTestCase(BaseTest):

    def setUp(self):
        self.sarmw = World()
        add_simplearm(self.sarmw)

        self.alea_pose = array([ [ 0. ,  5. ,  0. ,  0. ],
                                 [ 2.6,  1. ,  0. ,  4.2],
                                 [ 0. ,  0. ,  0. ,  0. ],
                                 [ 0. ,  9. ,  0. ,  3. ] ])
        self.alea_jac = array([ [ 4.2, 8.,  1.  ],
                                [ 0.,  0.,  0.  ],
                                [ 0.,  7.,  0.  ],
                                [ 2.6, 0.,  0.  ],
                                [ 0.,  0.,  0.5 ],
                                [ 8.8, 0.,  2.  ] ])
        self.alea_djac = array([ [ 1.,  0.8, 0.  ],
                                 [ 0.,  42., 0.  ],
                                 [ 0.,  4.2, 2.6 ],
                                 [ 1.,  0.,  0.  ],
                                 [ 0.,  1.,  0.  ],
                                 [ 7.1, 6.,  0.  ] ])
        self.alea_twist = [ 0., 0., 7.1, 1., 42, 0. ]

    def testConstruction(self):
        b = Body()
        b = Body(name='MyHead', mass=eye(6)*5, viscosity=None)

    def testSubFrames(self):
        body = Body()
        body.update_dynamic(self.alea_pose, self.alea_jac, self.alea_djac,
                            self.alea_twist)
        sframe = SubFrame(body, Hg.rotz(3.14/3.), 'Brand New Frame')
        self.assertListsAlmostEqual(sframe.bpose,
        array([ [ 0.50045969, -0.86575984,  0.        ,  0.        ],
                [ 0.86575984,  0.50045969,  0.        ,  0.        ],
                [ 0.        ,  0.        ,  1.        ,  0.        ],
                [ 0.        ,  0.        ,  0.        ,  1.        ]]))
        self.assertListsAlmostEqual(sframe.pose,
            [ [ 4.3287992 ,  2.50229845, 0., 0.  ],
              [ 2.16695503, -1.75051589, 0., 4.2 ],
              [ 0.        ,  0.        , 0., 0.  ],
              [ 7.79183856,  4.5041372 , 0., 3.  ] ])
        self.assertListsAlmostEqual(sframe.jacobian,
            [ [ 2.10193069,  4.00367751,  0.50045969],
              [-3.63619133, -6.92607872, -0.86575984],
              [ 0.        ,  7.        ,  0.        ],
              [ 1.30119519,  0.        ,  0.43287992],
              [-2.25097558,  0.        ,  0.25022984],
              [ 8.8       ,  0.        ,  2.        ] ])
        self.assertListsAlmostEqual(sframe.djacobian,
            [ [  0.50045969,  36.76228101,   0.        ],
              [ -0.86575984,  20.32669907,   0.        ],
              [  0.        ,   4.2       ,   2.6       ],
              [  0.50045969,   0.86575984,   0.        ],
              [ -0.86575984,   0.50045969,   0.        ],
              [  7.1       ,   6.        ,   0.        ] ])
        self.assertListsAlmostEqual(sframe.twist,
            [ 0., 0., 7.1, 36.86237295, 20.1535471, 0. ])


class TestAccessors(BodyTestCase):

    def testBodiesIteration(self):
        bodies = self.sarmw.getbodies()

        for b1, b2 in zip(bodies[0].iter_descendant_bodies(), bodies[1:]):
            self.assertEqual(b1, b2)

        forearm = bodies['ForeArm']
        forearm_ancs = [ bodies['Arm'], bodies['ground'] ]
        for b1, b2 in zip(forearm.iter_ancestor_bodies(), forearm_ancs):
            self.assertEqual(b1, b2)

    def testJointsIteration(self):
        bodies = self.sarmw.getbodies()
        joints = self.sarmw.getjoints()

        arm_desc_j = [ joints['Elbow'], joints['Wrist'] ]
        for j1, j2 in zip(bodies['Arm'].iter_descendant_joints(), arm_desc_j):
            self.assertEqual(j1, j2)

        arm_asc_j = [ joints['Shoulder'] ]
        for j1, j2 in zip(bodies['Arm'].iter_descendant_joints(), arm_desc_j):
            self.assertEqual(j1, j2)


class TestUpdates(BodyTestCase):

    def testGeometricUpdate(self):
        bodies = self.sarmw.getbodies()
        arm = bodies['Arm']
        forearm = bodies['ForeArm']
        hand = bodies['Hand']
        pose = array([ [ 0.,  5.,  0.,  0.  ],
                       [ 2.6, 1.,  0.,  4.2 ],
                       [ 0.,  0.,  0.,  0.  ],
                       [ 0.,  9.,  0.,  3.  ] ])
        arm.update_geometric(pose)
        self.assertListsAlmostEqual(forearm.pose,
            [ [ 0. ,  5. ,  0. ,  2.5],
              [ 2.6,  1. ,  0. ,  4.7],
              [ 0. ,  0. ,  0. ,  0. ],
              [ 0. ,  9. ,  0. ,  7.5] ])
        self.assertEqual(forearm.jacobian, None)
        self.assertListsAlmostEqual(hand.pose,
            [ [  0. ,   5. ,   0. ,   4.5 ],
              [  2.6,   1. ,   0. ,   5.1 ],
              [  0. ,   0. ,   0. ,   0.  ],
              [  0. ,   9. ,   0. ,  11.1 ] ])
        self.assertEqual(hand.jacobian, None)

    def testDynamicUpdate(self):
        bodies = self.sarmw.getbodies()
        arm = bodies['Arm']
        forearm = bodies['ForeArm']
        hand = bodies['Hand']

        arm.update_dynamic(self.alea_pose, self.alea_jac, self.alea_djac,
                            self.alea_twist)

        self.assertListsAlmostEqual(forearm.pose,
            [ [ 0. ,  5. ,  0. ,  2.5],
              [ 2.6,  1. ,  0. ,  4.7],
              [ 0. ,  0. ,  0. ,  0. ],
              [ 0. ,  9. ,  0. ,  7.5] ])
        self.assertListsAlmostEqual(hand.jacobian,
            [ [  4.2 ,   8.  ,   1.  ],
              [  0.  ,   0.  ,   0.  ],
              [  0.  ,   8.  ,   1.  ],
              [  2.6 ,  -6.7 ,   0.  ],
              [  0.  ,   0.  ,   0.5 ],
              [ 12.58,   7.2 ,   2.9 ] ])
        self.assertListsAlmostEqual(forearm.djacobian,
            [ [  1. ,   0.8,   0. ],
              [  0. ,  42. ,   0. ],
              [  0. ,   4.2,   2.6],
              [  1. ,  -2.1,  -1.3],
              [  0. ,   1. ,   0. ],
              [  7.6,   6.4,   0. ] ])
        self.assertListsAlmostEqual(hand.twist,
            [  0.  ,   0.  ,   7.1 ,  -5.39,  42.  ,   0.  ])


ts = unittest.TestSuite()
ts.addTest(BodyTestCase('testConstruction'))
ts.addTest(BodyTestCase('testSubFrames'))
ts.addTest(TestAccessors('testBodiesIteration'))
ts.addTest(TestAccessors('testJointsIteration'))
ts.addTest(TestUpdates('testGeometricUpdate'))
ts.addTest(TestUpdates('testDynamicUpdate'))
