#! /usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import ConstraintsTests, JointsTests, homogeneousmatrixTest
import Human36Tests, FrameTests, WorldTests, ControllersTests

tests = unittest.TestSuite([ JointsTests.ts, FrameTests.ts, ConstraintsTests.ts,
                             ControllersTests.ts, homogeneousmatrixTest.ts,
                             WorldTests.ts, Human36Tests.ts ])

unittest.TextTestRunner(verbosity=2).run(tests)
