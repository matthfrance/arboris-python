#! /usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import ConstraintsTests, JointsTests, UpdateDynamicTests, homogeneousmatrixTest
import Human36Tests, FrameTests

tests = unittest.TestSuite([ConstraintsTests.ts, JointsTests.ts,
                            UpdateDynamicTests.ts, homogeneousmatrixTest.ts,
                            Human36Tests.ts, FrameTests.ts])

unittest.TextTestRunner(verbosity=2).run(tests)
