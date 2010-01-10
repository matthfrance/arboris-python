#! /usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import ConstraintsTests
import JointsTests
import UpdateDynamicTests
import homogeneousmatrixTest
import Human36Tests

tests = unittest.TestSuite([ConstraintsTests.ts, JointsTests.ts,
                            UpdateDynamicTests.ts, homogeneousmatrixTest.ts, Human36Tests.ts])

unittest.TextTestRunner(verbosity=2).run(tests)
