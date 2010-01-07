#! /usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import ConstraintsTests
import JointsTests
import UpdateDynamicTests

tests = unittest.TestSuite([ConstraintsTests.ts, JointsTests.ts,
                            UpdateDynamicTests.ts])

unittest.TextTestRunner(verbosity=2).run(tests)
