#! /usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import ConstraintsTests

tests = unittest.TestSuite([ConstraintsTests.ts])

unittest.TextTestRunner(verbosity=2).run(tests)
