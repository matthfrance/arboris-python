# -*- encoding: UTF-8 -*-

from unittest import TestCase

class BaseTest(TestCase):

    def assertListsAlmostEqual(self, l1, l2, places=7):
        if len(l1) != len(l2):
            raise AssertionError("%s != %s"%(str(l1), str(l2)))
        for i, j in zip(l1, l2):
            if isinstance(i, list) and isinstance(j, list):
                self.assertListsAlmostEqual(i, j, places)
            else:
                self.assertAlmostEqual(i, j, places)
