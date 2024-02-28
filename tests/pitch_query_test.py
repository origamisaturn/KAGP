import unittest
import numpy as np
import math
import openmdao.api as om

import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, '..', '..', 'core')))
from cherry_guidance_refactor import PitchQuery

class TestPitchQuery(unittest.TestCase):
    def setUp(self):
        self.prob = om.Problem(PitchQueryGroup())
        self.prob.setup()
        set_pitch_query_default(self.prob)

    def test_case_1(self):
        