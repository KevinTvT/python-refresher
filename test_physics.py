import unittest
import physics


class TestPhysics(unittest.TestCase):
    def test_calculate_buoyancy(self):
        self.assertEqual(physics.calculate_buoyancy(10, 10), 981)
        self.assertNotEqual(physics.calculate_buoyancy(10, 10), 1000)
        self.assertEqual(physics.calculate_buoyancy(15, 2.3), 338.445)

    def test_will_it_float(self):
        self.assertTrue(physics.will_it_float(10, 10))
        self.assertFalse(physics.will_it_float(0.01, 100))
        self.assertFalse(physics.will_it_float(0.999, 1000))

    def test_calculate_pressure(self):
        self.assertEqual(physics.calculate_pressure(20), 196200)
        self.assertNotEqual(physics.calculate_pressure(20), 200000)
        self.assertEqual(physics.calculate_pressure(0), 0)
