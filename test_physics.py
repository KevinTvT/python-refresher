import unittest
import physics  # IF ITS IN A CLASS I CAN INSTEAD TYPE: import physics.physics as physics
import numpy as np


class TestPhysics(unittest.TestCase):
    def test_calculate_buoyancy(self):
        self.assertEqual(physics.calculate_buoyancy(10, 10), 981)
        self.assertNotEqual(physics.calculate_buoyancy(10, 10), 1000)
        with self.assertRaises(ValueError):
            physics.calculate_buoyancy(-1, 2.3)
        # Another way to assertRaises
        # self.assertRaises(ValueError, lambda: physics.calculate_buoyancy(0,0))

    def test_will_it_float(self):
        self.assertTrue(physics.will_it_float(10, 10))  # will float
        self.assertEqual(
            physics.will_it_float(0.01, 10), "NEUTRAL BUOYANCY"
        )  # neutral buoyancy
        self.assertFalse(physics.will_it_float(0.999, 1000))  # will sink
        with self.assertRaises(ValueError):
            physics.will_it_float(-1, 2.3)

    def test_calculate_pressure(self):
        self.assertEqual(physics.calculate_pressure(20), 297525.0)
        self.assertNotEqual(physics.calculate_pressure(20), 200000)
        with self.assertRaises(ValueError):
            physics.calculate_pressure(-1)
        with self.assertRaises(ValueError):
            physics.calculate_pressure(0)

    def test_calculate_acceleration(self):
        self.assertEqual(physics.calculate_acceleration(5, 2), 2.5)
        self.assertNotEqual(physics.calculate_acceleration(5, 2), 3)
        self.assertEqual(physics.calculate_acceleration(-5, 2), -2.5)
        self.assertRaises(ValueError, lambda: physics.calculate_acceleration(5, 0))

    def test_calculate_angular_acceleration(self):
        self.assertEqual(physics.calculate_angular_acceleration(5, 2), 2.5)
        self.assertNotEqual(physics.calculate_angular_acceleration(5, 2), 3)
        self.assertEqual(physics.calculate_angular_acceleration(-5, 2), -2.5)
        self.assertRaises(
            ValueError, lambda: physics.calculate_angular_acceleration(5, 0)
        )

    def test_calculate_torque(self):
        self.assertEqual(physics.calculate_torque(5, 2, 30), 5.235)
        self.assertNotEqual(physics.calculate_torque(5, 2, 30), -5.33)
        self.assertEqual(physics.calculate_torque(5, 90, 2), 10)
        self.assertEqual(physics.calculate_torque(-2, 2, 2), -0.14)
        self.assertEqual(physics.calculate_torque(2, -10, 2), -0.695)
        self.assertRaises(ValueError, lambda: physics.calculate_torque(5, 2, -1))

    def test_calculate_moment_of_inertia(self):
        self.assertEqual(physics.calculate_moment_of_inertia(5, 2), 20)
        self.assertNotEqual(physics.calculate_moment_of_inertia(5, 2), 21)
        self.assertEqual(physics.calculate_moment_of_inertia(5, -2), 20)
        self.assertRaises(
            ValueError, lambda: physics.calculate_moment_of_inertia(-5, 2)
        )

    def test_calculate_auv_acceleration(self):
        self.assertTrue(
            np.allclose(physics.calculate_auv_acceleration(5, np.pi / 2, 2), [0, 2.5])
        )
        self.assertFalse(
            np.allclose(physics.calculate_auv_acceleration(5, np.pi / 2, 2), [0, 2])
        )
        self.assertTrue(
            np.allclose(
                physics.calculate_auv_acceleration(
                    500, np.pi / 4, volume=100, thruster_distance=100
                ),
                [2.5 * np.sqrt(2), 2.5 * np.sqrt(2)],
            )
        )
        self.assertFalse(
            np.allclose(
                physics.calculate_auv_acceleration(
                    500, 1, volume=100, thruster_distance=100
                ),
                [2, 2],
            ),
        )
        self.assertRaises(
            ValueError, lambda: physics.calculate_auv_acceleration(5, 1, -5, 5)
        )
        self.assertRaises(
            ValueError, lambda: physics.calculate_auv_acceleration(5, 1, 5, -5)
        )
        self.assertRaises(
            ValueError, lambda: physics.calculate_auv_acceleration(5, 1, -5, -5)
        )

    def test_calculate_auv_angular_acceleration(self):
        self.assertEqual(physics.calculate_auv_angular_acceleration(5, 1, 2), 1.052)
        self.assertNotEqual(physics.calculate_auv_angular_acceleration(5, 1, 2), 1)
        self.assertEqual(
            physics.calculate_auv_angular_acceleration(5, np.pi / 2, 5, 15), 15
        )
        self.assertNotEqual(
            physics.calculate_auv_angular_acceleration(5, np.pi / 2, 5, 15), 10
        )
        self.assertRaises(
            ValueError,
            lambda: physics.calculate_auv_angular_acceleration(5, np.pi / 2, -1, 10),
        )

    def test_calculate_auv2_acceleration(self):
        testOne = physics.calculate_auv2_acceleration([20, 20, 10, 10], np.pi / 4, 0)
        self.assertTrue(
            np.allclose(
                testOne,
                [10 * np.sqrt(2), 0],
            )
        )
        self.assertFalse(np.allclose(testOne, [10, 0]))

        testTwo = physics.calculate_auv2_acceleration(
            [40, 20, 30, 20], np.pi / 3, np.pi / 6, 500
        )  # π/6 is 30˚
        self.assertTrue(np.allclose(testTwo, [5 * np.sqrt(3), 5]))
        self.assertFalse(np.allclose(testTwo, [10 * np.sqrt(3), 10]))

        testThree = physics.calculate_auv2_acceleration(
            [-40, -20, 30, 20], np.pi / 3, np.pi / 6
        )
        self.assertTrue(np.allclose(testThree, [-70 / 2 * np.sqrt(3), 5]))
        self.assertFalse(np.allclose(testThree, [-140 / 2 * np.sqrt(3), 10]))
        self.assertRaises(
            ValueError,
            lambda: physics.calculate_auv2_acceleration(
                [-40, -20, 30, 20], np.pi / 3, np.pi / 6, -20
            ),
        )

    def test_calculate_auv2_angular_acceleration(self):
        testOne = physics.calculate_auv2_angular_acceleration(
            [40, 20, 30, 10], np.pi / 2, 10, 5
        )
        self.assertEqual(testOne, 4.0)
        self.assertNotEqual(testOne, 20)

        testTwo = physics.calculate_auv2_angular_acceleration(
            [40, 20, 30, 10], 3 * np.pi / 2, 10, 5
        )
        self.assertAlmostEqual(testTwo, -4.0)
        self.assertNotAlmostEqual(testTwo, 4.0)

        self.assertRaises(
            ValueError,
            lambda: physics.calculate_auv2_angular_acceleration(
                [40, 20, 30, 10],
                np.pi,
                -1,
                0,
            ),
        )
