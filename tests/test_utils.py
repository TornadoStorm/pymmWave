import random
import unittest

import numpy as np

import src.pymmWave.utils as utils


class TestCoordinateConversion(unittest.TestCase):
    def test_zero_range(self):
        for _ in range(100):
            angle = random.uniform(-np.pi, np.pi)
            elev = random.uniform(-np.pi / 2, np.pi / 2)

            self.assertEqual(
                utils.spherical_to_cartesian(0, angle, elev),
                (0, 0, 0),
            )

    def test_cartesian_to_spherical(self):
        x, y, z = 3.5467, 98.65, 0.01
        e_range, e_angle, e_elev = utils.cartesian_to_spherical(x, y, z)
        self.assertAlmostEqual(e_range, 98.7137, places=4)
        self.assertAlmostEqual(e_angle, 1.5349, places=4)
        self.assertAlmostEqual(e_elev, 1.0130e-04, places=4)

        x, y, z = 0.0, 0.0, 0.0
        e_range, e_angle, e_elev = utils.cartesian_to_spherical(x, y, z)
        self.assertEqual(e_range, 0)
        self.assertEqual(e_angle, 0)
        self.assertEqual(e_elev, 0)

        x, y, z = 69, 420, 99999
        e_range, e_angle, e_elev = utils.cartesian_to_spherical(x, y, z)
        self.assertLess(abs(e_range - 1.0000e05), 0.1)
        self.assertLess(abs(e_angle - 1.4080), 0.1)
        self.assertLess(abs(e_elev - 1.5665), 0.1)

    def test_convert_back(self):
        for _ in range(100):
            x = random.uniform(-100.0, 100.0)
            y = random.uniform(-100.0, 100.0)
            z = random.uniform(-100.0, 100.0)

            range_, angle, elev = utils.cartesian_to_spherical(x, y, z)
            x_, y_, z_ = utils.spherical_to_cartesian(range_, angle, elev)

            self.assertAlmostEqual(x_, x, places=3)
            self.assertAlmostEqual(y_, y, places=3)
            self.assertAlmostEqual(z_, z, places=3)


class TestTransformPoint(unittest.TestCase):
    def test_transform_zero(self):
        for _ in range(100):
            height = random.uniform(0.0, 100.0)
            elevation_tilt = random.uniform(-90.0, 90.0)
            azimuth_tilt = random.uniform(-90.0, 90.0)

            self.assertEqual(
                utils.transform_point(0, 0, 0, height, elevation_tilt, azimuth_tilt),
                (0, 0, height),
            )

    def test_transform_origin(self):
        for _ in range(100):
            x = random.uniform(-100.0, 100.0)
            y = random.uniform(-100.0, 100.0)
            z = random.uniform(-100.0, 100.0)

            self.assertEqual(
                utils.transform_point(x, y, z, 0, 0, 0),
                (x, y, z),
            )

    def _test_transformation(
        self, height, elevation_tilt, azimuth_tilt, points, transformed
    ):
        for i in range(len(points)):
            original = points[i]
            result = utils.transform_point(
                original[0],
                original[1],
                original[2],
                height,
                elevation_tilt,
                azimuth_tilt,
            )
            expected = transformed[i]

            self.assertAlmostEqual(result[0], expected[0], places=3)
            self.assertAlmostEqual(result[1], expected[1], places=3)
            self.assertAlmostEqual(result[2], expected[2], places=3)

    def test_transform_point_basic(self):
        # Testing some manually entered basic values
        for _ in range(100):
            height = random.uniform(-100.0, 100.0)
            azimuth_tilt = 0
            elevation_tilt = np.radians(-90)

            points = [
                (0, 0, 1),
                (0, 1, 0),
                (1, 0, 0),
            ]

            transformed = [
                (0, 1, height),
                (0, 0, -1 + height),
                (1, 0, height),
            ]

            self._test_transformation(
                height, elevation_tilt, azimuth_tilt, points, transformed
            )

    def test_transform_point(self):
        """Test a bunch of values generated from MATLAB to ensure correctness."""

        # Offset 1
        height = 5
        elevation_tilt = np.radians(68)
        azimuth_tilt = np.radians(256)

        points = [
            (0.46171, 9.5022, 7.952),
            (0.97132, 0.34446, 1.8687),
            (8.2346, 4.3874, 4.8976),
            (6.9483, 3.8156, 4.4559),
            (3.171, 7.6552, 6.4631),
        ]

        transformed = [
            (-3.8118, 0.47454, 16.789),
            (-1.791, -0.55452, 6.0194),
            (-4.8035, -7.289, 10.903),
            (-4.3027, -6.0882, 10.207),
            (-3.7991, -2.3208, 14.519),
        ]

        self._test_transformation(
            height, elevation_tilt, azimuth_tilt, points, transformed
        )

        # Offset 2
        height = 3
        azimuth_tilt = np.radians(245)
        elevation_tilt = np.radians(59)

        points = [
            (0.46171, 9.5022, 7.952),
            (0.97132, 0.34446, 1.8687),
            (8.2346, 4.3874, 4.8976),
            (6.9483, 3.8156, 4.4559),
            (3.171, 7.6552, 6.4631),
        ]

        transformed = [
            (-1.9372, 0.3939, 15.241),
            (-1.7014, -0.27834, 4.2577),
            (-5.2369, -6.6439, 9.2832),
            (-4.617, -5.5136, 8.5655),
            (-2.7877, -2.1989, 12.891),
        ]

        self._test_transformation(
            height, elevation_tilt, azimuth_tilt, points, transformed
        )

        # Offset 3
        height = 20
        azimuth_tilt = np.radians(42)
        elevation_tilt = np.radians(45)

        points = [
            (0.46171, 9.5022, 7.952),
            (0.97132, 0.34446, 1.8687),
            (8.2346, 4.3874, 4.8976),
            (6.9483, 3.8156, 4.4559),
            (3.171, 7.6552, 6.4631),
        ]

        transformed = [
            (-0.39036, 1.1236, 32.342),
            (1.443, -0.15104, 21.565),
            (6.3609, 5.2419, 26.565),
            (5.4665, 4.3129, 25.849),
            (1.7925, 2.7482, 29.983),
        ]

        self._test_transformation(
            height, elevation_tilt, azimuth_tilt, points, transformed
        )
