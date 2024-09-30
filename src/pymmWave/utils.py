import numpy as np


def load_cfg_file(filepath: str) -> list[str]:
    """Loads a config file to a list of strings. Wrapper around readlines() with some simple verification.

    Args:
        filepath (str): Filepath, relative paths should be ok

    Returns:
        list[str]: List of lines as str
    """

    assert len(filepath) > 4
    assert filepath[-4:] == ".cfg"

    data: list[str]
    with open(filepath, "r") as f:
        data = f.readlines()

    return data


def spherical_to_cartesian(
    range: float, angle: float, elev: float
) -> tuple[float, float, float]:
    """
    Convert spherical coordinates to cartesian coordinates.

    :param range: Range in meters.
    :param angle: Azimuth angle in radians.
    :param elev: Elevation angle in radians.
    :return: Cartesian coordinates.
    """

    x: float = float(range * np.cos(elev) * np.cos(angle))
    y: float = float(range * np.cos(elev) * np.sin(angle))
    z: float = float(range * np.sin(elev))

    return x, y, z


def cartesian_to_spherical(x: float, y: float, z: float) -> tuple[float, float, float]:
    """
    Convert cartesian coordinates to spherical coordinates.

    :param x: X coordinate.
    :param y: Y coordinate.
    :param z: Z coordinate.
    :return: Spherical coordinates in the order: range, angle, elev.
    """

    range: float = float(np.sqrt(x**2 + y**2 + z**2))

    if range == 0:
        angle: float = 0.0
        elev: float = 0.0
    else:
        angle: float = float(np.arctan2(y, x))
        # Ensure the argument for arcsin is within the valid range [-1, 1]
        sin_elev_arg = z / range
        sin_elev_arg = np.clip(sin_elev_arg, -1.0, 1.0)
        elev: float = float(np.arcsin(sin_elev_arg))

    return range, angle, elev


def transform_point(
    x: float,
    y: float,
    z: float,
    height: float,
    elevation_tilt: float,
    azimuth_tilt: float = 0,
) -> tuple[float, float, float]:
    """
    Transform a point using a given height from the ground and elevation tilt.

    :param x: Raw X coordinate.
    :param y: Raw Y coordinate.
    :param z: Raw Z coordinate.
    :param height: Height of the sensor from the ground in meters.
    :param elevation_tilt: Elevation tilt of the sensor in radians.
    :param azimuth_tilt: Azimuth tilt of the sensor in radians.
    :return: Transformed point
    """

    rotmat_az = np.array(
        [
            [np.cos(azimuth_tilt), -np.sin(azimuth_tilt), 0],
            [np.sin(azimuth_tilt), np.cos(azimuth_tilt), 0],
            [0, 0, 1],
        ]
    )

    rotmat_el = np.array(
        [
            [1, 0, 0],
            [0, np.cos(elevation_tilt), -np.sin(elevation_tilt)],
            [0, np.sin(elevation_tilt), np.cos(elevation_tilt)],
        ]
    )

    trans = np.dot(rotmat_az, np.dot(rotmat_el, np.array([[x], [y], [z]])))
    corr_x: float = float(trans[0, :][0])
    corr_y: float = float(trans[1, :][0])
    corr_z: float = float(trans[2, :][0] + height)

    return corr_x, corr_y, corr_z


def transform_direction(
    vx: float,
    vy: float,
    vz: float,
    elevation_tilt: float,
    azimuth_tilt: float = 0,
) -> tuple[float, float, float]:
    """
    Transform a direction vector using elevation and azimuth tilts.

    :param vx: Raw X component of the vector.
    :param vy: Raw Y component of the vector.
    :param vz: Raw Z component of the vector.
    :param elevation_tilt: Elevation tilt of the sensor in radians.
    :param azimuth_tilt: Azimuth tilt of the sensor in radians.
    :return: Transformed velocity vector
    """

    rotmat_az = np.array(
        [
            [np.cos(azimuth_tilt), -np.sin(azimuth_tilt), 0],
            [np.sin(azimuth_tilt), np.cos(azimuth_tilt), 0],
            [0, 0, 1],
        ]
    )

    rotmat_el = np.array(
        [
            [1, 0, 0],
            [0, np.cos(elevation_tilt), -np.sin(elevation_tilt)],
            [0, np.sin(elevation_tilt), np.cos(elevation_tilt)],
        ]
    )

    trans = np.dot(rotmat_az, np.dot(rotmat_el, np.array([[vx], [vy], [vz]])))
    corr_vx: float = float(trans[0, :][0])
    corr_vy: float = float(trans[1, :][0])
    corr_vz: float = float(trans[2, :][0])

    return corr_vx, corr_vy, corr_vz


def transform_spherical_point(
    range: float,
    angle: float,
    elev: float,
    height: float,
    elevation_tilt: float,
    azimuth_tilt: float = 0,
) -> tuple[float, float, float]:
    """
    Transform a point in spherical coordinates using a given height from the ground and elevation tilt.

    :param r: Range in meters.
    :param angle: Azimuth angle in radians.
    :param elev: Elevation angle in radians.
    :param height: Height of the sensor from the ground in meters.
    :param elevation_tilt: Elevation tilt of the sensor in radians.
    :param azimuth_tilt: Azimuth tilt of the sensor in radians
    :return: Transformed point in spherical coordinates (range, angle, elev).
    """
    x, y, z = spherical_to_cartesian(range, angle, elev)
    x, y, z = transform_point(x, y, z, height, elevation_tilt, azimuth_tilt)
    return cartesian_to_spherical(x, y, z)
