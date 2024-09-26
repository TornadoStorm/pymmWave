class DynamicPoint:
    target_id: int
    """
        If point n was associated to a target, the value of Target ID will be the ID of the associated Target. Otherwise, the value will indicated the reason the point was not associated.<br/>
        Target ID can have the following values:
        <250: The ID of the tracked object the point has been associated to<br/>
        253: Point not associated, doesn't meet SNR requirements<br/>
        254: Point not associated, located outside Boundary Box area<br/>
        255: Point not associated, considered noise<br/>
    """
    range: float
    angle: float
    elev: float
    doppler: float
    snr: int
    noise: int

class StaticPoint:
    x: float
    y: float
    z: float
    doppler: float
    snr: int
    noise: int

class TrackedObject:
    target_id: int
    pos_x: float
    pos_y: float
    vel_x: float
    vel_y: float
    acc_x: float
    acc_y: float
    pos_z: float
    vel_z: float
    acc_z: float