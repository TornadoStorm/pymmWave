from dataclasses import dataclass
from typing import Dict


@dataclass(frozen=True)
class TrackedObject:
    target_id: int
    pos_x: float
    pos_y: float
    pos_z: float
    vel_x: float
    vel_y: float
    vel_z: float
    acc_x: float
    acc_y: float
    acc_z: float

    def __init__(self, data: Dict):
        object.__setattr__(self, "target_id", data["target_id"])
        object.__setattr__(self, "pos_x", data["pos_x"])
        object.__setattr__(self, "pos_y", data["pos_y"])
        object.__setattr__(self, "pos_z", data["pos_z"])
        object.__setattr__(self, "vel_x", data["vel_x"])
        object.__setattr__(self, "vel_y", data["vel_y"])
        object.__setattr__(self, "vel_z", data["vel_z"])
        object.__setattr__(self, "acc_x", data["acc_x"])
        object.__setattr__(self, "acc_y", data["acc_y"])
        object.__setattr__(self, "acc_z", data["acc_z"])


@dataclass(frozen=True)
class DynamicPoint:
    target_id: int
    """
    The payload consists of 1 byte for EACH point in the dynamic point cloud. If point n was associated to a target, the value of Target ID will be the ID of the associated Target. Otherwise, the value will indicated the reason the point was not associated.</br>
    Target ID can have the following values:</br>
    <b><250</b>: The ID of the tracked object the point has been associated to</br>
    <b>253</b>: Point not associated, doesn't meet SNR requirements</br>
    <b>254</b>: Point not associated, located outside Boundary Box area</br>
    <b>255</b>: Point not associated, considered noise
    """
    range: float
    angle: float
    elev: float
    doppler: float
    snr: int
    noise: int

    def __init__(self, data: Dict):
        object.__setattr__(self, "target_id", data["target_id"])
        object.__setattr__(self, "range", data["range"])
        object.__setattr__(self, "angle", data["angle"])
        object.__setattr__(self, "elev", data["elev"])
        object.__setattr__(self, "doppler", data["doppler"])
        object.__setattr__(self, "snr", data["snr"])
        object.__setattr__(self, "noise", data["noise"])


@dataclass(frozen=True)
class StaticPoint:
    x: float
    y: float
    z: float
    doppler: float
    snr: int
    noise: int

    def __init__(self, data: Dict):
        object.__setattr__(self, "x", data["x"])
        object.__setattr__(self, "y", data["y"])
        object.__setattr__(self, "z", data["z"])
        object.__setattr__(self, "doppler", data["doppler"])
        object.__setattr__(self, "snr", data["snr"])
        object.__setattr__(self, "noise", data["noise"])


@dataclass(frozen=True)
class AreaScannerData:
    major_num: int
    minor_num: int
    bugfix_num: int
    build_num: int
    total_packet_len: int
    platform_type: int
    frame_number: int
    time_cpu_cycles: int
    num_detected_obj: int
    num_tlvs: int
    subframe_number: int
    num_static_detected_obj: int

    dynamic_points: list[DynamicPoint]
    static_points: list[StaticPoint]
    tracked_objects: list[TrackedObject]

    def __init__(self, data: Dict):
        object.__setattr__(self, "major_num", data["major_num"])
        object.__setattr__(self, "minor_num", data["minor_num"])
        object.__setattr__(self, "bugfix_num", data["bugfix_num"])
        object.__setattr__(self, "build_num", data["build_num"])
        object.__setattr__(self, "total_packet_len", data["total_packet_len"])
        object.__setattr__(self, "platform_type", data["platform_type"])
        object.__setattr__(self, "frame_number", data["frame_number"])
        object.__setattr__(self, "time_cpu_cycles", data["time_cpu_cycles"])
        object.__setattr__(self, "num_detected_obj", data["num_detected_obj"])
        object.__setattr__(self, "num_tlvs", data["num_tlvs"])
        object.__setattr__(self, "subframe_number", data["subframe_number"])
        object.__setattr__(
            self, "num_static_detected_obj", data["num_static_detected_obj"]
        )

        # Parse dynamic points
        dynamic_points = []
        if "dynamic_points" in data:
            for point in data["dynamic_points"]:
                dynamic_points.append(DynamicPoint(point))
        object.__setattr__(self, "dynamic_points", dynamic_points)

        # Parse static points
        static_points = []
        if "static_points" in data:
            for point in data["static_points"]:
                static_points.append(StaticPoint(point))
        object.__setattr__(self, "static_points", static_points)

        # Parse tracked objects
        tracked_objects = []
        if "tracked_objects" in data:
            for obj in data["tracked_objects"]:
                tracked_objects.append(TrackedObject(obj))
        object.__setattr__(self, "tracked_objects", tracked_objects)
