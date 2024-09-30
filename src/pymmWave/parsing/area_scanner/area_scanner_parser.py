import math
import struct
from typing import Dict

import numpy as np
from serial import Serial

from ...utils import transform_direction, transform_point, transform_spherical_point
from ..sensor_parser import SensorParser


class AreaScannerParser(SensorParser):
    """
    Parsing data in the format of the area scanner example project.<br/>
    Refer to the <a href="https://dev.ti.com/tirex/explore/content/mmwave_industrial_toolbox_4_12_1/labs/Area_Scanner/docs/area_scanner_users_guide.html#data-output-format">documentation</a> for more information.
    """

    height: float = 0
    """Height of the sensor from the ground in meters"""
    elevation_tilt: float = 0
    """Elevation tilt of the sensor in radians."""

    def parse(self, s: Serial) -> Dict:

        result = {}

        # Version
        packet_version = int.from_bytes(s.read(4), byteorder="little")
        result["major_num"] = (packet_version >> 24) & 0xFF
        result["minor_num"] = (packet_version >> 16) & 0xFF
        result["bugfix_num"] = (packet_version >> 8) & 0xFF
        result["build_num"] = packet_version & 0xFF

        # Total packet length
        result["total_packet_len"] = int.from_bytes(s.read(4), byteorder="little")

        # Platform type
        result["platform_type"] = int.from_bytes(s.read(4), byteorder="little")

        # Frame number
        result["frame_number"] = int.from_bytes(s.read(4), byteorder="little")

        # Time in CPU cycles when the message was created
        result["time_cpu_cycles"] = int.from_bytes(s.read(4), byteorder="little")

        # Number of detected objects
        result["num_detected_obj"] = int.from_bytes(s.read(4), byteorder="little")

        # Number of TLVs
        result["num_tlvs"] = int.from_bytes(s.read(4), byteorder="little")

        # Subframe number
        result["subframe_number"] = int.from_bytes(s.read(4), byteorder="little")

        # Number of static detected objects
        result["num_static_detected_obj"] = int.from_bytes(
            s.read(4), byteorder="little"
        )

        if result["num_tlvs"] != 0:
            # Parse TLVs
            for _ in range(result["num_tlvs"]):
                tlv_type = int.from_bytes(s.read(4), byteorder="little")
                tlv_length = int.from_bytes(s.read(4), byteorder="little")
                tlv_payload = s.read(tlv_length)

                match tlv_type:
                    case 1:
                        result["dynamic_points"] = []
                        for i in range(0, len(tlv_payload), 16):
                            current_point = tlv_payload[i : i + 16]

                            # Transformed spherical coordinates
                            pos = transform_spherical_point(
                                struct.unpack("f", current_point[0:4])[0],
                                struct.unpack("f", current_point[4:8])[0],
                                struct.unpack("f", current_point[8:12])[0],
                                self.height,
                                self.elevation_tilt,
                            )

                            result["dynamic_points"].append(
                                {
                                    "target_id": 255,  # Default value
                                    "range": pos[0],
                                    "angle": pos[1],
                                    "elev": pos[2],
                                    "doppler": struct.unpack("f", current_point[12:16])[
                                        0
                                    ],
                                    "snr": 0,  # Default value
                                    "noise": 0,  # Default value
                                }
                            )
                    case 7:
                        for i in range(0, len(tlv_payload) // 4, 1):
                            current_point = tlv_payload[(i * 4) : (i * 4) + 4]
                            result["dynamic_points"][i]["snr"] = int.from_bytes(
                                current_point[0:2], byteorder="little"
                            )
                            result["dynamic_points"][i]["noise"] = int.from_bytes(
                                current_point[2:4], byteorder="little"
                            )
                    case 8:
                        result["static_points"] = []
                        for i in range(0, len(tlv_payload), 16):
                            current_point = tlv_payload[i : i + 16]

                            # Transformed position
                            pos = transform_point(
                                struct.unpack("f", current_point[0:4])[0],
                                struct.unpack("f", current_point[4:8])[0],
                                struct.unpack("f", current_point[8:12])[0],
                                self.height,
                                self.elevation_tilt,
                            )

                            result["static_points"].append(
                                {
                                    "x": pos[0],
                                    "y": pos[1],
                                    "z": pos[2],
                                    "doppler": struct.unpack("f", current_point[12:16])[
                                        0
                                    ],
                                    "snr": 0,  # Default value
                                    "noise": 0,  # Default value
                                }
                            )
                    case 9:
                        for i in range(0, len(tlv_payload) // 4, 1):
                            current_point = tlv_payload[(i * 4) : (i * 4) + 4]
                            result["static_points"][i]["snr"] = int.from_bytes(
                                current_point[0:2], byteorder="little"
                            )
                            result["static_points"][i]["noise"] = int.from_bytes(
                                current_point[2:4], byteorder="little"
                            )
                    case 10:
                        result["tracked_objects"] = []
                        for i in range(0, len(tlv_payload), 40):
                            current_point = tlv_payload[i : i + 40]

                            # Transformed position
                            pos = transform_point(
                                struct.unpack("f", current_point[4:8])[0],
                                struct.unpack("f", current_point[8:12])[0],
                                struct.unpack("f", current_point[28:32])[0],
                                self.height,
                                self.elevation_tilt,
                            )

                            # Transformed velocity
                            vel = transform_direction(
                                struct.unpack("f", current_point[12:16])[0],
                                struct.unpack("f", current_point[16:20])[0],
                                struct.unpack("f", current_point[32:36])[0],
                                self.elevation_tilt,
                            )

                            # Transformed acceleration
                            accel = transform_direction(
                                struct.unpack("f", current_point[20:24])[0],
                                struct.unpack("f", current_point[24:28])[0],
                                struct.unpack("f", current_point[36:40])[0],
                                self.elevation_tilt,
                            )

                            result["tracked_objects"].append(
                                {
                                    "target_id": int.from_bytes(
                                        current_point[0:4], byteorder="little"
                                    ),
                                    "pos_x": pos[0],
                                    "pos_y": pos[1],
                                    "vel_x": vel[0],
                                    "vel_y": vel[1],
                                    "acc_x": accel[0],
                                    "acc_y": accel[1],
                                    "pos_z": pos[2],
                                    "vel_z": vel[2],
                                    "acc_z": accel[2],
                                }
                            )
                    case 11:
                        for i in range(len(tlv_payload)):
                            result["dynamic_points"][i]["target_id"] = int.from_bytes(
                                [tlv_payload[i]], byteorder="little"
                            )
                    case _:
                        print(f"Unknown TLV type: {tlv_type}")

        return result
