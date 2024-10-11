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

        data = s.read(8)

        packet_version, total_packet_len = struct.unpack("<2I", data)

        # Version
        result["major_num"] = (packet_version >> 24) & 0xFF
        result["minor_num"] = (packet_version >> 16) & 0xFF
        result["bugfix_num"] = (packet_version >> 8) & 0xFF
        result["build_num"] = packet_version & 0xFF

        # Total packet length
        result["total_packet_len"] = total_packet_len

        data = s.read(total_packet_len - 8)  # Read the rest of the packet

        offset = 28
        (
            result["platform_type"],  # Platform type
            result["frame_number"],  # Frame number
            result["time_cpu_cycles"],  # Time in cycles when the message was created
            result["num_detected_obj"],  # Number of detected objects
            result["num_tlvs"],  # Number of TLVs
            result["subframe_number"],  # Subframe number
            result["num_static_detected_obj"],  # Number of static detected objects
        ) = struct.unpack("<7I", data[:offset])

        if result["num_tlvs"] != 0:
            result["dynamic_points"] = []

            # Parse TLVs
            for _ in range(result["num_tlvs"]):
                tlv_type, tlv_length = struct.unpack("<2I", data[offset : offset + 8])
                offset += 8
                tlv_payload = data[offset : offset + tlv_length]
                offset += tlv_length

                match tlv_type:
                    case 1:
                        result["dynamic_points"] = []
                        for i in range(0, len(tlv_payload), 16):
                            vals = tlv_payload[i : i + 16]

                            raw_range, raw_angle, raw_elev, raw_doppler = struct.unpack(
                                "<4f", vals
                            )

                            # Transformed spherical coordinates
                            pos = transform_spherical_point(
                                raw_range,
                                raw_angle,
                                raw_elev,
                                self.height,
                                self.elevation_tilt,
                            )

                            result["dynamic_points"].append(
                                {
                                    "target_id": 255,  # Default value
                                    "range": pos[0],
                                    "angle": pos[1],
                                    "elev": pos[2],
                                    "doppler": raw_doppler,
                                    "snr": 0,  # Default value
                                    "noise": 0,  # Default value
                                }
                            )
                    case 7:
                        j = 0
                        for i in range(0, len(tlv_payload), 4):
                            vals = tlv_payload[i : i + 4]
                            (
                                result["dynamic_points"][j]["snr"],
                                result["dynamic_points"][j]["noise"],
                            ) = struct.unpack("<2H", vals)
                            j += 1
                    case 8:
                        result["static_points"] = []
                        for i in range(0, len(tlv_payload), 16):
                            vals = tlv_payload[i : i + 16]

                            raw_x, raw_y, raw_z, raw_doppler = struct.unpack(
                                "<4f", vals
                            )

                            # Transformed position
                            pos = transform_point(
                                raw_x,
                                raw_y,
                                raw_z,
                                self.height,
                                self.elevation_tilt,
                            )

                            result["static_points"].append(
                                {
                                    "x": pos[0],
                                    "y": pos[1],
                                    "z": pos[2],
                                    "doppler": raw_doppler,
                                    "snr": 0,  # Default value
                                    "noise": 0,  # Default value
                                }
                            )
                    case 9:
                        j = 0
                        for i in range(0, len(tlv_payload), 4):
                            vals = tlv_payload[i : i + 4]
                            (
                                result["static_points"][j]["snr"],
                                result["static_points"][j]["noise"],
                            ) = struct.unpack("<2H", vals)
                            j += 1
                    case 10:
                        result["tracked_objects"] = []
                        for i in range(0, len(tlv_payload), 40):
                            vals = tlv_payload[i : i + 40]

                            target_id = int.from_bytes(vals[0:4], byteorder="little")
                            (
                                raw_pos_x,
                                raw_pos_y,
                                raw_vel_x,
                                raw_vel_y,
                                raw_accel_x,
                                raw_accel_y,
                                raw_pos_z,
                                raw_vel_z,
                                raw_accel_z,
                            ) = struct.unpack("<9f", vals[4:40])

                            # Transformed position
                            pos = transform_point(
                                raw_pos_x,
                                raw_pos_y,
                                raw_pos_z,
                                self.height,
                                self.elevation_tilt,
                            )

                            # Transformed velocity
                            vel = transform_direction(
                                raw_vel_x,
                                raw_vel_y,
                                raw_vel_z,
                                self.elevation_tilt,
                            )

                            # Transformed acceleration
                            accel = transform_direction(
                                raw_accel_x,
                                raw_accel_y,
                                raw_accel_z,
                                self.elevation_tilt,
                            )

                            result["tracked_objects"].append(
                                {
                                    "target_id": target_id,
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
