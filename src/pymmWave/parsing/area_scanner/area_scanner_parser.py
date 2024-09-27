import struct
from typing import Dict

from serial import Serial

from ..sensor_parser import SensorParser


class AreaScannerParser(SensorParser):
    """Parses data from the firmware of the area scanner example project."""

    def parse(self, s: Serial) -> Dict:
        """
        Parse the data from the sensor.
        """

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
                            result["dynamic_points"].append(
                                {
                                    "range": struct.unpack("f", current_point[0:4])[0],
                                    "angle": struct.unpack("f", current_point[4:8])[0],
                                    "elev": struct.unpack("f", current_point[8:12])[0],
                                    "doppler": struct.unpack("f", current_point[12:16])[
                                        0
                                    ],
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
                            result["static_points"].append(
                                {
                                    "x": struct.unpack("f", current_point[0:4])[0],
                                    "y": struct.unpack("f", current_point[4:8])[0],
                                    "z": struct.unpack("f", current_point[8:12])[0],
                                    "doppler": struct.unpack("f", current_point[12:16])[
                                        0
                                    ],
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
                            result["tracked_objects"].append(
                                {
                                    "target_id": int.from_bytes(
                                        current_point[0:4], byteorder="little"
                                    ),
                                    "pos_x": struct.unpack("f", current_point[4:8])[0],
                                    "pos_y": struct.unpack("f", current_point[8:12])[0],
                                    "vel_x": struct.unpack("f", current_point[12:16])[
                                        0
                                    ],
                                    "vel_y": struct.unpack("f", current_point[16:20])[
                                        0
                                    ],
                                    "acc_x": struct.unpack("f", current_point[20:24])[
                                        0
                                    ],
                                    "acc_y": struct.unpack("f", current_point[24:28])[
                                        0
                                    ],
                                    "pos_z": struct.unpack("f", current_point[28:32])[
                                        0
                                    ],
                                    "vel_z": struct.unpack("f", current_point[32:36])[
                                        0
                                    ],
                                    "acc_z": struct.unpack("f", current_point[36:40])[
                                        0
                                    ],
                                }
                            )
                    case 11:
                        for i in range(len(tlv_payload)):
                            # If point n was associated to a target, the value of Target ID will be the ID of the associated Target.
                            # Otherwise, the value will indicated the reason the point was not associated.
                            # Target ID can have the following values:
                            # <250: The ID of the tracked object the point has been associated to
                            # 253: Point not associated, doesn't meet SNR requirements
                            # 254: Point not associated, located outside Boundary Box area
                            # 255: Point not associated, considered noise
                            result["dynamic_points"][i]["target_id"] = int.from_bytes(
                                [tlv_payload[i]], byteorder="little"
                            )
                    case _:
                        print(f"Unknown TLV type: {tlv_type}")

        return result
