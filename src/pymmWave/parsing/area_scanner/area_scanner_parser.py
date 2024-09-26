from serial import Serial
from ..sensor_parser import SensorParser
from typing import Any
from .models import DynamicPoint, StaticPoint, TrackedObject
import struct

class AreaScannerParser(SensorParser):
    """Parses data from the firmware of the area scanner example project.
    """

    def parse(self, s: Serial) -> dict:
        """
        Parse the data from the sensor.
        """

        result = {}

        # Version
        packet_version = int.from_bytes(s.read(4), byteorder='little')
        result['major_num'] = (packet_version >> 24) & 0xFF
        result['minor_num'] = (packet_version >> 16) & 0xFF
        result['bugfix_num'] = (packet_version >> 8) & 0xFF
        result['build_num'] = packet_version & 0xFF

        # Total packet length
        result['total_packet_len'] = int.from_bytes(s.read(4), byteorder='little')

        # Platform type
        result['platform_type'] = int.from_bytes(s.read(4), byteorder='little')

        # Frame number
        result['frame_number'] = int.from_bytes(s.read(4), byteorder='little')

        # Time in CPU cycles when the message was created
        result['time_cpu_cycles'] = int.from_bytes(s.read(4), byteorder='little')

        # Number of detected objects
        result['num_detected_obj'] = int.from_bytes(s.read(4), byteorder='little')

        # Number of TLVs
        result['num_tlvs'] = int.from_bytes(s.read(4), byteorder='little')

        # Subframe number
        result['subframe_number'] = int.from_bytes(s.read(4), byteorder='little')

        # Number of static detected objects
        result['num_static_detected_obj'] = int.from_bytes(s.read(4), byteorder='little')

        if result['num_tlvs'] != 0:
            # Parse TLVs
            for _ in range(result['num_tlvs']):
                tlv_type = int.from_bytes(s.read(4), byteorder='little')
                tlv_length = int.from_bytes(s.read(4), byteorder='little')
                tlv_payload = s.read(tlv_length)

                match tlv_type:
                    case 1:
                        result['dynamic_points'] = []
                        for i in range(0, len(tlv_payload), 16):
                            current_point = tlv_payload[i:i+16]
                            new_point = DynamicPoint()
                            new_point.range = struct.unpack('f', current_point[0:4])[0]
                            new_point.angle = struct.unpack('f', current_point[4:8])[0]
                            new_point.elev = struct.unpack('f', current_point[8:12])[0]
                            new_point.doppler = struct.unpack('f', current_point[12:16])[0]
                            result['dynamic_points'].append(new_point)
                    case 7:
                        for i in range(0, len(tlv_payload) // 4, 1):
                            current_point = tlv_payload[(i*4):(i*4)+4]
                            result['dynamic_points'][i].snr = int.from_bytes(current_point[0:2], byteorder='little')
                            result['dynamic_points'][i].noise = int.from_bytes(current_point[2:4], byteorder='little')
                    case 8:
                        result['static_points'] = []
                        for i in range(0, len(tlv_payload), 16):
                            current_point = tlv_payload[i:i+16]
                            new_point = StaticPoint()
                            new_point.x = struct.unpack('f', current_point[0:4])[0]
                            new_point.y = struct.unpack('f', current_point[4:8])[0]
                            new_point.z = struct.unpack('f', current_point[8:12])[0]
                            new_point.doppler = struct.unpack('f', current_point[12:16])[0]
                            result['static_points'].append(new_point)
                    case 9:
                        for i in range(0, len(tlv_payload) // 4, 1):
                            current_point = tlv_payload[(i*4):(i*4)+4]
                            result['static_points'][i].snr = int.from_bytes(current_point[0:2], byteorder='little')
                            result['static_points'][i].noise = int.from_bytes(current_point[2:4], byteorder='little')
                    case 10:
                        result['tracked_objects'] = []
                        for i in range(0, len(tlv_payload), 40):
                            current_point = tlv_payload[i:i+40]
                            new_point = TrackedObject()
                            new_point.target_id = int.from_bytes(current_point[0:4], byteorder='little')
                            new_point.pos_x = struct.unpack('f', current_point[4:8])[0]
                            new_point.pos_y = struct.unpack('f', current_point[8:12])[0]
                            new_point.vel_x = struct.unpack('f', current_point[12:16])[0]
                            new_point.vel_y = struct.unpack('f', current_point[16:20])[0]
                            new_point.acc_x = struct.unpack('f', current_point[20:24])[0]
                            new_point.acc_y = struct.unpack('f', current_point[24:28])[0]
                            new_point.pos_z = struct.unpack('f', current_point[28:32])[0]
                            new_point.vel_z = struct.unpack('f', current_point[32:36])[0]
                            new_point.acc_z = struct.unpack('f', current_point[36:40])[0]
                            result['tracked_objects'].append(new_point)
                    case 11:
                        for i in range(len(tlv_payload)):
                            result['dynamic_points'][i].target_id = int.from_bytes([tlv_payload[i]], byteorder='little')
                    case _:
                        print(f"Unknown TLV type: {tlv_type}")

        
        return result