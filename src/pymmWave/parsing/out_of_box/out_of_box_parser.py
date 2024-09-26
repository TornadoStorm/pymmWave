from dataclasses import dataclass
from serial import Serial, SerialException
from ..sensor_parser import SensorParser
from typing import Any, Optional
import struct
from ...constants import TLV_type, BYTE_MULT, MAGIC_NUMBER
from ...IWR6843AOP import Frame
from ...data_model import DopplerPointCloud
import numpy as np
from struct import unpack

# ! BROKEN CODE, stored here for reference

@dataclass
class _light_doppler_cloud:
    x_coord: np.ndarray
    y_coord: np.ndarray
    z_coord: np.ndarray
    doppler: np.ndarray

def _getXYZ_type2(vec: list[int], vecIdx: int, Params: Frame, num_detected_obj: int, sizeObj: int, raw_bv: Any):
        num_detected_obj = int(num_detected_obj)
        data = _light_doppler_cloud(np.zeros(num_detected_obj), np.zeros(num_detected_obj), np.zeros(num_detected_obj), np.zeros(num_detected_obj))  #type: ignore
        i: int
        start_idx: int

        for i in range(num_detected_obj):  
            # /*start index in bytevec for this detected obj*/
            start_idx = vecIdx + i * sizeObj
            try:
                data.x_coord[i] = unpack('f',bytes(vec[start_idx+0:start_idx+4]))[0]
                data.y_coord[i] = unpack('f',bytes(vec[start_idx+4:start_idx+8]))[0]
                data.z_coord[i] = unpack('f',bytes(vec[start_idx+8:start_idx+12]))[0]
                data.doppler[i] = unpack('f',bytes(vec[start_idx+12:start_idx+16]))[0]
            except:
                pass

        return data

def _processDetectedPoints(bv: list[int], idx: int, dt: Frame, raw_bv: Any) -> Optional[_light_doppler_cloud]:
        """Processes detected points from a byte vector, unpacks floats as well.
        This is some preset structure that we are breaking down
        """
        if (dt.numDetectedObj > 0):
            sizeofObj: int = 16
                    
            return _getXYZ_type2(bv, idx, dt, dt.numDetectedObj, sizeofObj, raw_bv)

        return None

class OutOfBoxParser(SensorParser):
    _doppler_filtering = 0

    def __init__(self, doppler_filtering: float = 0):
        self._doppler_filtering = doppler_filtering

    def parse(self, s: Serial) -> dict:
        """
        Parse the data from the sensor.
        """

        result = {}
        a: Optional[bytes] = b''

        chunks: list[bytes] = []
        a += s.read_all()  # type: ignore
        print(a)
        if a is None:
            raise SerialException()
        b = MAGIC_NUMBER
        index = [x for x in range(len(a)) if a[x:x+len(b)] == b]
        # print(len(a))
        if len(index) > 0:

            dt = Frame()
            # Header

            ## Magic word, {0x0102,0x0304,0x0506,0x0708}
            byteVecIdx = index[0]+8 # magic word (4 unit16)
            #numDetectedObj = 0

            ## Version, uint32: MajorNum * 2^24 + MinorNum * 2^16 + BugfixNum * 2^8 + BuildNum
            dt.tlv_version = a[byteVecIdx:byteVecIdx + 4]
            dt.tlv_version_uint16 = dt.tlv_version[2] + (dt.tlv_version[3] << 8)
            byteVecIdx += 4

            ## Total packet length (including header) in bytes, uint32
            # bf = np.array([x for x in a[byteVecIdx:byteVecIdx + 4]])  # type: ignore
            # b_m = np.array(BYTE_MULT)  # type: ignore
            # totalPacketLen = int(np.sum(np.dot(bf, b_m)))  # type: ignore
            totalPacketLen = int.from_bytes(a[byteVecIdx:byteVecIdx + 4], byteorder='little')  # type: ignore
            print("Total Packet Length:", totalPacketLen)
            byteVecIdx += 4

            chunks.append(a)
            chunks.append(s.read(totalPacketLen-8))  # type: ignore
            raw_bv = b''.join(chunks)
            bv = [x for x in raw_bv]
            
            if (len(bv) >= totalPacketLen):
                # Platform type, uint32: 0xA1643 or 0xA1443 
                dt.tlv_platform = np.sum(np.dot(bv[byteVecIdx:byteVecIdx + 4], BYTE_MULT))  # type: ignore
                byteVecIdx += 4

                # Frame number, uint32
                dt.frameNumber = np.sum(np.dot(bv[byteVecIdx:byteVecIdx + 4], BYTE_MULT))  # type: ignore
                byteVecIdx += 4

                # Time in CPU cycles when the message was created. For AR16xx: DSP CPU cycles
                # timeCpuCycles = np.sum(np.dot(bv[byteVecIdx:byteVecIdx + 4], BYTE_MULT))  # type: ignore
                byteVecIdx += 4

                # Number of detected objects, uint32
                dt.numDetectedObj = np.sum(np.dot(bv[byteVecIdx:byteVecIdx + 4], BYTE_MULT))  # type: ignore
                byteVecIdx += 4

                # Number of TLVs, uint32
                numTLVs = int(np.sum(np.dot(bv[byteVecIdx:byteVecIdx + 4], BYTE_MULT)))  # type: ignore
                byteVecIdx += 4

                byteVecIdx += 4

                #  start_tlv_ticks: int
                for _ in range(numTLVs):
                    if(byteVecIdx>len(bv)):
                        break
                    tlv_type = np.sum(np.dot(bv[byteVecIdx:byteVecIdx + 4], BYTE_MULT))  # type: ignore
                    byteVecIdx += 4
                    tlv_length = int(np.sum(np.dot(bv[byteVecIdx:byteVecIdx + 4], BYTE_MULT)))  # type: ignore
                    byteVecIdx += 4
                    # print(numTLVs, tlv_type, tlv_
                    # length)

                    # tlv payload
                    if (TLV_type(tlv_type) == TLV_type.MMWDEMO_OUTPUT_MSG_DETECTED_POINTS):
                        # will not get this type if numDetectedObj == 0 even though gui monitor selects this type
                        dt.detectedPoints_byteVecIdx = byteVecIdx
                    # elif (tlv_type == TLV_type.MMWDEMO_OUTPUT_MSG_RANGE_PROFILE):
                    #     #Params.rangeProfile_byteVecIdx = byteVecIdx
                    #     pass
                    # elif (tlv_type == TLV_type.MMWDEMO_OUTPUT_MSG_NOISE_PROFILE):
                    #     # processRangeNoiseProfile(bytevec, byteVecIdx, Params, false)
                    #     # gatherParamStats(Params.plot.noiseStats, getTimeDiff(start_tlv_ticks))
                    #     pass
                    # elif (tlv_type == TLV_type.MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP):
                    #     pass
                    #     # processAzimuthHeatMap(bytevec, byteVecIdx, Params)
                    #     # gatherParamStats(Params.plot.azimuthStats, getTimeDiff(start_tlv_ticks))
                    # elif (tlv_type == TLV_type.MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP):
                    #     pass
                    #     # processRangeDopplerHeatMap(bytevec, byteVecIdx, Params)
                    #     # gatherParamStats(Params.plot.dopplerStats, getTimeDiff(start_tlv_ticks))
                    # elif (tlv_type == TLV_type.MMWDEMO_OUTPUT_MSG_STATS):
                    #     pass
                    #     # processStatistics(bytevec, byteVecIdx, Params)
                    #     # gatherParamStats(Params.plot.cpuloadStats, getTimeDiff(start_tlv_ticks))
                    # elif (tlv_type == TLV_type.MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO):
                    #     pass
                    #     # Params.sideInfo_byteVecIdx = byteVecIdx
                    # elif (tlv_type == TLV_type.MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP):
                    #     pass
                    #     # processAzimuthElevHeatMap(bytevec, byteVecIdx, Params)
                    #     # gatherParamStats(Params.plot.azimuthElevStats, getTimeDiff(start_tlv_ticks))
                    # elif (tlv_type == TLV_type.MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS):
                    #     pass
                    #     # processTemperatureStatistics(bytevec, byteVecIdx, Params)
                    #     # gatherParamStats(Params.plot.temperatureStats, getTimeDiff(start_tlv_ticks))
                    

                    byteVecIdx += tlv_length
                
                # print("TLV loop took {} seconds".format(time.time()-st_tlv))
                if(dt.detectedPoints_byteVecIdx > -1):
                    detObjRes = _processDetectedPoints(bv, dt.detectedPoints_byteVecIdx, dt, raw_bv)

                    if detObjRes is not None:
                        valid_doppler = np.greater(np.abs(detObjRes.doppler), self._doppler_filtering)

                        obj_np = np.array([                                 # type: ignore
                                detObjRes.x_coord[valid_doppler],
                                detObjRes.y_coord[valid_doppler],
                                detObjRes.z_coord[valid_doppler],
                                detObjRes.doppler[valid_doppler]]).T 
                        obj: DopplerPointCloud = DopplerPointCloud(obj_np)

                        result['detected_points'] = obj
                        return result
            a = b''
        else:
            pass
            # print("No data.")