'''
The Python library for the Parker-LORD MicroStrain Sensing AHRS IMUs.
The library currently only supports reading Euler angles.

:reference: http://lord-microstrain.github.io/MSCL/Documentation/Getting%20Started/index.html?python#inertial
:device: 3DMGX5-AHRS

:authors: Carlos Carrasquillo, Emily Upton
:contact: ccarrasquillo3@gatech.edu
:created: September 23, 2021
:modified: September 24, 2021

'''

from os.path import dirname, abspath
import sys

LIB_DIR = abspath(dirname(__file__))
sys.path.append(LIB_DIR)

import mscl


class AHRS_RPY():
    '''
    A class used to pass around data for the Microstrain AHRS. 

    :methods: to_degrees(): returns an instance of AHRS_Data where all IMU measurements are 
                            expressed in degrees.

    :param: roll: roll, expressed in radians by default
    :param: pitch: pitch, expressed in radians by default
    :param: yaw: yaw, expressed in radians by default
    '''
    def __init__(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
    
    def to_degrees(self):
        RAD_2_DEG = 57.295779513
        roll = self.roll * RAD_2_DEG
        pitch = self.pitch * RAD_2_DEG
        yaw = self.yaw * RAD_2_DEG

        return AHRS_RPY(roll, pitch, yaw)

    def __str__(self):
        return "AHRS RPY | Roll: {:3.4f}, Pitch: {:3.4f}, Yaw: {:3.4f}".format(self.roll, self.pitch, self.yaw)


class AHRS_Raw():
        def __init__(self, ax, ay, az, gx, gy, gz):
                self.ax = ax
                self.ay = ay
                self.az = az
                self.gx = gx
                self.gy = gy
                self.gz = gz
        
        def __str__(self):
                return "AHRS Raw | AX: {:3.2f}, AY: {:3.2f}, AZ: {:3.2f}, GX: {:3.2f}, GY: {:3.2f}, GZ: {:3.2f}".format(self.ax, self.ay, self.az, self.gx, self.gy, self.gz)


class AHRS():
    '''
    A class used to read roll, pitch, and yaw data from the Microstrain AHRS IMUs. 

    :methods: update(): reads the latest packets from the AHRS IMU
    :methods: _get_latest_data(): returns an array of the data points from channels
                                  provided the channel names.
    :methods: get_rpy_rad(): fetches roll, pitch, and yaw data in radians
    :methods: get_rpy_deg(): fetches roll, pitch, and yaw data in degrees
    :methods: get_raw_data(): fetches scaledAccelX, scaledAccelY, scaledAccelZ, 
                              scaledGyroX, scaledGyroY, scaledGyroZ data

    :param: com: the COM port on the Raspberry Pi that the AHRS is connected to
    
    :attribute: node: the active AHRS node- every AHRS is its own node. contains connection 
                      and channel info
    '''
    def __init__(self, com):
        # create a serial connection with the specified COM port, at a specified baud rate
        connection = mscl.Connection.Serial(com, 115200)
        
        # create an InertialNode with each connection
        node = mscl.InertialNode(connection)

        # attemting to establish communication with the node. the program will exit if not successful
        success = node.ping()
        if not success:
            sys.exit("Unable to establish a connection with the Microstrain AHRS.")

        # if the node supports AHRS/IMU
        ahrsImuChannels = mscl.MipChannels()

        # append the channels of data ypu want to read
        ahrsImuChannels.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_EULER_ANGLES, mscl.SampleRate.Hertz(100)))
        ahrsImuChannels.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_SCALED_ACCEL_VEC, mscl.SampleRate.Hertz(100)))
        ahrsImuChannels.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_SCALED_GYRO_VEC, mscl.SampleRate.Hertz(100)))
        # ~ ahrsImuChannels.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER, mscl.SampleRate.Hertz(100)))
        # ~ ahrsImuChannels.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_ORIENTATION_QUATERNION, mscl.SampleRate.Hertz(100)))

        node.setActiveChannelFields(mscl.MipTypes.CLASS_AHRS_IMU, ahrsImuChannels)
        node.enableDataStream(mscl.MipTypes.CLASS_AHRS_IMU)

        self.node = node
        self.latest_packets = None

    def _read(self):
        '''
        Reads the latest packets from the AHRS IMU (timeout = 500).
        '''
        self.latest_packets = self.node.getDataPackets(500)

    def update(self, degrees=False):
        '''
        Reads the latest packets from the AHRS IMU.

        :return: a tuple containing AHRS_RPY [0] and a AHRS_Raw [1] objects
        '''
        self._read()
        
        rpy = self.get_rpy_deg() if degrees else self.get_rpy_rad()
        raw = self.get_raw_data()
        
        return (rpy, raw)

    def _get_latest_data(self, names):
        '''
        Returns an array of the data points from the provided channels.

        :param: names: the names of the channels to be read from the MSCL packet

        :return: a list of the data points from the channel with index n
        '''
        if self.latest_packets is None:
            return None
        
        measurements = dict.fromkeys(names, 0)
        for packet in self.latest_packets:
            point = packet.data()
            
            for dataPoint in point:
                channelName = dataPoint.channelName()
                if channelName in measurements:
                    measurements[channelName] = dataPoint.as_float()

        return measurements.values()

    def get_rpy_rad(self):
        '''
        Fetches roll, pitch, and yaw data in radians.

        :return: an AHRS_RPY object containing roll, pitch, and yaw data in radians 
        '''
        data = self._get_latest_data(["roll", "pitch", "yaw"])
        
        roll, pitch, yaw = tuple(data)
            
        rpy = AHRS_RPY(roll, pitch, yaw)
        return rpy

    def get_rpy_deg(self):
        '''
        Fetches roll, pitch, and yaw data in degrees.

        :return: an AHRS_RPY object containing roll, pitch, and yaw data in degrees 
        '''
        rpy = self.get_rpy_rad()
        return rpy.to_degrees()
        
    def get_raw_data(self):
        '''
        Fetches scaledAccelX, scaledAccelY, scaledAccelZ, scaledGyroX, 
        scaledGyroY, scaledGyroZ data.

        :return: an AHRS_Raw object containing scaledAccelX, 
                 scaledAccelY, scaledAccelZ, scaledGyroX, scaledGyroY, 
                 scaledGyroZ
        '''
        accel_data = self._get_latest_data(["scaledAccelX", "scaledAccelY", "scaledAccelZ"])
        gyro_data = self._get_latest_data(["scaledGyroX", "scaledGyroY", "scaledGyroZ"])
        
        ax, ay, az = tuple(accel_data)
        gx, gy, gz = tuple(gyro_data)
            
        raw_data = AHRS_Raw(ax, ay, az, gx, gy, gz)
        return raw_data
