#!/usr/bin/env python
"""
This script parses and converts Trimble GSOF messages incoming from a receiver and publishes the relevant ROS messages.
It  has been adapted from https://kb.unavco.org/kb/article/trimble-netr9-receiver-gsof-messages-806.html

@author: Michael Hutchinson (mhutchinson@sagarobotics.com)
"""


from struct import unpack
from trimble_gnss_driver.parser import parse_maps
from trimble_gnss_driver.gps_qualities import gps_qualities
import socket
import sys
import math
import time
import rclpy
from rclpy.node import Node

# from tf_transformations import  quaternion_from_euler
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu # For lat lon h
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


"""
GSOF messages from https://www.trimble.com/OEM_ReceiverHelp/#GSOFmessages_Overview.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257COverview%257C_____0
"""
# Records we most care about
ATTITUDE = 27              # Attitude information with errors
LAT_LON_H = 2              # Position lat lon h
POSITION_SIGMA = 12        # Errors in position
POSITION_TYPE = 38
BASE_POSITION_QUALITY = 41 # Needed for gps quality indicator
INS_FULL_NAV = 49          # INS fused full nav info pose, attittude etc
INS_RMS = 50               # RMS errors from reported fused position
RECEIVED_BASE_INFO = 35    # Received base information

# Others (but still not the entire list of GSOF msgs available.)
VELOCITY = 8
SERIAL_NUM = 15
GPS_TIME = 1
UTC_TIME = 16
ECEF_POS = 3
LOCAL_DATUM = 4
LOCAL_ENU= 5


def quaternion_from_euler(roll, pitch, yaw):
    """
    Conveniance function to convert quaternion to euluer.
    To be replaced by tf_transformations when we upgrade to Ubuntu 22
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q



class GSOFDriver(Node):
    """ A Ros node to parse GSOF messages from a TCP stream. """

    def __init__(self):
        super().__init__('trimble_gnss_driver')

        self.port = self.declare_parameter('rtk_port', 21098).value
        self.ip = self.declare_parameter('rtk_ip', '192.168.0.50').value
        self.output_frame_id = self.declare_parameter('output_frame_id','gps_main').value

        apply_dual_antenna_offset = self.declare_parameter('apply_dual_antenna_offset', False).value
        gps_main_frame_id = self.declare_parameter('gps_main_frame_id', 'gps_link').value
        gps_aux_frame_id = self.declare_parameter('gps_aux_frame_id', 'gps_link').value


        if apply_dual_antenna_offset:
            self.heading_offset = self.get_heading_offset(gps_main_frame_id, gps_aux_frame_id)
        else:
            self.heading_offset = 0.0
        self.get_logger().info(f'Heading offset is {self.heading_offset}')

        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)

        # For attitude, use IMU msg to keep compatible with robot_localization
        # But note that this is not only from an IMU
        self.attitude_pub = self.create_publisher(Imu, 'attitude', 10)

        # yaw from the dual antennas fills an Imu msg simply to keep consistent
        # with the current setup
        # Keep separate from attitude to avoid accidentally fusing zeros when
        # we don't measure roll/pitch
        self.yaw_pub = self.create_publisher(Imu, 'yaw', 10)

        self.client = self.setup_connection()

        self.buffer = b''
        self.msg_dict = {}
        self.msg_bytes = None
        self.checksum = True
        self.rec_dict = {}

        current_time = self.get_clock().now()
        self.current_time_msg = current_time.to_msg()
        self.current_time_seconds = current_time.nanoseconds * 1e-9
        self.ins_rms_ts = 0
        self.pos_sigma_ts = 0
        self.quality_ts = 0
        self.error_info_timeout = 1.0
        self.base_info_timeout = 5.0

        self.create_timer(1/20, self.run)

    def run(self):

        # READ GSOF STREAM
        self.records = []
        current_time = self.get_clock().now()
        self.current_time_msg = current_time.to_msg()
        self.current_time_seconds = current_time.nanoseconds * 1e-9
        if not self.get_message():
            return
        self.get_records()

        # Make sure we have the information required to publish msgs and
        # that its not too old
        if INS_FULL_NAV in self.records:
            if INS_RMS in self.records or self.current_time_seconds - self.ins_rms_ts < self.error_info_timeout:
                # print "Full INS info, filling ROS messages"
                self.send_ins_fix()
                self.send_ins_attitude()
            else:
                self.get_logger().warn(f'Skipping INS output as no matching errors within the timeout. Current time: {self.current_time_seconds}, last error msg {self.ins_rms_ts}')
        else:
            if LAT_LON_H in self.records:
                if (POSITION_SIGMA in self.records or self.current_time_seconds - self.pos_sigma_ts < self.error_info_timeout) and (POSITION_TYPE in self.records or self.current_time_seconds - self.quality_ts < self.base_info_timeout):
                    self.send_fix()
                else:
                    self.get_logger().warn(f'Skipping fix output as no corresponding sigma errors or gps quality within the timeout. Current time: {self.current_time_seconds}, last sigma msg {self.pos_sigma_ts}, last gps quality msg {self.quality_ts}')
            if ATTITUDE in self.records:
                self.send_yaw()
        # if RECEIVED_BASE_INFO in self.records or LOCAL_DATUM in self.records or LOCAL_ENU in self.records:
        #     print(self.rec_dict)

            # print("Base Info: \n", self.rec_dict['BASE_NAME_1'], self.rec_dict['BASE_ID_1'], self.rec_dict['BASE_LATITUDE'], self.rec_dict['BASE_LONGITUDE'], self.rec_dict['BASE_HEIGHT'])
        # if INS_FULL_NAV in self.records and LAT_LON_H in self.records:
        #     print("Altitude INS: ", self.rec_dict['FUSED_ALTITUDE'], "Height LLH (WGS84): ", self.rec_dict['HEIGHT_WGS84'])


    def send_ins_fix(self):
        if self.rec_dict['FUSED_LATITUDE'] == 0 and self.rec_dict['FUSED_LONGITUDE']  == 0 and self.rec_dict['FUSED_ALTITUDE'] == 0:
            self.get_logger().warn("Invalid fix, skipping")
            return

        fix = NavSatFix()
        fix.header.stamp = self.current_time_msg
        fix.header.frame_id = self.output_frame_id

        gps_qual = gps_qualities[self.rec_dict['GPS_QUALITY']]
        fix.status.service = NavSatStatus.SERVICE_GPS # TODO: Fill correctly
        fix.status.status = gps_qual[0]
        fix.position_covariance_type = gps_qual[1]

        fix.latitude = self.rec_dict['FUSED_LATITUDE']
        fix.longitude = self.rec_dict['FUSED_LONGITUDE']

        # To follow convention set in the NavSatFix definition, altitude should be:
        # Altitude [m]. Positive is above the WGS 84 ellipsoid
        # Ref - http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html

        fix.altitude = self.rec_dict['FUSED_ALTITUDE']  # <-- CHECK

        fix.position_covariance[0] = self.rec_dict['FUSED_RMS_LONGITUDE'] ** 2
        fix.position_covariance[4] = self.rec_dict['FUSED_RMS_LATITUDE'] ** 2
        fix.position_covariance[8] = self.rec_dict['FUSED_RMS_ALTITUDE'] ** 2

        self.fix_pub.publish(fix)


    def send_ins_attitude(self):
        """
        We send the GNSS fused attitude information as an IMU msg to
        keep compatible with the robot_localization package. Although it should
        be noted that this is not just from an IMU but fused with dual gnss
        antennas information.
        """

        if self.rec_dict['FUSED_ROLL'] == 0 and self.rec_dict['FUSED_PITCH']  == 0 and self.rec_dict['FUSED_YAW'] == 0:
            self.get_logger().warn("Invalid yaw, skipping")
            return

        attitude = Imu()

        attitude.header.stamp = self.current_time_msg
        attitude.header.frame_id = self.output_frame_id  # Assume transformation handled by receiver

        heading_enu = 2*math.pi - self.normalize_angle(math.radians(self.rec_dict['FUSED_YAW']) + 3*math.pi/2)
        orientation_quat = quaternion_from_euler(math.radians(self.rec_dict['FUSED_ROLL']),     #  roll sign stays the same
                                             - math.radians(self.rec_dict['FUSED_PITCH']),  # -ve for robots coord system (+ve down)
                                             heading_enu)
        # print 'r p y_enu receiver_heading [degs]: ', self.rec_dict['FUSED_ROLL'], self.rec_dict['FUSED_PITCH'], math.degrees(heading_enu), self.rec_dict['FUSED_YAW']

        attitude.orientation = orientation_quat

        attitude.orientation_covariance[0] = math.radians(self.rec_dict['FUSED_RMS_ROLL']) ** 2  # [36] size array
        attitude.orientation_covariance[4] = math.radians(self.rec_dict['FUSED_RMS_PITCH']) ** 2  # [36] size array
        attitude.orientation_covariance[8] = math.radians(self.rec_dict['FUSED_RMS_YAW']) ** 2 # [36] size array

        self.attitude_pub.publish(attitude)


    def send_fix(self):
        if self.rec_dict['LATITUDE'] == 0 and self.rec_dict['LONGITUDE']  == 0 and self.rec_dict['HEIGHT_WGS84'] == 0:
            self.get_logger().warn("Invalid fix, skipping")
            return

        fix = NavSatFix()

        fix.header.stamp = self.current_time_msg
        fix.header.frame_id = self.output_frame_id
        gps_qual = self.get_gps_quality()
        fix.status.service = NavSatStatus.SERVICE_GPS # TODO: Fill correctly
        fix.status.status = gps_qual[0]
        fix.position_covariance_type = gps_qual[1]

        fix.latitude = math.degrees(self.rec_dict['LATITUDE'])
        fix.longitude = math.degrees(self.rec_dict['LONGITUDE'])

        # To follow convention set in the NavSatFix definition, altitude should be:
        # Altitude [m]. Positive is above the WGS 84 ellipsoid
        # Ref - http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html

        fix.altitude = self.rec_dict['HEIGHT_WGS84']  # <-- CHECK

        fix.position_covariance[0] = self.rec_dict['SIG_EAST'] ** 2  # Check east north order
        fix.position_covariance[4] = self.rec_dict['SIG_NORT'] ** 2
        fix.position_covariance[8] = self.rec_dict['SIG_UP'] ** 2

        self.fix_pub.publish(fix)


    def send_yaw(self):
        """
        We send yaw (without ins) as an IMU message for compatibility with our
        other software
        """
        if self.rec_dict['ROLL'] == 0 and self.rec_dict['PITCH']  == 0 and self.rec_dict['YAW'] == 0:
            self.get_logger().warn("Invalid yaw, skipping")
            return

        yaw = Imu()

        yaw.header.stamp = self.current_time_msg
        yaw.header.frame_id = self.output_frame_id  # Assume transformation handled by receiver

        heading_ned = self.normalize_angle(self.rec_dict['YAW'] + self.heading_offset)
        heading_enu = 2*math.pi - self.normalize_angle(heading_ned + 3*math.pi/2)

        orientation_quat = quaternion_from_euler(self.rec_dict['ROLL'],     #  roll sign stays the same
                                             - self.rec_dict['PITCH'],  # -ve for robots coord system (+ve down)
                                             heading_enu)
        # print 'r p y receiver_heading [rads]: ', self.rec_dict['ROLL'], self.rec_dict['PITCH'], heading_enu, self.rec_dict['YAW']

        # yaw.orientation = Quaternion(*orientation_quat)
        yaw.orientation = orientation_quat

        yaw.orientation_covariance[0] = self.rec_dict['ROLL_VAR']  # [9]
        yaw.orientation_covariance[4] = self.rec_dict['PITCH_VAR']  # [9]
        yaw.orientation_covariance[8] = self.rec_dict['YAW_VAR'] # [9]

        self.yaw_pub.publish(yaw)


    def get_gps_quality(self):
        """Get ROS NavSatStatus position type from trimbles
        """
        trimble_position_type = self.rec_dict['POSITION_TYPE']

        if trimble_position_type >= 9:
            position_type = 4 # fix
        elif trimble_position_type >= 7:
            position_type = 5 # float
        elif trimble_position_type >= 1:
            position_type = 2
        else:
            position_type = 0

        return gps_qualities[position_type]


    @staticmethod
    def normalize_angle(angle_in):
        while angle_in > 2*math.pi:
            angle_in = angle_in - 2*math.pi
        while angle_in < 0:
            angle_in = angle_in + 2*math.pi
        return angle_in


    def get_heading_offset(self, gps_main_frame_id, gps_aux_frame_id):

        if gps_main_frame_id == gps_aux_frame_id:
            self.get_logger().error("Cannot offset antenna yaw if they have the same frame_id's, will assume 0.0")
            return 0.0

        #        Get GPS antenna tf
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer)

        self.get_logger().info( "Waiting for GPS tf between antennas.")
        got_gps_tf = False
        while not got_gps_tf:
            try:
                (trans, rot) = tf_buffer.lookupTransform(gps_main_frame_id, gps_aux_frame_id, rclpy.time.Time())
                got_gps_tf = True
            except (TransformException) as ex:
                got_gps_tf = False
                self.get_logger().info(f'Could not get tf from {gps_main_frame_id} to {gps_aux_frame_id}: {ex}')
                continue

        self.get_logger().info( "Got gps tf")
        dy_antennas = trans[1]
        dx_antennas = trans[0]

        if got_gps_tf:
            heading_offset = math.atan2(dy_antennas,dx_antennas)
        else:
            heading_offset = 0

        return heading_offset


    def setup_connection(self):
        attempts_limit = 10
        current_attempt = 0
        connected = False
        self.buffer = b'' # Reset buffer

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        while not connected and rclpy.ok() and current_attempt < attempts_limit:
            current_attempt += 1

            try:
                client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client.settimeout(5)
                ip = socket.gethostbyname(self.ip)
                address = (ip, self.port)

                self.get_logger().info(f'Attempting connection to {ip}:{self.port}')
                client.connect(address)

                self.get_logger().info("=====================================")
                self.get_logger().info(f'Connected to {ip}:{self.port}')
                self.get_logger().info("=====================================")
                connected = True
            except Exception as e:
                self.get_logger().warn(f'{e.__str__()}. Retrying, attempt: {current_attempt}/{attempts_limit}')
                time.sleep(2.0)

        if not connected:
            self.get_logger().error("No connection established. Node shutting down")
            sys.exit()

        return client


    def get_message(self):
        while len(self.buffer) < 7:
            try:
                self.buffer += self.client.recv(7)
            except socket.timeout:
                self.get_logger().warn("Socket timeout while reading message header")
                self.setup_connection()
                return False
        msg_field_names = ('STX', 'STATUS', 'TYPE', 'LENGTH',
                           'T_NUM', 'PAGE_INDEX', 'MAX_PAGE_INDEX')
        self.msg_dict = dict(zip(msg_field_names, unpack('>7B', self.buffer[:7])))

        total_length = 7 + self.msg_dict['LENGTH']
        while len(self.buffer) < total_length:
            try:
                self.buffer += self.client.recv(total_length - len(self.buffer))
            except socket.timeout:
                self.get_logger().warn("Socket timeout while reading message")
                self.setup_connection()
                return False

        # The message payload is from byte 7 up to (but not including) the last 3 bytes (checksum and ETX)
        self.msg_bytes = self.buffer[7:7 + self.msg_dict['LENGTH'] - 3]

        # The checksum is the first of the last 2 bytes before the ETX
        checksum_start = 7 + self.msg_dict['LENGTH'] - 3
        checksum_end = 7 + self.msg_dict['LENGTH'] - 1
        (checksum, _) = unpack('>2B', self.buffer[checksum_start:checksum_end])

        if checksum - self.checksum256(self.msg_bytes + self.buffer[1:7]) == 0:
            self.checksum = True
        else:
            self.get_logger().warn("Invalid checksum detected.")
            self.checksum = False

        self.buffer = self.buffer[total_length-1:]  # Remove the message header and data from the buffer

        return self.checksum

    def checksum256(self, st):
        """Calculate checksum"""
        if sys.version_info[0] >= 3:
            return sum(st) % 256
        else:
            return reduce(lambda x, y: x+y, map(ord, st)) % 256


    def get_records(self):
        self.byte_position = 0
        while self.byte_position < len(self.msg_bytes):
            # READ THE FIRST TWO BYTES FROM RECORD HEADER
            record_type, record_length = unpack('>2B', self.msg_bytes[self.byte_position:self.byte_position + 2])
            self.byte_position += 2
            self.records.append(record_type)
            # self.select_record(record_type, record_length)
            if record_type in parse_maps:
                self.rec_dict.update(dict(zip(parse_maps[record_type][0], unpack(parse_maps[record_type][1], self.msg_bytes[self.byte_position:self.byte_position + record_length]))))
            else:
                self.get_logger().warn(f'Record type {record_type} is not in parse maps.')
            self.byte_position += record_length

        if INS_RMS in self.records:
            self.ins_rms_ts = self.current_time_seconds
        if POSITION_SIGMA in self.records:
            self.pos_sigma_ts = self.current_time_seconds
        if POSITION_TYPE in self.records:
            self.quality_ts = self.current_time_seconds


def main(args=None):
    rclpy.init(args=args)

    gsof_driver = GSOFDriver()

    rclpy.spin(gsof_driver)

    gsof_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
