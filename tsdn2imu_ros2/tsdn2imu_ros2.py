import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header

import signal
import math

import threading
import serial

from typing import List
import numpy as np

CMD_HEADER: int = 0x9A

def add_bcc(cmd: List[int]):
    """Compute BCC (= Block Checking Charactor) and append to command sequence.
    
    Returns:
        list of binary data with BCC code.
    """
    check: int = 0x00
    for b in cmd:
        check = check ^ b
    cmd.append(check)
    return cmd

class tsdn121_interface(object):
    def __init__(self, data_port, baudrate) :
        self.gyro = [0.0, 0.0, 0.0]
        self.acc  = [0.0, 0.0, 0.0]
        self.geomag=[0.0, 0.0, 0.0]

        self.port = serial.Serial(data_port, baudrate)

    def sensor_init(self) :
        self.port.write(bytes(add_bcc([CMD_HEADER, 0x16, 0x10, 0x05, 0x00])))
        self.port.write(bytes(add_bcc([CMD_HEADER, 0x18, 0x10, 0x05, 0x00])))
    
    def sensor_start_freerun(self) :
        self.port.write(bytes(add_bcc([CMD_HEADER, 0x13, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00])))

    def sensor_stop(self) :
        self.port.write(bytes(add_bcc([CMD_HEADER, 0x15, 0x00])))
        
    def sensor_read(self,size) :
        byte_buffer=self.port.read(size)
        return byte_buffer
    
    def sensor_read_until(self,exp) :
        byte_buffer=self.port.read_until(exp)
        return byte_buffer
    
    def decode_data(self, byte_buffer) :
        #print(list(byte_buffer))
        check_buf = list(byte_buffer)
        if len(byte_buffer)<4:
            return "err"
        if add_bcc([CMD_HEADER]+check_buf[:-2])[-1] != byte_buffer[-2]:
            return "err"

        if byte_buffer[0] == 0x80:
            self.acc=[
                int.from_bytes(byte_buffer[5:8], "little", signed=True),
                int.from_bytes(byte_buffer[8:11], "little", signed=True),
                int.from_bytes(byte_buffer[11:14], "little", signed=True),
            ]
            self.gyro=[
                int.from_bytes(byte_buffer[14:17], "little", signed=True),
                int.from_bytes(byte_buffer[17:20], "little", signed=True),
                int.from_bytes(byte_buffer[20:23], "little", signed=True),
            ]
            #print("acc",self.acc)
            #print("gyro",self.gyro)
            return "imu"
            
        elif byte_buffer[0] == 0x81 :
            self.geomag=[
                int.from_bytes(byte_buffer[5:8], "little", signed=True),
                int.from_bytes(byte_buffer[8:11], "little", signed=True),
                int.from_bytes(byte_buffer[11:14], "little", signed=True),
            ]
            #print("geomag",self.geomag)
            return "meg"
        else:
            return "none"

class imu_publisher(Node):
    def __init__(self):
        super().__init__('tsdn121_imu_pub') 

        self.pub_imu = self.create_publisher(Imu, '/sensor/tsdn121/Imu', 10)
        self.pub_mag = self.create_publisher(MagneticField, '/sensor/tsdn121/MagneticField', 10)
        self.timer_pub = self.create_timer(0.005, self.timer_pub_callback)
        self.timer_get = self.create_timer(0.0005, self.timer_get_callback)

        self.imu_sensor = tsdn121_interface(data_port = "/dev/ttyACM0", baudrate = 9600)
        self.get_logger().info('senser init')
        self.imu_sensor.sensor_init()
        self.get_logger().info('senser start')
        self.imu_sensor.sensor_start_freerun()
        self.buffer = bytes()
        self.ret = "err"

    def stop(self):
        self.imu_sensor.sensor_stop()

    def timer_get_callback(self):
        self.buffer = self.buffer+self.imu_sensor.sensor_read(1)
        if self.buffer[-1] == 0x9A:
            self.ret = self.imu_sensor.decode_data(self.buffer)
            self.buffer = bytes()

    def timer_pub_callback(self):
        msg_imu = Imu()
        msg_mag = MagneticField()

        if self.ret == "imu":
            msg_imu.header = Header()
            msg_imu.header.stamp = self.get_clock().now().to_msg()
            msg_imu.header.frame_id = "imu_link"
            msg_imu.orientation.x = 0.0
            msg_imu.orientation.y = 0.0
            msg_imu.orientation.z = 0.0
            msg_imu.orientation.w = 1.0
            msg_imu.angular_velocity.x = math.radians(self.imu_sensor.gyro[0]/100.0)
            msg_imu.angular_velocity.y = math.radians(self.imu_sensor.gyro[1]/100.0)
            msg_imu.angular_velocity.z = math.radians(self.imu_sensor.gyro[2]/100.0)
            msg_imu.linear_acceleration.x = (self.imu_sensor.acc[0]*9.8)/10000.0
            msg_imu.linear_acceleration.y = (self.imu_sensor.acc[1]*9.8)/10000.0
            msg_imu.linear_acceleration.z = (self.imu_sensor.acc[2]*9.8)/10000.0
            self.pub_imu.publish(msg_imu)
        elif self.ret == "meg":
            msg_mag.header = Header()
            msg_mag.header.stamp = self.get_clock().now().to_msg()
            msg_mag.header.frame_id = "imu_link"
            msg_mag.magnetic_field.x = self.imu_sensor.geomag[0]/10.0
            msg_mag.magnetic_field.y = self.imu_sensor.geomag[0]/10.0
            msg_mag.magnetic_field.z = self.imu_sensor.geomag[0]/10.0
            self.pub_mag.publish(msg_mag)

global imu_pub

def ctrlc_handler(signum, frame):        #ctrl+c handle
    global imu_pub
    imu_pub.stop()
    print("Exiting")
    exit(1)

def main(argv=None):
    global imu_pub

    signal.signal(signal.SIGINT, ctrlc_handler) #set ctrl-c handle 

    rclpy.init()
    imu_pub = imu_publisher()   
    rclpy.spin(imu_pub)
    
    #shutdown
    imu_pub.stop()
    imu_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

