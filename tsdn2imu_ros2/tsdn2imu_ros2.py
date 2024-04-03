import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, FluidPressure, Temperature
from std_msgs.msg import Header

import signal
import math

import serial

from typing import List

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
        self.airPT=[0.0, 0.0]

        self.port = serial.Serial(data_port, baudrate)

    def sensor_init(self) :
        self.port.write(bytes(add_bcc([CMD_HEADER, 0x16, 40, 10, 0x00])))
        self.port.write(bytes(add_bcc([CMD_HEADER, 0x18, 40, 10, 0x00])))
        self.port.write(bytes(add_bcc([CMD_HEADER, 0x1A, 4, 10, 0x00])))
    
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
        check_buf = list(byte_buffer)
        if len(byte_buffer)<4:
            return -1
        if add_bcc([CMD_HEADER]+check_buf[:-2])[-1] != byte_buffer[-2]:
            return -2

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
            return 1
            
        elif byte_buffer[0] == 0x81 :
            self.geomag=[
                int.from_bytes(byte_buffer[5:8], "little", signed=True),
                int.from_bytes(byte_buffer[8:11], "little", signed=True),
                int.from_bytes(byte_buffer[11:14], "little", signed=True),
            ]
            return 2
        
        elif byte_buffer[0] == 0x82 :
            self.airPT=[
                int.from_bytes(byte_buffer[5:8], "little", signed=True),
                int.from_bytes(byte_buffer[8:10], "little", signed=True),
            ]
            return 3
        
        else:
            return 0

class imu_publisher(Node):
    def __init__(self):
        super().__init__('tsdn121_imu_pub') 

        self.pub_imu = self.create_publisher(Imu, '/sensor/tsdn121/Imu', 10)
        self.pub_mag = self.create_publisher(MagneticField, '/sensor/tsdn121/MagneticField', 10)
        self.pub_airP = self.create_publisher(FluidPressure, '/sensor/tsdn121/AirPressure', 10)
        self.pub_airT = self.create_publisher(Temperature, '/sensor/tsdn121/AirTemperature', 10)
        self.timer_pub = self.create_timer(0.001, self.timer_pub_callback)

        self.imu_sensor = tsdn121_interface(data_port = "/dev/ttyACM0", baudrate = 9600)
        self.get_logger().info('senser init')
        self.imu_sensor.sensor_init()
        self.get_logger().info('senser start')
        self.imu_sensor.sensor_start_freerun()
        self.buffer = bytes()
        self.ret = -1

    def stop(self):
        self.imu_sensor.sensor_stop()

    def timer_pub_callback(self):
        self.buffer = self.imu_sensor.sensor_read_until(b"\x9A")
        if self.buffer[-1] == 0x9A:
            self.ret = self.imu_sensor.decode_data(self.buffer)

            msg_imu = Imu()
            msg_mag = MagneticField()
            msg_airP = FluidPressure()
            msg_airT = Temperature()

            time_stamp = self.get_clock().now().to_msg()

            if self.ret == 1:
                msg_imu.header = Header()
                msg_imu.header.stamp = time_stamp
                msg_imu.header.frame_id = "imu_link"

                msg_imu.orientation_covariance[0] = -1
                msg_imu.orientation.x = 0.0
                msg_imu.orientation.y = 0.0
                msg_imu.orientation.z = 0.0
                msg_imu.orientation.w = 0.0

                msg_imu.angular_velocity_covariance = [0,0,0,0,0,0,0,0,0]
                msg_imu.angular_velocity.x = math.radians(self.imu_sensor.gyro[0]/100.0)
                msg_imu.angular_velocity.y = math.radians(self.imu_sensor.gyro[1]/100.0)
                msg_imu.angular_velocity.z = math.radians(self.imu_sensor.gyro[2]/100.0)

                msg_imu.linear_acceleration_covariance  = [0,0,0,0,0,0,0,0,0]
                msg_imu.linear_acceleration.x = (self.imu_sensor.acc[0]*9.8)/10000.0
                msg_imu.linear_acceleration.y = (self.imu_sensor.acc[1]*9.8)/10000.0
                msg_imu.linear_acceleration.z = (self.imu_sensor.acc[2]*9.8)/10000.0
                self.pub_imu.publish(msg_imu)

            elif self.ret == 2:
                msg_mag.header = Header()
                msg_mag.header.stamp = time_stamp
                msg_mag.header.frame_id = "imu_link"
                msg_mag.magnetic_field.x = self.imu_sensor.geomag[0]/10.0
                msg_mag.magnetic_field.y = self.imu_sensor.geomag[1]/10.0
                msg_mag.magnetic_field.z = self.imu_sensor.geomag[2]/10.0
                self.pub_mag.publish(msg_mag)

            elif self.ret == 3:
                msg_airP.header = Header()
                msg_airP.header.stamp = time_stamp
                msg_airP.header.frame_id = "imu_link"
                msg_airP.fluid_pressure = (float)(self.imu_sensor.airPT[0])
                self.pub_airP.publish(msg_airP)
                msg_airT.header = Header()
                msg_airT.header.stamp = time_stamp
                msg_airT.header.frame_id = "imu_link"
                msg_airT.temperature = self.imu_sensor.airPT[1]/10.0
                self.pub_airT.publish(msg_airT)
            
            self.ret = 0
        
global imu_pub

def ctrlc_handler(signum, frame):        #ctrl+c handle
    global imu_pub
    imu_pub.stop()
    print("Exiting")
    exit(0)

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

