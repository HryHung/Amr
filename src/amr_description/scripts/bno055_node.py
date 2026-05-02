#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

import board
import adafruit_bno055


def tuple_has_none(data, expected_len):
    if data is None:
        return True
    if len(data) < expected_len:
        return True
    return any(v is None for v in data[:expected_len])


class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')

        self.declare_parameter('i2c_address', 0x28)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('mag_topic', '/imu/mag')
        self.declare_parameter('publish_rate', 30.0)

        self.i2c_address = int(self.get_parameter('i2c_address').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.mag_topic = str(self.get_parameter('mag_topic').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 20)
        self.mag_pub = self.create_publisher(MagneticField, self.mag_topic, 20)

        self.get_logger().info(f'Opening BNO055 on I2C address 0x{self.i2c_address:02X}')

        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=self.i2c_address)

        _ = self.sensor.temperature
        time.sleep(0.5)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_callback)

        self.get_logger().info(
            f'BNO055 node started: frame_id={self.frame_id}, '
            f'imu_topic={self.imu_topic}, rate={self.publish_rate} Hz'
        )

    def publish_callback(self):
        try:
            quat = self.sensor.quaternion
            gyro = self.sensor.gyro
            lin_acc = self.sensor.linear_acceleration
            mag = self.sensor.magnetic
            calib = self.sensor.calibration_status

            if tuple_has_none(gyro, 3):
                self.get_logger().warn('Gyro incomplete, skipping publish', throttle_duration_sec=2.0)
                return

            if tuple_has_none(lin_acc, 3):
                self.get_logger().warn('Linear acceleration incomplete, skipping publish', throttle_duration_sec=2.0)
                return

            stamp = self.get_clock().now().to_msg()

            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = self.frame_id

            if not tuple_has_none(quat, 4):
                # BNO055 / Adafruit tuple order is (w, x, y, z)
                imu_msg.orientation.w = float(quat[0])
                imu_msg.orientation.x = float(quat[1])
                imu_msg.orientation.y = float(quat[2])
                imu_msg.orientation.z = float(quat[3])

                imu_msg.orientation_covariance = [
                    0.02, 0.0, 0.0,
                    0.0, 0.02, 0.0,
                    0.0, 0.0, 0.05
                ]
            else:
                imu_msg.orientation_covariance[0] = -1.0

            imu_msg.angular_velocity.x = float(gyro[0])
            imu_msg.angular_velocity.y = float(gyro[1])
            imu_msg.angular_velocity.z = float(gyro[2])

            imu_msg.linear_acceleration.x = float(lin_acc[0])
            imu_msg.linear_acceleration.y = float(lin_acc[1])
            imu_msg.linear_acceleration.z = float(lin_acc[2])

            imu_msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.02
            ]

            imu_msg.linear_acceleration_covariance = [
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.15
            ]

            self.imu_pub.publish(imu_msg)

            if not tuple_has_none(mag, 3):
                mag_msg = MagneticField()
                mag_msg.header.stamp = stamp
                mag_msg.header.frame_id = self.frame_id

                mag_msg.magnetic_field.x = float(mag[0]) * 1e-6
                mag_msg.magnetic_field.y = float(mag[1]) * 1e-6
                mag_msg.magnetic_field.z = float(mag[2]) * 1e-6

                mag_msg.magnetic_field_covariance = [
                    1e-6, 0.0, 0.0,
                    0.0, 1e-6, 0.0,
                    0.0, 0.0, 1e-6
                ]

                self.mag_pub.publish(mag_msg)

            if calib is not None:
                sys_c, gyro_c, accel_c, mag_c = calib
                self.get_logger().debug(
                    f'Calibration: Sys={sys_c} Gyro={gyro_c} Accel={accel_c} Mag={mag_c}'
                )

        except Exception as e:
            self.get_logger().warn(f'BNO055 publish failed: {e}', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BNO055Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()