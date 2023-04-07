#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from moa_msgs.msg import CAN
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
from random import randint, random

class PublishAckermannMsg(Node):
    def __init__(self):
        super().__init__("ackermann_publiser_node")
        self.ackerman_publisher = self.create_publisher(AckermannDrive, "/cmd_vel", 10, callback_group = ReentrantCallbackGroup())
        self.create_timer(2, self.publish_msg)
    
    def publish_msg(self):
        args = {"steering_angle": random()*2,
                "steering_angle_velocity": random(),
                "speed": float(randint(0,60)),
                "acceleration": float(randint(-30,30)),
                "jerk": float(randint(-7,7))}
        ackermsg = AckermannDrive(**args)
        self.ackerman_publisher.publish(ackermsg)

class PublishCANMsg(Node):
    def __init__(self):
        super().__init__("can_publisher_node")
        self.ackerman_subscriber = self.create_subscription(AckermannDrive, "/cmd_vel", self.publish_msg, 10, callback_group = ReentrantCallbackGroup())
        self.CAN_publisher = self.create_publisher(CAN, "/pub_raw_can", 10, callback_group = ReentrantCallbackGroup())

    def publish_msg(self, msg: AckermannDrive):
        # get ackerman message
        # steering angle - convert to bytes (2 uint 8/bytes)
        sta = np.float16(msg.steering_angle).tobytes()
        # steering angle velocity - same conversion process as steering angle
        stav = np.float16(msg.steering_angle_velocity).tobytes()
        # speed - already a uint8 value
        sp = int(msg.speed)
        # acceleration - between -127 to 127 so take remainder of 256 (uint8)
        acc = int(msg.acceleration) % 256
        # jerk - same conversion process as acceleration
        jk = int(msg.jerk) % 256
        # CAN msg instance
        cmd = CAN()
        cmd.id = randint(0, 65535)
        cmd.is_rtr = False
        cmd.data = np.array([sta[0], sta[1], stav[0], stav[1], sp, acc, jk, 0], dtype=np.uint8) 
        self.CAN_publisher.publish(cmd)
        convertedValues = {'steering angle':np.frombuffer(int(cmd.data[0]).to_bytes(1,'big')+int(cmd.data[1]).to_bytes(1,'big'), np.float16)[0],
                           'steering angle velocity':np.frombuffer(int(cmd.data[2]).to_bytes(1,'big')+int(cmd.data[3]).to_bytes(1,'big'), np.float16)[0],
                           'speed':cmd.data[4],
                           'acceleration':[cmd.data[5] if cmd.data[5]<=127 else cmd.data[5]-256][0],
                           'jerk':[cmd.data[6] if cmd.data[6]<=127 else cmd.data[6]-256][0]}
        print(f"converted values: {convertedValues} \n")

def main():
    rclpy.init()
    executioner = MultiThreadedExecutor(num_threads=2)
    executioner.add_node(PublishAckermannMsg())
    executioner.add_node(PublishCANMsg())
    try:
        executioner.spin()
    except KeyboardInterrupt:
        executioner.get_nodes()[0].get_logger().info('Keyboard interrupt, shutting down.\n')
    rclpy.shutdown()

if __name__ == '__main__':
    main()

