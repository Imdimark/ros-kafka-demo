#!/usr/bin/env python2.7

import rospy
import copy
import sys

from geometry_msgs.msg import Point, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lib.kafka_producer import KafkaProducer
from lib.ur5_move_group import UR5Movegroup

KAFKA_BOOTSTRAP_SERVER = '192.168.137.1:9093,192.168.137.1:9094,192.168.137.1:9095'
KAFKA_API_KEY = 'theengineroom'
KAFKA_API_SECRET = "1tY=ZP43t20"

# API key values for connecting to Confluent's Kafka server
#KAFKA_BOOTSTRAP_SERVER = 'pkc-38xx2.eu-south-1.aws.confluent.cloud:9092'
#KAFKA_API_KEY = 'KIX3UBJKDYM747P5'
#KAFKA_API_SECRET = 'G4CzaTO/ZJbnRoUCwlixbYrttBFCJtNlU0Pe2HbvNXnTZ/Pr3IjSh4kUcCdrqx9k'

# Fixed Kafka topic for UR5 robot (always generated with Id == 0)
KAFKA_TOPIC = "robot_1_joint_trajectory"

class UR5Soccer:
   
    def __init__(self, ur5_arm, kp):

      # Pass along move group and Kafka producer
      self.ur5_arm = ur5_arm
      self.kp = kp

      # Fixed Kafka topic
      self.topic = KAFKA_TOPIC

      # misc
      self.ball_location = None
      self.ball_location_received = False
      self.sleep_time = 7.5

      # UR5 idle pose
      self.idle_pose = Pose()
      self.idle_pose.position.x = 0.356
      self.idle_pose.position.y = -0.198
      self.idle_pose.position.z = 0.4557
      self.idle_pose.orientation.x = -0.7071
      self.idle_pose.orientation.y = 0.7071
      self.idle_pose.orientation.z = 0.0
      self.idle_pose.orientation.w = 0.0
      self.idle_joint_state = [-0.78, -1.79, 1.70, -1.47, 4.72, -0.77]


      # Sub for ball location
      rospy.Subscriber("/ur5/ball_location", Point, self.on_location_received, queue_size=10)

   
    # Callback for latest ball location
    def on_location_received(self, msg):
      self.ball_location_received = True
      self.ball_location = msg


    def get_debug_pose(self):
      print(self.ur5_arm.get_ee_pose())
      print(self.ur5_arm.get_joint_state())
      sys.exit(0)


    # Routine to set robot to idle pose
    def home_routine(self):
        
      joint_trajectory_msg = JointTrajectory()
      joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
      joint_trajectory_msg.joint_names = joint_names

      point = JointTrajectoryPoint()
      point.positions = [0.0] * 6
      joint_trajectory_msg.points.append(point)

      point = JointTrajectoryPoint()
      point.positions = self.idle_joint_state
      joint_trajectory_msg.points.append(point)

      # Build and publish Kafka record from ROS message 
      self.kp.produce_record(self.topic, joint_trajectory_msg)


    def soccer_routine(self):
      
      # Plan to reach pose
      reach_pose = copy.deepcopy(self.idle_pose)
      reach_pose.position.x = -self.ball_location.x + 4.0
      reach_pose.position.y = -self.ball_location.y
      reach_pose.position.z = 0.25

      print(reach_pose)

      plan, fraction = self.ur5_arm.plan_cartesian_path(self.idle_joint_state, reach_pose)
      # Make sure plan is valid..
      if len(plan.joint_trajectory.points) > 2 and fraction > 0.99:
        self.kp.produce_record(self.topic, plan.joint_trajectory)

      else:
        return
      
      # Save last joint state as new initial state for next stage...
      last_joint_state = list(plan.joint_trajectory.points[-1].positions)

      rospy.sleep(self.sleep_time)

      # Move back to idle pose
      plan = self.ur5_arm.plan_joint_state(last_joint_state, self.idle_joint_state)
      if len(plan.joint_trajectory.points) > 2:
        self.kp.produce_record(self.topic, plan.joint_trajectory)



def main():
    
    # Init UR5's move group for planning...
    ur5_arm = UR5Movegroup()
    # Init Kafka Producer...
    kp = KafkaProducer(KAFKA_BOOTSTRAP_SERVER, KAFKA_API_KEY, KAFKA_API_SECRET)

    # Init UR5's soccer routine class
    ppr = UR5Soccer(ur5_arm, kp)
    #ppr.get_debug_pose()
    # Perform home routine to put robot in idle pose...
    ppr.home_routine()
    
    # Repeat indefinitely
    while not rospy.is_shutdown():
      
      if ppr.ball_location_received and ppr.ball_location is not None:
         
        ppr.soccer_routine()
        ppr.ball_location_received = False
        ppr.ball_location = None

      # wait for ball location to be published...
      else:
         
         rospy.sleep(1.0)
         

       
if __name__ == '__main__':
  main()