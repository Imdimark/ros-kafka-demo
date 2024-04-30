#!/usr/bin/env python2.7

import rospy
import copy
import sys

from geometry_msgs.msg import Point, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lib.kafka_producer import KafkaProducer
from lib.panda_move_group import PandaMoveGroup

KAFKA_BOOTSTRAP_SERVER = '192.168.126.219:9096,192.168.126.219:9097,192.168.126.219.19:9098'
KAFKA_API_KEY = 'theengineroom'
KAFKA_API_SECRET = "1tY=ZP43t20"

# API key values for connecting to Confluent's Kafka server
#KAFKA_BOOTSTRAP_SERVER = 'pkc-38xx2.eu-south-1.aws.confluent.cloud:9092'
#KAFKA_API_KEY = 'KIX3UBJKDYM747P5'
#KAFKA_API_SECRET = 'G4CzaTO/ZJbnRoUCwlixbYrttBFCJtNlU0Pe2HbvNXnTZ/Pr3IjSh4kUcCdrqx9k'

# Fixed Kafka topic for Panda robot (always generated with Id == 0)
KAFKA_TOPIC = "robot_0_joint_trajectory"

class PandaSoccer:
   
    def __init__(self, panda_arm, kp):

      # Pass along move group and Kafka producer
      self.panda_arm = panda_arm
      self.kp = kp

      # Fixed Kafka topic
      self.topic = KAFKA_TOPIC

      # misc
      self.ball_location = None
      self.ball_location_received = False
      self.sleep_time = 7.5

      # Panda idle pose
      self.idle_pose = Pose()
      self.idle_pose.position.x = 0.3
      self.idle_pose.position.y = 0.0
      self.idle_pose.position.z = 0.6
      self.idle_pose.orientation.x = -0.38
      self.idle_pose.orientation.y = 0.923
      self.idle_pose.orientation.z = 0.0
      self.idle_pose.orientation.w = 0.0
      self.idle_joint_state = [-0.444, -0.83, 0.28, -2.349, 0.206, 1.54, 2.108]

      # Sub for ball location
      rospy.Subscriber("/panda/ball_location", Point, self.on_location_received, queue_size=10)

   
    # Callback for latest ball location
    def on_location_received(self, msg):
      self.ball_location_received = True
      self.ball_location = msg

    
    def get_debug_pose(self):
      print(self.panda_arm.get_ee_pose())
      print(self.panda_arm.get_joint_state())
      sys.exit(0)


    # Routine to set robot to idle pose
    def home_routine(self):

      joint_trajectory_msg = JointTrajectory()

      joint_trajectory_msg = JointTrajectory()
      joint_names = ["panda_joint{0}".format(i) for i in range(1,8)]
      joint_trajectory_msg.joint_names = joint_names

      point = JointTrajectoryPoint()
      point.positions = [0.0] * 7
      joint_trajectory_msg.points.append(point)

      point = JointTrajectoryPoint()
      point.positions = self.idle_joint_state
      joint_trajectory_msg.points.append(point)

      # Build and publish Kafka record from ROS message 
      self.kp.produce_record(self.topic, joint_trajectory_msg)
      

    def soccer_routine(self):
      
      # Plan to reach pose
      reach_pose = copy.deepcopy(self.idle_pose)
      reach_pose.position.x = self.ball_location.x
      reach_pose.position.y = self.ball_location.y
      reach_pose.position.z = 0.125
      
      print ("entro in soccer routine" )
      print (reach_pose.position)
      print (self.idle_joint_state, self.idle_pose)
      plan, fraction = self.panda_arm.plan_cartesian_path(self.idle_joint_state, reach_pose)
      print ("prima if")
      # Make sure plan is valid..
      if len(plan.joint_trajectory.points) > 2 and fraction > 0.99:
        self.kp.produce_record(self.topic, plan.joint_trajectory)
        print ("if")

      else:
        print ("else")
        return
      
      # Save last joint state as new initial state for next stage...
      last_joint_state = list(plan.joint_trajectory.points[-1].positions)

      rospy.sleep(self.sleep_time)

      # Move back to idle pose
      plan = self.panda_arm.plan_joint_state(last_joint_state, self.idle_joint_state)
      if len(plan.joint_trajectory.points) > 2:
        self.kp.produce_record(self.topic, plan.joint_trajectory)



def main():
    
    # Init Panda's move group for planning...
    panda_arm = PandaMoveGroup()
    # Init Kafka Producer...
    kp = KafkaProducer(KAFKA_BOOTSTRAP_SERVER, KAFKA_API_KEY, KAFKA_API_SECRET)

    # Init Panda's soccer routine class
    ppr = PandaSoccer(panda_arm, kp)
    #ppr.get_debug_pose()
    # Perform home routine to put robot in idle pose...
    ppr.home_routine()
    
    # Repeat indefinitely
    while not rospy.is_shutdown():
      if ppr.ball_location_received:
        print ("1")
      if ppr.ball_location is not None:
        print ("2")
      if ppr.ball_location_received and ppr.ball_location is not None:
         
        ppr.soccer_routine()
        ppr.ball_location_received = False
        ppr.ball_location = None

      # wait for ball location to be published...
      else:
         
         rospy.sleep(1.0)
         

       
if __name__ == '__main__':
  main()
