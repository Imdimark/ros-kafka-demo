#!/usr/bin/env python2.7

import sys
import rospy

from geometry_msgs.msg import Point

from confluent_kafka import Consumer

# API key values for connecting to local Kafka server
KAFKA_BOOTSTRAP_SERVER = '192.168.126.219:9096,192.168.126.219:9097,192.168.126.219:9098'
KAFKA_API_KEY = 'theengineroom'
KAFKA_API_SECRET = "1tY=ZP43t20"

# API key values for connecting to Confluent's Kafka server
#KAFKA_BOOTSTRAP_SERVER = 'pkc-38xx2.eu-south-1.aws.confluent.cloud:9092'
#KAFKA_API_KEY = 'KIX3UBJKDYM747P5'
#KAFKA_API_SECRET = 'G4CzaTO/ZJbnRoUCwlixbYrttBFCJtNlU0Pe2HbvNXnTZ/Pr3IjSh4kUcCdrqx9k'

# Fixed Kafka topic ball location
KAFKA_TOPIC = "object_0_pose"

class KafkaConsumerNode:

    def __init__(self, bootstrap_serv, api_key, api_secret):

        rospy.init_node('kafka_consumer_node', anonymous=True, disable_signals=True)


        self.consumer = Consumer({
            'bootstrap.servers': bootstrap_serv,
            'sasl.mechanism': 'PLAIN',
            'security.protocol': 'SASL_PLAINTEXT',
            #'security.protocol': 'SASL_SSL',
            'sasl.username': api_key,
            'sasl.password': api_secret,
            'group.id': 'mygroup',
            'auto.offset.reset': 'earliest'
        })
        # subscribe to topic
        self.consumer.subscribe([KAFKA_TOPIC])

        # ROS publishers
        self.panda_goal_pub = rospy.Publisher("/panda/ball_location", Point, queue_size=10)
        self.ur5_goal_pub = rospy.Publisher("/ur5/ball_location", Point, queue_size=10)

        
    # Internal loop routine for listerning to kafka's trajectory streaming
    def loop_routine(self):

        try:
            while True:
                # Poll Kafka server with 1 sec timeout
                msg = self.consumer.poll(1.0)
                
                # Proceed to next iteration if timeout expired...
                if msg is None:
                    continue
                if msg.error():
                    print("Consumer error: {}".format(msg.error()))
                    continue
                # If read message, decode value()
                point_msg_str = msg.value().split()

                
                # Parse message based on robot type
                p = Point()
                p.x = -0.01 * float(point_msg_str[4][:-1])
                p.y = 0.01 * float(point_msg_str[7][:-1])
                p.z = float(point_msg_str[10][:-1])
                print (p)
                #rospy.loginfo("Got ball location, current index is: {0}".format(i))

                if p.z == 1:
                    self.panda_goal_pub.publish(p)
                else:
                    self.ur5_goal_pub.publish(p)

                #i += 1
               
        except KeyboardInterrupt:
            # Close Kafka connection
            self.consumer.close()
            rospy.signal_shutdown(None)
            sys.exit(0)


def main():
    
    # Instantiate consumer
    kc = KafkaConsumerNode(KAFKA_BOOTSTRAP_SERVER, KAFKA_API_KEY, KAFKA_API_SECRET)

    rospy.sleep(2.0)
    rospy.loginfo("Kafka consumer initialized, listening to topic '{0}'".format(KAFKA_TOPIC))

    # Perform loop routine...
    kc.loop_routine()




if __name__ == '__main__':
  main()
