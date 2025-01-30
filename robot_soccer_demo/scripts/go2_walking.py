#http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29 #RICO-RM progetto
import rospy
from std_msgs.msg import String
from lib.kafka_producer import KafkaProducer
import tf2_ros
import tf.transformations

KAFKA_BOOTSTRAP_SERVERS = '192.168.1.7:9096,192.168.1.7:9097,192.168.1.7:9098'
KAFKA_API_KEY = 'theengineroom'
KAFKA_API_SECRET = "1tY=ZP43t20"
KAFKA_TOPIC = "go2Positi_onRotation"

class tfSender:
    def __init__(self):
        self.producer = KafkaProducer(KAFKA_BOOTSTRAP_SERVERS, KAFKA_API_KEY, KAFKA_API_SECRET)
        self.buffer=tf2_ros.Buffer()
        self.listener=tf2_ros.TransformListener(self.buffer)

    def send_tf(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                trans = self.buffer.lookup_transform('map', 'trunk', rospy.Time(0), rospy.Duration(1.0))
                x = trans.transform.translation.x
                y = trans.transform.translation.y
                quaternion = trans.transform.rotation
                
                eulerFromQuaternion = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
                #zRotationDegrees = eulerFromQuaternion[2] * (180.0 / 3.14159) 
                message = String()
                message.data =",{},{},{},".format(x, y, eulerFromQuaternion[2]) ## Due to python 2.7 cannot use f"{}" syntax
                
                self.producer.produce_record(KAFKA_TOPIC, message)
                rospy.loginfo("Robot position has been sent by go2Positi_onRotation kafka topic" )

            except Exception as e:
                rospy.loginfo("Error while sending data by Kafka: {}".format(e))

            rate.sleep()



if __name__ == '__main__':
    rospy.init_node('robotPositionSender')
    tfSendertf_sender = tfSender()
    tfSendertf_sender.send_tf() 