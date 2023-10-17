import json
from confluent_kafka import Producer
from rospy_message_converter import message_converter


class KafkaProducer:

    def __init__(self, bootstrap_serv, api_key, api_secret):
      self.p = Producer({
          'bootstrap.servers': bootstrap_serv,
          'sasl.mechanism': 'PLAIN',
          'security.protocol': 'SASL_PLAINTEXT',
          #'security.protocol': 'SASL_SSL',
          'sasl.username': api_key,
          'sasl.password': api_secret,
          'allow.auto.create.topics': 'true'
      })

    def delivery_report(self, err, msg):
        if err is not None:
            print('Message delivery failed: {}'.format(err))
        else:
            print('Message delivered to {} [{}]'.format(msg.topic(), msg.partition()))


    def produce_record(self, topic,  msg):
      record = json.dumps(message_converter.convert_ros_message_to_dictionary(msg))
      self.p.produce(topic, record.encode('utf-8'), callback=self.delivery_report)
      self.p.flush()