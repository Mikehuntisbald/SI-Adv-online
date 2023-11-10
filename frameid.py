#!/usr/bin/env python

import rosbag
from sensor_msgs.msg import PointCloud
from rospy import Time

def modify_frame_id(input_bag_path, output_bag_path, new_frame_id='base_link'):
    with rosbag.Bag(output_bag_path, 'w') as outbag:
        flag = False
        for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
            global time
            if topic == "/tf":
                for transform in msg.transforms:
                    # 修改frame_id
                    if transform.header.frame_id == "map":
                        #print("Processing message from topic: ", topic)
                        transform.header.frame_id = "world"
                        time = t
                        flag = True
                        outbag.write(topic, msg, t)
            if topic == "/livox/lidar" and flag:
                #print("Processing message from topic: ", topic)
                #print(time)
                #time_obj = nano_to_time(time)
                msg.header.frame_id = "base_link"
                msg.header.stamp = time
                outbag.write(topic, msg, time)
                
            #else:
            #    outbag.write(topic, msg, t)
            #print(type(msg))
            # Check if the current message is a PointCloud2 message
            #if isinstance(msg, PointCloud):
            #    print("Processing message from topic: ", topic)
            #    msg.header.frame_id = new_frame_id
            #outbag.write(topic, msg, t)

if __name__ == '__main__':
    input_bag_path = './test_2023-08-11-19-05-12.bag'
    output_bag_path = './test2.bag'
    modify_frame_id(input_bag_path, output_bag_path)
