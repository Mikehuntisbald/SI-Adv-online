#!/usr/bin/env python

import rosbag
from sensor_msgs.msg import PointCloud

def modify_frame_id(input_bag_path, output_bag_path, new_frame_id='base_link'):
    with rosbag.Bag(input_bag_path, 'r') as inbag, rosbag.Bag(output_bag_path, 'w') as outbag:
        start_time = inbag.get_start_time()
        for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
            if t.to_sec() > start_time + 10:
                if topic == "/kitti/velo/pointcloud" :
                    #print("Processing message from topic: ", topic)
                    msg.header.frame_id = "base_link"
                    outbag.write(topic, msg, t)
                else : outbag.write(topic, msg, t)
            #else:
            #    outbag.write(topic, msg, t)
            #print(type(msg))
            # Check if the current message is a PointCloud2 message
            #if isinstance(msg, PointCloud):
            #    print("Processing message from topic: ", topic)
            #    msg.header.frame_id = new_frame_id
            #outbag.write(topic, msg, t)

if __name__ == '__main__':
    input_bag_path = './kitti_2011_09_26_drive_0005_synced.bag'
    output_bag_path = './kitti.bag'
    modify_frame_id(input_bag_path, output_bag_path)

