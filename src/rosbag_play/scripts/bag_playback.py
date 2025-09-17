#!/usr/bin/env python3
import rospy
import rosbag
from sensor_msgs.msg import Image, Imu
from rosgraph_msgs.msg import Clock
import time
import sys

def playback_with_remap(bag_path):
    # Khởi tạo node
    rospy.init_node('bag_playback_node', anonymous=True)
    
    # Kiểm tra tham số /use_sim_time
    use_sim_time = rospy.get_param('/use_sim_time', False)
    
    # Publisher cho topics remap
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    imu_pub = rospy.Publisher('/imu_raw', Imu, queue_size=10)
    clock_pub = None
    if use_sim_time:
        clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
    
    # Mở bag file
    bag = rosbag.Bag(bag_path)
    start_time = None
    
    for topic, msg, t in bag.read_messages(topics=['/cam0/image_raw', '/imu0']):
        if start_time is None:
            start_time = time.time() - t.to_sec()  # Để simulate real-time từ timestamp đầu
        
        # Xuất bản /clock nếu use_sim_time bật
        if use_sim_time and clock_pub:
            clock_msg = Clock()
            clock_msg.clock = t  # Sử dụng timestamp từ bag file
            clock_pub.publish(clock_msg)
        
        # Remap và publish
        if topic == '/cam0/image_raw':
            msg.header.stamp = t if use_sim_time else rospy.Time.now()
            image_pub.publish(msg)
        elif topic == '/imu0':
            msg.header.stamp = t if use_sim_time else rospy.Time.now()
            imu_pub.publish(msg)
        
        # Sleep để simulate real-time (dựa trên timestamp giữa messages)
        current_time = time.time() - start_time
        sleep_duration = t.to_sec() - current_time
        if sleep_duration > 0:
            time.sleep(sleep_duration)
    
    bag.close()
    rospy.loginfo("Playback hoàn thành.")

if __name__ == '__main__':
    try:
        rospy.init_node('bag_playback_node', anonymous=True)
        bag_path = rospy.get_param('~bag_path')  # Lấy từ param private (~)
        playback_with_remap(bag_path)
    except rospy.ROSInterruptException:
        pass