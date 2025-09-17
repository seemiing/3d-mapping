#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CameraInfo
import std_msgs.msg

def publish_camera_info():
    # Khởi tạo node ROS
    rospy.init_node('camera_info_publisher', anonymous=True)
    
    # Tạo publisher cho topic camera_info
    pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
    
    # Tạo message CameraInfo
    cam_info = CameraInfo()
    
    # Điền header
    cam_info.header = std_msgs.msg.Header()
    cam_info.header.frame_id = "image"  # Frame ID của camera
    cam_info.header.stamp = rospy.Time.now()
    
    # Độ phân giải từ kalibr_imucam_chain.yaml
    cam_info.width = 640
    cam_info.height = 480
    
    # Mô hình méo
    cam_info.distortion_model = "equidistant"
    
    # Hệ số méo
    cam_info.D = [-0.013721808247486035, 0.020727425669427896, -0.012786476702685545, 0.0025242267320687625]
    
    # Ma trận nội tại K
    fx = 278.66723066149086
    fy = 278.48991409740296
    cx = 319.75221200593535
    cy = 241.96858910358173
    cam_info.K = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
    
    # Ma trận chỉnh sửa R (ma trận đơn vị vì không có chỉnh sửa)
    cam_info.R = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
    
    # Ma trận chiếu P
    cam_info.P = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
    
    # Tần số publish (ví dụ: 30 Hz)
    rate = rospy.Rate(30)
    
    while not rospy.is_shutdown():
        # Cập nhật timestamp
        cam_info.header.stamp = rospy.Time.now()
        
        # Publish message
        pub.publish(cam_info)
        
        # Ngủ để duy trì tần số
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_info()
    except rospy.ROSInterruptException:
        pass