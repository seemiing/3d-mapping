#!/usr/bin/env python3
import rospy
import tf
from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf2_sensor_msgs

def callback(data):
    try:
        # Lấy transform từ frame "image" sang "global"
        trans = tf_buffer.lookup_transform("global", data.header.frame_id, data.header.stamp, rospy.Duration(1.0))
        # Transform point cloud
        transformed_cloud = tf2_sensor_msgs.do_transform_cloud(data, trans)
        transformed_cloud.header.frame_id = "global"
        # Publish point cloud đã transform
        pub.publish(transformed_cloud)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Transform error: %s", e)

if __name__ == '__main__':
    rospy.init_node('transform_point_cloud', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    pub = rospy.Publisher('/voxel_cloud_transformed', PointCloud2, queue_size=10)
    sub = rospy.Subscriber('/voxel_cloud', PointCloud2, callback)
    rospy.spin()