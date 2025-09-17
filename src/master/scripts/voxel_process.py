#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.expanduser('~/catkin_ws_ov/src/master'))
import rospy
import ros_numpy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2

class VoxelProcessor:
    def __init__(self, voxel_size=0.5, min_points_per_voxel=2):
        rospy.init_node('voxel_processor', anonymous=True)
        self.sub = rospy.Subscriber('/voxel_cloud', PointCloud2, self.cloud_callback)
        self.pub = rospy.Publisher('/voxel_cloud_filtered', PointCloud2, queue_size=10)
        self.voxel_size = voxel_size
        self.min_points_per_voxel = min_points_per_voxel
        self.global_voxel_dict = {}  # Lưu điểm và màu theo chỉ số voxel
        self.has_rgb = False  # Kiểm tra trường RGB
        rospy.loginfo(f"Đã subscribe topic /voxel_cloud, publish topic /voxel_cloud_filtered, voxel_size = {self.voxel_size}m, min_points_per_voxel = {self.min_points_per_voxel}")

    def cloud_callback(self, msg):
        # Kiểm tra trường RGB
        if not self.has_rgb:
            for field in msg.fields:
                if field.name == 'rgb':
                    self.has_rgb = True
                    rospy.loginfo("Point cloud có trường RGB, sẽ giữ màu trong xử lý.")
                    break

        # Chuyển PointCloud2 thành mảng numpy
        cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        points = np.array([cloud_array['x'], cloud_array['y'], cloud_array['z']]).T
        if self.has_rgb:
            colors = cloud_array['rgb']
            # Kiểm tra kiểu dữ liệu của rgb
            if colors.dtype == np.uint32:
                colors = np.array([self.rgb_to_float(c) for c in colors], dtype=np.float32)
            elif colors.dtype == np.float32:
                # Giả sử rgb là float32, đã được gộp thành một giá trị
                colors = np.array([self.rgb_float_to_array(c) for c in colors], dtype=np.float32)
            else:
                rospy.logwarn("Kiểu dữ liệu RGB không hỗ trợ, bỏ qua màu.")
                self.has_rgb = False
                colors = None
        else:
            colors = None
        rospy.loginfo(f"Nhận được point cloud với {points.shape[0]} điểm")

        # Kiểm tra phạm vi tọa độ
        if points.shape[0] > 0:
            min_coords = np.min(points, axis=0)
            max_coords = np.max(points, axis=0)
            rospy.loginfo(f"Phạm vi tọa độ: min = {min_coords}, max = {max_coords}")

        # Chuyển thành Open3D PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        if self.has_rgb and colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors)

        # Voxelization
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=self.voxel_size)
        try:
            voxels = voxel_grid.get_voxels()  # Sử dụng .get_voxels()
            rospy.loginfo(f"Đã tạo voxel grid với {len(voxels)} voxels")
        except AttributeError:
            try:
                voxels = voxel_grid.voxels  # Fallback về .voxels
                rospy.loginfo(f"Đã tạo voxel grid với {len(voxels)} voxels (dùng .voxels thay vì .get_voxels())")
            except AttributeError as e:
                rospy.logerr(f"Lỗi: {e}. Không thể truy xuất voxel grid.")
                return

        # Cập nhật global voxel dictionary
        for point_idx in range(len(pcd.points)):
            point = np.asarray(pcd.points)[point_idx]
            voxel_idx = tuple(voxel_grid.get_voxel(point))
            if voxel_idx not in self.global_voxel_dict:
                self.global_voxel_dict[voxel_idx] = {'points': [], 'colors': []}
            self.global_voxel_dict[voxel_idx]['points'].append(point)
            if self.has_rgb and colors is not None:
                color = np.asarray(pcd.colors)[point_idx]
                self.global_voxel_dict[voxel_idx]['colors'].append(color)

        # Lọc và lấy điểm trung bình từ global dictionary
        filtered_points = []
        filtered_colors = []
        for voxel_idx, data in self.global_voxel_dict.items():
            points = data['points']
            if len(points) >= self.min_points_per_voxel:
                mean_point = np.mean(points, axis=0)
                filtered_points.append(mean_point)
                if self.has_rgb and len(data['colors']) > 0:
                    mean_color = np.mean(data['colors'], axis=0)
                    filtered_colors.append(mean_color)

        filtered_points = np.array(filtered_points)
        rospy.loginfo(f"Point cloud sau khi loại bỏ overlap: {len(filtered_points)} điểm")

        # In điểm mẫu sau lọc
        if len(filtered_points) > 0:
            rospy.loginfo(f"Điểm mẫu sau lọc: {filtered_points[:5]}")
            if self.has_rgb and len(filtered_colors) > 0:
                rospy.loginfo(f"Màu mẫu sau lọc: {filtered_colors[:5]}")
        else:
            rospy.loginfo("Không có điểm nào sau khi lọc (có thể voxel_size quá lớn hoặc dữ liệu nhiễu).")
            return

        # Publish point cloud đã lọc
        if len(filtered_points) > 0:
            if self.has_rgb and len(filtered_colors) > 0:
                structured_array = np.array(
                    [(p[0], p[1], p[2], self.array_to_rgb(c)) for p, c in zip(filtered_points, filtered_colors)],
                    dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)]
                )
            else:
                structured_array = np.array(
                    [(p[0], p[1], p[2]) for p in filtered_points],
                    dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)]
                )
            cloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(
                structured_array,
                stamp=msg.header.stamp,
                frame_id=msg.header.frame_id
            )
            self.pub.publish(cloud_msg)
            rospy.loginfo("Đã publish point cloud đã lọc tới /voxel_cloud_filtered")

    def rgb_to_float(self, rgb):
        # Chuyển RGB từ uint32 sang [r, g, b] trong [0, 1]
        r = ((rgb >> 16) & 255) / 255.0
        g = ((rgb >> 8) & 255) / 255.0
        b = (rgb & 255) / 255.0
        return np.array([r, g, b], dtype=np.float32)

    def rgb_float_to_array(self, rgb_float):
        # Giả sử rgb_float là float32, chuyển sang [r, g, b] trong [0, 1]
        rgb_int = int(rgb_float)
        r = ((rgb_int >> 16) & 255) / 255.0
        g = ((rgb_int >> 8) & 255) / 255.0
        b = (rgb_int & 255) / 255.0
        return np.array([r, g, b], dtype=np.float32)

    def array_to_rgb(self, color):
        # Chuyển [r, g, b] trong [0, 1] sang uint32 RGB
        r = int(color[0] * 255) << 16
        g = int(color[1] * 255) << 8
        b = int(color[2] * 255)
        return np.uint32(r | g | b)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        processor = VoxelProcessor(voxel_size=0.5, min_points_per_voxel=2)
        processor.run()
    except rospy.ROSInterruptException:
        pass