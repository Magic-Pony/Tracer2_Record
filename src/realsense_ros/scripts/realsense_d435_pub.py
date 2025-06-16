

import rospy
import pyrealsense2 as rs
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import open3d as o3d
import json
import rospkg

def save_intrinsics(color_intrin, depth_intrin, png_depth_scale):
    # 创建 data 目录
    data_dir = os.path.join(os.getcwd(), "data")
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)
    
    # 内参文件路径
    intrin_file_path = os.path.join(data_dir, "intrinsics.txt")
    
    # 构建内参矩阵
    color_intrin_matrix = np.array([
        [color_intrin.fx, 0, color_intrin.ppx],
        [0, color_intrin.fy, color_intrin.ppy],
        [0, 0, 1]
    ])
    depth_intrin_matrix = np.array([
        [depth_intrin.fx, 0, depth_intrin.ppx],
        [0, depth_intrin.fy, depth_intrin.ppy],
        [0, 0, 1]
    ])
    
    # 写入内参信息
    with open(intrin_file_path, "w") as f:
        f.write("RGB Camera Intrinsics:\n")
        f.write("Matrix:\n")
        f.write(str(color_intrin_matrix) + "\n")
        f.write("Values:\n")
        f.write(f"fx: {color_intrin.fx}, fy: {color_intrin.fy}, ppx: {color_intrin.ppx}, ppy: {color_intrin.ppy}\n\n")
        
        f.write("Depth Camera Intrinsics:\n")
        f.write("Matrix:\n")
        f.write(str(depth_intrin_matrix) + "\n")
        f.write("Values:\n")
        f.write(f"fx: {depth_intrin.fx}, fy: {depth_intrin.fy}, ppx: {depth_intrin.ppx}, ppy: {depth_intrin.ppy}\n\n")
        
        f.write("Aligned Depth Camera Intrinsics (Same as RGB):\n")
        f.write("Matrix:\n")
        f.write(str(color_intrin_matrix) + "\n")
        f.write("Values:\n")
        f.write(f"fx: {color_intrin.fx}, fy: {color_intrin.fy}, ppx: {color_intrin.ppx}, ppy: {color_intrin.ppy}\n\n")
        
        f.write("PNG Depth Scale:\n")
        f.write(f"{png_depth_scale}\n")

def main():
    # ROS 配置
    # 初始化ROS节点
    rospy.init_node('realsense_node', anonymous=True)
    # 创建图像发布者
    rgb_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=10)
    # 创建CvBridge对象，用于OpenCV图像与ROS图像消息的转换
    bridge = CvBridge()
    # 定义 frame_id
    rgb_frame_id = "camera_color_optical_frame"
    depth_frame_id = "camera_depth_optical_frame"

    # RealSense管道 配置
    # 配置RealSense管道
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 15)
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 15)
    profile = pipeline.start(config)
    # realsense高级模式配置
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('realsense_ros')
    json_file_path = os.path.join(pkg_path, "config/Costume_D435.json")
    with open(json_file_path, "r") as f:
        json_data = f.read()
    dev = pipeline.get_active_profile().get_device()
    adv_mode = rs.rs400_advanced_mode(dev)
    adv_mode.load_json(json_data)

    # 对齐深度到彩色图像
    align_to = rs.stream.color
    align = rs.align(align_to)

    # 高质量深度图 配置
    # 深度图空间滤波器
    spatial_filter = rs.spatial_filter()
    spatial_filter.set_option(rs.option.filter_magnitude, 2)
    spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.5)
    spatial_filter.set_option(rs.option.filter_smooth_delta, 20)
    spatial_filter.set_option(rs.option.holes_fill, 1)
    # 创建深度到视差和视差到深度的转换器
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    ## 可视化设置
    # 创建RealSense颜色化器
    colorizer = rs.colorizer()
    # 创建Open3D可视化器
    # vis = o3d.visualization.Visualizer()
    # vis.create_window("Point Cloud", width=848, height=480)
    # pcd = o3d.geometry.PointCloud()
    # first_pcd = True

    ## 内参保存设置
    # 获取并保存相机内参
    depth_sensor = profile.get_device().first_depth_sensor()
    png_depth_scale = depth_sensor.get_depth_scale()
    color_intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    depth_intrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    save_intrinsics(color_intrin, depth_intrin, png_depth_scale)

    try:
        while not rospy.is_shutdown():
            # 等待一帧数据
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            # 依次应用空间滤波器、时间滤波器和洞填充滤波器
            depth_frame = aligned_frames.get_depth_frame()
            depth_frame = depth_to_disparity.process(depth_frame)
            depth_frame = spatial_filter.process(depth_frame)
            depth_frame = disparity_to_depth.process(depth_frame)

            depth_frame = align.process(frames).get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # 转换为numpy数组
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # 发布ROS图像消息
            rgb_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
            current_time = rospy.Time.now()
            rgb_msg.header.stamp = current_time
            depth_msg.header.stamp = current_time
            rgb_msg.header.frame_id = rgb_frame_id
            depth_msg.header.frame_id = depth_frame_id
            rgb_pub.publish(rgb_msg)
            depth_pub.publish(depth_msg)

            # 可选：显示图像（用于调试）
            colorizer_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            cv2.imshow('RGB Image', color_image)
    #         cv2.imshow('Depth Image', depth_image)
    #         cv2.imshow('colorizer depth',colorizer_depth)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    #         # 转换为Open3D的图像格式
    #         color_o3d = o3d.geometry.Image(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
    #         depth_o3d = o3d.geometry.Image(depth_image)

    #         # 创建RGBD图像
    #         rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    #             color_o3d, depth_o3d, convert_rgb_to_intensity=False)

    #         # 创建Open3D的内参对象
    #         o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(
    #             color_intrin.width, color_intrin.height, color_intrin.fx, color_intrin.fy, color_intrin.ppx, color_intrin.ppy)

    #         # 创建点云
    #         new_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    #             rgbd_image, o3d_intrinsics)

    #         # 转换坐标系以匹配RealSense的坐标系
    #         new_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    #         if first_pcd:
    #             pcd = new_pcd
    #             vis.add_geometry(pcd)
    #             first_pcd = False
    #         else:
    #             pcd.points = new_pcd.points
    #             pcd.colors = new_pcd.colors
    #             vis.update_geometry(pcd)

    #         vis.poll_events()
    #         vis.update_renderer()

    except rospy.ROSInterruptException:
        pass
    finally:
        pipeline.stop() # 停止管道
        cv2.destroyAllWindows() # 关闭OpenCV窗口
    #     vis.destroy_window()  # 关闭Open3D可视化窗口

if __name__ == '__main__':
    main()