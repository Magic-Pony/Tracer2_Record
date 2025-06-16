

import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices, quaternion_slerp
import tf
import collections
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rospkg

# 使用 rospkg 获取包路径
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('remote_record')
base_path = os.path.join(pkg_path, "recorded_episode/room303")

# 时间戳阈值，单位：秒，设置为 1 毫秒
TIMESTAMP_THRESHOLD = 0.001
# 存储最近的里程计消息，最多保存 100 条
odom_buffer = collections.deque(maxlen=100)

# 创建文件夹
def create_folders():
    if not os.path.exists(f"{base_path}/rgb"):
        os.makedirs(f"{base_path}/rgb")
    if not os.path.exists(f"{base_path}/depth"):
        os.makedirs(f"{base_path}/depth")
    
    traj_file_path = f"{base_path}/traj.txt"
    if not os.path.exists(traj_file_path):
        with open(traj_file_path, "w") as traj_file:
            traj_file.write("")

# 保存 RGB 图像
def save_rgb_image(bridge, rgb_msg):
    rgb_image = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
    timestamp = "%.6f" % rgb_msg.header.stamp.to_sec()
    filename = f"{base_path}/rgb/{timestamp}.jpg"
    cv2.imwrite(filename, rgb_image)

# 保存 Depth 图像
def save_depth_image(bridge, depth_msg):
    depth_image = bridge.imgmsg_to_cv2(depth_msg, "16UC1")
    timestamp = "%.6f" % depth_msg.header.stamp.to_sec()
    filename = f"{base_path}/depth/{timestamp}.png"
    cv2.imwrite(filename, depth_image)

# 保存 RGB 和 Depth 图像
def save_results(bridge, rgb_msg, depth_msg):
    rgb_image = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
    depth_image = bridge.imgmsg_to_cv2(depth_msg, "16UC1")
    timestamp = "%.6f" % rgb_msg.header.stamp.to_sec()
    results_dir = os.path.join(base_path, "results")
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    rgb_filename = os.path.join(results_dir, f"frame{timestamp}.jpg")
    depth_filename = os.path.join(results_dir, f"depth{timestamp}.png")
    cv2.imwrite(rgb_filename, rgb_image)
    cv2.imwrite(depth_filename, depth_image)

# 保存相机 Odom 数据
def save_odom(transformed_pose, traj_file_path):
    se3_matrix = transformed_pose
    # 将 SE3 矩阵保存为一行
    se3_flat = se3_matrix.flatten()
    with open(traj_file_path, "a") as traj_file:
        traj_file.write(" ".join(map(str, se3_flat)) + "\n")

# 对里程计数据进行插值
def interpolate_odom(image_timestamp, odom_buffer):
    if len(odom_buffer) < 2:
        return None

    # 将时间戳转换为纳秒进行比较
    image_timestamp_ns = image_timestamp.secs * 1e9 + image_timestamp.nsecs
    last_odom_timestamp_ns = odom_buffer[-1].header.stamp.secs * 1e9 + odom_buffer[-1].header.stamp.nsecs
    first_odom_timestamp_ns = odom_buffer[0].header.stamp.secs * 1e9 + odom_buffer[0].header.stamp.nsecs

    if last_odom_timestamp_ns < image_timestamp_ns:
        # 所有里程计时间戳都在图像时间戳前面，使用最后两个元素插值
        i = len(odom_buffer) - 2
        j = len(odom_buffer) - 1
    elif first_odom_timestamp_ns > image_timestamp_ns:
        # 所有里程计时间戳都在图像时间戳后面，使用前两个元素插值
        i = 0
        j = 1
    else:
        # 正常情况，找到合适的插值区间
        for i in range(len(odom_buffer) - 1):
            t1_ns = odom_buffer[i].header.stamp.secs * 1e9 + odom_buffer[i].header.stamp.nsecs
            t2_ns = odom_buffer[i + 1].header.stamp.secs * 1e9 + odom_buffer[i + 1].header.stamp.nsecs
            if t1_ns <= image_timestamp_ns <= t2_ns:
                j = i + 1
                break
        else:
            # 未找到合适区间，使用最近的两个时间戳
            odom_timestamps_ns = [odom.header.stamp.secs * 1e9 + odom.header.stamp.nsecs for odom in odom_buffer]
            closest_indices = sorted(range(len(odom_timestamps_ns)), key=lambda idx: abs(odom_timestamps_ns[idx] - image_timestamp_ns))[:2]
            closest_indices.sort()
            i = closest_indices[0]
            j = closest_indices[1]

    t1_ns = odom_buffer[i].header.stamp.secs * 1e9 + odom_buffer[i].header.stamp.nsecs
    t2_ns = odom_buffer[j].header.stamp.secs * 1e9 + odom_buffer[j].header.stamp.nsecs

    # 计算插值比例
    alpha = (image_timestamp_ns - t1_ns) / (t2_ns - t1_ns)

    # 处理除零错误
    if t2_ns == t1_ns:
        alpha = 0

    # 插值位置
    pos1 = odom_buffer[i].pose.pose.position
    pos2 = odom_buffer[j].pose.pose.position
    interpolated_pos = [
        pos1.x + alpha * (pos2.x - pos1.x),
        pos1.y + alpha * (pos2.y - pos1.y),
        pos1.z + alpha * (pos2.z - pos1.z)
    ]

    # 插值四元数
    quat1 = [
        odom_buffer[i].pose.pose.orientation.x,
        odom_buffer[i].pose.pose.orientation.y,
        odom_buffer[i].pose.pose.orientation.z,
        odom_buffer[i].pose.pose.orientation.w
    ]
    quat2 = [
        odom_buffer[j].pose.pose.orientation.x,
        odom_buffer[j].pose.pose.orientation.y,
        odom_buffer[j].pose.pose.orientation.z,
        odom_buffer[j].pose.pose.orientation.w
    ]
    interpolated_quat = quaternion_slerp(quat1, quat2, alpha)

    return interpolated_pos, interpolated_quat

# 图像回调函数
def image_callback(rgb_msg, depth_msg, bridge, traj_file_path, base_to_camera_transform):
    rospy.loginfo("Image callback triggered!")

    if len(odom_buffer) == 0:
        rospy.logwarn("No odom data available, skipping.")
        return
    
    image_timestamp = rgb_msg.header.stamp
    # 保存图像
    save_results(bridge, rgb_msg, depth_msg)

    # 找到最近的里程计时间戳
    closest_odom = min(odom_buffer, key=lambda odom: abs(odom.header.stamp.to_sec() - image_timestamp.to_sec()))
    odom_timestamp = closest_odom.header.stamp
    time_diff = abs(image_timestamp.to_sec() - odom_timestamp.to_sec())

    if time_diff <= TIMESTAMP_THRESHOLD:
        # 时间戳在阈值范围内，直接保存里程计信息
        pose = closest_odom.pose.pose
        position = [pose.position.x, pose.position.y, pose.position.z]
        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        odom_matrix = concatenate_matrices(translation_matrix(position), quaternion_matrix(orientation))
    else:
        # 时间戳不在阈值内，进行插值
        interpolated_data = interpolate_odom(image_timestamp, odom_buffer)
        if interpolated_data:
            interpolated_pos, interpolated_quat = interpolated_data
            odom_matrix = concatenate_matrices(
                translation_matrix(interpolated_pos),
                quaternion_matrix(interpolated_quat)
            )
        else:
            rospy.logwarn("Failed to interpolate odom data, skipping.")
            return

    # 进行坐标变换
    camera_odom_matrix = concatenate_matrices(odom_matrix, base_to_camera_transform)
    save_odom(camera_odom_matrix, traj_file_path)

# 里程计回调函数
def odom_callback(odom_msg):
    # 将最新的里程计消息添加到缓冲区
    odom_buffer.append(odom_msg)

def main():
    rospy.init_node("data_recorder", anonymous=True)

    # 创建文件夹
    create_folders()

    traj_file_path = f"{base_path}/traj.txt"
    if not os.path.exists(traj_file_path):
        with open(traj_file_path, "w") as traj_file:
            traj_file.write("")

    # 创建 CvBridge
    bridge = CvBridge()

    listener = tf.TransformListener()
    listener.waitForTransform('/base_link', '/camera_color_optical_frame', rospy.Time(0), rospy.Duration(5.0))
    (trans, rot) = listener.lookupTransform('/base_link', '/camera_color_optical_frame', rospy.Time(0))
    base_to_camera_transform = concatenate_matrices(translation_matrix(trans), quaternion_matrix(rot))

    # 创建订阅者
    rgb_sub = Subscriber("/camera/color/image_raw", Image)
    depth_sub = Subscriber("/camera/depth/image_raw", Image)
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback)

    ats = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=20, slop=0.001, allow_headerless=False)
    ats.registerCallback(image_callback, bridge, traj_file_path, base_to_camera_transform)

    rospy.loginfo("Data recorder started, synchronizing and saving data...")
    rospy.spin()

if __name__ == "__main__":
    main()