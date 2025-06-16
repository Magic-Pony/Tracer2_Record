
import rosbag
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import rospkg
import os

# 定义颜色列表，用于不同 odom 轨迹显示不同颜色
COLORS = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

def extract_odom_data(bag_path, topic):
    """
    从指定的 rosbag 包和话题中提取里程计数据

    :param bag_path: rosbag 包的路径
    :param topic: 里程计话题名称
    :return: x 坐标列表，y 坐标列表
    """
    x_list = []
    y_list = []
    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            for topic_name, msg, t in bag.read_messages(topics=[topic]):
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                x_list.append(x)
                y_list.append(y)
    except Exception as e:
        print(f"Error reading bag file {bag_path}: {e}")
    return x_list, y_list

def visualize_odom_bags(bag_paths, topics):
    """
    可视化多个 rosbag 包中指定话题的里程计数据，不同话题用不同颜色表示，标记起点和终点

    :param bag_paths: rosbag 包路径列表
    :param topics: 里程计话题名称列表
    """
    plt.figure(figsize=(10, 8))
    color_index = 0
    for i, bag_path in enumerate(bag_paths):
        for topic in topics:
            x, y = extract_odom_data(bag_path, topic)
            if x and y:
                color = COLORS[color_index % len(COLORS)]
                # 绘制轨迹
                plt.plot(x, y, color=color, label=f'{bag_path} - {topic}')
                # 标记起点
                plt.scatter(x[0], y[0], color=color, marker='o', s=100, label=f'{topic} Start')
                # 标记终点
                plt.scatter(x[-1], y[-1], color=color, marker='x', s=100, label=f'{topic} End')
                color_index += 1

    plt.title('Odometry Comparison')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

if __name__ == '__main__':
    # 使用 rospkg 获取当前包所在的工作空间路径
    rospack = rospkg.RosPack()
    ws_path = os.path.dirname(rospack.get_path('odom_test'))
    # 只保留文件名，自动拼接路径
    bag_filenames = ['odom_wheel_2025-06-11-21-16-28.bag']
    bag_paths = [os.path.join(ws_path, filename) for filename in bag_filenames]

    # 请替换为实际的里程计话题列表
    topics = ['/odom']
    visualize_odom_bags(bag_paths, topics)