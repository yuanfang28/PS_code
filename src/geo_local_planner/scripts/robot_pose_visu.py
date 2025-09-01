#!/usr/bin/env python3

import os
import rosbag
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
from matplotlib.patches import Circle

# 定义读取bag文件并绘制图形的函数
def read_bag_and_plot(bag_file):
    # 打开 .bag 文件
    bag = rosbag.Bag(bag_file)

    # 存储 x 和 y 的列表
    robot_x_vals = []
    robot_y_vals = []
    plan_x_vals = []
    plan_y_vals = []
    circle_centers = []  
    circle_radii = []  
    # 读取指定话题的数据
    for topic, msg, t in bag.read_messages(topics=['/move_base/GeoPlannerROS/robot_pose',
                                                     '/move_base/GlobalPlanner/plan',
                                                     '/move_base/GeoPlannerROS/obs_center', 
                                                    '/move_base/GeoPlannerROS/obs_radius']):
        if topic == '/move_base/GeoPlannerROS/robot_pose':
            # 提取 robot_pose 信息
            robot_x_vals.append(msg.x)
            robot_y_vals.append(msg.y)
        elif topic == '/move_base/GlobalPlanner/plan':
            # 提取 GlobalPlanner/plan 信息
            for pose in msg.poses:
                plan_x_vals.append(pose.pose.position.x)
                plan_y_vals.append(pose.pose.position.y)
        elif topic == '/move_base/GeoPlannerROS/obs_center':  # 读取圆心信息
            for pose in msg.poses:
                center = pose.position
                #print(f"Circle center found: ({center.x}, {center.y})")
                circle_centers.append((center.x, center.y))
        elif topic == '/move_base/GeoPlannerROS/obs_radius':  # 读取半径信息
            for data in msg.data:
                circle_radii.append(data)


    # 关闭 .bag 文件
    bag.close()

    # 使用 matplotlib 绘制图形
    plt.figure()
    # 绘制 Robot Path
    plt.plot(robot_x_vals, robot_y_vals, label="Robot Path", color="blue")

    # 绘制 Global Planner Path
    plt.plot(plan_x_vals, plan_y_vals, label="Global Plan", color="red")

    # 绘制障碍物的圆
    for i in range(len(circle_centers)):
        center_x, center_y = circle_centers[i]
        radius = circle_radii[i] if i < len(circle_radii) else 0

        #for radius in radii:
        circle = Circle((center_x, center_y), radius, color='green', alpha=0.5)
        plt.gca().add_patch(circle)  # 将每个圆添加到图形中

    # 添加 y=3 和 y=-3 的边界线
    plt.axhline(y=3, color='g', linestyle='--', label='y = 3')
    plt.axhline(y=-3, color='b', linestyle='--', label='y = -3')

    # 图形设置
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Robot Position and Global Plan with Obstacles")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")

    # 显示图形
    plt.show()


# 初始化 ROS 节点
import rospy
rospy.init_node('bag_reader', anonymous=True)

# 使用 os.path.expanduser 展开路径
bag_file = os.path.expanduser('~/ros_ws/bagfiles/2025-01-22-11-36-01.bag')

# 调用读取并绘图函数
read_bag_and_plot(bag_file)