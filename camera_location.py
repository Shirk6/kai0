# -- coding: UTF-8
import os
import time
import numpy as np
import h5py
import argparse
import dm_env

import collections
from collections import deque

import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import sys
import cv2
import tty
import termios, select

import asyncio

# 保存数据函数
def save_data(args, timesteps, actions, dataset_path):
    # 数据字典
    data_size = len(actions)
    data_dict = {
        # 一个是奖励里面的qpos，qvel， effort ,一个是实际发的acition
        '/observations/qpos': [],
        '/observations/qvel': [],
        '/observations/effort': [],
        '/action': [],
        '/base_action': [],
        # '/base_action_t265': [],
    }

    # 相机字典  观察的图像
    for cam_name in args.camera_names:
        data_dict[f'/observations/images/{cam_name}'] = []
        if args.use_depth_image:
            data_dict[f'/observations/images_depth/{cam_name}'] = []

    # len(action): max_timesteps, len(time_steps): max_timesteps + 1
    # 动作长度 遍历动作
    while actions:
        # 循环弹出一个队列
        action = actions.pop(0)   # 动作  当前动作
        ts = timesteps.pop(0)     # 奖励  前一帧

        # 往字典里面添值
        # Timestep返回的qpos，qvel,effort
        data_dict['/observations/qpos'].append(ts.observation['qpos'])
        data_dict['/observations/qvel'].append(ts.observation['qvel'])
        data_dict['/observations/effort'].append(ts.observation['effort'])

        # 实际发的action
        data_dict['/action'].append(action)
        data_dict['/base_action'].append(ts.observation['base_vel'])

        # 相机数据
        # data_dict['/base_action_t265'].append(ts.observation['base_vel_t265'])
        for cam_name in args.camera_names:
            data_dict[f'/observations/images/{cam_name}'].append(ts.observation['images'][cam_name])
            if args.use_depth_image:
                data_dict[f'/observations/images_depth/{cam_name}'].append(ts.observation['images_depth'][cam_name])

    t0 = time.time()
    with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
        # 文本的属性：
        # 1 是否仿真
        # 2 图像是否压缩
        #
        root.attrs['sim'] = False
        root.attrs['compress'] = False

        # 创建一个新的组observations，观测状态组
        # 图像组
        obs = root.create_group('observations')
        image = obs.create_group('images')
        for cam_name in args.camera_names:

            _ = image.create_dataset(cam_name, (data_size, 480, 640, 3), dtype='uint8',
                                         chunks=(1, 480, 640, 3), )
        if args.use_depth_image:
            image_depth = obs.create_group('images_depth')
            for cam_name in args.camera_names:
                _ = image_depth.create_dataset(cam_name, (data_size, 480, 640), dtype='uint16',
                                             chunks=(1, 480, 640), )

        _ = obs.create_dataset('qpos', (data_size, 14))
        _ = obs.create_dataset('qvel', (data_size, 14))
        _ = obs.create_dataset('effort', (data_size, 14))
        _ = root.create_dataset('action', (data_size, 14))
        _ = root.create_dataset('base_action', (data_size, 2))

        # data_dict write into h5py.File
        for name, array in data_dict.items():  
            root[name][...] = array
    print(f'\033[32m\nSaving: {time.time() - t0:.1f} secs. %s \033[0m\n'%dataset_path)


class RosOperator:
    def __init__(self, args):
        self.robot_base_deque = None
        self.puppet_arm_right_deque = None
        self.puppet_arm_left_deque = None
        self.master_arm_right_deque = None
        self.master_arm_left_deque = None
        self.img_front_deque = None
        self.img_right_deque = None
        self.img_left_deque = None
        self.img_front_depth_deque = None
        self.img_right_depth_deque = None
        self.img_left_depth_deque = None
        self.bridge = None
        self.args = args
        self.init()
        self.init_ros()

    def init(self):
        self.bridge = CvBridge()
        self.img_left_deque = deque()
        self.img_right_deque = deque()
        self.img_front_deque = deque()
        self.img_left_depth_deque = deque()
        self.img_right_depth_deque = deque()
        self.img_front_depth_deque = deque()
        self.master_arm_left_deque = deque()
        self.master_arm_right_deque = deque()
        self.puppet_arm_left_deque = deque()
        self.puppet_arm_right_deque = deque()
        self.robot_base_deque = deque()

    def get_frame(self):
        if len(self.img_front_deque) == 0:
            return False
        frame_time =  self.img_front_deque[-1].header.stamp.to_sec()
        
        while self.img_front_deque[0].header.stamp.to_sec() < frame_time:
            self.img_front_deque.popleft()
        img_front = self.bridge.imgmsg_to_cv2(self.img_front_deque.popleft(), 'passthrough')

        return (img_front)


    def img_front_callback(self, msg):
        if len(self.img_front_deque) >= 2000:
            self.img_front_deque.popleft()
        self.img_front_deque.append(msg)

    def img_left_depth_callback(self, msg):
        if len(self.img_left_depth_deque) >= 2000:
            self.img_left_depth_deque.popleft()
        self.img_left_depth_deque.append(msg)

    def img_right_depth_callback(self, msg):
        if len(self.img_right_depth_deque) >= 2000:
            self.img_right_depth_deque.popleft()
        self.img_right_depth_deque.append(msg)

    def img_front_depth_callback(self, msg):
        if len(self.img_front_depth_deque) >= 2000:
            self.img_front_depth_deque.popleft()
        self.img_front_depth_deque.append(msg)

    def master_arm_left_callback(self, msg):
        if len(self.master_arm_left_deque) >= 2000:
            self.master_arm_left_deque.popleft()
        self.master_arm_left_deque.append(msg)

    def master_arm_right_callback(self, msg):
        if len(self.master_arm_right_deque) >= 2000:
            self.master_arm_right_deque.popleft()
        self.master_arm_right_deque.append(msg)

    def puppet_arm_left_callback(self, msg):
        if len(self.puppet_arm_left_deque) >= 2000:
            self.puppet_arm_left_deque.popleft()
        self.puppet_arm_left_deque.append(msg)

    def puppet_arm_right_callback(self, msg):
        if len(self.puppet_arm_right_deque) >= 2000:
            self.puppet_arm_right_deque.popleft()
        self.puppet_arm_right_deque.append(msg)

    def robot_base_callback(self, msg):
        if len(self.robot_base_deque) >= 2000:
            self.robot_base_deque.popleft()
        self.robot_base_deque.append(msg)

    def init_ros(self):
        rospy.init_node('record_episodes', anonymous=True)
        rospy.Subscriber(self.args.img_front_topic, Image, self.img_front_callback, queue_size=1000, tcp_nodelay=True)


    def process(self):
        timesteps = []
        actions = []
        # 图像数据
        image = np.random.randint(0, 255, size=(480, 640, 3), dtype=np.uint8)
        image_dict = dict()
        for cam_name in self.args.camera_names:
            image_dict[cam_name] = image
        count = 0
        
        # input_key = input("please input s:")
        # while input_key != 's' and not rospy.is_shutdown():
        #     input_key = input("please input s:")

        rate = rospy.Rate(self.args.frame_rate)
        print_flag = True
        
        print("按空格结束采集")
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        while not rospy.is_shutdown():
            # 2 收集数据
            result = self.get_frame()
            if not result:
                if print_flag:
                    print("syn fail")
                    print_flag = False
                rate.sleep()
                continue
            print_flag = True
            count += 1
            (img_front, img_front_depth, img_left_depth, img_right_depth,
            puppet_arm_left, puppet_arm_right, master_arm_left, master_arm_right, robot_base) = result
            # 2.1 图像信息
            image_dict = dict()
            image_dict[self.args.camera_names[0]] = img_front

            # 2.2 从臂的信息从臂的状态 机械臂示教模式时 会自动订阅
            obs = collections.OrderedDict()  # 有序的字典
            obs['images'] = image_dict
            if self.args.use_depth_image:
                image_dict_depth = dict()
                image_dict_depth[self.args.camera_names[0]] = img_front_depth

                obs['images_depth'] = image_dict_depth
            obs['qpos'] = np.concatenate((np.array(puppet_arm_left.position), np.array(puppet_arm_right.position)), axis=0)
            obs['qvel'] = np.concatenate((np.array(puppet_arm_left.velocity), np.array(puppet_arm_right.velocity)), axis=0)
            obs['effort'] = np.concatenate((np.array(puppet_arm_left.effort), np.array(puppet_arm_right.effort)), axis=0)
            if self.args.use_robot_base:
                obs['base_vel'] = [robot_base.twist.twist.linear.x, robot_base.twist.twist.angular.z]
            else:
                obs['base_vel'] = [0.0, 0.0]

            # 第一帧 只包含first， fisrt只保存StepType.FIRST
            if count == 1:
                ts = dm_env.TimeStep(
                    step_type=dm_env.StepType.FIRST,
                    reward=None,
                    discount=None,
                    observation=obs)
                timesteps.append(ts)
                continue

            # 时间步
            ts = dm_env.TimeStep(
                step_type=dm_env.StepType.MID,
                reward=None,
                discount=None,
                observation=obs)

            # 主臂保存状态
            left_gripper = np.array(master_arm_left.position)[-1:]
            right_gripper = np.array(master_arm_right.position)[-1:]

            # left_action = np.concatenate((np.array(puppet_arm_left.position[0:-1]), left_gripper))
            # right_action = np.concatenate((np.array(puppet_arm_left.position[0:-1]), right_gripper))

            left_action = np.concatenate((np.array(puppet_arm_left.position[0:-1]), left_gripper))
            right_action = np.concatenate((np.array(puppet_arm_right.position[0:-1]), right_gripper))

            action = np.concatenate((left_action, right_action), axis=0)
            actions.append(action)
            timesteps.append(ts)
            print("Frame data: ", count)
            if rospy.is_shutdown():
                exit(-1)
            rate.sleep()
            r, w, x = select.select([sys.stdin], [], [], 0)
            if r:
                key = sys.stdin.read(1)
                if key == ' ':
                    print("采集结束")
                    break
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        termios.tcflush(fd, termios.TCIFLUSH)
        #while len(timesteps) > len(actions):
        #    timesteps.pop()
        print("len(timesteps): ", len(timesteps))
        print("len(actions)  : ", len(actions))
        #print("KeyboardInterrupt")
        return timesteps, actions


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_dir', action='store', type=str, help='Dataset_dir.',
                        default="./data", required=False)
    parser.add_argument('--task_name', action='store', type=str, help='Task name.',
                        default="test", required=False)
    parser.add_argument('--episode_idx', action='store', type=int, help='Episode index.',
                        default=9, required=False)
    
    parser.add_argument('--max_timesteps', action='store', type=int, help='Max_timesteps.',
                        default=100, required=False)

    parser.add_argument('--camera_names', action='store', type=str, help='camera_names',
                        default=['cam_high'], required=False)
    #  topic name of color image
    parser.add_argument('--img_front_topic', action='store', type=str, help='img_front_topic',
                        default='/camera_h/color/image_raw', required=False)
    parser.add_argument('--img_left_topic', action='store', type=str, help='img_left_topic',
                        default='/camera_h/color/image_raw', required=False)
    parser.add_argument('--img_right_topic', action='store', type=str, help='img_right_topic',
                        default='/camera_h/color/image_raw', required=False)
    
    # topic name of depth image
    parser.add_argument('--img_front_depth_topic', action='store', type=str, help='img_front_depth_topic',
                        default='/camera_f/depth/image_raw', required=False)
    parser.add_argument('--img_left_depth_topic', action='store', type=str, help='img_left_depth_topic',
                        default='/camera_l/depth/image_raw', required=False)
    parser.add_argument('--img_right_depth_topic', action='store', type=str, help='img_right_depth_topic',
                        default='/camera_r/depth/image_raw', required=False)
    
    # topic name of arm
    parser.add_argument('--master_arm_left_topic', action='store', type=str, help='master_arm_left_topic',
                        default='/master/joint_left', required=False)
    parser.add_argument('--master_arm_right_topic', action='store', type=str, help='master_arm_right_topic',
                        default='/master/joint_right', required=False)
    parser.add_argument('--puppet_arm_left_topic', action='store', type=str, help='puppet_arm_left_topic',
                        default='/puppet/joint_left', required=False)
    parser.add_argument('--puppet_arm_right_topic', action='store', type=str, help='puppet_arm_right_topic',
                        default='/puppet/joint_right', required=False)
    
    # topic name of robot_base
    parser.add_argument('--robot_base_topic', action='store', type=str, help='robot_base_topic',
                        default='/odom', required=False)
    
    parser.add_argument('--use_robot_base', action='store', type=bool, help='use_robot_base',
                        default=False, required=False)
    # collect depth image
    parser.add_argument('--use_depth_image', action='store', type=bool, help='use_depth_image',
                        default=False, required=False)
    
    parser.add_argument('--frame_rate', action='store', type=int, help='frame_rate',
                        default=30, required=False)
    args = parser.parse_args()
    return args


def justify_data(path, ros_operator):
    """
    验证数据的函数，打开并显示first_person_photo.jpg图片
    
    Args:
        path (str): 数据路径
    """

    
    # 构建图片路径
    # third_person_image_path = os.path.join(path, "folding_cloth_1_14.jpg")
    # third_person_image_path = os.path.join(path, "folding_1_8.jpg")
    # third_person_image_path = os.path.join(path, "depositing_1_7.jpg")
    # third_person_image_path = os.path.join(path, "sweeping_1_20.jpg")
    # third_person_image_path = os.path.join(path, "stacking_1_7.jpg")
    # third_person_image_path = os.path.join(path, "passing_1_7.jpg")
    # third_person_image_path = os.path.join(path, "folding_T_1_14.jpg")
    # third_person_image_path = os.path.join(path, "folding_T_1_14.jpg")
    # third_person_image_path = os.path.join(path, "folding_2_11.jpg")
    # third_person_image_path = os.path.join(path, "quanxin_task_3_13.jpg")
    # third_person_image_path = os.path.join(path, "test_math.jpg")
    #third_person_image_path = os.path.join(path, "table_bussing_quanxin_3_16.jpg")
    # third_person_image_path = os.path.join(path, "math_reasoning_55.jpg")
    #third_person_image_path = os.path.join(path, "math_reasoning_429.jpg")
    # third_person_image_path = os.path.join(path, "pack_3_objects.jpg")
    # third_person_image_path = os.path.join(path, "table_sweeping.jpg")
    #third_person_image_path = os.path.join(path, "table_cleaning.jpg")
    # third_person_image_path = os.path.join(path, "pack_3_objects_plus.jpg")
    third_person_image_path = os.path.join(path, "battery_assemble.jpg")
    # third_person_image_path = os.path.join(path, "stack_3_cups_gen.jpg")
    # third_person_image_path = os.path.join(path, "stack_3_cups_gen.jpg")
    # third_person_image_path = os.path.join(path, "pick_1_cup.jpg")

    
    # 检查文件是否存在
    if not os.path.exists(third_person_image_path):
        print(f"图片文件不存在: {third_person_image_path}")
        return
    
    # 使用cv2读取图片
    image = cv2.imread(third_person_image_path)
    sign = False
    while not rospy.is_shutdown() and sign == False:
        result = ros_operator.get_frame()
        if not isinstance(result, np.ndarray):
            print("syn fail")
            time.sleep(0.1)
            continue
        (img_front) = result
        image_np = np.array(image)
        image_get_from_topic = np.array(img_front)
        diff = cv2.absdiff(image_np, image_get_from_topic)
        diff = np.clip(diff, 0, 255).astype(np.uint8)
        
        diff_mean = np.mean(diff)
        print(f"MAE: {diff_mean}")
        time.sleep(0.1)

        cv2.imshow("diff", diff)
        key = cv2.waitKey(1) & 0xFF

    print("Reset Success")
    #cv2.destroyWindow('diff')





def main():
    
    args = get_arguments()
    ros_operator = RosOperator(args)
    episode_idx = args.episode_idx
    
    justify_data("/home/agilex/World_Action_Model/cobot_magic/collect_data/game", ros_operator)
    #if(len(actions) < args.max_timesteps):
    #    print("\033[31m\nSave failure, please record %s timesteps of data.\033[0m\n" %args.max_timesteps)
    #    exit(-1)

    


if __name__ == '__main__':
    main()

# python collect_data.py --dataset_dir ~/data --max_timesteps 500 --episode_idx 0
