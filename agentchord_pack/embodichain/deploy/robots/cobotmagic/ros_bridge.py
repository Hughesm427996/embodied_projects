# import logging

# logger = logging.getLogger(__name__)

try:
    import rospy
except ImportError:
    rospy = None
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time as RclpyTime
except ImportError:
    rclpy = None

if rospy is None:
    print("Failed to import rospy; ROS 1 features unavailable.")
if rclpy is None:
    print("Failed to import rclpy; ROS 2 features unavailable.")
import threading
import numpy as np
import json
import os
import time

from collections import deque
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from piper_msgs.msg import PosCmd

BUFFERSIZE = 2000

ROS_BRIDGE_PARAMS = None
with open(os.path.join(os.path.dirname(__file__), "bridge_params.json"), "r") as f:
    ROS_BRIDGE_PARAMS = json.load(f)


    class Args:
        def __init__(self, **entries):
            self.__dict__.update(entries)


    args = Args(**ROS_BRIDGE_PARAMS)


class RosOperator:
    def __init__(self, args=args):
        self.args = args
        print(
            "Initializing ROS operator (ROS1=%s, ROS2=%s)",
            rospy is not None,
            rclpy is not None,
        )
        self.init()
        print("RosOperator init() completed, now initializing ROS interfaces...")
        self.init_ros()
        print("ROS interfaces initialized.")

    def init(self):
        self.ctrl_state = False
        self.ctrl_state_lock = threading.Lock()
        self.puppet_arm_publish_thread = None
        self.puppet_arm_publish_lock = None
        self.bridge = CvBridge()
        self.img_left_deque = deque()
        self.img_right_deque = deque()
        self.img_front_deque = deque()
        self.img_left_depth_deque = deque()
        self.img_right_depth_deque = deque()
        self.img_front_depth_deque = deque()
        self.puppet_arm_left_deque = deque()
        self.puppet_arm_right_deque = deque()
        self.robot_base_deque = deque()
        self.puppet_arm_publish_lock = threading.Lock()
        self.puppet_arm_publish_lock.acquire()
        self.puppet_arm_pos_left_deque = deque()
        self.puppet_arm_pos_right_deque = deque()
        self.joint_names = [
            "joint0",
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]

    @property
    def running(self):
        return not rospy.is_shutdown()

    def init_ros(self):
        self.create_ros_node()
        self.rate = self.create_rate(self.args.publish_rate)

        # 修复话题名 - 跳过空字符串
        def should_create_subscription(topic):
            return topic and topic.strip() and topic != "null"

        # RGB图像订阅
        if should_create_subscription(self.args.img_right_topic):
            print("Creating subscriber for %s", self.args.img_right_topic)
            self.create_subscriber(
                self.args.img_right_topic, self.img_right_callback, Image
            )
        # else:
        #     print("Warning: img_right_topic is invalid, using default")
        #     self.args.img_right_topic = "/camera/camera_r/color/image_raw"
        #     self.create_subscriber(
        #         self.args.img_right_topic, self.img_right_callback, Image
        #     )

        # 深度图像订阅（如果启用）
        if self.args.use_depth_image:
            if should_create_subscription(self.args.img_right_depth_topic):
                print("Creating subscriber for depth topic %s", self.args.img_right_depth_topic)
                self.create_subscriber(
                    self.args.img_right_depth_topic, self.img_right_depth_callback, Image
                )
            else:
                print("Warning: img_right_depth_topic is invalid, using default")
                self.args.img_right_depth_topic = "/camera/camera_r/aligned_depth_to_color/image_raw"
                self.create_subscriber(
                    self.args.img_right_depth_topic, self.img_right_depth_callback, Image
                )

        # 跳过其他空话题 - 不创建订阅
        # img_left_topic, img_front_topic 等留空

        # 机械臂话题
        print("Subscribing puppet arm left topic %s", self.args.puppet_arm_left_topic)
        self.create_subscriber(
            self.args.puppet_arm_left_topic, self.puppet_arm_left_callback, JointState
        )
        print("Subscribing puppet arm right topic %s", self.args.puppet_arm_right_topic)
        self.create_subscriber(
            self.args.puppet_arm_right_topic, self.puppet_arm_right_callback, JointState
        )

        # 创建发布器
        self.puppet_arm_left_publisher = self.create_publisher(
            self.args.puppet_arm_left_cmd_topic, JointState
        )
        self.puppet_arm_right_publisher = self.create_publisher(
            self.args.puppet_arm_right_cmd_topic, JointState
        )

    def puppet_arm_publish(self, left, right):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_current_time_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = left
        self.puppet_arm_left_publisher.publish(joint_state_msg)
        joint_state_msg.position = right
        self.puppet_arm_right_publisher.publish(joint_state_msg)
        self.rate.sleep()

    def robot_base_publish(self, vel):
        vel_msg = Twist()
        vel_msg.linear.x = vel[0]
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = vel[1]
        self.robot_base_publisher.publish(vel_msg)

    def puppet_arm_publish_continuous(self, left, right):
        rate = self.create_rate(self.args.publish_rate)
        left_arm = None
        right_arm = None
        while True and self.running:
            if len(self.puppet_arm_left_deque) != 0:
                left_arm = list(self.puppet_arm_left_deque[-1].position)
            if len(self.puppet_arm_right_deque) != 0:
                right_arm = list(self.puppet_arm_right_deque[-1].position)
            if left_arm is None or right_arm is None:
                rate.sleep()
                continue
            else:
                break
        left_symbol = [1 if left[i] - left_arm[i] > 0 else -1 for i in range(len(left))]
        right_symbol = [
            1 if right[i] - right_arm[i] > 0 else -1 for i in range(len(right))
        ]
        flag = True
        step = 0
        while flag and self.running:
            if self.puppet_arm_publish_lock.acquire(False):
                return
            left_diff = [abs(left[i] - left_arm[i]) for i in range(len(left))]
            right_diff = [abs(right[i] - right_arm[i]) for i in range(len(right))]
            flag = False
            for i in range(len(left)):
                if left_diff[i] < self.args.arm_steps_length[i]:
                    left_arm[i] = left[i]
                else:
                    left_arm[i] += left_symbol[i] * self.args.arm_steps_length[i]
                    flag = True
            for i in range(len(right)):
                if right_diff[i] < self.args.arm_steps_length[i]:
                    right_arm[i] = right[i]
                else:
                    right_arm[i] += right_symbol[i] * self.args.arm_steps_length[i]
                    flag = True
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = self.get_current_time_msg()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = left_arm
            self.puppet_arm_left_publisher.publish(joint_state_msg)
            joint_state_msg.position = right_arm
            self.puppet_arm_right_publisher.publish(joint_state_msg)
            step += 1
            print("puppet_arm_publish_continuous:", step)
            rate.sleep()

    def puppet_arm_publish_linear(self, left, right):
        num_step = 100
        rate = self.create_rate(200)

        left_arm = None
        right_arm = None

        while True and self.running:
            if len(self.puppet_arm_left_deque) != 0:
                left_arm = list(self.puppet_arm_left_deque[-1].position)
            if len(self.puppet_arm_right_deque) != 0:
                right_arm = list(self.puppet_arm_right_deque[-1].position)
            if left_arm is None or right_arm is None:
                rate.sleep()
                continue
            else:
                break

        traj_left_list = np.linspace(left_arm, left, num_step)
        traj_right_list = np.linspace(right_arm, right, num_step)

        for i in range(len(traj_left_list)):
            traj_left = traj_left_list[i]
            traj_right = traj_right_list[i]
            traj_left[-1] = left[-1]
            traj_right[-1] = right[-1]
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = self.get_current_time_msg()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = traj_left
            self.puppet_arm_left_publisher.publish(joint_state_msg)
            joint_state_msg.position = traj_right
            self.puppet_arm_right_publisher.publish(joint_state_msg)
            rate.sleep()

    def puppet_arm_publish_continuous_thread(self, left, right):
        if self.puppet_arm_publish_thread is not None:
            self.puppet_arm_publish_lock.release()
            self.puppet_arm_publish_thread.join()
            self.puppet_arm_publish_lock.acquire(False)
            self.puppet_arm_publish_thread = None
        self.puppet_arm_publish_thread = threading.Thread(
            target=self.puppet_arm_publish_continuous, args=(left, right)
        )
        self.puppet_arm_publish_thread.start()

    def get_current_frame_time(self):
        frame_time_list = []
        if self.args.img_left_topic:
            # wait until self.img_left_deque is not empty
            while len(self.img_left_deque) == 0:
                time.sleep(0.001)
            frame_time_list.append(
                self.time_converter(self.img_left_deque[-1].header.stamp)
            )
        if self.args.img_right_topic:
            while len(self.img_right_deque) == 0:
                time.sleep(0.001)
            print("await for img_right")
            frame_time_list.append(
                self.time_converter(self.img_right_deque[-1].header.stamp)
            )
        if self.args.img_front_topic:
            while len(self.img_front_deque) == 0:
                time.sleep(0.001)
            frame_time_list.append(
                self.time_converter(self.img_front_deque[-1].header.stamp)
            )

        return min(frame_time_list) if len(frame_time_list) !=0 else 0

    def get_frame(self):
        img_front, img_left, img_right = self.get_image_frame()
        puppet_arm_left, puppet_arm_right = self.get_puppet_arm_frame()
        return img_front, img_left, img_right, puppet_arm_left, puppet_arm_right

    def get_image_frame(self):
        # TODO: currently only support rgb image.
        frame_time = self.get_current_frame_time()

        if self.args.img_right_topic and (
                len(self.img_right_deque) == 0
                or self.time_converter(self.img_right_deque[-1].header.stamp) < frame_time
        ):
            return False
        if self.args.img_front_topic and (
                len(self.img_front_deque) == 0
                or self.time_converter(self.img_front_deque[-1].header.stamp) < frame_time
        ):
            return False

        img_left = None
        if self.args.img_left_topic:
            while (
                    len(self.img_left_deque) > 0
                    and self.time_converter(self.img_left_deque[0].header.stamp)
                    < frame_time
            ):
                self.img_left_deque.popleft()
            if len(self.img_left_deque) == 0:
                return False
            img_left = self.bridge.imgmsg_to_cv2(
                self.img_left_deque.popleft(), "passthrough"
            )

        while self.time_converter(self.img_right_deque[0].header.stamp) < frame_time:
            self.img_right_deque.popleft()
        img_right = self.bridge.imgmsg_to_cv2(
            self.img_right_deque.popleft(), "passthrough"
        )

        img_front = None
        if self.args.img_front_topic:
            while (
                    len(self.img_front_deque) > 0
                    and self.time_converter(self.img_front_deque[0].header.stamp)
                    < frame_time
            ):
                self.img_front_deque.popleft()
            if len(self.img_front_deque) == 0:
                return False
            img_front = self.bridge.imgmsg_to_cv2(
                self.img_front_deque.popleft(), "passthrough"
            )

        return img_front, img_left, img_right

    def get_depth_frame(self):
        """
        获取深度图像帧
        返回: (depth_front, depth_left, depth_right)
        """
        if not self.args.use_depth_image:
            print("深度图像功能未启用")
            return None, None, None

        frame_time = self.get_current_frame_time()

        # 检查深度图像是否准备好
        if self.args.img_right_depth_topic:
            if len(self.img_right_depth_deque) == 0:
                # print("等待深度图像...")
                return None, None, None
            # 检查时间戳
            latest_stamp = self.time_converter(self.img_right_depth_deque[-1].header.stamp)
            if latest_stamp < frame_time:
                # print(f"深度图像延迟: {frame_time - latest_stamp:.3f}s")
                return None, None, None

        depth_left = None
        depth_right = None
        depth_front = None

        # 获取左深度图像
        if self.args.img_left_depth_topic and len(self.img_left_depth_deque) > 0:
            # 找到时间戳匹配的图像
            for i in range(len(self.img_left_depth_deque)):
                msg = self.img_left_depth_deque[i]
                if abs(self.time_converter(msg.header.stamp) - frame_time) < 0.05:  # 50ms容差
                    depth_left = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                    # 移除已处理的
                    for _ in range(i + 1):
                        self.img_left_depth_deque.popleft()
                    break

        # 获取右深度图像
        if self.args.img_right_depth_topic and len(self.img_right_depth_deque) > 0:
            for i in range(len(self.img_right_depth_deque)):
                msg = self.img_right_depth_deque[i]
                if abs(self.time_converter(msg.header.stamp) - frame_time) < 0.05:
                    depth_right = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                    for _ in range(i + 1):
                        self.img_right_depth_deque.popleft()
                    break

        # 获取前深度图像
        if self.args.img_front_depth_topic and len(self.img_front_depth_deque) > 0:
            for i in range(len(self.img_front_depth_deque)):
                msg = self.img_front_depth_deque[i]
                if abs(self.time_converter(msg.header.stamp) - frame_time) < 0.05:
                    depth_front = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                    for _ in range(i + 1):
                        self.img_front_depth_deque.popleft()
                    break

        return depth_front, depth_left, depth_right

    def get_puppet_arm_frame(self):
        frame_time = self.get_current_frame_time()

        while (
                len(self.puppet_arm_left_deque) == 0
                or len(self.puppet_arm_right_deque) == 0
        ):
            print("Waiting for puppet arm JointState messages...")
            time.sleep(0.0001)

        while (
                self.time_converter(self.puppet_arm_left_deque[0].header.stamp) < frame_time
        ):
            self.puppet_arm_left_deque.popleft()
            while len(self.puppet_arm_left_deque) == 0:
                time.sleep(0.0002)
        puppet_arm_left = self.puppet_arm_left_deque.popleft()

        while (
                self.time_converter(self.puppet_arm_right_deque[0].header.stamp)
                < frame_time
        ):
            self.puppet_arm_right_deque.popleft()
            while len(self.puppet_arm_right_deque) == 0:
                time.sleep(0.0002)
        puppet_arm_right = self.puppet_arm_right_deque.popleft()

        return puppet_arm_left, puppet_arm_right

    def get_puppet_arm_frame_realtime(self):
        while len(self.puppet_arm_left_deque) == 0 or len(self.puppet_arm_right_deque) == 0:
            time.sleep(0.0005)

        puppet_arm_left = self.puppet_arm_left_deque[-1]
        puppet_arm_right = self.puppet_arm_right_deque[-1]
        return puppet_arm_left, puppet_arm_right

    def get_puppet_arm_pos_frame(self, name: str):
        if (
                len(self.puppet_arm_pos_left_deque) == 0
                or len(self.puppet_arm_pos_right_deque) == 0
        ):
            return None

        if name not in ["left_arm", "right_arm"]:
            print("Invalid arm name")
            return None

        if name == "left_arm":
            return self.puppet_arm_pos_left_deque[-1]
        else:
            return self.puppet_arm_pos_right_deque[-1]

    def img_left_callback(self, msg):
        if len(self.img_left_deque) >= BUFFERSIZE:
            self.img_left_deque.popleft()
        self.img_left_deque.append(msg)

    def img_right_callback(self, msg):
        if len(self.img_right_deque) >= BUFFERSIZE:
            self.img_right_deque.popleft()
        self.img_right_deque.append(msg)

    def img_front_callback(self, msg):
        if len(self.img_front_deque) >= BUFFERSIZE:
            self.img_front_deque.popleft()
        self.img_front_deque.append(msg)

    def img_left_depth_callback(self, msg):
        if len(self.img_left_depth_deque) >= BUFFERSIZE:
            self.img_left_depth_deque.popleft()
        self.img_left_depth_deque.append(msg)

    def img_right_depth_callback(self, msg):
        if len(self.img_right_depth_deque) >= BUFFERSIZE:
            self.img_right_depth_deque.popleft()
        self.img_right_depth_deque.append(msg)

    def img_front_depth_callback(self, msg):
        if len(self.img_front_depth_deque) >= BUFFERSIZE:
            self.img_front_depth_deque.popleft()
        self.img_front_depth_deque.append(msg)

    def puppet_arm_left_callback(self, msg):
        if len(self.puppet_arm_left_deque) >= BUFFERSIZE:
            self.puppet_arm_left_deque.popleft()
        self.puppet_arm_left_deque.append(msg)

    def puppet_arm_right_callback(self, msg):
        if len(self.puppet_arm_right_deque) >= BUFFERSIZE:
            self.puppet_arm_right_deque.popleft()
        self.puppet_arm_right_deque.append(msg)

    def puppet_arm_pos_left_callback(self, msg):
        if len(self.puppet_arm_pos_left_deque) >= BUFFERSIZE:
            self.puppet_arm_pos_left_deque.popleft()
        self.puppet_arm_pos_left_deque.append(msg)

    def puppet_arm_pos_right_callback(self, msg):
        if len(self.puppet_arm_pos_right_deque) >= BUFFERSIZE:
            self.puppet_arm_pos_right_deque.popleft()
        self.puppet_arm_pos_right_deque.append(msg)

    def robot_base_callback(self, msg):
        if len(self.robot_base_deque) >= BUFFERSIZE:
            self.robot_base_deque.popleft()
        self.robot_base_deque.append(msg)

    def ctrl_callback(self, msg):
        self.ctrl_state_lock.acquire()
        self.ctrl_state = msg.data
        self.ctrl_state_lock.release()

    def get_ctrl_state(self):
        self.ctrl_state_lock.acquire()
        state = self.ctrl_state
        self.ctrl_state_lock.release()
        return state

    def create_ros_node(self):
        print("Creating ROS node...")
        print("Ensuring ROS node is initialized...")
        if not rospy.core.is_initialized():
            # import ipdb; ipdb.set_trace()
            rospy.init_node("joint_state_publisher", anonymous=True)
        print("ROS node initialized (ROS1).")

    def create_subscriber(self, topic_name, callback, msg_type, msg_queue_size=1000):
        rospy.Subscriber(
            topic_name,
            msg_type,
            callback,
            queue_size=msg_queue_size,
            tcp_nodelay=True,
        )

    def create_publisher(self, topic_name, msg_type, msg_queue_size=10):
        publisher = rospy.Publisher(topic_name, msg_type, queue_size=msg_queue_size)
        return publisher

    def create_rate(self, rate):
        return rospy.Rate(rate)

    def get_current_time_msg(self):
        return rospy.Time.now()

    def time_converter(self, stamp):
        return stamp.to_sec()


class RosOperatorForRos2(RosOperator):
    def __init__(self, args=args):
        super().__init__(args)
        self.init_for_ros2()

    @property
    def running(self):
        return rclpy.ok()

    def init_for_ros2(self):
        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "gripper",
        ]
        self.processing_thread = threading.Thread(
            target=self._processing_loop, daemon=True
        )
        self._stop_event = threading.Event()
        self.processing_thread.start()

    def _processing_loop(self):
        """独立线程中的处理循环"""
        while self.running and not self._stop_event.is_set():
            rclpy.spin_once(self.ros_node, timeout_sec=0.02)

    def shutdown(self):
        """安全关闭资源"""
        self._stop_event.set()
        self.processing_thread.join()
        if self.processing_thread.is_alive():
            print("Thread not stopping, forcing shutdown")
        self.ros_node.destroy_node()
        rclpy.try_shutdown()

    def __del__(self):
        self.shutdown()

    def create_ros_node(self):
        rclpy.init(args=None)
        self.ros_node = Node("joint_state_publisher")

    def create_subscriber(self, topic_name, callback, msg_type, msg_queue_size=1000):
        if not self.ros_node:
            raise ValueError("ROS2 requires node instance")
        qos_profile = rclpy.qos.QoSProfile(
            depth=msg_queue_size,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )
        self.ros_node.create_subscription(msg_type, topic_name, callback, qos_profile)

    def create_publisher(self, topic_name, msg_type, msg_queue_size=10):
        if not self.ros_node:
            raise ValueError("ROS2 requires node instance")
        qos_profile = rclpy.qos.QoSProfile(
            depth=msg_queue_size,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )
        publisher = self.ros_node.create_publisher(msg_type, topic_name, qos_profile)
        return publisher

    def create_rate(self, rate):
        if not self.ros_node:
            raise ValueError("ROS2 requires node instance")
        return self.ros_node.create_rate(rate)

    def get_current_time_msg(self):
        if not self.ros_node:
            raise ValueError("ROS2 requires node instance")
        return self.ros_node.get_clock().now().to_msg()

    def time_converter(self, stamp):
        return RclpyTime.from_msg(stamp).nanoseconds


if __name__ == "__main__":
    opt = RosOperatorForRos2()
    try:
        while opt.running:
            time.sleep(0.1)
            _, img_left, img_right, puppet_arm_left, puppet_arm_right = opt.get_frame()
            print(
                f"img_left:{img_left.shape},img_right:{img_right.shape},puppet_arm_left:{puppet_arm_left},puppet_arm_right:{puppet_arm_right}"
            )
    except KeyboardInterrupt:
        pass
    finally:
        opt.shutdown()
