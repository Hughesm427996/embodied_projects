import numpy as np
import torch
from copy import deepcopy
from typing import List, Union

from sensor_msgs.msg import JointState
from embodichain.deploy.robots.cobotmagic.ros_bridge import (
    RosOperator,
    RosOperatorForRos2,
)
from scipy.spatial.transform import Rotation as R
from embodichain.utils import logger

try:
    from piper_msgs.msg import PosCmd
except ImportError as exc:
    logger.log_error(f"Failed to import PosCmd from piper_msgs: {exc}")
import time
from pathlib import Path
import cv2
import yaml


def mul_linear_expand(
    arr: np.ndarray, expand_times: Union[int, List[int]], is_interp: bool = True
):
    arr_len = arr.shape[0]
    dim = arr.shape[1]
    if isinstance(expand_times, int):
        interp_path = np.zeros(shape=(arr_len * expand_times, dim), dtype=float)
    else:
        assert len(expand_times) == arr_len - 1, "Invalid expand_times size."
        interp_path = np.zeros(shape=(sum(expand_times), dim), dtype=float)

    idx = 0
    for i in range(0, arr_len - 1):
        if isinstance(expand_times, int):
            sample_times = expand_times
        else:
            sample_times = expand_times[i]
        for k in range(sample_times):
            if is_interp:
                v = (sample_times - k) / sample_times * arr[i] + k / sample_times * arr[
                    i + 1
                ]
            else:
                v = arr[i]
            interp_path[idx] = v
            idx += 1
    interp_path = interp_path[:idx]
    return interp_path


class _RealRobotAdapter:
    """Expose cached real-robot state through the Robot interface used by agents."""

    def __init__(self, controller, model_robot):
        self._controller = controller
        self._model_robot = model_robot

    @property
    def uid(self):
        return self._model_robot.uid

    @property
    def device(self):
        return self._model_robot.device

    def get_qpos(self, name: str = None, target: bool = False):
        qpos = torch.zeros(16, dtype=torch.float32, device=self.device)
        left_arm = torch.as_tensor(
            getattr(self._controller, "left_arm_current_qpos", torch.zeros(6)),
            dtype=torch.float32,
            device=self.device,
        )
        right_arm = torch.as_tensor(
            getattr(self._controller, "right_arm_current_qpos", torch.zeros(6)),
            dtype=torch.float32,
            device=self.device,
        )
        left_gripper = torch.as_tensor(
            getattr(self._controller, "left_arm_current_gripper_state", torch.zeros(1)),
            dtype=torch.float32,
            device=self.device,
        ).reshape(-1)[0]
        right_gripper = torch.as_tensor(
            getattr(
                self._controller,
                "right_arm_current_gripper_state",
                torch.zeros(1),
            ),
            dtype=torch.float32,
            device=self.device,
        ).reshape(-1)[0]

        qpos[self._controller.left_arm_joints] = left_arm
        qpos[self._controller.right_arm_joints] = right_arm
        qpos[self._controller.left_eef_joints] = left_gripper
        qpos[self._controller.right_eef_joints] = right_gripper
        qpos = qpos.unsqueeze(0)

        if name is None:
            return qpos
        return qpos[:, self.get_joint_ids(name=name)]

    def get_joint_ids(self, name: str = None, remove_mimic: bool = False):
        return self._model_robot.get_joint_ids(
            name=name,
            remove_mimic=remove_mimic,
        )

    def get_control_part_base_pose(self, *args, **kwargs):
        return self._model_robot.get_control_part_base_pose(*args, **kwargs)

    def compute_ik(self, *args, **kwargs):
        return self._model_robot.compute_ik(*args, **kwargs)

    def compute_fk(self, *args, **kwargs):
        return self._model_robot.compute_fk(*args, **kwargs)


class _PerceptionRigidObject:
    """Rigid-object facade backed by AgentAgilexController perception."""

    def __init__(self, controller, obj_name: str):
        self._controller = controller
        self.obj_name = obj_name

    def get_local_pose(self, to_matrix: bool = True):
        if not to_matrix:
            logger.log_error("Real perception object poses only support matrix pose.")
        pose = self._controller.get_object_pose(self.obj_name, update=True)
        return pose.unsqueeze(0)


class _PerceptionSceneAdapter:
    """Small scene facade for monitor code that expects env.sim."""

    def __init__(self, controller):
        self._controller = controller

    def get_rigid_object_uid_list(self):
        obj_names = list(getattr(self._controller, "obj_info", {}).keys())
        for obj_name in getattr(self._controller, "object_names", []):
            if obj_name not in obj_names:
                obj_names.append(obj_name)
        return obj_names

    def get_rigid_object(self, obj_name: str):
        if obj_name not in self.get_rigid_object_uid_list():
            logger.log_error(
                f"Rigid object '{obj_name}' not found in perception scene."
            )
        return _PerceptionRigidObject(self._controller, obj_name)

    def draw_marker(self, *args, **kwargs):
        return None

    def update(self, *args, **kwargs):
        return None


class AgilexController:
    def __init__(
        self,
        ros_operator: Union[RosOperator, RosOperatorForRos2],
        init_controller=False,
    ):
        self.ros_operator = ros_operator

        self.left_gripper_joint_limits = (0, 0.1)  # 0.1表示夹爪最大能够张开0.1m，且是全张开到底的。
        self.right_gripper_joint_limits = (0, 0.1)

        self.has_controller = False
        if init_controller:
            from embodichain.lab.sim import SimulationManager, SimulationManagerCfg
            from embodichain.lab.sim.objects import Robot
            from embodichain.lab.sim.robots import CobotMagicCfg

            # Initialize simulation
            robot_sim = SimulationManager(SimulationManagerCfg(headless=True))
            robot_sim.set_manual_update(False)

            # Robot configuration
            cfg_dict = {"uid": "CobotMagic"}
            self.sim_robot: Robot = robot_sim.add_robot(
                cfg=CobotMagicCfg.from_dict(cfg_dict)
            )
            self.has_controller = True

            self.right_base_xpos = self.sim_robot.get_control_part_base_pose(
                "right_arm", to_matrix=True
            ).squeeze(0)
            self.left_base_xpos = self.sim_robot.get_control_part_base_pose(
                "left_arm", to_matrix=True
            ).squeeze(0)

        self._previous_qpos = self.get_current_qpos()

    def get_current_qpos(self, name: str = None):
        left_arm_frame, right_arm_frame = self.ros_operator.get_puppet_arm_frame()
        left_arm_frame: JointState
        right_arm_frame: JointState
        qpos_left = np.array(left_arm_frame.position)
        qpos_right = np.array(right_arm_frame.position)
        # print("Left arm joint : {}.".format(np.round(qpos_left, 4)))
        # print("Right arm joint : {}.".format(np.round(qpos_right, 4)))
        if name == None:
            return np.concatenate([qpos_left, qpos_right])
        elif name == "left_arm":
            return qpos_left[:6]
        elif name == "right_arm":
            return qpos_right[:6]
        elif name == "left_eef":
            return qpos_left[6]
        elif name == "right_eef":
            return qpos_right[6]
        else:
            logger.log_error("Invalid name for get_current_qpos")

    def set_current_qpos(
        self, qpos: np.ndarray, name: str = None, interp_num=0, wait: bool = True
    ):
        qpos_target = deepcopy(qpos)
        qpos_left = qpos_target[:7]
        qpos_right = qpos_target[7:]

        if interp_num == 0:
            self.ros_operator.puppet_arm_publish(
                qpos_left.tolist(), qpos_right.tolist()
            )
            if wait:
                self._wait_qpos_for_motion_done(target_qpos=qpos)
            self._previous_qpos = np.concatenate((qpos_left, qpos_right))
        else:
            expand_times = [interp_num]
            expand_left = np.stack((self._previous_qpos[:7], qpos_left), axis=0)
            expand_right = np.stack((self._previous_qpos[7:], qpos_right), axis=0)
            new_qpos_left = mul_linear_expand(expand_left, expand_times)
            new_qpos_right = mul_linear_expand(expand_right, expand_times)
            for i in range(len(new_qpos_left)):
                # print(f"left arm joint : {np.round(new_qpos_left[i], 4)}")
                # print(f"right arm joint : {np.round(new_qpos_right[i], 4)}")
                self.ros_operator.puppet_arm_publish(
                    new_qpos_left[i].tolist(), new_qpos_right[i].tolist()
                )
            if wait:
                self._wait_qpos_for_motion_done(target_qpos=qpos)
            self._previous_qpos = np.concatenate(
                (new_qpos_left[-1], new_qpos_right[-1])
            )
        return True

    def set_current_xpos(self, name: str, xpos: np.ndarray):
        if self.has_controller is False:
            logger.log_warning("No controller is initialized")
            return False

        if name not in ["left_arm", "right_arm"]:
            logger.log_error("Invalid name for set_current_xpos")
            return False

        ret, qpos = self.get_arm_ik(
            target_xpos=xpos,
            is_left=name == "left_arm",
            qpos_seed=self.get_current_qpos(name=name),
        )

        if ret is False:
            logger.log_warning("Failed to get IK solution")
            return False
        else:
            current_qpos = self.get_current_qpos()
            if name == "left_arm":
                current_qpos[:6] = qpos
            elif name == "right_arm":
                current_qpos[7:13] = qpos
            self.set_current_qpos(current_qpos)
            return True

    def set_current_xpos_L(
        self,
        name: str,
        xpos: np.ndarray,
        num_points: int = 10,
    ):
        """tcp末端笛卡尔直线运动"""
        if not isinstance(name, str) or name not in ["left_arm", "right_arm"]:
            logger.log_error("Invalid name for set_current_xpos")
            return

        if not isinstance(xpos, (np.ndarray, torch.Tensor)) or xpos.shape != (4, 4):
            logger.log_error("xpos must be a 4x4 homogeneous transformation matrix")
            return

        current_pose = self.get_current_xpos(name)

        start_pos = current_pose[:3, 3]
        end_pos = xpos[:3, 3]
        positions = np.linspace(start_pos, end_pos, num_points)

        start_rot = R.from_matrix(current_pose[:3, :3])
        end_rot = R.from_matrix(xpos[:3, :3])
        rotations = []

        for t in np.linspace(0, 1, num_points):
            rotations.append((start_rot * (end_rot * start_rot.inv()) ** t).as_matrix())

        interpolated_poses = []
        for pos, rot in zip(positions, rotations):
            pose = np.eye(4)
            pose[:3, :3] = rot
            pose[:3, 3] = pos
            interpolated_poses.append(pose)
        interpolated_qpos_list = []
        for interpolated_pose in interpolated_poses:
            res, qpos = self.get_arm_ik(
                target_xpos=interpolated_pose,
                is_left=name == "left_arm",
                qpos_seed=self.get_current_qpos(name),
            )
            if not res:
                logger.log_warning("Failed to get IK solution on the line")
                return False
            else:
                interpolated_qpos_list.append(qpos)

        for qpos in interpolated_qpos_list:
            current_qpos = self.get_current_qpos()
            if name == "left_arm":
                current_qpos[:6] = qpos
            elif name == "right_arm":
                current_qpos[7:13] = qpos
            self.set_current_qpos(current_qpos)
        return True

    def get_current_xpos(self, name: str):
        """获取末端tcp的姿态,返回是齐次矩阵"""
        if self.has_controller is False:
            logger.log_error("No controller is initialized")

        if name not in ["left_arm", "right_arm"]:
            logger.log_error("Invalid name for get_current_xpos")

        return self.get_arm_fk(
            qpos=self.get_current_qpos(name=name), is_left=name == "left_arm"
        )

    def get_arm_ik(self, target_xpos, is_left, qpos_seed=None):
        control_part = "left_arm" if is_left else "right_arm"
        robot = getattr(self, "robot", None)
        if robot is None:
            robot = getattr(self, "sim_robot", None)
        if robot is None:
            logger.log_error("No simulation robot is initialized")
        ret, qpos = robot.compute_ik(
            name=control_part,
            pose=target_xpos,
            joint_seed=qpos_seed,
        )
        success = ret.all().item() if isinstance(ret, torch.Tensor) else bool(ret)
        return bool(success), qpos.squeeze(0)

    def get_arm_fk(self, qpos, is_left):
        control_part = "left_arm" if is_left else "right_arm"
        robot = getattr(self, "robot", None)
        if robot is None:
            robot = getattr(self, "sim_robot", None)
        if robot is None:
            logger.log_error("No simulation robot is initialized")
        xpos = robot.compute_fk(
            name=control_part,
            qpos=torch.as_tensor(qpos, dtype=torch.float32, device=robot.device),
            to_matrix=True,
        )
        return xpos.squeeze(0)

    def get_current_xpos_origin(self, name: str):  # 一般不用
        """使用监听话题的方式获得末端法兰盘的xyz,roll,pitch,yaw(角度制)"""
        if name not in ["left_arm", "right_arm"]:
            logger.log_error("Invalid name for get_current_xpos")

        arm_xpos: PosCmd = self.ros_operator.get_puppet_arm_pos_frame(name)

        xpos = np.array(
            [
                arm_xpos.x,
                arm_xpos.y,
                arm_xpos.z,
                arm_xpos.roll,
                arm_xpos.pitch,
                arm_xpos.yaw,
            ]
        )
        return xpos

    def _wait_qpos_for_motion_done(
        self,
        timeout=10.0,
        poll_interval=0.1,
        distance_threshold=0.1,
        gripper_threshold=0.02,
        count_threshold=3,
        target_qpos=None,
    ):
        """阻塞直到指定机械臂关节运动完成"""
        start_time = time.time()
        ret = self.ros_operator.get_puppet_arm_frame()
        arm_val = np.concatenate([np.array(ret[0].position), np.array(ret[1].position)])

        count = 1
        while True:
            ret = self.ros_operator.get_puppet_arm_frame()
            new_arm_val = np.concatenate(
                [np.array(ret[0].position), np.array(ret[1].position)]
            )
            target_constraint = (
                np.linalg.norm(arm_val - new_arm_val) <= distance_threshold
            )
            # from IPython import embed
            # embed()
            if target_constraint:
                count += 1

            if count > count_threshold:
                return True
            arm_val = new_arm_val
            if time.time() - start_time > timeout:
                raise TimeoutError(f"qpos motion timeout after {timeout}s")
            time.sleep(poll_interval)

    def set_gripper(self, name: str, gripper_cmd: float):
        if name not in ["left_arm", "right_arm"]:
            logger.log_error("Invalid name for set_gripper")
        current_qpos = self.get_current_qpos()
        if name == None:
            current_qpos[6] = gripper_cmd
            current_qpos[13] = gripper_cmd
            self.set_current_qpos(current_qpos)
        elif name == "left_arm":
            current_qpos[6] = gripper_cmd
            self.set_current_qpos(current_qpos)
        elif name == "right_arm":
            current_qpos[13] = gripper_cmd
            self.set_current_qpos(current_qpos)
        else:
            logger.log_error("Invalid name for set_gripper")


class AgentAgilexController(AgilexController):
    """
    High-level agent controller built on top of AgilexController.
    Adds agent-specific logic such as policy execution, safety checks, etc.
    """

    def __init__(
        self,
        ros_operator,
        init_controller=False,
        agent_config=None,
        task_name=None,
        agent_config_path=None,
        object_names=None,
        affordance_datas=None,
        camera_config_path=None,
        kingfisher_ip: str = "192.168.1.188",
    ):
        # 调用父类构造函数
        super().__init__(
            ros_operator=ros_operator,
            init_controller=init_controller,
        )
        self.ros_operator = ros_operator
        self.current_step = 0
        self.task_name = task_name
        self.backend = "real"
        self.real_world = True
        self.kingfisher_ip = kingfisher_ip
        self.camera_config_path = camera_config_path
        self.object_names = list(object_names or [])
        self.affordance_datas = {} if affordance_datas is None else affordance_datas
        self.obj_info = {}

        if hasattr(self, "sim_robot"):
            self.robot = _RealRobotAdapter(self, self.sim_robot)
            self.sim = _PerceptionSceneAdapter(self)

        self._init_kingfisher()
        self._init_foundation_stereo_model()
        self._init_sam3_predictor()

        if agent_config is not None:
            self._init_agents(
                agent_config=agent_config,
                task_name=task_name,
                agent_config_path=agent_config_path,
            )

        self.get_states()

    def _init_kingfisher(self):
        try:
            import kingfisher

            self.kingfisher = kingfisher
            self.kingfisher.connect(self.kingfisher_ip)
        except Exception as exc:
            logger.log_error(f"Failed to connect to Kingfisher: {exc}")

    def _init_foundation_stereo_model(self):
        self.max_disp = 416
        try:
            from glia.dl.models.foundation_stereo.build import (
                build_foundation_stereo_model,
            )

            self.foundation_stereo_model = (
                build_foundation_stereo_model(
                    encoder="vits",
                    max_disp=self.max_disp,
                )
                .cuda()
                .eval()
            )
        except Exception as exc:
            logger.log_error(f"Failed to initialize Foundation Stereo Model: {exc}")

    def _init_sam3_predictor(self):
        try:
            from embodichain.deploy.tools.sam3.mask_detector import (
                DEFAULT_MODEL_PATH,
                DEFAULT_SAVE_DIR,
                MySAM3SemanticPredictor,
            )

            overrides = dict(
                conf=0.5,  # only for initialization, not used
                task="segment",
                mode="predict",
                model=str(DEFAULT_MODEL_PATH),
                half=True,  # Use FP16 for faster inference
                save=True,
            )
            self.predictor = MySAM3SemanticPredictor(overrides=overrides)
            self.predictor.save_dir = DEFAULT_SAVE_DIR
            self.predictor.setup_model()

            from datetime import datetime

            day_time = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.sam_results_dir = Path("sam_results") / self.task_name / day_time
        except Exception as exc:
            logger.log_error(f"Failed to initialize SAM3SemanticPredictor: {exc}")

    def get_states(self):
        if not hasattr(self, "robot"):
            logger.log_error(
                "AgentAgilexController requires init_controller=True to initialize the kinematic robot model."
            )

        self.left_arm_joints = [0, 2, 4, 6, 8, 10]
        self.right_arm_joints = [1, 3, 5, 7, 9, 11]
        self.left_eef_joints = [12, 13]
        self.right_eef_joints = [14, 15]

        self.left_arm_init_qpos = np.array([0, 0, 0, 0, 0, 0])
        self.right_arm_init_qpos = np.array([0, 0, 0, 0, 0, 0])

        self.left_arm_init_xpos = self.get_arm_fk(
            qpos=self.left_arm_init_qpos, is_left=True
        )
        self.right_arm_init_xpos = self.get_arm_fk(
            qpos=self.right_arm_init_qpos, is_left=False
        )

        self.left_arm_base_pose = self.left_base_xpos
        self.right_arm_base_pose = self.right_base_xpos

        self.left_arm_current_qpos = self.left_arm_init_qpos
        self.right_arm_current_qpos = self.right_arm_init_qpos

        self.left_arm_current_xpos = self.left_arm_init_xpos
        self.right_arm_current_xpos = self.right_arm_init_xpos

        self.open_state = np.array([1.0])
        self.close_state = np.array([0.0])

        self.left_arm_current_gripper_state = self.open_state
        self.right_arm_current_gripper_state = self.open_state
        self.init_qpos = self.robot.get_qpos().squeeze(0)
        self.update_obj_info()

    def _init_agents(self, agent_config, task_name, agent_config_path=None):
        try:
            from embodichain.agents.hierarchy.task_agent import TaskAgent
            from embodichain.agents.hierarchy.compile_agent import CompileAgent
            from embodichain.agents.hierarchy.recovery_agent import RecoveryAgent
            from embodichain.agents.hierarchy.llm import (
                task_llm,
                compile_llm,
                recovery_llm,
            )

            task_name = task_name or agent_config["Task"]["name"]
            self.task_agent = TaskAgent(
                task_llm,
                **agent_config["Agent"],
                **agent_config["TaskAgent"],
                task_name=task_name,
                config_dir=agent_config_path,
            )
            self.recovery_agent = RecoveryAgent(
                recovery_llm,
                **agent_config["Agent"],
                **agent_config["RecoveryAgent"],
                task_name=task_name,
                config_dir=agent_config_path,
            )
            self.compile_agent = CompileAgent(
                compile_llm,
                **agent_config["Agent"],
                **agent_config["CompileAgent"],
                task_name=task_name,
                config_dir=agent_config_path,
            )
        except Exception as exc:
            logger.log_error(f"Failed to initialize Agents: {exc}")

    def get_current_qpos(self, name: str = None):
        (
            left_arm_frame,
            right_arm_frame,
        ) = self.ros_operator.get_puppet_arm_frame_realtime()
        left_arm_frame: JointState
        right_arm_frame: JointState
        qpos_left = np.array(left_arm_frame.position)
        qpos_right = np.array(right_arm_frame.position)
        # print("Left arm joint : {}.".format(np.round(qpos_left, 4)))
        # print("Right arm joint : {}.".format(np.round(qpos_right, 4)))
        if name == None:
            return np.concatenate([qpos_left, qpos_right])
        elif name == "left_arm":
            return qpos_left[:6]
        elif name == "right_arm":
            return qpos_right[:6]
        elif name == "left_eef":
            return qpos_left[6]
        elif name == "right_eef":
            return qpos_right[6]
        else:
            logger.log_error("Invalid name for get_current_qpos")

    def get_current_qpos_scaled_gripper(self):
        left_range = self.left_gripper_joint_limits
        right_range = self.right_gripper_joint_limits
        qpos = self.get_current_qpos()
        qpos = qpos.copy()
        qpos[6] = (qpos[6] - left_range[0]) / (left_range[1] - left_range[0] + 1e-8)
        qpos[13] = (qpos[13] - right_range[0]) / (
            right_range[1] - right_range[0] + 1e-8
        )

        return qpos

    def get_obs_for_agent(self):
        # ---------------------------------- Realsense camera----------------------------------
        # cam_k = np.array([
        #     [607.080810546875, 0.0, 318.8660583496094],
        #     [0.0, 606.98681640625, 247.5299835205078],
        #     [0.0, 0.0, 1.0]
        # ], dtype=np.float32)  # got by the command ros2 topic echo /camera/camera_r/color/camera_info --once

        # cam_pose = None # TODO
        #
        # _, _, rgb = self.ros_operator.get_image_frame()
        # _, _, depth = self.ros_operator.get_depth_frame()
        #
        # depth = depth.astype(np.float32) / 1000.0

        # ---------------------------------- Kingfisher camera----------------------------------
        # use left camera as default
        from embodichain.deploy.devices.camera.king_fisher import (
            DEFAULT_CAM_CONFIG,
            get_kinfisher_images,
        )

        config_path = Path(self.camera_config_path or DEFAULT_CAM_CONFIG)
        left_rgb, right_rgb, rect_cam_k, baseline = get_kinfisher_images(
            scale=4,
            convert_to_rgb=True,
            cam_config=str(config_path),
            ip=self.kingfisher_ip,
        )
        # from glia.utility.image_utils import draw_rect_imgs
        # original_pair = draw_rect_imgs(left_rgb, right_rgb)
        # cv2.imwrite("original_pair.png", original_pair)

        cam_k = rect_cam_k.astype(np.float32)

        with open(config_path, "r") as f:
            data = yaml.safe_load(f)
            T_left_cam_to_left_arm = np.array(
                data["T_left_cam_to_left_arm"], dtype=np.float32
            )
            T_left_cam_to_right_arm = np.array(
                data["T_left_cam_to_right_arm"], dtype=np.float32
            )

        T_to_left_arm = T_left_cam_to_left_arm
        T_to_right_arm = T_left_cam_to_right_arm

        obs = {
            "rgb": left_rgb,
            "left_rgb": left_rgb,
            "right_rgb": right_rgb,
            "baseline": baseline,
            "cam_k": cam_k,
            "T_to_left_arm": T_to_left_arm,
            "T_to_right_arm": T_to_right_arm,
        }
        self.last_observations = obs
        return obs

    def get_rgb_for_agent(self):
        # ---------------------------------- Kingfisher camera----------------------------------
        try:
            rgb_1, rgb_2 = self.kingfisher.captureQuarterSize()
            rgb_1 = cv2.cvtColor(rgb_1, cv2.COLOR_BGR2RGB)
            rgb_2 = cv2.cvtColor(rgb_2, cv2.COLOR_BGR2RGB)
        except Exception as exc:
            logger.log_error(f"Failed to capture Kingfisher RGB images: {exc}")

        # ---------------------------------- Realsense camera----------------------------------
        _, _, rgb_3 = self.ros_operator.get_image_frame()

        return rgb_1, rgb_2, rgb_3

    def estimate_object_pose(
        self, obj_name: str, robot_name: str = "left_arm", **kwargs
    ):
        from embodichain.deploy.perception.pose_estimation import (
            get_obj_pose_from_perception,
        )

        perception_kwargs = {
            "predictor": getattr(self, "predictor", None),
            "foundation_stereo_model": getattr(self, "foundation_stereo_model", None),
            "max_disp": getattr(self, "max_disp", 416),
        }
        perception_kwargs.update(kwargs)
        pose = get_obj_pose_from_perception(
            self,
            obj_name=obj_name,
            robot_name=robot_name,
            kwargs=perception_kwargs,
        )
        return torch.as_tensor(pose, dtype=torch.float32)

    def get_object_pose(
        self,
        obj_name: str,
        robot_name: str = "left_arm",
        update: bool = False,
        **kwargs,
    ):
        obj_info = getattr(self, "obj_info", {})
        if not update and obj_name in obj_info:
            return obj_info[obj_name]["pose"]

        pose = self.estimate_object_pose(
            obj_name=obj_name,
            robot_name=robot_name,
            **kwargs,
        )
        if obj_name in obj_info:
            obj_info[obj_name]["pose"] = pose
        else:
            obj_info[obj_name] = {
                "pose": pose,
                "height": pose[2, 3],
                "grasp_pose_obj": None,
            }
        self.obj_info = obj_info
        return pose

    def update_obj_info(
        self,
        obj_names=None,
        robot_name: str = "left_arm",
        use_perception: bool = True,
        **kwargs,
    ):
        obj_info = getattr(self, "obj_info", {})
        if obj_names is None:
            obj_names = list(getattr(self, "object_names", []))
            for obj_name in obj_info.keys():
                if obj_name not in obj_names:
                    obj_names.append(obj_name)

        for obj_name in obj_names:
            pose_kwargs = dict(kwargs)
            pose_robot_name = pose_kwargs.pop("robot_name", robot_name)
            if use_perception:
                pose = self.estimate_object_pose(
                    obj_name=obj_name,
                    robot_name=pose_robot_name,
                    **pose_kwargs,
                )
            elif obj_name in obj_info:
                pose = obj_info[obj_name]["pose"]
            else:
                continue

            obj_grasp_pose = self.affordance_datas.get(
                f"{obj_name}_grasp_pose_object",
                None,
            )
            if isinstance(obj_grasp_pose, torch.Tensor) and obj_grasp_pose.dim() == 3:
                obj_grasp_pose = obj_grasp_pose.squeeze(0)

            if obj_name not in obj_info:
                obj_info[obj_name] = {
                    "pose": pose,
                    "height": pose[2, 3],
                    "grasp_pose_obj": obj_grasp_pose,
                }
            else:
                obj_info[obj_name]["pose"] = pose
                if obj_grasp_pose is not None:
                    obj_info[obj_name]["grasp_pose_obj"] = obj_grasp_pose

        self.obj_info = obj_info
        return obj_info

    def get_current_qpos_agent(self):
        return self.left_arm_current_qpos, self.right_arm_current_qpos

    def set_current_qpos_agent(self, arm_qpos, is_left):
        if is_left:
            self.left_arm_current_qpos = arm_qpos
        else:
            self.right_arm_current_qpos = arm_qpos

    def get_current_xpos_agent(self):
        return self.left_arm_current_xpos, self.right_arm_current_xpos

    def set_current_xpos_agent(self, arm_xpos, is_left):
        if is_left:
            self.left_arm_current_xpos = arm_xpos
        else:
            self.right_arm_current_xpos = arm_xpos

    def get_current_gripper_state_agent(self):
        return self.left_arm_current_gripper_state, self.right_arm_current_gripper_state

    def set_current_gripper_state_agent(self, arm_gripper_state, is_left):
        if is_left:
            self.left_arm_current_gripper_state = arm_gripper_state
        else:
            self.right_arm_current_gripper_state = arm_gripper_state

    # -------------------- IK / FK --------------------
    def get_arm_ik(self, target_xpos, is_left, qpos_seed=None):
        return super().get_arm_ik(target_xpos, is_left=is_left, qpos_seed=qpos_seed)

    def get_arm_fk(self, qpos, is_left):
        return super().get_arm_fk(qpos=qpos, is_left=is_left)

    # -------------------- get compiled graph for action list --------------------
    def generate_graph_for_actions(self, regenerate=False, recovery=False, **kwargs):
        from embodichain.data import database_agent_prompt_dir

        kwargs.setdefault(
            "log_dir",
            Path(database_agent_prompt_dir) / "real" / self.compile_agent.task_name,
        )
        logger.log_info(
            f"Generate graph for creating {'recovery' if recovery else ''} action list for {self.compile_agent.task_name}.",
            color="yellow" if recovery else "green",
        )

        print(f"\033[92m\nStart task graph generation.\n\033[0m")
        task_agent_input = self.task_agent.get_composed_observations(
            env=self,
            regenerate=regenerate,
            observations=self.get_obs_for_agent(),
            **kwargs,
        )
        task_graph = self.task_agent.generate(**task_agent_input)

        recovery_spec = None
        if recovery:
            print(f"\033[91m\nStart recovery spec generation.\n\033[0m")
            recovery_agent_input = self.recovery_agent.get_composed_observations(
                env=self,
                regenerate=regenerate,
                task_graph=task_graph,
                **kwargs,
            )
            recovery_spec = self.recovery_agent.generate(**recovery_agent_input)

        print(f"\033[94m\nStart graph compilation.\n\033[0m")
        compile_agent_input = self.compile_agent.get_composed_observations(
            env=self,
            regenerate=regenerate,
            task_graph=task_graph,
            recovery_spec=recovery_spec,
            recovery_enabled=recovery,
            **kwargs,
        )
        graph_file_path, kwargs, graph_content = self.compile_agent.generate(
            **compile_agent_input
        )

        return graph_file_path, kwargs, graph_content

    # -------------------- get action list --------------------
    def create_demo_action_list(
        self, regenerate=False, recovery=False, *args, **kwargs
    ):
        graph_file_path, compile_kwargs, _ = self.generate_graph_for_actions(
            regenerate=regenerate, recovery=recovery, **kwargs
        )
        compile_kwargs["interactive_error_injection"] = kwargs.get(
            "interactive_error_injection", False
        )
        return self.compile_agent.act(graph_file_path, **compile_kwargs)

    def create_demo_action_list_with_self_correction(self, **kwargs):
        return self.create_demo_action_list(recovery=True, **kwargs)
