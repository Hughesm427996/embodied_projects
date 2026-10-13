import os
import sys
from typing import Dict, Tuple, Union
import yaml
import cv2
import numpy as np
from embodichain.utils.logger import log_info, log_warning

_KINGFISHER_SDK = None
_KINGFISHER_IP = None


def _get_kingfisher(ip: str | None = None):
    global _KINGFISHER_SDK, _KINGFISHER_IP

    camera_ip = ip or os.environ.get("KINGFISHER_IP")
    if not camera_ip:
        raise ValueError(
            "Kingfisher IP is required. Pass ip=... or set KINGFISHER_IP."
        )

    if _KINGFISHER_SDK is None:
        sdk_path = os.path.join(os.path.dirname(__file__), "kingfisher")
        if sdk_path not in sys.path:
            sys.path.append(sdk_path)
        try:
            import kingfisher
        except Exception as exc:
            raise RuntimeError("Failed to import Kingfisher SDK.") from exc
        _KINGFISHER_SDK = kingfisher

    if _KINGFISHER_IP != camera_ip:
        try:
            _KINGFISHER_SDK.connect(camera_ip)
        except Exception as exc:
            raise RuntimeError(
                f"Failed to connect to Kingfisher camera at {camera_ip}."
            ) from exc
        _KINGFISHER_IP = camera_ip

    return _KINGFISHER_SDK


RESOLUTION = (540, 960)
FULL_RESOLUTION = (540 * 4, 960 * 4)
CUHKSZ_ROKAE_HIKVISION_RESOLUTION = (540, 960)


def parse_yaml_to_np_array(input_data, is_file: bool = True) -> dict:
    if is_file:
        with open(input_data, "r") as file:
            data = yaml.safe_load(file)
    else:
        data = yaml.safe_load(input_data)

    result = {}

    for key, value in data.items():
        if isinstance(value, list):
            matrix = []
            for row in value:
                matrix.append(list(map(float, row)))
            result[key] = np.array(matrix)
        else:
            result[key] = value

    return result


###########################################################
# read camera parameters, intrinsic and extrinsic.
DEFAULT_CAM_CONFIG = os.path.join(
    os.path.dirname(__file__), "kingfisher_params.yaml"
)

def save_calib_data_to_yaml(output_path: str, ip: str | None = None):
    """
    Save calibration data obtained from kingfisher SDK directly to a YAML file.
    :param output_path: Path to the YAML file where calibration data will be saved.
    """
    try:
        calib_data_str = _get_kingfisher(ip).getCalibData()
        calib_data_dict = yaml.safe_load(calib_data_str)

        with open(output_path, "w", encoding="utf-8") as file:
            yaml.dump(
                calib_data_dict, file, default_flow_style=False, allow_unicode=True
            )

        log_info(f"Calibration data saved to {output_path}")
    except Exception as e:
        log_warning(f"Failed to save calibration data: {e}")
        raise


def get_kingfisher_params(
    print_flag: bool = False,
    cam_config: str = DEFAULT_CAM_CONFIG,
    scale: int = 4,
    use_sdk: bool = True,
    rectify: bool = False,
    ip: str | None = None,
) -> Union[Tuple[np.ndarray], Dict]:
    if use_sdk:
        calib_data_str = _get_kingfisher(ip).getCalibData()
        input_calib_params = parse_yaml_to_np_array(calib_data_str, is_file=False)
        log_info("use sdk to get calib data")
    else:
        if not os.path.exists(cam_config):
            log_warning(
                f"cam_config path '{cam_config}' does not exist. use DEFAULT_CAM_CONFIG."
            )
            cam_config = DEFAULT_CAM_CONFIG
        input_calib_params = parse_yaml_to_np_array(cam_config, is_file=True)
        log_info("get calib from yaml file")

    input_calib_params["cam1_k"][:2, ...] = (
        input_calib_params["cam1_k"][:2, ...] / scale
    )
    input_calib_params["cam2_k"][:2, ...] = (
        input_calib_params["cam2_k"][:2, ...] / scale
    )

    camera_matrix_l = input_calib_params["cam1_k"]
    dist_coeffs_l = input_calib_params["dist_1"]
    camera_matrix_r = input_calib_params["cam2_k"]
    dist_coeffs_r = input_calib_params["dist_2"]
    R_l_r = input_calib_params["R_l_r"]
    t_l_r = input_calib_params["t_l_r"]

    if rectify:
        R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
            cameraMatrix1=camera_matrix_l,
            distCoeffs1=dist_coeffs_l,
            cameraMatrix2=camera_matrix_r,
            distCoeffs2=dist_coeffs_r,
            imageSize=RESOLUTION[::-1],
            R=R_l_r,
            T=t_l_r,
            flags=1024,
            newImageSize=(0, 0),
        )

        rect_cam_k = P2[:3, :3]
        baseline = -P2[0, 3] / P2[0, 0] * 0.001  # m
        input_calib_params["rect_cam_k"] = rect_cam_k
        input_calib_params["baseline"] = baseline

    if print_flag:
        print("camera_matrix_l:", camera_matrix_l)
        print("dist_coeffs_l:", dist_coeffs_l)
        print("camera_matrix_r:", camera_matrix_r)
        print("dist_coeffs_r:", dist_coeffs_r)
        print("R_l_r:", R_l_r)
        print("t_l_r:", t_l_r)
    return input_calib_params


def get_kinfisher_images(
    scale=4,
    cam_config=DEFAULT_CAM_CONFIG,
    convert_to_rgb=False,
    ip: str | None = None,
):
    from embodichain.toolkits.vision.stereo import StereoRectify

    assert scale in [1, 4], "scale must be 1, 4 but {}.".format(scale)

    resolution = RESOLUTION if scale == 4 else FULL_RESOLUTION

    calib_dict = get_kingfisher_params(scale=scale, cam_config=cam_config, ip=ip)
    rectifier = StereoRectify(
        height=resolution[0],
        width=resolution[1],
        calib_file=cam_config,
        calib_dict=calib_dict,
    )

    if scale == 4:
        left, right = _get_kingfisher(ip).captureQuarterSize()
    else:
        left, right = _get_kingfisher(ip).capture()
    if convert_to_rgb:
        left = cv2.cvtColor(left, cv2.COLOR_BGR2RGB)
        right = cv2.cvtColor(right, cv2.COLOR_BGR2RGB)

    left_img = np.reshape(left, resolution + (3,))
    right_img = np.reshape(right, resolution + (3,))
    left_img, right_img = rectifier.rectify_imgs(left_img, right_img)

    return left_img, right_img, rectifier.rect_cam_k, rectifier.baseline
