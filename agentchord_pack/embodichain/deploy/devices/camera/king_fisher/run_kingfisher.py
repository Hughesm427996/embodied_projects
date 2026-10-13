import argparse
import os
import time
import cv2
import numpy as np
from datetime import datetime

from embodichain.deploy.devices.camera.king_fisher import get_kinfisher_images


def make_save_dir(task_name, root):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir = os.path.join(root, task_name, ts)
    os.makedirs(save_dir, exist_ok=True)
    return save_dir


def adjust_gamma(img, gamma=1.5):
    inv_gamma = 1.0 / gamma
    table = np.array(
        [(i / 255.0) ** inv_gamma * 255 for i in range(256)]
    ).astype("uint8")
    return cv2.LUT(img, table)


def brighten(img, alpha=1.2, beta=20):
    # alpha: contrast (>1 brighter)
    # beta: brightness offset
    return cv2.convertScaleAbs(img, alpha=alpha, beta=beta)


def parse_args():
    parser = argparse.ArgumentParser(description="Capture images from Kingfisher.")
    parser.add_argument(
        "--task-name",
        default="DualArmPourWater",
        help="Task name for saved images.",
    )
    parser.add_argument(
        "--save-root",
        default="./rgb_logs",
        help="Root directory for saved images.",
    )
    parser.add_argument(
        "--ip",
        default=os.environ.get("KINGFISHER_IP", "192.168.1.188"),
        help="Kingfisher camera IP. Defaults to KINGFISHER_IP.",
    )
    parser.add_argument("--scale", type=int, choices=[1, 4], default=4)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    save_dir = make_save_dir(args.task_name, args.save_root)
    print(f"[INFO] Saving images to: {save_dir}")

    frame_idx = 0
    start_time = time.time()

    try:
        while True:
            left, right, _, _ = get_kinfisher_images(scale=args.scale, ip=args.ip)
            right = brighten(right, alpha=1.2, beta=25)

            right_path = os.path.join(
                save_dir,
                f"right_{frame_idx:06d}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png",
            )
            cv2.imwrite(right_path, right)

            frame_idx += 1

    except KeyboardInterrupt:
        print("[INFO] Capture interrupted by user.")

    finally:
        end_time = time.time()
        total_time = end_time - start_time

        time_path = os.path.join(save_dir, "0_total_time.txt")
        with open(time_path, "w") as f:
            f.write(f"Total capture time (seconds): {total_time:.3f}\n")
            f.write(f"Total frames: {frame_idx}\n")
            if total_time > 0:
                f.write(f"Average FPS: {frame_idx / total_time:.2f}\n")

        print(f"[INFO] Total time saved to: {time_path}")
