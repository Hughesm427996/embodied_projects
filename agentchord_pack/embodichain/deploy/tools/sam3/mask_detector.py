# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ----------------------------------------------------------------------------

from __future__ import annotations

import argparse
from pathlib import Path

import torch
import torch.nn.functional as F
from ultralytics.engine.results import Results
from ultralytics.models.sam import SAM3SemanticPredictor
from ultralytics.utils import ops

__all__ = ["MySAM3SemanticPredictor", "build_predictor", "predict_local_image"]


DEFAULT_MODEL_PATH = Path(__file__).resolve().parent / "sam3.pt"
DEFAULT_SAVE_DIR = Path(__file__).resolve().parent / "runs"
DEFAULT_IMAGE_PATH = Path(__file__).resolve().parent / "test_fig.png"


class MySAM3SemanticPredictor(SAM3SemanticPredictor):
    def __init__(self, overrides, score_thresh: float = 0.05):
        super().__init__(overrides=overrides)
        self.score_thresh = score_thresh

    def postprocess(self, preds, img, orig_imgs):
        """Post-process the predictions to apply non-overlapping constraints if required."""
        pred_boxes = preds["pred_boxes"]  # (nc, num_query, 4)
        pred_logits = preds["pred_logits"]
        pred_masks = preds["pred_masks"]
        pred_scores = pred_logits.sigmoid()
        presence_score = preds["presence_logit_dec"].sigmoid().unsqueeze(1)
        pred_scores = (pred_scores * presence_score).squeeze(-1)
        pred_cls = torch.tensor(
            list(range(pred_scores.shape[0])),
            dtype=pred_scores.dtype,
            device=pred_scores.device,
        )[:, None].expand_as(pred_scores)
        pred_boxes = torch.cat(
            [pred_boxes, pred_scores[..., None], pred_cls[..., None]], dim=-1
        )

        keep = (pred_scores == pred_scores.max(dim=1, keepdim=True).values) & (
            pred_scores > self.score_thresh
        )
        pred_masks = pred_masks[keep]
        pred_boxes = pred_boxes[keep]
        pred_boxes[:, :4] = ops.xywh2xyxy(pred_boxes[:, :4])

        names = getattr(
            self.model,
            "names",
            [str(i) for i in range(pred_scores.shape[0])],
        )
        if not isinstance(orig_imgs, list):  # input images are a torch.Tensor, not a list
            orig_imgs = ops.convert_torch2numpy_batch(orig_imgs)
        results = []
        for masks, boxes, orig_img, img_path in zip(
            [pred_masks],
            [pred_boxes],
            orig_imgs,
            self.batch[0],
        ):
            if masks.shape[0] == 0:
                masks, boxes = None, torch.zeros((0, 6), device=pred_masks.device)
            else:
                masks = (
                    F.interpolate(
                        masks.float()[None],
                        orig_img.shape[:2],
                        mode="bilinear",
                    )[0]
                    > 0.5
                )
                boxes[..., [0, 2]] *= orig_img.shape[1]
                boxes[..., [1, 3]] *= orig_img.shape[0]
            results.append(
                Results(orig_img, path=img_path, names=names, masks=masks, boxes=boxes)
            )
        return results


def build_predictor(
    model: str | Path = DEFAULT_MODEL_PATH,
    save_dir: str | Path = DEFAULT_SAVE_DIR,
    conf: float = 0.5,
    score_thresh: float = 0.05,
    half: bool = True,
    save: bool = True,
) -> MySAM3SemanticPredictor:
    overrides = dict(
        conf=conf,
        task="segment",
        mode="predict",
        model=str(model),
        half=half,
        save=save,
    )
    predictor = MySAM3SemanticPredictor(
        overrides=overrides,
        score_thresh=score_thresh,
    )
    predictor.save_dir = Path(save_dir)
    return predictor


def predict_local_image(
    image: str | Path,
    text: list[str],
    model: str | Path = DEFAULT_MODEL_PATH,
    save_dir: str | Path = DEFAULT_SAVE_DIR,
    conf: float = 0.5,
    score_thresh: float = 0.05,
    half: bool = True,
    save: bool = True,
) -> list[Results]:
    image = Path(image)
    if not image.exists():
        raise FileNotFoundError(f"Image path does not exist: {image}")
    predictor = build_predictor(
        model=model,
        save_dir=save_dir,
        conf=conf,
        score_thresh=score_thresh,
        half=half,
        save=save,
    )
    predictor.set_image(str(image))
    return predictor(text=text)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test SAM3 on a local image.")
    parser.add_argument(
        "image",
        nargs="?",
        default=str(DEFAULT_IMAGE_PATH),
        help="Path to the local image.",
    )
    parser.add_argument(
        "--text",
        action="append",
        default=None,
        help='Text prompt. Repeat for multiple prompts, for example: --text "center cup"',
    )
    parser.add_argument(
        "--model",
        default=str(DEFAULT_MODEL_PATH),
        help="SAM3 model path.",
    )
    parser.add_argument(
        "--save-dir",
        default=str(DEFAULT_SAVE_DIR),
        help="Output directory.",
    )
    parser.add_argument("--conf", type=float, default=0.2, help="Confidence threshold.")
    parser.add_argument(
        "--score-thresh",
        type=float,
        default=0.05,
        help="Score threshold for keeping the best query of each prompt.",
    )
    parser.add_argument("--fp32", action="store_true", help="Disable FP16 inference.")
    parser.add_argument("--no-save", action="store_true", help="Do not save visualization.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    results = predict_local_image(
        image=args.image,
        text=args.text or ["center cup"],
        model=args.model,
        save_dir=args.save_dir,
        conf=args.conf,
        score_thresh=args.score_thresh,
        half=not args.fp32,
        save=not args.no_save,
    )
    mask_count = 0
    for result in results:
        if result.masks is not None:
            mask_count += len(result.masks)
    print(f"[INFO] Detected {mask_count} mask(s).")
    if not args.no_save:
        print(f"[INFO] Results saved to: {args.save_dir}")


if __name__ == "__main__":
    main()
