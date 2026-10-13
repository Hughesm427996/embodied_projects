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

from pathlib import Path
from typing import Any

__all__ = ["get_sam_mask"]


def get_sam_mask(
    predictor: Any,
    obj_name: str,
    img_path: str | Path,
    save_path: str | Path | None = None,
) -> Any:
    """Run a SAM-style text-prompted segmentation predictor on one image."""
    label = obj_name.replace("_", " ")
    if save_path is not None:
        predictor.save_dir = Path(save_path)
    predictor.set_image(str(img_path))
    return predictor(text=[label])
