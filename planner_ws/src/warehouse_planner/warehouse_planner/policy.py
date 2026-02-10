from __future__ import annotations

import os
from typing import Optional

import numpy as np


def get_default_model_path(package_share_dir: str) -> str:
    return os.path.join(package_share_dir, "models", "warehouse_maskable_ppo.zip")
