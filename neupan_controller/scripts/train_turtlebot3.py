#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Train upstream NeuPAN's Waffle DUNE on CUDA and export a libneupan model."""

import argparse
from pathlib import Path
import sys

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
import yaml


def main():
    package_dir = Path(get_package_share_directory("neupan_controller"))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path,
        default=package_dir / "config/turtlebot3_waffle_train.yaml",
    )
    parser.add_argument(
        "--neupan-source", type=Path,
        default=Path("/root/scout_mini_ws/src/NeuPAN"),
        help="Upstream NeuPAN source checkout",
    )
    parser.add_argument(
        "--output-dir", type=Path,
        default=Path.home() / ".ros/neupan/turtlebot3_waffle",
    )
    parser.add_argument("--device", help="Override YAML CUDA device, e.g. cuda:1")
    args = parser.parse_args()

    sys.path.insert(0, str(args.neupan_source.expanduser().resolve()))
    # CMake installs the existing exporter and NPTF writer next to this script.
    sys.path.insert(
        0, str(Path(get_package_prefix("neupan_controller")) / "lib/neupan_controller")
    )
    import numpy as np
    import torch
    from neupan import configuration
    from neupan.blocks import DUNETrain, ObsPointNet
    from neupan.configuration import np_to_tensor
    from neupan.robot import robot
    from export_dune_weights import export

    with args.config.expanduser().open() as stream:
        config = yaml.safe_load(stream)
    device = torch.device(args.device or config["device"])
    if device.type != "cuda" or not torch.cuda.is_available():
        raise RuntimeError("CUDA training requires an available GPU and CUDA-enabled PyTorch")

    seed = config.get("seed", 7)
    np.random.seed(seed)
    torch.manual_seed(seed)
    configuration.device = device
    config["device"] = str(device)

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    config_path = output_dir / "training_config.yaml"
    with config_path.open("w") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)

    footprint = robot(**config["robot"])
    model = ObsPointNet(2, len(footprint.G)).to(device)
    trainer = DUNETrain(
        model, np_to_tensor(footprint.G), np_to_tensor(footprint.h), str(output_dir)
    )
    print(f"Training device: {device} ({torch.cuda.get_device_name(device)})")
    print("ECOS generates geometry labels on CPU; DUNE optimization uses CUDA.")
    trainer.start(**config["train"])

    # Always export the final epoch, including when epoch is not a save_freq multiple.
    checkpoint = output_dir / "model_final.pth"
    torch.save(model.state_dict(), checkpoint)
    export(str(checkpoint), str(output_dir / "turtlebot3_waffle.bin"), str(config_path))
    print(f"Model ready: {output_dir / 'turtlebot3_waffle.bin'}")


if __name__ == "__main__":
    main()
