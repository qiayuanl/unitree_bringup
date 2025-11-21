# launch/wandb.launch.py
from pathlib import Path

import wandb
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration


def _download_onnx(run_path: str, dest_dir: str) -> str:
    api = wandb.Api()
    run = api.run(run_path)

    # Look in run files
    for f in run.files():
        if f.name.endswith(".onnx"):
            Path(dest_dir).mkdir(parents=True, exist_ok=True)
            f.download(root=dest_dir, replace=True)
            return str(Path(dest_dir) / f.name)

    # Look in artifacts too (if logged as artifact)
    for art in run.logged_artifacts():
        for af in art.files():
            if af.name.endswith(".onnx"):
                Path(dest_dir).mkdir(parents=True, exist_ok=True)
                af.download(root=dest_dir)
                return str(Path(dest_dir) / af.name)

    raise RuntimeError(f"No .onnx file found in run {run_path}")


def _pull_and_set(context, *_, **__):
    run_path = LaunchConfiguration("wandb_path").perform(context)
    dest = LaunchConfiguration("dest_dir").perform(context)

    local_path = _download_onnx(run_path, dest)
    print(f"[W&B] Downloaded ONNX to: {local_path}")
    return [SetLaunchConfiguration("policy_path", local_path)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("wandb_path"),
        DeclareLaunchArgument("dest_dir", default_value="/tmp/wandb_onnx"),
        OpaqueFunction(function=_pull_and_set),
    ])
