from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('smart_community')

    # Gazebo模型路径，把包内models追加到原有GAZEBO_MODEL_PATH前面
    model_dir = os.path.join(pkg_share, "models")
    old_gazebo_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    if old_gazebo_path:
        gazebo_model_value = model_dir + ":" + old_gazebo_path
    else:
        gazebo_model_value = model_dir

    set_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=gazebo_model_value
    )

    # 虚拟机软件渲染，解决黑屏闪退
    set_render = SetEnvironmentVariable(
        name="LIBGL_ALWAYS_SOFTWARE",
        value="1"
    )

    world_file = os.path.join(pkg_share, "worlds", "competition_world.sdf")

    # 启动gazebo加载world
    start_gazebo = ExecuteProcess(
        cmd=["gazebo", world_file],
        output="screen"
    )

    return LaunchDescription([
        set_render,
        set_model_path,
        start_gazebo
    ])

