import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("kr6_r700_2", package_name="kuka_R6_R700_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Ajusta esta ruta si moviste el script a otro lugar
    script_path = "/home/laura/ws_kuka_R6_R700-2/mover_a_pick.py"

    kr6_moveit_py_node = Node(
        executable=sys.executable,   # el interprete python3 actual
        arguments=[script_path],
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([kr6_moveit_py_node])
