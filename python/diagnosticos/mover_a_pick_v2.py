#!/usr/bin/env python3
"""
Mueve el KR6 R700-2 a la pose 'pick' usando pymoveit2.
Se conecta al move_group ya activo (lanzado con demo.launch.py),
en vez de crear una instancia interna de MoveIt.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

from pymoveit2 import MoveIt2


def main():
    rclpy.init()

    node = Node("kr6_pymoveit2_pick")
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=[
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6",
        ],
        base_link_name="base_link",
        end_effector_name="tool0",
        group_name="manipulator",
        callback_group=callback_group,
    )

    # Ejecutamos el nodo en un hilo aparte para que las llamadas de
    # servicio/acción de MoveIt2 puedan procesarse mientras esperamos
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    # --- Pose 'pick' ---
    position = [0.229, 0.580, 0.284]
    quat_xyzw = [0.728, -0.683, -0.013, -0.055]

    node.get_logger().info("Planeando y moviendo a 'pick'...")
    moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=False)
    moveit2.wait_until_executed()
    node.get_logger().info("Movimiento a 'pick' completado.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
