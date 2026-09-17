#!/usr/bin/env python3
"""
Parte 4C - Traslado pick -> pre-place (evasion de obstaculo).
Mismo tipo de movimiento libre que 4A, pero ya con el planeador
seleccionado (RRTConnect, ganador de la comparacion en 4A).

NOTA: este script asume que el robot esta fisicamente en 'pick'
(con la pieza ya adjunta a tool0, tal como quedo tras 4B), y que
demo.launch.py sigue corriendo desde entonces (para que la escena
y el objeto adjunto sigan vigentes).
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

from pymoveit2 import MoveIt2


# ============================================================
#  PARAMETROS
# ============================================================
PLACE_POSITION = np.array([0.093, -0.588, 0.631])
PLACE_QUAT_XYZW = [0.078, -0.995, 0.055, -0.004]  # rotada 270 grados sobre Z local (libre de colision)
PLACE_S = np.array([0.0165, -0.1088, -0.9922])  # recalculado tras rotar orientacion (S se preserva)
D_RETROCESO = 0.1

PLANEADOR = "RRTConnectkConfigDefault"  # ya seleccionado en 4A
# ============================================================


def calcular_pre_place(place_pos, S_vector, d):
    S_normalizado = S_vector / np.linalg.norm(S_vector)
    return (place_pos - d * S_normalizado).tolist()


def main():
    rclpy.init()
    node = Node("kr6_traslado_4c")
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        base_link_name="base_link",
        end_effector_name="tool0",
        group_name="manipulator",
        callback_group=callback_group,
    )

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    time.sleep(2.0)

    pre_place_position = calcular_pre_place(PLACE_POSITION, PLACE_S, D_RETROCESO)
    node.get_logger().info(f"pre-place calculado: {pre_place_position}")

    node.get_logger().info(f"Planeando pick -> pre-place con {PLANEADOR} (evasion de obstaculo)...")
    moveit2.planner_id = PLANEADOR

    t0 = time.time()
    moveit2.move_to_pose(
        position=pre_place_position,
        quat_xyzw=PLACE_QUAT_XYZW,
        cartesian=False,  # movimiento libre, igual que 4A
    )
    moveit2.wait_until_executed()
    t1 = time.time()

    node.get_logger().info(f"Movimiento pick -> pre-place completado en {t1 - t0:.4f} s")

    executor.shutdown()
    thread.join(timeout=2.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
