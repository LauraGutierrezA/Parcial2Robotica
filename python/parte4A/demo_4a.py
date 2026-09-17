#!/usr/bin/env python3
"""
Parte 4A - Version DEMO (para el ciclo animado fluido).
Sin comparacion de planeadores (eso ya se hizo y quedo documentado en
comparar_planeadores.py) -- aqui solo se planea y ejecuta UNA vez con
el ganador ya decidido (RRTConnect), para evitar animaciones dobles
en RViz durante la grabacion del ciclo completo.
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

from pymoveit2 import MoveIt2
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject


PICK_POSITION = [0.229, 0.580, 0.284]
PICK_QUAT_XYZW = [0.728, -0.683, -0.013, -0.055]
PICK_S = np.array([0.056, 0.098, -0.994])

PLACE_POSITION = [0.093, -0.588, 0.631]
PLACE_QUAT_XYZW = [0.078, -0.995, 0.055, -0.004]

D_RETROCESO = 0.1

OBSTACULO_POSICION = [0.38, 0.48, 0.17]
OBSTACULO_RADIO = 0.1
OBSTACULO_ALTURA = 0.35

MESA_SIZE = [0.5, 0.4, 0.05]
MESA_BORDE_MARGEN = 0.03
ESFERA_RADIO = 0.03

GANADOR = "RRTConnectkConfigDefault"


def calcular_mesa(pos_referencia, mesa_size, esfera_radio, borde_margen):
    superficie_z = pos_referencia[2] - esfera_radio
    centro_z = superficie_z - mesa_size[2] / 2.0
    centro_x = pos_referencia[0] - (mesa_size[0] / 2.0 - borde_margen)
    centro_y = pos_referencia[1]
    return [centro_x, centro_y, centro_z]


def calcular_pre_pick(pick_pos, S_vector, d):
    S_normalizado = S_vector / np.linalg.norm(S_vector)
    return (np.array(pick_pos) - d * S_normalizado).tolist()


def main():
    rclpy.init()
    node = Node("kr6_demo_4a")
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

    # --- Por si la corrida anterior quedo a medias (pieza aun adjunta) ---
    pub_attach = node.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
    aco_reset = AttachedCollisionObject()
    aco_reset.object.id = "pieza_esfera"
    aco_reset.link_name = "tool0"
    aco_reset.object.operation = CollisionObject.REMOVE
    pub_attach.publish(aco_reset)
    time.sleep(0.5)

    # --- SIEMPRE empezar desde 'home', sin importar donde quedo el robot ---
    node.get_logger().info("Moviendo a 'home'...")
    moveit2.move_to_pose(
        position=[0.815, 0.000, 0.425],
        quat_xyzw=[0.000, 0.707, -0.000, 0.707],
        cartesian=False,
    )
    moveit2.wait_until_executed()
    node.get_logger().info("Listo: robot en home.")

    for _ in range(2):
        moveit2.add_collision_cylinder(
            id="obstaculo_poste", position=OBSTACULO_POSICION,
            quat_xyzw=[0.0, 0.0, 0.0, 1.0], height=OBSTACULO_ALTURA, radius=OBSTACULO_RADIO,
        )
        time.sleep(0.5)

    mesa_pick_pos = calcular_mesa(PICK_POSITION, MESA_SIZE, ESFERA_RADIO, MESA_BORDE_MARGEN)
    for _ in range(2):
        moveit2.add_collision_box(
            id="mesa_pick", position=mesa_pick_pos,
            quat_xyzw=[0.0, 0.0, 0.0, 1.0], size=MESA_SIZE,
        )
        time.sleep(0.5)

    mesa_place_pos = calcular_mesa(PLACE_POSITION, MESA_SIZE, ESFERA_RADIO, MESA_BORDE_MARGEN)
    for _ in range(2):
        moveit2.add_collision_box(
            id="mesa_place", position=mesa_place_pos,
            quat_xyzw=[0.0, 0.0, 0.0, 1.0], size=MESA_SIZE,
        )
        time.sleep(0.5)

    for _ in range(2):
        moveit2.add_collision_sphere(id="pieza_esfera", position=PICK_POSITION, radius=ESFERA_RADIO)
        time.sleep(0.5)

    time.sleep(1.0)

    pre_pick_position = calcular_pre_pick(PICK_POSITION, PICK_S, D_RETROCESO)
    node.get_logger().info(f"pre-pick: {pre_pick_position}")

    node.get_logger().info(f"Moviendo home -> pre-pick con {GANADOR}...")
    moveit2.planner_id = GANADOR
    moveit2.move_to_pose(position=pre_pick_position, quat_xyzw=PICK_QUAT_XYZW, cartesian=False)
    moveit2.wait_until_executed()
    node.get_logger().info("Listo: robot en pre-pick.")

    executor.shutdown()
    thread.join(timeout=2.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
