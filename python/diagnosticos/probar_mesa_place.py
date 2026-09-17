#!/usr/bin/env python3
"""
Prueba RAPIDA: define una mesa_place mas chica y menos desplazada,
y verifica con /check_state_validity si 'place' queda libre de
colision -- sin correr el ciclo completo cada vez.
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node

from moveit_msgs.srv import GetStateValidity, GetPositionIK
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


PLACE_POSITION = [0.093, -0.588, 0.631]
PLACE_QUAT_XYZW = [0.648, 0.759, -0.036, 0.042]
ESFERA_RADIO = 0.03
GROUP_NAME = "manipulator"

# --- Nueva geometria de mesa_place a probar (mas chica, menos desplazada) ---
MESA_SIZE = [0.25, 0.25, 0.03]   # antes: [0.5, 0.4, 0.05]
MESA_BORDE_MARGEN = 0.05          # antes: 0.03 (con tabla mas chica, desplaza menos en absoluto)
HOLGURA_VERTICAL_EXTRA = 0.05     # <-- NUEVO: espacio extra entre pieza y mesa (m)


def calcular_mesa(pos_referencia, mesa_size, esfera_radio, borde_margen, holgura_extra=0.0):
    superficie_z = pos_referencia[2] - esfera_radio - holgura_extra
    centro_z = superficie_z - mesa_size[2] / 2.0
    centro_x = pos_referencia[0] - (mesa_size[0] / 2.0 - borde_margen)
    centro_y = pos_referencia[1]
    return [centro_x, centro_y, centro_z]


def main():
    rclpy.init()
    node = Node("probar_mesa_place")

    pub_collision = node.create_publisher(CollisionObject, "/collision_object", 10)
    cli_ik = node.create_client(GetPositionIK, "/compute_ik")
    cli_validity = node.create_client(GetStateValidity, "/check_state_validity")
    cli_ik.wait_for_service(timeout_sec=10.0)
    cli_validity.wait_for_service(timeout_sec=10.0)

    mesa_pos = calcular_mesa(PLACE_POSITION, MESA_SIZE, ESFERA_RADIO, MESA_BORDE_MARGEN, HOLGURA_VERTICAL_EXTRA)
    node.get_logger().info(f"Probando mesa_place: size={MESA_SIZE}, pos={mesa_pos}")

    # Republicar mesa_place con la nueva geometria (reemplaza la anterior, mismo id)
    caja = SolidPrimitive()
    caja.type = SolidPrimitive.BOX
    caja.dimensions = MESA_SIZE
    pose_caja = Pose()
    pose_caja.position.x, pose_caja.position.y, pose_caja.position.z = mesa_pos
    pose_caja.orientation.w = 1.0

    obj = CollisionObject()
    obj.id = "mesa_place"
    obj.header.frame_id = "base_link"
    obj.primitives = [caja]
    obj.primitive_poses = [pose_caja]
    obj.operation = CollisionObject.ADD

    for _ in range(2):
        pub_collision.publish(obj)
        time.sleep(0.5)
    time.sleep(1.0)

    # IK para 'place'
    req_ik = GetPositionIK.Request()
    req_ik.ik_request.group_name = GROUP_NAME
    req_ik.ik_request.pose_stamped.header.frame_id = "base_link"
    req_ik.ik_request.pose_stamped.pose.position.x = PLACE_POSITION[0]
    req_ik.ik_request.pose_stamped.pose.position.y = PLACE_POSITION[1]
    req_ik.ik_request.pose_stamped.pose.position.z = PLACE_POSITION[2]
    (req_ik.ik_request.pose_stamped.pose.orientation.x,
     req_ik.ik_request.pose_stamped.pose.orientation.y,
     req_ik.ik_request.pose_stamped.pose.orientation.z,
     req_ik.ik_request.pose_stamped.pose.orientation.w) = PLACE_QUAT_XYZW

    future = cli_ik.call_async(req_ik)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    resultado_ik = future.result()

    if resultado_ik is None or resultado_ik.error_code.val != 1:
        node.get_logger().error("IK para 'place' fallo.")
        rclpy.shutdown()
        return

    req_val = GetStateValidity.Request()
    req_val.group_name = GROUP_NAME
    req_val.robot_state.joint_state = resultado_ik.solution.joint_state

    future2 = cli_validity.call_async(req_val)
    rclpy.spin_until_future_complete(node, future2, timeout_sec=10.0)
    resultado_val = future2.result()

    node.get_logger().info(f"'place' con nueva mesa: valido = {resultado_val.valid}")
    for c in resultado_val.contacts:
        node.get_logger().info(f"  CONTACTO: '{c.contact_body_1}' <-> '{c.contact_body_2}'")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
