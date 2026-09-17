#!/usr/bin/env python3
"""
Barre varias posiciones XY para mesa_place (mismo tamano/altura
originales -- no cambia donde reposa la pieza), buscando una
ubicacion real donde 'place' quede libre de colision.
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

# Tamano y altura ORIGINALES (no cambiamos donde reposa la pieza)
MESA_SIZE = [0.5, 0.4, 0.05]
SUPERFICIE_Z = PLACE_POSITION[2] - ESFERA_RADIO
CENTRO_Z = SUPERFICIE_Z - MESA_SIZE[2] / 2.0

# Offsets XY a probar, relativos a place (metros)
OFFSETS_A_PROBAR = [
    (-0.22, 0.0),   # el original (para confirmar que falla)
    (0.0, 0.0),     # mesa centrada justo bajo place
    (0.22, 0.0),    # desplazada al lado opuesto
    (0.0, 0.22),    # desplazada en +Y
    (0.0, -0.22),   # desplazada en -Y
    (-0.15, 0.15),  # diagonal
]


def publicar_mesa(pub, centro_x, centro_y, centro_z):
    caja = SolidPrimitive()
    caja.type = SolidPrimitive.BOX
    caja.dimensions = MESA_SIZE
    pose_caja = Pose()
    pose_caja.position.x, pose_caja.position.y, pose_caja.position.z = centro_x, centro_y, centro_z
    pose_caja.orientation.w = 1.0

    obj = CollisionObject()
    obj.id = "mesa_place"
    obj.header.frame_id = "base_link"
    obj.primitives = [caja]
    obj.primitive_poses = [pose_caja]
    obj.operation = CollisionObject.ADD

    for _ in range(2):
        pub.publish(obj)
        time.sleep(0.3)
    time.sleep(0.5)


def main():
    rclpy.init()
    node = Node("barrer_mesa_place")

    pub_collision = node.create_publisher(CollisionObject, "/collision_object", 10)
    cli_ik = node.create_client(GetPositionIK, "/compute_ik")
    cli_validity = node.create_client(GetStateValidity, "/check_state_validity")
    cli_ik.wait_for_service(timeout_sec=10.0)
    cli_validity.wait_for_service(timeout_sec=10.0)

    req_ik_base = GetPositionIK.Request()
    req_ik_base.ik_request.group_name = GROUP_NAME
    req_ik_base.ik_request.pose_stamped.header.frame_id = "base_link"
    req_ik_base.ik_request.pose_stamped.pose.position.x = PLACE_POSITION[0]
    req_ik_base.ik_request.pose_stamped.pose.position.y = PLACE_POSITION[1]
    req_ik_base.ik_request.pose_stamped.pose.position.z = PLACE_POSITION[2]
    (req_ik_base.ik_request.pose_stamped.pose.orientation.x,
     req_ik_base.ik_request.pose_stamped.pose.orientation.y,
     req_ik_base.ik_request.pose_stamped.pose.orientation.z,
     req_ik_base.ik_request.pose_stamped.pose.orientation.w) = PLACE_QUAT_XYZW

    resultados = []
    for dx, dy in OFFSETS_A_PROBAR:
        cx, cy = PLACE_POSITION[0] + dx, PLACE_POSITION[1] + dy
        publicar_mesa(pub_collision, cx, cy, CENTRO_Z)

        future = cli_ik.call_async(req_ik_base)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        resultado_ik = future.result()

        if resultado_ik is None or resultado_ik.error_code.val != 1:
            node.get_logger().warn(f"offset=({dx},{dy}): IK fallo")
            continue

        req_val = GetStateValidity.Request()
        req_val.group_name = GROUP_NAME
        req_val.robot_state.joint_state = resultado_ik.solution.joint_state

        future2 = cli_validity.call_async(req_val)
        rclpy.spin_until_future_complete(node, future2, timeout_sec=10.0)
        resultado_val = future2.result()

        contactos = [f"{c.contact_body_1}<->{c.contact_body_2}" for c in resultado_val.contacts]
        node.get_logger().info(
            f"offset=({dx:+.2f},{dy:+.2f}) centro=({cx:.3f},{cy:.3f}): "
            f"valido={resultado_val.valid} {contactos if contactos else ''}"
        )
        resultados.append((dx, dy, resultado_val.valid))

    print("\n" + "=" * 50)
    print("RESUMEN:")
    for dx, dy, valido in resultados:
        estado = "LIBRE" if valido else "CHOCA"
        print(f"  offset ({dx:+.2f}, {dy:+.2f}): {estado}")
    print("=" * 50)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
