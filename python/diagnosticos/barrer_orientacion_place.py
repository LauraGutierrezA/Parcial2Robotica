#!/usr/bin/env python3
"""
Prueba varias rotaciones de la orientacion de 'place' ALREDEDOR DE SU
PROPIO EJE DE APROXIMACION (Z local del efector) -- esto mantiene el
vector S (y por lo tanto pre-place) sin cambios, solo gira la muneca.
Busca un angulo que quede libre de colision con mesa_place.
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
PLACE_QUAT_XYZW_ORIGINAL = [0.648, 0.759, -0.036, 0.042]
ESFERA_RADIO = 0.03
GROUP_NAME = "manipulator"

# Mesa ORIGINAL (sin desplazar) -- si cambiamos orientacion, probemos
# primero contra la posicion mas simple/logica de la mesa
MESA_SIZE = [0.5, 0.4, 0.05]
MESA_BORDE_MARGEN = 0.03


def calcular_mesa(pos_referencia, mesa_size, esfera_radio, borde_margen):
    superficie_z = pos_referencia[2] - esfera_radio
    centro_z = superficie_z - mesa_size[2] / 2.0
    centro_x = pos_referencia[0] - (mesa_size[0] / 2.0 - borde_margen)
    centro_y = pos_referencia[1]
    return [centro_x, centro_y, centro_z]


def quat_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ]


def rotacion_z_local(angulo_rad):
    """Cuaternion de rotacion 'angulo_rad' alrededor del eje Z LOCAL
    (el eje de aproximacion del efector, en su propio frame)."""
    return [0.0, 0.0, np.sin(angulo_rad / 2.0), np.cos(angulo_rad / 2.0)]


def main():
    rclpy.init()
    node = Node("barrer_orientacion_place")

    pub_collision = node.create_publisher(CollisionObject, "/collision_object", 10)
    cli_ik = node.create_client(GetPositionIK, "/compute_ik")
    cli_validity = node.create_client(GetStateValidity, "/check_state_validity")
    cli_ik.wait_for_service(timeout_sec=10.0)
    cli_validity.wait_for_service(timeout_sec=10.0)

    mesa_pos = calcular_mesa(PLACE_POSITION, MESA_SIZE, ESFERA_RADIO, MESA_BORDE_MARGEN)
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
        time.sleep(0.3)
    time.sleep(0.5)
    node.get_logger().info(f"mesa_place en posicion original: {mesa_pos}")

    resultados = []
    for angulo_grados in range(0, 360, 30):
        angulo_rad = np.radians(angulo_grados)
        q_rot = rotacion_z_local(angulo_rad)
        # Rotacion LOCAL: se aplica multiplicando por la derecha
        q_nuevo = quat_mult(PLACE_QUAT_XYZW_ORIGINAL, q_rot)

        req_ik = GetPositionIK.Request()
        req_ik.ik_request.group_name = GROUP_NAME
        req_ik.ik_request.pose_stamped.header.frame_id = "base_link"
        req_ik.ik_request.pose_stamped.pose.position.x = PLACE_POSITION[0]
        req_ik.ik_request.pose_stamped.pose.position.y = PLACE_POSITION[1]
        req_ik.ik_request.pose_stamped.pose.position.z = PLACE_POSITION[2]
        (req_ik.ik_request.pose_stamped.pose.orientation.x,
         req_ik.ik_request.pose_stamped.pose.orientation.y,
         req_ik.ik_request.pose_stamped.pose.orientation.z,
         req_ik.ik_request.pose_stamped.pose.orientation.w) = q_nuevo

        future = cli_ik.call_async(req_ik)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        resultado_ik = future.result()
        if resultado_ik is None or resultado_ik.error_code.val != 1:
            node.get_logger().warn(f"angulo={angulo_grados}: IK fallo (inalcanzable)")
            continue

        req_val = GetStateValidity.Request()
        req_val.group_name = GROUP_NAME
        req_val.robot_state.joint_state = resultado_ik.solution.joint_state
        future2 = cli_validity.call_async(req_val)
        rclpy.spin_until_future_complete(node, future2, timeout_sec=10.0)
        resultado_val = future2.result()

        contactos = [f"{c.contact_body_1}<->{c.contact_body_2}" for c in resultado_val.contacts]
        node.get_logger().info(
            f"angulo={angulo_grados}°: valido={resultado_val.valid} "
            f"quat={[round(v,3) for v in q_nuevo]} {contactos if contactos else ''}"
        )
        resultados.append((angulo_grados, resultado_val.valid, q_nuevo))

    print("\n" + "=" * 70)
    print("RESUMEN:")
    for angulo, valido, q in resultados:
        print(f"  {angulo}°: {'LIBRE' if valido else 'choca'}  quat={[round(v,3) for v in q]}")
    print("=" * 70)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
