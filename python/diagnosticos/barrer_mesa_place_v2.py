#!/usr/bin/env python3
"""
Barre offsets Y y tamanos de mesa_place, verificando DOS condiciones:
1) 'place' queda libre de colision (check_state_validity)
2) 'place' cae DENTRO del area de la mesa (no en el borde/fuera)
"""

import time
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

MARGEN_SEGURIDAD = 0.05  # 'place' debe quedar al menos 5cm dentro del borde

# (tamano_y_mesa, offset_y) a probar -- agrandamos la mesa para poder
# desplazarla sin que 'place' quede fuera de su superficie
CONFIGURACIONES = [
    (0.4, -0.22),   # la original que probamos (referencia, sabemos que falla el margen)
    (0.6, -0.22),   # mesa mas ancha, mismo offset
    (0.6, -0.15),   # mesa mas ancha, offset mas chico
    (0.8, -0.22),   # mesa aun mas ancha
    (0.6, -0.10),
]

MESA_SIZE_X = 0.5
MESA_SIZE_Z = 0.05
SUPERFICIE_Z = PLACE_POSITION[2] - ESFERA_RADIO
CENTRO_Z = SUPERFICIE_Z - MESA_SIZE_Z / 2.0


def main():
    rclpy.init()
    node = Node("barrer_mesa_place_v2")

    pub_collision = node.create_publisher(CollisionObject, "/collision_object", 10)
    cli_ik = node.create_client(GetPositionIK, "/compute_ik")
    cli_validity = node.create_client(GetStateValidity, "/check_state_validity")
    cli_ik.wait_for_service(timeout_sec=10.0)
    cli_validity.wait_for_service(timeout_sec=10.0)

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

    resultados = []
    for mesa_size_y, offset_y in CONFIGURACIONES:
        centro_y = PLACE_POSITION[1] + offset_y

        # Verificar si 'place' queda dentro de la mesa, con margen
        borde_cercano = centro_y + mesa_size_y / 2.0
        margen_real = PLACE_POSITION[1] - borde_cercano  # negativo = dentro
        dentro_de_mesa = margen_real <= -MARGEN_SEGURIDAD

        caja = SolidPrimitive()
        caja.type = SolidPrimitive.BOX
        caja.dimensions = [MESA_SIZE_X, mesa_size_y, MESA_SIZE_Z]
        pose_caja = Pose()
        pose_caja.position.x, pose_caja.position.y, pose_caja.position.z = (
            PLACE_POSITION[0], centro_y, CENTRO_Z
        )
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

        future = cli_ik.call_async(req_ik)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        resultado_ik = future.result()
        if resultado_ik is None or resultado_ik.error_code.val != 1:
            node.get_logger().warn(f"size_y={mesa_size_y}, offset={offset_y}: IK fallo")
            continue

        req_val = GetStateValidity.Request()
        req_val.group_name = GROUP_NAME
        req_val.robot_state.joint_state = resultado_ik.solution.joint_state
        future2 = cli_validity.call_async(req_val)
        rclpy.spin_until_future_complete(node, future2, timeout_sec=10.0)
        resultado_val = future2.result()

        sin_colision = resultado_val.valid
        ok = sin_colision and dentro_de_mesa

        node.get_logger().info(
            f"size_y={mesa_size_y}, offset_y={offset_y}: "
            f"sin_colision={sin_colision}, dentro_de_mesa={dentro_de_mesa} "
            f"(margen={-margen_real*100:.1f}cm) -> {'*** OK ***' if ok else 'no sirve'}"
        )
        resultados.append((mesa_size_y, offset_y, sin_colision, dentro_de_mesa, ok))

    print("\n" + "=" * 70)
    print("RESUMEN:")
    for size_y, offset_y, sin_col, dentro, ok in resultados:
        print(f"  size_y={size_y}, offset_y={offset_y}: "
              f"sin_colision={sin_col}, dentro_de_mesa={dentro} -> {'OK' if ok else 'NO'}")
    print("=" * 70)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
