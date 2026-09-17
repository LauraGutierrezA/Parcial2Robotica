#!/usr/bin/env python3
"""
Consulta rapida: dado una posicion XYZ y un cuaternion, imprime los
angulos articulares (IK) correspondientes. Reutilizable para cualquier
punto (pre-pick, pre-place, o cualquier otro que necesites).

Uso: edita POSICION y QUAT_XYZW abajo, y corre el script.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK

GROUP_NAME = "manipulator"
BASE_LINK = "base_link"

# --- EDITA AQUI la posicion/orientacion que quieras consultar ---
POSICION = [0.2234, 0.5702, 0.3834]        # ejemplo: pre-pick
QUAT_XYZW = [0.728, -0.683, -0.013, -0.055]  # misma orientacion que 'pick'
SEMILLAS_A_PROBAR = [None, [0,0,1.5,0,0,0], [0,0,0.8,0,0,0], [0,0,0,0,0,0],
                     [0,0,-0.8,0,0,0], [0,0,-1.5,0,0,0]]


def main():
    rclpy.init()
    node = Node("obtener_angulos")
    cli_ik = node.create_client(GetPositionIK, "/compute_ik")
    cli_ik.wait_for_service(timeout_sec=10.0)

    for semilla in SEMILLAS_A_PROBAR:
        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP_NAME
        req.ik_request.pose_stamped.header.frame_id = BASE_LINK
        req.ik_request.pose_stamped.pose.position.x = float(POSICION[0])
        req.ik_request.pose_stamped.pose.position.y = float(POSICION[1])
        req.ik_request.pose_stamped.pose.position.z = float(POSICION[2])
        (req.ik_request.pose_stamped.pose.orientation.x,
         req.ik_request.pose_stamped.pose.orientation.y,
         req.ik_request.pose_stamped.pose.orientation.z,
         req.ik_request.pose_stamped.pose.orientation.w) = QUAT_XYZW

        if semilla is not None:
            req.ik_request.robot_state.joint_state.name = [
                "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"
            ]
            req.ik_request.robot_state.joint_state.position = semilla

        future = cli_ik.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        resultado = future.result()

        if resultado is not None and resultado.error_code.val == 1:
            angulos = list(resultado.solution.joint_state.position[:6])
            print(f"Semilla={semilla} -> ANGULOS (rad): {np.round(angulos, 4).tolist()}")
            print(f"ANGULOS (grados): {np.round(np.degrees(angulos), 2).tolist()}")
            break
        else:
            codigo = resultado.error_code.val if resultado else None
            print(f"Semilla={semilla} -> fallo (error_code={codigo})")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
