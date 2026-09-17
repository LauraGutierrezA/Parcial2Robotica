#!/usr/bin/env python3
"""
Llama a /compute_ik + /check_state_validity 5 veces seguidas, con
EXACTAMENTE la misma pose y escena, para verificar si KDL da
resultados distintos cada vez (no deterministico).
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
MESA_SIZE = [0.5, 0.4, 0.05]


def main():
    rclpy.init()
    node = Node("probar_repetibilidad")

    pub_collision = node.create_publisher(CollisionObject, "/collision_object", 10)
    cli_ik = node.create_client(GetPositionIK, "/compute_ik")
    cli_validity = node.create_client(GetStateValidity, "/check_state_validity")
    cli_ik.wait_for_service(timeout_sec=10.0)
    cli_validity.wait_for_service(timeout_sec=10.0)

    centro_y = PLACE_POSITION[1] - 0.22
    centro_z = PLACE_POSITION[2] - ESFERA_RADIO - MESA_SIZE[2] / 2.0

    caja = SolidPrimitive()
    caja.type = SolidPrimitive.BOX
    caja.dimensions = MESA_SIZE
    pose_caja = Pose()
    pose_caja.position.x, pose_caja.position.y, pose_caja.position.z = (
        PLACE_POSITION[0], centro_y, centro_z
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
    for intento in range(8):
        future = cli_ik.call_async(req_ik)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        resultado_ik = future.result()
        if resultado_ik is None or resultado_ik.error_code.val != 1:
            node.get_logger().warn(f"intento {intento}: IK fallo")
            continue

        req_val = GetStateValidity.Request()
        req_val.group_name = GROUP_NAME
        req_val.robot_state.joint_state = resultado_ik.solution.joint_state
        future2 = cli_validity.call_async(req_val)
        rclpy.spin_until_future_complete(node, future2, timeout_sec=10.0)
        resultado_val = future2.result()

        juntas = [round(p, 3) for p in resultado_ik.solution.joint_state.position[:6]]
        node.get_logger().info(f"intento {intento}: valido={resultado_val.valid}, juntas={juntas}")
        resultados.append(resultado_val.valid)

    print("\n" + "=" * 50)
    print(f"Resultados de los 8 intentos: {resultados}")
    print(f"¿Todos iguales? {'SI' if len(set(resultados)) == 1 else 'NO -- IK es no-deterministico'}")
    print("=" * 50)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
