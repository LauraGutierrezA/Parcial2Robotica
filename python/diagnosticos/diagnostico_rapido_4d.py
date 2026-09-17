#!/usr/bin/env python3
"""Diagnostico rapido: que esta chocando ahora con la nueva orientacion."""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity, GetPositionIK

PLACE_POSITION = [0.093, -0.588, 0.631]
PLACE_QUAT_XYZW = [0.078, -0.995, 0.055, -0.004]
PRE_PLACE_POSITION = [0.0913, -0.577, 0.7304]  # aprox, S casi igual que antes
GROUP_NAME = "manipulator"


def diagnosticar(node, cli_ik, cli_validity, position, etiqueta):
    req_ik = GetPositionIK.Request()
    req_ik.ik_request.group_name = GROUP_NAME
    req_ik.ik_request.pose_stamped.header.frame_id = "base_link"
    req_ik.ik_request.pose_stamped.pose.position.x = position[0]
    req_ik.ik_request.pose_stamped.pose.position.y = position[1]
    req_ik.ik_request.pose_stamped.pose.position.z = position[2]
    (req_ik.ik_request.pose_stamped.pose.orientation.x,
     req_ik.ik_request.pose_stamped.pose.orientation.y,
     req_ik.ik_request.pose_stamped.pose.orientation.z,
     req_ik.ik_request.pose_stamped.pose.orientation.w) = PLACE_QUAT_XYZW

    future = cli_ik.call_async(req_ik)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    resultado_ik = future.result()
    if resultado_ik is None or resultado_ik.error_code.val != 1:
        node.get_logger().error(f"[{etiqueta}] IK fallo")
        return

    req_val = GetStateValidity.Request()
    req_val.group_name = GROUP_NAME
    req_val.robot_state.joint_state = resultado_ik.solution.joint_state
    future2 = cli_validity.call_async(req_val)
    rclpy.spin_until_future_complete(node, future2, timeout_sec=10.0)
    resultado_val = future2.result()

    node.get_logger().info(f"[{etiqueta}] valido: {resultado_val.valid}")
    for c in resultado_val.contacts:
        node.get_logger().info(f"  CONTACTO [{etiqueta}]: '{c.contact_body_1}' <-> '{c.contact_body_2}'")


def main():
    rclpy.init()
    node = Node("diagnostico_rapido")
    cli_ik = node.create_client(GetPositionIK, "/compute_ik")
    cli_validity = node.create_client(GetStateValidity, "/check_state_validity")
    cli_ik.wait_for_service(timeout_sec=10.0)
    cli_validity.wait_for_service(timeout_sec=10.0)

    diagnosticar(node, cli_ik, cli_validity, PRE_PLACE_POSITION, "pre-place")
    punto_medio = [(a+b)/2 for a, b in zip(PRE_PLACE_POSITION, PLACE_POSITION)]
    diagnosticar(node, cli_ik, cli_validity, punto_medio, "punto_medio")
    diagnosticar(node, cli_ik, cli_validity, PLACE_POSITION, "place")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
