#!/usr/bin/env python3
"""
PRUEBA DE DESCARTE - version con pre-pick HARD-CODED (no calculado)
y colisiones ACTIVAS (avoid_collisions=True), para verificar si el
problema de fraction=0.0 tiene algo que ver con como se calcula
pre-pick, o es completamente independiente de eso.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclDuration

from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import time as _time


# ============================================================
#  PARAMETROS -- pre-pick HARD-CODED a proposito para esta prueba
# ============================================================
PRE_PICK_POSITION = np.array([0.2234, 0.5702, 0.3834])  # <-- HARD-CODED
PICK_POSITION = np.array([0.229, 0.580, 0.284])
PICK_QUAT_XYZW = [0.728, -0.683, -0.013, -0.055]

N_INTERMEDIOS = 3
VMAX_ROJO = 0.200
AMAX_ROJO = 0.300
ESFERA_RADIO = 0.03

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GROUP_NAME = "manipulator"
LINK_NAME = "tool0"
CONTROLLER_ACTION = "/manipulator_controller/follow_joint_trajectory"
# ============================================================


def tiempo_total_cubico(d, vmax, amax):
    return max(1.5 * d / vmax, np.sqrt(6.0 * d / amax))


def perfil_cubico(d, T, t):
    u = np.clip(t / T, 0.0, 1.0)
    pos = d * (3 * u**2 - 2 * u**3)
    vel = (d / T) * (6 * u - 6 * u**2)
    acc = (d / T**2) * (6 - 12 * u)
    return pos, vel, acc


def generar_waypoints_y_tiempos(pre_pick, pick, tiempo_fn, perfil_fn, vmax, amax, n_intermedios):
    d = float(np.linalg.norm(pick - pre_pick))
    T = tiempo_fn(d, vmax, amax)
    t_intermedios = np.linspace(0, T, n_intermedios + 2)[1:-1]
    t_puntos = np.append(t_intermedios, T)
    s_puntos, _, _ = perfil_fn(d, T, t_puntos)
    direccion = (pick - pre_pick) / d
    posiciones_xyz = [pre_pick + s * direccion for s in s_puntos]
    return t_puntos, posiciones_xyz, T, d


class PruebaHardcode(Node):
    def __init__(self):
        super().__init__("kr6_prueba_hardcode")
        self.cli_cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.action_client = ActionClient(self, FollowJointTrajectory, CONTROLLER_ACTION)
        self.pub_attach = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)

    def esperar_servicios(self):
        self.cli_cartesian.wait_for_service(timeout_sec=10.0)
        self.action_client.wait_for_server(timeout_sec=10.0)

    def adjuntar_pieza(self):
        self.get_logger().info("Adjuntando 'pieza_esfera' a tool0...")
        esfera = SolidPrimitive()
        esfera.type = SolidPrimitive.SPHERE
        esfera.dimensions = [ESFERA_RADIO]

        pose_relativa = Pose()
        pose_relativa.orientation.w = 1.0

        aco = AttachedCollisionObject()
        aco.link_name = LINK_NAME
        aco.object.id = "pieza_esfera"
        aco.object.header.frame_id = LINK_NAME
        aco.object.primitives = [esfera]
        aco.object.primitive_poses = [pose_relativa]
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = [LINK_NAME, "link_6", "flange", "link_5"]

        for _ in range(2):
            self.pub_attach.publish(aco)
            _time.sleep(0.5)
        _time.sleep(0.5)

    def calcular_trayectoria_cartesiana(self, poses, max_step):
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.group_name = GROUP_NAME
        req.link_name = LINK_NAME
        req.waypoints = poses
        req.max_step = max_step
        req.jump_threshold = 0.0
        req.avoid_collisions = True  # <-- COLISIONES ACTIVAS para esta prueba
        future = self.cli_cartesian.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        return future.result()

    def enviar_trayectoria(self, joint_traj):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("El controlador rechazo la trayectoria.")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        return True


def main():
    print(f"pre-pick (HARD-CODED): {PRE_PICK_POSITION.tolist()}")

    rclpy.init()
    node = PruebaHardcode()
    node.esperar_servicios()
    node.adjuntar_pieza()

    t_puntos, posiciones_xyz, T, d = generar_waypoints_y_tiempos(
        PRE_PICK_POSITION, PICK_POSITION, tiempo_total_cubico, perfil_cubico,
        VMAX_ROJO, AMAX_ROJO, N_INTERMEDIOS,
    )
    node.get_logger().info(f"T={T:.4f}s, d={d:.4f}m")

    poses = []
    for p in posiciones_xyz:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = p.tolist()
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = PICK_QUAT_XYZW
        poses.append(pose)

    resultado = node.calcular_trayectoria_cartesiana(poses, max_step=d)

    if resultado is None:
        node.get_logger().error("El servicio no respondio.")
    else:
        node.get_logger().info(
            f"RESULTADO: fraction={resultado.fraction}, "
            f"error_code={resultado.error_code.val}, "
            f"n_puntos={len(resultado.solution.joint_trajectory.points)}"
        )
        if resultado.fraction >= 0.99:
            node.get_logger().info("Ejecutando trayectoria (colisiones activas)...")
            node.enviar_trayectoria(resultado.solution.joint_trajectory)
            node.get_logger().info("Listo.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
