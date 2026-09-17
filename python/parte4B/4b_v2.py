#!/usr/bin/env python3
"""
Parte 4B - Acercamiento fino pre-pick -> pick. CON ATTACH.
(Restaurado desde la version que confirmamos que funciona.)
"""

import time
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


PICK_POSITION = np.array([0.229, 0.580, 0.284])
PICK_QUAT_XYZW = [0.728, -0.683, -0.013, -0.055]
PICK_S = np.array([0.056, 0.098, -0.994])
D_RETROCESO = 0.1
N_INTERMEDIOS = 3
VMAX_ROJO = 0.200
AMAX_ROJO = 0.300
ESFERA_RADIO = 0.03
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GROUP_NAME = "manipulator"
LINK_NAME = "tool0"
CONTROLLER_ACTION = "/manipulator_controller/follow_joint_trajectory"


def calcular_pre_pick(pick_pos, S_vector, d):
    S_normalizado = S_vector / np.linalg.norm(S_vector)
    return pick_pos - d * S_normalizado


PRE_PICK_POSITION = calcular_pre_pick(PICK_POSITION, PICK_S, D_RETROCESO)


def tiempo_total_cubico(d, vmax, amax):
    return max(1.5 * d / vmax, np.sqrt(6.0 * d / amax))


def tiempo_total_quintico(d, vmax, amax):
    return max(1.875 * d / vmax, np.sqrt((10.0 / np.sqrt(3.0)) * d / amax))


def perfil_cubico(d, T, t):
    u = np.clip(t / T, 0.0, 1.0)
    pos = d * (3 * u**2 - 2 * u**3)
    return pos


def perfil_quintico(d, T, t):
    u = np.clip(t / T, 0.0, 1.0)
    pos = d * (10 * u**3 - 15 * u**4 + 6 * u**5)
    return pos


def generar_waypoints_y_tiempos(pre_pick, pick, tiempo_fn, perfil_fn, vmax, amax, n_intermedios):
    d = float(np.linalg.norm(pick - pre_pick))
    T = tiempo_fn(d, vmax, amax)
    t_intermedios = np.linspace(0, T, n_intermedios + 2)[1:-1]
    t_puntos = np.append(t_intermedios, T)
    s_puntos = perfil_fn(d, T, t_puntos)
    direccion = (pick - pre_pick) / d
    posiciones_xyz = [pre_pick + s * direccion for s in s_puntos]
    return t_puntos, posiciones_xyz, T, d


class Ejecutor4B(Node):
    def __init__(self):
        super().__init__("kr6_perfil_4b_v2")
        self.cli_cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.cli_fk = self.create_client(GetPositionFK, "/compute_fk")
        self.action_client = ActionClient(self, FollowJointTrajectory, CONTROLLER_ACTION)
        self.pub_attach = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)

    def esperar_servicios(self):
        self.cli_cartesian.wait_for_service(timeout_sec=10.0)
        self.cli_fk.wait_for_service(timeout_sec=10.0)
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
            time.sleep(0.5)
        time.sleep(0.5)

    def calcular_trayectoria_cartesiana(self, poses, max_step):
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.group_name = GROUP_NAME
        req.link_name = LINK_NAME
        req.waypoints = poses
        req.max_step = max_step
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        future = self.cli_cartesian.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        return future.result()

    def obtener_posicion_fk(self, joint_positions):
        req = GetPositionFK.Request()
        req.header = Header(frame_id="base_link")
        req.fk_link_names = [LINK_NAME]
        req.robot_state.joint_state.name = JOINT_NAMES
        req.robot_state.joint_state.position = list(joint_positions)
        future = self.cli_fk.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        resultado = future.result()
        if resultado is None or len(resultado.pose_stamped) == 0:
            return None
        p = resultado.pose_stamped[0].pose.position
        return np.array([p.x, p.y, p.z])

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

    def mover_a_pre_pick(self):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = PRE_PICK_POSITION.tolist()
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = PICK_QUAT_XYZW
        resultado = self.calcular_trayectoria_cartesiana([pose], max_step=0.01)
        if resultado is None or resultado.fraction < 0.99:
            self.get_logger().error("No se pudo llegar a pre-pick.")
            return False
        self.enviar_trayectoria(resultado.solution.joint_trajectory)
        return True


def evaluar_perfil_real(node, nombre, tiempo_fn, perfil_fn, ejecutar=True):
    t_puntos, posiciones_xyz, T, d = generar_waypoints_y_tiempos(
        PRE_PICK_POSITION, PICK_POSITION, tiempo_fn, perfil_fn,
        VMAX_ROJO, AMAX_ROJO, N_INTERMEDIOS,
    )
    node.get_logger().info(f"--- {nombre}: T={T:.4f}s, d={d:.4f}m ---")

    poses = []
    for p in posiciones_xyz:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = p.tolist()
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = PICK_QUAT_XYZW
        poses.append(pose)

    resultado = node.calcular_trayectoria_cartesiana(poses, max_step=d)
    if resultado is None or resultado.fraction < 0.99:
        frac = resultado.fraction if resultado else None
        node.get_logger().error(f"{nombre}: computeCartesianPath incompleto (fraction={frac})")
        return None

    if ejecutar:
        node.get_logger().info(f"Ejecutando perfil {nombre}...")
        node.enviar_trayectoria(resultado.solution.joint_trajectory)

    # --- Reparametrizar el tiempo correctamente antes de guardar ---
    # computeCartesianPath NO respeta la ley de tiempos quintica que
    # calculamos -- reasigna sus propios tiempos (AddTimeOptimalParameterization).
    # Hay que recuperar, para cada punto, la distancia REAL recorrida (via FK)
    # y de ahi invertir la ley quintica para saber el tiempo CORRECTO.
    puntos = resultado.solution.joint_trajectory.points
    posiciones_articulares = [list(p.positions) for p in puntos]

    posiciones_cartesianas = []
    for pos_art in posiciones_articulares:
        pos_fk = node.obtener_posicion_fk(pos_art)
        if pos_fk is None:
            node.get_logger().error(f"{nombre}: fallo el FK al reparametrizar.")
            return {"T": T}
        posiciones_cartesianas.append(pos_fk)

    distancias = np.array([
        np.linalg.norm(pc - PRE_PICK_POSITION) for pc in posiciones_cartesianas
    ])
    t_tabla = np.linspace(0, T, 500)
    pos_tabla = perfil_fn(d, T, t_tabla)
    tiempos_correctos = np.interp(distancias, pos_tabla, t_tabla)
    tiempos_correctos[0] = 0.0
    tiempos_correctos[-1] = T

    import json
    datos = {
        "tiempos": tiempos_correctos.tolist(),
        "posiciones": posiciones_articulares,
    }
    with open("trayectoria_4b.json", "w") as f:
        json.dump(datos, f)
    node.get_logger().info("Trayectoria guardada (tiempos reparametrizados) en trayectoria_4b.json.")

    return {"T": T}


def main():
    rclpy.init()
    node = Ejecutor4B()
    node.esperar_servicios()

    node.get_logger().info(
        "NOTA: este script asume que el robot YA esta en pre-pick "
        "(corre primero comparar_planeadores.py)."
    )

    node.adjuntar_pieza()

    r_quintico = evaluar_perfil_real(node, "QUINTICO", tiempo_total_quintico, perfil_quintico, ejecutar=True)

    if r_quintico:
        print(f"\nTiempo total: {r_quintico['T']:.4f} s")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
