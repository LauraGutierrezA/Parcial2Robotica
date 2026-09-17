#!/usr/bin/env python3
"""
Parte 4D - Acercamiento fino pre-place -> place (deposito). CON ATTACH.

Estrategia: en vez de dejar que /compute_cartesian_path elija la
semilla (que parece sesgarse siempre a la misma rama de codo), se
calcula el camino punto por punto con /compute_ik, probando
EXPLICITAMENTE varias semillas de codo (distintos valores iniciales
de joint_3) combinadas con varias orientaciones (inclinacion + giro),
validando cada punto con /check_state_validity (incluyendo
correctamente la pieza adjunta) antes de ejecutar nada.
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclDuration

from moveit_msgs.srv import GetPositionIK, GetStateValidity, GetPlanningScene
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
from pymoveit2 import MoveIt2


PLACE_POSITION = np.array([0.093, -0.588, 0.631])
PLACE_QUAT_XYZW_ORIGINAL = [0.648, 0.759, -0.036, 0.042]
PLACE_S = np.array([0.017, -0.110, -0.994])
D_RETROCESO = 0.1
N_INTERMEDIOS = 3
VMAX_AZUL = 0.100
AMAX_AZUL = 0.020
ESFERA_RADIO = 0.03

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GROUP_NAME = "manipulator"
LINK_NAME = "tool0"
CONTROLLER_ACTION = "/manipulator_controller/follow_joint_trajectory"

# Semillas de codo a probar para joint_3 (rad) -- cubre un rango amplio
# para forzar ramas de solucion distintas (codo arriba / abajo)
SEMILLAS_JOINT3 = [1.5, 0.8, 0.0, -0.8, -1.5]


def calcular_pre_place(place_pos, S_vector, d):
    S_normalizado = S_vector / np.linalg.norm(S_vector)
    return place_pos - d * S_normalizado


PRE_PLACE_POSITION = calcular_pre_place(PLACE_POSITION, PLACE_S, D_RETROCESO)


def quat_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ]


def rotacion_z_local(a):
    return [0.0, 0.0, np.sin(a/2), np.cos(a/2)]


def rotacion_x_local(a):
    return [np.sin(a/2), 0.0, 0.0, np.cos(a/2)]


def generar_candidatos_orientacion():
    candidatos = []
    for inclinacion in [-20, 0, 20]:
        base = quat_mult(PLACE_QUAT_XYZW_ORIGINAL, rotacion_x_local(np.radians(inclinacion)))
        for giro in [0, 90, 180, 270]:
            q = quat_mult(base, rotacion_z_local(np.radians(giro)))
            candidatos.append(((inclinacion, giro), q))
    return candidatos


def tiempo_total_quintico(d, vmax, amax):
    return max(1.875 * d / vmax, np.sqrt((10.0 / np.sqrt(3.0)) * d / amax))


def perfil_quintico(d, T, t):
    u = np.clip(t / T, 0.0, 1.0)
    return d * (10 * u**3 - 15 * u**4 + 6 * u**5)


def generar_waypoints_y_tiempos(pre_place, place, tiempo_fn, perfil_fn, vmax, amax, n_intermedios):
    d = float(np.linalg.norm(place - pre_place))
    T = tiempo_fn(d, vmax, amax)
    t_intermedios = np.linspace(0, T, n_intermedios + 2)[1:-1]
    t_puntos = np.append(t_intermedios, T)
    s_puntos = perfil_fn(d, T, t_puntos)
    direccion = (place - pre_place) / d
    posiciones_xyz = [pre_place + s * direccion for s in s_puntos]
    return t_puntos, posiciones_xyz, T, d


class Ejecutor4D(Node):
    def __init__(self):
        super().__init__("kr6_acercamiento_4d")
        self.cli_ik = self.create_client(GetPositionIK, "/compute_ik")
        self.cli_validity = self.create_client(GetStateValidity, "/check_state_validity")
        self.cli_scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.action_client = ActionClient(self, FollowJointTrajectory, CONTROLLER_ACTION)
        self.pub_attach = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
        self.pub_collision_object = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.ultimo_joint_state = None
        self.create_subscription(JointState, "/joint_states", self._callback_joint_state, 10)

    def _callback_joint_state(self, msg):
        self.ultimo_joint_state = msg

    def esperar_servicios(self):
        self.cli_ik.wait_for_service(timeout_sec=10.0)
        self.cli_validity.wait_for_service(timeout_sec=10.0)
        self.cli_scene.wait_for_service(timeout_sec=10.0)
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

    def depositar_pieza(self):
        self.get_logger().info("Depositando 'pieza_esfera' en 'place'...")
        aco = AttachedCollisionObject()
        aco.object.id = "pieza_esfera"
        aco.link_name = LINK_NAME
        aco.object.operation = CollisionObject.REMOVE
        for _ in range(2):
            self.pub_attach.publish(aco)
            time.sleep(0.5)
        time.sleep(0.5)

        esfera = SolidPrimitive()
        esfera.type = SolidPrimitive.SPHERE
        esfera.dimensions = [ESFERA_RADIO]
        pose_mundo = Pose()
        pose_mundo.position.x, pose_mundo.position.y, pose_mundo.position.z = PLACE_POSITION.tolist()
        pose_mundo.orientation.w = 1.0
        obj = CollisionObject()
        obj.id = "pieza_esfera"
        obj.header.frame_id = "base_link"
        obj.primitives = [esfera]
        obj.primitive_poses = [pose_mundo]
        obj.operation = CollisionObject.ADD
        self.pub_collision_object.publish(obj)
        time.sleep(0.5)

    def obtener_escena_con_adjuntos(self):
        """Escena viva CON los objetos adjuntos incluidos correctamente."""
        req = GetPlanningScene.Request()
        req.components.components = 1 | 2 | 64  # ROBOT_STATE + ATTACHED_OBJECTS + ACM
        future = self.cli_scene.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result().scene.robot_state

    def ik_con_semilla(self, position, quat_xyzw, semilla_joint3=None, semilla_completa=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP_NAME
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose.position.x = float(position[0])
        req.ik_request.pose_stamped.pose.position.y = float(position[1])
        req.ik_request.pose_stamped.pose.position.z = float(position[2])
        (req.ik_request.pose_stamped.pose.orientation.x,
         req.ik_request.pose_stamped.pose.orientation.y,
         req.ik_request.pose_stamped.pose.orientation.z,
         req.ik_request.pose_stamped.pose.orientation.w) = quat_xyzw

        if semilla_completa is not None:
            req.ik_request.robot_state.joint_state.name = JOINT_NAMES
            req.ik_request.robot_state.joint_state.position = list(semilla_completa)
        elif semilla_joint3 is not None:
            seed = [0.0, 0.0, semilla_joint3, 0.0, 0.0, 0.0]
            req.ik_request.robot_state.joint_state.name = JOINT_NAMES
            req.ik_request.robot_state.joint_state.position = seed

        future = self.cli_ik.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        resultado = future.result()
        if resultado is None or resultado.error_code.val != 1:
            return None
        return list(resultado.solution.joint_state.position[:6])

    def es_valido(self, joint_positions):
        robot_state = self.obtener_escena_con_adjuntos()
        robot_state.joint_state.name = JOINT_NAMES
        robot_state.joint_state.position = list(joint_positions)
        req = GetStateValidity.Request()
        req.group_name = GROUP_NAME
        req.robot_state = robot_state
        future = self.cli_validity.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        resultado = future.result()
        return resultado.valid, resultado.contacts

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


def intentar_camino_completo(node, posiciones_xyz, t_puntos, quat, semilla_joint3):
    """Calcula TODO el camino via IK secuencial (semilla explicita en el
    primer punto, continuidad despues), validando cada punto (con la
    pieza adjunta correctamente incluida). Devuelve la lista de
    soluciones articulares si TODO el camino es valido, si no None."""
    soluciones = []
    semilla_actual = None
    for i, p in enumerate(posiciones_xyz):
        if i == 0:
            sol = node.ik_con_semilla(p, quat, semilla_joint3=semilla_joint3)
        else:
            sol = node.ik_con_semilla(p, quat, semilla_completa=semilla_actual)

        if sol is None:
            return None

        valido, contactos = node.es_valido(sol)
        if not valido:
            return None

        soluciones.append(sol)
        semilla_actual = sol

    return soluciones


def main():
    rclpy.init()
    node = Ejecutor4D()
    node.esperar_servicios()

    node.adjuntar_pieza()

    t_puntos, posiciones_xyz, T, d = generar_waypoints_y_tiempos(
        PRE_PLACE_POSITION, PLACE_POSITION, tiempo_total_quintico, perfil_quintico,
        VMAX_AZUL, AMAX_AZUL, N_INTERMEDIOS,
    )
    node.get_logger().info(f"QUINTICO: T={T:.4f}s, d={d:.4f}m")

    candidatos_orientacion = generar_candidatos_orientacion()
    node.get_logger().info(
        f"Probando {len(candidatos_orientacion)} orientaciones x "
        f"{len(SEMILLAS_JOINT3)} semillas de codo = "
        f"{len(candidatos_orientacion) * len(SEMILLAS_JOINT3)} combinaciones..."
    )

    soluciones_ganadoras = None
    combo_ganadora = None

    for etiqueta, quat in candidatos_orientacion:
        for semilla in SEMILLAS_JOINT3:
            soluciones = intentar_camino_completo(node, posiciones_xyz, t_puntos, quat, semilla)
            if soluciones is not None:
                node.get_logger().info(
                    f"*** FUNCIONA: inclinacion={etiqueta[0]}, giro={etiqueta[1]}, "
                    f"semilla_joint3={semilla} ***"
                )
                soluciones_ganadoras = soluciones
                combo_ganadora = (etiqueta, quat, semilla)
                break
            else:
                node.get_logger().info(
                    f"  inclinacion={etiqueta[0]}, giro={etiqueta[1]}, semilla={semilla}: no valido"
                )
        if soluciones_ganadoras is not None:
            break

    if soluciones_ganadoras is None:
        node.get_logger().error(
            "NINGUNA combinacion de orientacion + semilla de codo funciono. "
            "El problema no se resuelve solo variando orientacion/codo."
        )
        rclpy.shutdown()
        return

    print(f"\n>>> GANADORA: {combo_ganadora[0]}, semilla_joint3={combo_ganadora[2]}\n")

    # Construir y ejecutar la trayectoria con las soluciones validadas
    traj = JointTrajectoryPoint
    from trajectory_msgs.msg import JointTrajectory
    joint_traj = JointTrajectory()
    joint_traj.joint_names = JOINT_NAMES
    for i, sol in enumerate(soluciones_ganadoras):
        pt = JointTrajectoryPoint()
        pt.positions = sol
        pt.time_from_start = RclDuration(seconds=float(t_puntos[i])).to_msg()
        joint_traj.points.append(pt)

    node.get_logger().info("Ejecutando trayectoria validada hacia 'place'...")
    node.enviar_trayectoria(joint_traj)
    node.get_logger().info("Llegada a 'place'. Soltando la pieza...")
    node.depositar_pieza()
    node.get_logger().info("Ciclo pick-and-place completo.")

    # --- Guardar la trayectoria real para la verificacion del Jacobiano (Parte 5) ---
    # Se agrega tambien el punto inicial (pre-place, t=0), que la busqueda
    # de candidatos no incluye (solo itera sobre los puntos intermedios+final).
    sol_inicial = node.ik_con_semilla(PRE_PLACE_POSITION, combo_ganadora[1],
                                        semilla_joint3=combo_ganadora[2])
    import json
    if sol_inicial is not None:
        tiempos_completos = [0.0] + [float(t) for t in t_puntos]
        posiciones_completas = [list(sol_inicial)] + [list(s) for s in soluciones_ganadoras]
    else:
        node.get_logger().warn("No se pudo calcular el punto inicial (t=0) para guardar.")
        tiempos_completos = [float(t) for t in t_puntos]
        posiciones_completas = [list(s) for s in soluciones_ganadoras]

    datos = {"tiempos": tiempos_completos, "posiciones": posiciones_completas}
    with open("trayectoria_4d.json", "w") as f:
        json.dump(datos, f)
    node.get_logger().info("Trayectoria guardada en trayectoria_4d.json (para Parte 5).")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
