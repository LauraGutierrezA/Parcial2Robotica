#!/usr/bin/env python3
"""
Parte 4D - Acercamiento fino pre-place -> place (deposito).
Mismo tipo de tramo que 4B, pero con el perfil YA seleccionado
(quintico, ganador en 4B por su menor salto de aceleracion articular
en los extremos) y las restricciones del tramo AZUL (retorno):
0.100 m/s, 0.020 m/s^2.

NOTA: asume que el robot esta en 'pre-place' (tras correr 4C), con
la pieza aun adjunta a tool0, y que demo.launch.py sigue corriendo.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclDuration

from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetStateValidity
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


# ============================================================
#  PARAMETROS
# ============================================================
# Sube el destino Z unos milímetros o ajusta levemente para evitar la invasión de link_4
PLACE_POSITION = np.array([0.093, -0.588, 0.635]) # Subido de 0.631 a 0.635
PLACE_QUAT_XYZW = [0.648, 0.759, -0.036, 0.042]
PLACE_S = np.array([0.017, -0.110, -0.994])  # columna 3 de la matriz de place
D_RETROCESO = 0.12

N_INTERMEDIOS = 3

# Restricciones tramo AZUL (retorno)
VMAX_AZUL = 0.100
AMAX_AZUL = 0.020

ESFERA_RADIO = 0.03

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GROUP_NAME = "manipulator"
LINK_NAME = "tool0"
CONTROLLER_ACTION = "/manipulator_controller/follow_joint_trajectory"
# ============================================================


def calcular_pre_place(place_pos, S_vector, d):
    S_normalizado = S_vector / np.linalg.norm(S_vector)
    return place_pos - d * S_normalizado


PRE_PLACE_POSITION = calcular_pre_place(PLACE_POSITION, PLACE_S, D_RETROCESO)


def tiempo_total_quintico(d, vmax, amax):
    return max(1.875 * d / vmax, np.sqrt((10.0 / np.sqrt(3.0)) * d / amax))


def perfil_quintico(d, T, t):
    u = np.clip(t / T, 0.0, 1.0)
    pos = d * (10 * u**3 - 15 * u**4 + 6 * u**5)
    vel = (d / T) * (30 * u**2 - 60 * u**3 + 30 * u**4)
    acc = (d / T**2) * (60 * u - 180 * u**2 + 120 * u**3)
    return pos, vel, acc


def generar_waypoints_y_tiempos(pre_place, place, tiempo_fn, perfil_fn, vmax, amax, n_intermedios):
    d = float(np.linalg.norm(place - pre_place))
    T = tiempo_fn(d, vmax, amax)
    t_intermedios = np.linspace(0, T, n_intermedios + 2)[1:-1]
    t_puntos = np.append(t_intermedios, T)
    s_puntos, _, _ = perfil_fn(d, T, t_puntos)
    direccion = (place - pre_place) / d
    posiciones_xyz = [pre_place + s * direccion for s in s_puntos]
    return t_puntos, posiciones_xyz, T, d


class Ejecutor4D(Node):
    def __init__(self):
        super().__init__("kr6_acercamiento_4d")
        self.cli_cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.cli_fk = self.create_client(GetPositionFK, "/compute_fk")
        self.action_client = ActionClient(self, FollowJointTrajectory, CONTROLLER_ACTION)
        self.pub_attach = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
        self.pub_collision_object = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.cli_validity = self.create_client(GetStateValidity, "/check_state_validity")

    def esperar_servicios(self):
        self.cli_cartesian.wait_for_service(timeout_sec=10.0)
        self.cli_fk.wait_for_service(timeout_sec=10.0)
        self.action_client.wait_for_server(timeout_sec=10.0)

    def asegurar_pieza_adjunta(self):
        """Idempotente: asegura que la pieza exista en el mundo y este adjunta a tool0."""
        self.get_logger().info("Asegurando 'pieza_esfera' en la escena y adjuntándola a tool0...")
        
        # 1. Primero publicamos la esfera como objeto de colisión en el mundo (origen en tool0)
        esfera = SolidPrimitive()
        esfera.type = SolidPrimitive.SPHERE
        esfera.dimensions = [ESFERA_RADIO]
        
        col_obj = CollisionObject()
        col_obj.id = "pieza_esfera"
        col_obj.header.frame_id = LINK_NAME
        col_obj.primitives = [esfera]
        
        pose_relativa = Pose()
        pose_relativa.orientation.w = 1.0
        col_obj.primitive_poses = [pose_relativa]
        col_obj.operation = CollisionObject.ADD
        
        self.pub_collision_object.publish(col_obj)
        
        import time as _time
        _time.sleep(1.0)

        # 2. Ahora enviamos la orden de adjuntarlo al tool0
        aco = AttachedCollisionObject()
        aco.link_name = LINK_NAME
        aco.object.id = "pieza_esfera"
        aco.object.header.frame_id = LINK_NAME
        aco.object.primitives = [esfera]
        aco.object.primitive_poses = [pose_relativa]
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = [LINK_NAME, "link_6", "flange", "link_5"]

        for _ in range(3):
            self.pub_attach.publish(aco)
            _time.sleep(1.0)
            
        _time.sleep(2.0) # Pausa final para estabilizar la escena de planeación
        
    def depositar_pieza(self):
        """Suelta la pieza en 'place': la desadjunta de tool0 y la vuelve
        a dejar como objeto estatico del mundo, en la posicion actual."""
        self.get_logger().info("Depositando 'pieza_esfera' en 'place' (desadjuntando)...")
        aco = AttachedCollisionObject()
        aco.object.id = "pieza_esfera"
        aco.link_name = LINK_NAME
        aco.object.operation = CollisionObject.REMOVE
        self.pub_attach.publish(aco)

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

        import time as _time
        _time.sleep(0.5)
        self.pub_collision_object.publish(obj)
        _time.sleep(0.5)

    def diagnosticar_pose(self, position, quat_xyzw, etiqueta):
        """Calcula IK para una pose y revisa su validez usando el estado
        REAL de la escena (incluyendo objetos adjuntos Y la matriz de
        colision permitida) -- pidiendo los componentes correctos esta vez."""
        from moveit_msgs.srv import GetPositionIK, GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents

        cli_ik = self.create_client(GetPositionIK, "/compute_ik")
        cli_ik.wait_for_service(timeout_sec=10.0)
        cli_scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        cli_scene.wait_for_service(timeout_sec=10.0)

        req_ik = GetPositionIK.Request()
        req_ik.ik_request.group_name = GROUP_NAME
        req_ik.ik_request.pose_stamped.header.frame_id = "base_link"
        req_ik.ik_request.pose_stamped.pose.position.x = position[0]
        req_ik.ik_request.pose_stamped.pose.position.y = position[1]
        req_ik.ik_request.pose_stamped.pose.position.z = position[2]
        (req_ik.ik_request.pose_stamped.pose.orientation.x,
         req_ik.ik_request.pose_stamped.pose.orientation.y,
         req_ik.ik_request.pose_stamped.pose.orientation.z,
         req_ik.ik_request.pose_stamped.pose.orientation.w) = quat_xyzw

        future = cli_ik.call_async(req_ik)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        resultado_ik = future.result()
        if resultado_ik is None or resultado_ik.error_code.val != 1:
            self.get_logger().error(f"IK para '{etiqueta}' fallo")
            return

        # Componentes correctos: ROBOT_STATE(1) + ATTACHED_OBJECTS(2) + ACM(64)
        req_scene = GetPlanningScene.Request()
        req_scene.components.components = 1 | 2 | 64
        future_scene = cli_scene.call_async(req_scene)
        rclpy.spin_until_future_complete(self, future_scene, timeout_sec=10.0)
        resultado_scene = future_scene.result()
        n_adjuntos = len(resultado_scene.scene.robot_state.attached_collision_objects)
        self.get_logger().info(f"[{etiqueta}] Objetos adjuntos (correcto esta vez): {n_adjuntos}")

        req_val = GetStateValidity.Request()
        req_val.group_name = GROUP_NAME
        req_val.robot_state = resultado_scene.scene.robot_state
        req_val.robot_state.joint_state = resultado_ik.solution.joint_state

        future_val = self.cli_validity.call_async(req_val)
        rclpy.spin_until_future_complete(self, future_val, timeout_sec=10.0)
        resultado_val = future_val.result()

        self.get_logger().info(f"[{etiqueta}] valido: {resultado_val.valid}")
        for contacto in resultado_val.contacts:
            self.get_logger().info(
                f"  CONTACTO [{etiqueta}]: '{contacto.contact_body_1}' <-> '{contacto.contact_body_2}'"
            )

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


def evaluar_perfil_real(node, nombre, tiempo_fn, perfil_fn, vmax, amax, ejecutar=True):
    t_puntos, posiciones_xyz, T, d = generar_waypoints_y_tiempos(
        PRE_PLACE_POSITION, PLACE_POSITION, tiempo_fn, perfil_fn,
        vmax, amax, N_INTERMEDIOS,
    )
    node.get_logger().info(f"--- {nombre} (real, via IK): T={T:.4f}s, d={d:.4f}m ---")

    poses = []
    for p in posiciones_xyz:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = p.tolist()
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = PLACE_QUAT_XYZW
        poses.append(pose)

    resultado = node.calcular_trayectoria_cartesiana(poses, max_step=d)
    if resultado is None or resultado.fraction < 0.99:
        frac = resultado.fraction if resultado else None
        node.get_logger().error(f"{nombre}: computeCartesianPath incompleto (fraction={frac})")
        return None

    puntos = resultado.solution.joint_trajectory.points
    node.get_logger().info(f"{nombre}: computeCartesianPath devolvio {len(puntos)} puntos")

    posiciones_cartesianas = []
    for p in puntos:
        pos_fk = node.obtener_posicion_fk(p.positions)
        if pos_fk is None:
            node.get_logger().error(f"{nombre}: fallo el FK de un punto.")
            return None
        posiciones_cartesianas.append(pos_fk)

    distancias_acumuladas = np.array([
        np.linalg.norm(pc - PRE_PLACE_POSITION) for pc in posiciones_cartesianas
    ])
    s_tabla = np.linspace(0, d, 500)
    t_tabla = np.linspace(0, T, 500)
    pos_tabla, _, _ = perfil_fn(d, T, t_tabla)
    tiempos = np.interp(distancias_acumuladas, pos_tabla, t_tabla)
    tiempos[0] = 0.0
    tiempos[-1] = T

    n = len(puntos)
    posiciones_articulares = np.array([puntos[i].positions for i in range(n)])

    velocidades = np.gradient(posiciones_articulares, tiempos, axis=0)
    aceleraciones = np.gradient(velocidades, tiempos, axis=0)
    velocidades[0] = 0.0
    velocidades[-1] = 0.0
    aceleraciones[0] = 0.0
    aceleraciones[-1] = 0.0

    a_max = float(np.max(np.abs(aceleraciones)))
    jerk = np.gradient(aceleraciones, tiempos, axis=0)
    suavidad_rms = float(np.sqrt(np.mean(jerk**2)))

    node.get_logger().info(
        f"{nombre}: aceleracion articular max={a_max:.4f} rad/s^2, "
        f"suavidad(RMS jerk)={suavidad_rms:.4f}"
    )

    nueva_traj = resultado.solution.joint_trajectory
    nueva_traj.joint_names = JOINT_NAMES
    nueva_traj.points = []
    for i in range(n):
        pt = JointTrajectoryPoint()
        pt.positions = posiciones_articulares[i].tolist()
        pt.velocities = velocidades[i].tolist()
        pt.accelerations = aceleraciones[i].tolist()
        pt.time_from_start = RclDuration(seconds=float(tiempos[i])).to_msg()
        nueva_traj.points.append(pt)

    if ejecutar:
        node.get_logger().info(f"Ejecutando perfil {nombre}...")
        node.enviar_trayectoria(nueva_traj)

    return {"a_max": a_max, "suavidad_rms": suavidad_rms, "T": T}


def main():
    print(f"pre-place calculado (d={D_RETROCESO} m): {np.round(PRE_PLACE_POSITION, 4).tolist()}")

    rclpy.init()
    node = Ejecutor4D()
    node.esperar_servicios()

    node.get_logger().info(
        "NOTA: este script asume que el robot YA esta en pre-place "
        "(corre primero traslado_4c.py) y que la pieza sigue adjunta."
    )

    node.asegurar_pieza_adjunta()

    punto_medio = ((PRE_PLACE_POSITION + PLACE_POSITION) / 2).tolist()
    node.diagnosticar_pose(punto_medio, PLACE_QUAT_XYZW.tolist() if hasattr(PLACE_QUAT_XYZW, "tolist") else PLACE_QUAT_XYZW, "punto_medio")
    node.diagnosticar_pose(PLACE_POSITION.tolist(), PLACE_QUAT_XYZW, "place")

    resultado = evaluar_perfil_real(
        node, "QUINTICO", tiempo_total_quintico, perfil_quintico,
        VMAX_AZUL, AMAX_AZUL, ejecutar=True,
    )

    if resultado:
        print("\n" + "=" * 60)
        print(f"Tiempo total: {resultado['T']:.4f} s")
        print(f"Aceleracion articular max: {resultado['a_max']:.4f} rad/s^2")
        print(f"Suavidad (RMS jerk): {resultado['suavidad_rms']:.4f}")
        print("=" * 60)
        node.depositar_pieza()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
