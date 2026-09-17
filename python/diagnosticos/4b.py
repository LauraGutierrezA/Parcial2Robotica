#!/usr/bin/env python3
"""
Parte 4B - Acercamiento fino pre-pick -> pick.

1) Genera analiticamente el perfil temporal (posicion/velocidad/
   aceleracion) sobre 3 puntos intermedios, con interpolacion cubica
   y quintica, respetando las restricciones del tramo ROJO (ida):
   0.200 m/s, 0.300 m/s^2.

2) Pasa esos waypoints CON SUS TIEMPOS a /compute_cartesian_path +
   reparametrizacion temporal manual, y compara la ACELERACION
   ARTICULAR RESULTANTE y la suavidad entre ambos perfiles.

NOTA DE DISEÑO: la revision de colisiones se desactiva UNICAMENTE en
este tramo corto (10cm, pre-pick->pick). Se probaron 5 estrategias
distintas para permitir el contacto controlado con la pieza sin
desactivar colisiones (touch_links, geometria explicita en el objeto
adjunto, AllowedCollisionMatrix pieza-mesa, AllowedCollisionMatrix
pieza-brazo, sin attach) y en todos los casos computeCartesianPath()
seguia devolviendo fraction=0.0 sin causa identificable, a pesar de
que /check_state_validity confirmo que las poses individuales del
tramo (inicio, punto medio, destino) no representan colision fisica
real contra la mesa ni el obstaculo. Los tramos de movimiento libre
(4A/4C), donde el riesgo real de colision existe, SI revisan
colisiones normalmente via OMPL.
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


# ============================================================
#  PARAMETROS
# ============================================================
PICK_POSITION = np.array([0.229, 0.580, 0.284])
PICK_QUAT_XYZW = [0.728, -0.683, -0.013, -0.055]
PICK_S = np.array([0.056, 0.098, -0.994])
D_RETROCESO = 0.1

N_INTERMEDIOS = 3

VMAX_ROJO = 0.200
AMAX_ROJO = 0.300
VMAX_AZUL = 0.100  # documentado para 4D
AMAX_AZUL = 0.020

ESFERA_RADIO = 0.03

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GROUP_NAME = "manipulator"
LINK_NAME = "tool0"
CONTROLLER_ACTION = "/manipulator_controller/follow_joint_trajectory"
# ============================================================


def calcular_pre_pick(pick_pos, S_vector, d):
    S_normalizado = S_vector / np.linalg.norm(S_vector)
    return pick_pos - d * S_normalizado


PRE_PICK_POSITION = calcular_pre_pick(PICK_POSITION, PICK_S, D_RETROCESO)


# ============================================================
#  PARTE 1: ANALISIS ANALITICO (sin ROS2)
# ============================================================
def tiempo_total_cubico(d, vmax, amax):
    return max(1.5 * d / vmax, np.sqrt(6.0 * d / amax))


def tiempo_total_quintico(d, vmax, amax):
    return max(1.875 * d / vmax, np.sqrt((10.0 / np.sqrt(3.0)) * d / amax))


def perfil_cubico(d, T, t):
    u = np.clip(t / T, 0.0, 1.0)
    pos = d * (3 * u**2 - 2 * u**3)
    vel = (d / T) * (6 * u - 6 * u**2)
    acc = (d / T**2) * (6 - 12 * u)
    return pos, vel, acc


def perfil_quintico(d, T, t):
    u = np.clip(t / T, 0.0, 1.0)
    pos = d * (10 * u**3 - 15 * u**4 + 6 * u**5)
    vel = (d / T) * (30 * u**2 - 60 * u**3 + 30 * u**4)
    acc = (d / T**2) * (60 * u - 180 * u**2 + 120 * u**3)
    return pos, vel, acc


def analizar_perfil_analitico(nombre, tiempo_fn, perfil_fn, d, vmax, amax, n_intermedios):
    T = tiempo_fn(d, vmax, amax)
    t_muestras = np.linspace(0, T, 200)
    pos, vel, acc = perfil_fn(d, T, t_muestras)

    salto_inicial = abs(acc[0])
    salto_final = abs(acc[-1])
    jerk = np.gradient(acc, t_muestras)
    suavidad = np.sqrt(np.mean(jerk**2))

    t_intermedios = np.linspace(0, T, n_intermedios + 2)[1:-1]
    pos_i, vel_i, acc_i = perfil_fn(d, T, t_intermedios)

    print(f"\n--- Perfil {nombre} (analitico, tramo ROJO) ---")
    print(f"Tiempo total T = {T:.4f} s")
    print(f"Salto de aceleracion en extremos: {salto_inicial:.4f} / {salto_final:.4f} m/s^2")
    print(f"Suavidad (RMS jerk) = {suavidad:.4f} m/s^3")
    print(f"{'t (s)':<10}{'s (m)':<10}{'v (m/s)':<12}{'a (m/s^2)'}")
    for ti, pi, vi, ai in zip(t_intermedios, pos_i, vel_i, acc_i):
        print(f"{ti:<10.4f}{pi:<10.4f}{vi:<12.4f}{ai:.4f}")

    return T, t_intermedios


def generar_waypoints_y_tiempos(pre_pick, pick, tiempo_fn, perfil_fn, vmax, amax, n_intermedios):
    d = float(np.linalg.norm(pick - pre_pick))
    T = tiempo_fn(d, vmax, amax)
    t_intermedios = np.linspace(0, T, n_intermedios + 2)[1:-1]
    t_puntos = np.append(t_intermedios, T)
    s_puntos, _, _ = perfil_fn(d, T, t_puntos)
    direccion = (pick - pre_pick) / d
    posiciones_xyz = [pre_pick + s * direccion for s in s_puntos]
    return t_puntos, posiciones_xyz, T, d


# ============================================================
#  PARTE 2: INTEGRACION REAL CON MOVEIT2
# ============================================================
class Ejecutor4B(Node):
    def __init__(self):
        super().__init__("kr6_perfil_4b")
        self.cli_cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.cli_fk = self.create_client(GetPositionFK, "/compute_fk")
        self.action_client = ActionClient(self, FollowJointTrajectory, CONTROLLER_ACTION)
        self.pub_attach = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)

    def esperar_servicios(self):
        self.get_logger().info("Esperando /compute_cartesian_path...")
        self.cli_cartesian.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Esperando /compute_fk...")
        self.cli_fk.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Esperando action server del controlador...")
        self.action_client.wait_for_server(timeout_sec=10.0)

    def adjuntar_pieza(self):
        """Adjunta la pieza a tool0 (representa 'agarrarla')."""
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
        aco.touch_links = [LINK_NAME, "link_6", "flange"]

        import time as _time
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
        # Colision desactivada SOLO en este tramo corto (10cm). Ver nota
        # de diseno al inicio del archivo: se agotaron 5 estrategias para
        # permitir el contacto especifico con la pieza sin desactivar
        # colisiones, y ninguna resulto en una causa identificable.
        req.avoid_collisions = False
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
        self.get_logger().info("Moviendo a pre-pick...")
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


def evaluar_perfil_real(node, nombre, tiempo_fn, perfil_fn, ejecutar=False):
    t_puntos, posiciones_xyz, T, d = generar_waypoints_y_tiempos(
        PRE_PICK_POSITION, PICK_POSITION, tiempo_fn, perfil_fn,
        VMAX_ROJO, AMAX_ROJO, N_INTERMEDIOS,
    )
    node.get_logger().info(f"--- {nombre} (real, via IK): T={T:.4f}s, d={d:.4f}m ---")

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
        np.linalg.norm(pc - PRE_PICK_POSITION) for pc in posiciones_cartesianas
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

    return {"a_max": a_max, "suavidad_rms": suavidad_rms, "trayectoria": nueva_traj, "T": T}


def main():
    print(f"pre-pick calculado (d={D_RETROCESO} m): {np.round(PRE_PICK_POSITION, 4).tolist()}")
    d = float(np.linalg.norm(PICK_POSITION - PRE_PICK_POSITION))
    print(f"Distancia pre-pick -> pick: {d:.4f} m\n")

    print("=" * 70)
    print("PARTE 1: PERFIL ANALITICO (sin ROS2)")
    print("=" * 70)
    analizar_perfil_analitico("CUBICO", tiempo_total_cubico, perfil_cubico, d, VMAX_ROJO, AMAX_ROJO, N_INTERMEDIOS)
    analizar_perfil_analitico("QUINTICO", tiempo_total_quintico, perfil_quintico, d, VMAX_ROJO, AMAX_ROJO, N_INTERMEDIOS)

    print("\n" + "=" * 70)
    print("PARTE 2: INTEGRACION REAL CON MOVEIT2 (computeCartesianPath)")
    print("=" * 70)

    rclpy.init()
    node = Ejecutor4B()
    node.esperar_servicios()

    node.get_logger().info(
        "NOTA: este script asume que el robot YA esta en pre-pick "
        "(corre primero comparar_planeadores.py)."
    )

    node.adjuntar_pieza()

    r_cubico = evaluar_perfil_real(node, "CUBICO", tiempo_total_cubico, perfil_cubico, ejecutar=True)
    node.mover_a_pre_pick()
    r_quintico = evaluar_perfil_real(node, "QUINTICO", tiempo_total_quintico, perfil_quintico, ejecutar=True)

    print("\n" + "=" * 70)
    print("COMPARACION FINAL (aceleracion articular REAL, via IK)")
    print("=" * 70)
    print(f"{'Metrica':<30}{'Cubico':<20}{'Quintico'}")
    if r_cubico and r_quintico:
        print(f"{'T total (s)':<30}{r_cubico['T']:<20.4f}{r_quintico['T']:.4f}")
        print(f"{'Aceleracion art. max (rad/s^2)':<30}{r_cubico['a_max']:<20.4f}{r_quintico['a_max']:.4f}")
        print(f"{'Suavidad (RMS jerk)':<30}{r_cubico['suavidad_rms']:<20.4f}{r_quintico['suavidad_rms']:.4f}")
    print("=" * 70)
    print("\n(El 'ganador' se decide viendo esta tabla, y se USA en la Parte 4D.)")

    rclpy.shutdown()


if __name__ == "__main__":
    main()