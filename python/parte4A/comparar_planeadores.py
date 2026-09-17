#!/usr/bin/env python3
"""
Parte 4A - Comparacion de planeadores OMPL (RRTConnect vs RRTstar)
para el tramo home -> pre-pick, con evasion del obstaculo.
Se conecta al move_group ya activo (lanzado con demo.launch.py).
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

from pymoveit2 import MoveIt2


# ============================================================
#  PARAMETROS
# ============================================================
PICK_POSITION = [0.229, 0.580, 0.284]
PICK_QUAT_XYZW = [0.728, -0.683, -0.013, -0.055]
PICK_S = np.array([0.056, 0.098, -0.994])

# --- Pose 'place' (ya validada contra MATLAB en la Parte 3) ---
PLACE_POSITION = [0.093, -0.588, 0.631]
PLACE_QUAT_XYZW = [0.078, -0.995, 0.055, -0.004]  # rotada 270 grados sobre Z local (libre de colision)

D_RETROCESO = 0.1

OBSTACULO_POSICION = [0.38, 0.48, 0.17]
OBSTACULO_RADIO = 0.1
OBSTACULO_ALTURA = 0.35

# --- Mesas (representadas como cajas, ver justificacion en el chat) ---
MESA_SIZE = [0.5, 0.4, 0.05]     # largo x ancho x grosor de la tapa (m)
MESA_BORDE_MARGEN = 0.03         # cuanto sobresale la pieza del borde (m)

# --- Pieza a manipular: esfera que reposa en el borde de la mesa pick ---
ESFERA_RADIO = 0.03              # m

# Planeadores a comparar (deben existir en ompl_planning.yaml)
PLANEADORES = ["RRTConnectkConfigDefault", "RRTstarkConfigDefault"]
# ============================================================


def calcular_mesa(pos_referencia, mesa_size, esfera_radio, borde_margen):
    """Ubica el centro de una mesa (caja) de forma que 'pos_referencia'
    (el punto pick o place) quede justo sobre su borde en X, con la
    superficie de la mesa a la altura correcta para que una esfera de
    radio 'esfera_radio' apoyada ahi tenga su centro en pos_referencia."""
    superficie_z = pos_referencia[2] - esfera_radio
    centro_z = superficie_z - mesa_size[2] / 2.0
    centro_x = pos_referencia[0] - (mesa_size[0] / 2.0 - borde_margen)
    centro_y = pos_referencia[1]
    return [centro_x, centro_y, centro_z]


def calcular_pre_pick(pick_pos, S_vector, d):
    S_normalizado = S_vector / np.linalg.norm(S_vector)
    return (np.array(pick_pos) - d * S_normalizado).tolist()


def calcular_metricas(traj):
    """Calcula longitud del camino y un indice de suavidad a partir
    de la trayectoria devuelta por moveit2.plan() (espacio articular).
    NOTA: en esta version de pymoveit2, plan() devuelve directamente
    un JointTrajectory (no un RobotTrajectory), por eso usamos
    traj.points en vez de traj.joint_trajectory.points."""
    puntos = traj.points
    posiciones = np.array([p.positions for p in puntos])

    diffs = np.diff(posiciones, axis=0)
    longitud = float(np.sum(np.linalg.norm(diffs, axis=1)))

    if len(diffs) > 1:
        segundas_diffs = np.diff(diffs, axis=0)
        suavidad = float(np.sum(np.linalg.norm(segundas_diffs, axis=1)))
    else:
        suavidad = 0.0

    return longitud, suavidad, len(puntos)


def main():
    rclpy.init()
    node = Node("kr6_comparar_planeadores")
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        base_link_name="base_link",
        end_effector_name="tool0",
        group_name="manipulator",
        callback_group=callback_group,
    )

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    time.sleep(2.0)

    # --- Agregar el obstaculo (dos veces, por el problema de descubrimiento DDS) ---
    for _ in range(2):
        moveit2.add_collision_cylinder(
            id="obstaculo_poste",
            position=OBSTACULO_POSICION,
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            height=OBSTACULO_ALTURA,
            radius=OBSTACULO_RADIO,
        )
        time.sleep(0.5)

    # --- Mesa en 'pick' ---
    mesa_pick_pos = calcular_mesa(PICK_POSITION, MESA_SIZE, ESFERA_RADIO, MESA_BORDE_MARGEN)
    node.get_logger().info(f"Mesa pick en {mesa_pick_pos}")
    for _ in range(2):
        moveit2.add_collision_box(
            id="mesa_pick",
            position=mesa_pick_pos,
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            size=MESA_SIZE,
        )
        time.sleep(0.5)

    # --- Mesa en 'place' ---
    # Con la orientacion original de 'place' esta posicion generaba colision
    # real (link_4 vs mesa_place, confirmado con /check_state_validity).
    # Se roto la orientacion de 'place' 270 grados sobre su propio eje Z
    # local (ver barrer_orientacion_place.py) -- el vector S se preserva,
    # asi que la formula original de calcular_mesa vuelve a ser valida.
    mesa_place_pos = calcular_mesa(PLACE_POSITION, MESA_SIZE, ESFERA_RADIO, MESA_BORDE_MARGEN)
    node.get_logger().info(f"Mesa place en {mesa_place_pos}")
    for _ in range(2):
        moveit2.add_collision_box(
            id="mesa_place",
            position=mesa_place_pos,
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            size=MESA_SIZE,
        )
        time.sleep(0.5)

    # --- Pieza (esfera) reposando en el borde de la mesa pick ---
    node.get_logger().info(f"Pieza (esfera) en {PICK_POSITION}")
    for _ in range(2):
        moveit2.add_collision_sphere(
            id="pieza_esfera",
            position=PICK_POSITION,
            radius=ESFERA_RADIO,
        )
        time.sleep(0.5)

    time.sleep(1.0)

    pre_pick_position = calcular_pre_pick(PICK_POSITION, PICK_S, D_RETROCESO)
    node.get_logger().info(f"pre-pick: {pre_pick_position}")

    resultados = {}

    for planner in PLANEADORES:
        node.get_logger().info(f"--- Planeando con {planner} ---")
        moveit2.planner_id = planner

        t0 = time.time()
        traj = moveit2.plan(
            position=pre_pick_position,
            quat_xyzw=PICK_QUAT_XYZW,
            cartesian=False,
        )
        t1 = time.time()

        if traj is None:
            node.get_logger().warn(f"{planner}: PLANEACION FALLIDA")
            resultados[planner] = None
            continue

        tiempo_planeacion = t1 - t0
        longitud, suavidad, n_puntos = calcular_metricas(traj)

        resultados[planner] = {
            "tiempo_s": tiempo_planeacion,
            "longitud": longitud,
            "suavidad": suavidad,
            "n_puntos": n_puntos,
        }

    # --- Imprimir tabla comparativa ---
    print("\n" + "=" * 78)
    print(f"{'Planeador':<28}{'Tiempo (s)':<14}{'Long. camino':<16}{'Suavidad':<12}{'#Puntos'}")
    print("-" * 78)
    for planner, r in resultados.items():
        if r is None:
            print(f"{planner:<28}{'FALLO':<14}")
        else:
            print(
                f"{planner:<28}{r['tiempo_s']:<14.4f}"
                f"{r['longitud']:<16.4f}{r['suavidad']:<12.4f}{r['n_puntos']}"
            )
    print("=" * 78)
    print("(menor tiempo = mas rapido | menor longitud = camino mas directo |")
    print(" menor suavidad = trayectoria con menos cambios bruscos de direccion)\n")

    # --- Ejecutar el movimiento REAL hacia pre-pick, con el ganador ---
    GANADOR_4A = "RRTConnectkConfigDefault"  # ajusta si tu tabla da otro resultado
    node.get_logger().info(f"Ejecutando movimiento real home -> pre-pick con {GANADOR_4A}...")
    moveit2.planner_id = GANADOR_4A
    moveit2.move_to_pose(
        position=pre_pick_position,
        quat_xyzw=PICK_QUAT_XYZW,
        cartesian=False,
    )
    moveit2.wait_until_executed()
    node.get_logger().info("Robot ahora deberia estar fisicamente en pre-pick.")

    executor.shutdown()
    thread.join(timeout=2.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
