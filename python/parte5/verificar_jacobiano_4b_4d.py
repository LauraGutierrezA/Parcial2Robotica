#!/usr/bin/env python3
"""
Parte 5 - Verificacion del Jacobiano y de las velocidades en los
tramos de acercamiento fino (4B y 4D).

VERSION SIMPLE: no vuelve a resolver IK. Lee la trayectoria articular
REAL ya calculada y ejecutada por 4b_v2.py / 4d_simple.py (guardada en
trayectoria_4b.json / trayectoria_4d.json), calcula el Jacobiano
numerico en cada punto (via /compute_fk, mismo solver KDL de siempre),
y compara la velocidad cartesiana resultante contra la velocidad
teorica del perfil quintico en ese mismo instante.

REQUIERE: haber corrido 4b_v2.py y 4d_simple.py primero (para generar
los archivos .json), desde la MISMA carpeta donde corres este script.
"""

import json
import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import Header


JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
BASE_LINK = "base_link"
TIP_LINK = "tool0"
DELTA = 1e-4  # rad, para la diferencia finita del Jacobiano

# Distancias y restricciones de cada tramo (para la velocidad TEORICA)
D_4B, VMAX_4B, AMAX_4B = 0.1, 0.200, 0.300
D_4D, VMAX_4D, AMAX_4D = 0.1, 0.100, 0.020


def tiempo_total_quintico(d, vmax, amax):
    return max(1.875 * d / vmax, np.sqrt((10.0 / np.sqrt(3.0)) * d / amax))


def velocidad_teorica_quintico(d, T, t):
    u = np.clip(t / T, 0.0, 1.0)
    return (d / T) * (30 * u**2 - 60 * u**3 + 30 * u**4)


class Verificador(Node):
    def __init__(self):
        super().__init__("verificador_jacobiano")
        self.cli_fk = self.create_client(GetPositionFK, "/compute_fk")

    def fk_posicion(self, joint_positions):
        req = GetPositionFK.Request()
        req.header = Header(frame_id=BASE_LINK)
        req.fk_link_names = [TIP_LINK]
        req.robot_state.joint_state.name = JOINT_NAMES
        req.robot_state.joint_state.position = list(joint_positions)
        future = self.cli_fk.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        resultado = future.result()
        if resultado is None or len(resultado.pose_stamped) == 0:
            return None
        p = resultado.pose_stamped[0].pose.position
        return np.array([p.x, p.y, p.z])

    def jacobiano_lineal(self, joint_positions):
        J = np.zeros((3, 6))
        for i in range(6):
            q_mas, q_menos = list(joint_positions), list(joint_positions)
            q_mas[i] += DELTA
            q_menos[i] -= DELTA
            p_mas, p_menos = self.fk_posicion(q_mas), self.fk_posicion(q_menos)
            if p_mas is None or p_menos is None:
                return None
            J[:, i] = (p_mas - p_menos) / (2 * DELTA)
        return J


def verificar_desde_archivo(node, nombre, archivo, d, vmax, amax):
    print(f"\n{'='*70}\nTRAMO: {nombre}\n{'='*70}")
    try:
        with open(archivo) as f:
            datos = json.load(f)
    except FileNotFoundError:
        print(f"  No se encontro '{archivo}'. Corre primero el script correspondiente.")
        return

    tiempos = np.array(datos["tiempos"])
    posiciones = np.array(datos["posiciones"])
    T = tiempo_total_quintico(d, vmax, amax)

    if len(tiempos) < 2:
        print("  Muy pocos puntos guardados para calcular velocidad.")
        return

    theta_dot = np.gradient(posiciones, tiempos, axis=0)

    # --- Imprimir la matriz Jacobiana COMPLETA en cada punto ---
    jacobianos = []
    print(f"\n--- Matrices Jacobianas (lineal, 3x6) en cada punto de {nombre} ---")
    for i, t in enumerate(tiempos):
        J = node.jacobiano_lineal(posiciones[i])
        jacobianos.append(J)
        print(f"\nJ en t={t:.4f} s (angulos reales={np.round(posiciones[i], 4).tolist()}):")
        if J is None:
            print("  FK fallo en este punto -- Jacobiano no disponible.")
        else:
            with np.printoptions(precision=4, suppress=True, linewidth=120):
                print(J)

    # --- Tabla resumen de velocidades (igual que antes) ---
    print(f"\n--- Tabla resumen: velocidad teorica vs. real ---")
    print(f"{'t (s)':<10}{'v_teorica (m/s)':<18}{'v_real |J*qdot| (m/s)':<22}{'error (%)'}")
    for i, t in enumerate(tiempos):
        J = jacobianos[i]
        if J is None:
            print(f"{t:<10.4f} FK fallo en este punto")
            continue
        v_real = np.linalg.norm(J @ theta_dot[i])
        v_teo = velocidad_teorica_quintico(d, T, t)
        error_pct = abs(v_real - v_teo) / v_teo * 100 if v_teo > 1e-6 else 0.0
        print(f"{t:<10.4f}{v_teo:<18.4f}{v_real:<22.4f}{error_pct:.2f}%")


def main():
    rclpy.init()
    node = Verificador()
    node.cli_fk.wait_for_service(timeout_sec=10.0)

    verificar_desde_archivo(node, "4B (pre-pick -> pick)", "trayectoria_4b.json", D_4B, VMAX_4B, AMAX_4B)
    verificar_desde_archivo(node, "4D (pre-place -> place)", "trayectoria_4d.json", D_4D, VMAX_4D, AMAX_4D)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
