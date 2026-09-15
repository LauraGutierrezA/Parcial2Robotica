#!/usr/bin/env python3
"""
Parte 4B - Perfil temporal (posicion/velocidad/aceleracion) para el
acercamiento fino pre-pick -> pick, comparando interpolacion cubica vs
quintica, bajo las restricciones del tramo 'rojo' (ida).

Esta primera parte es puramente analitica (no requiere ROS2 corriendo),
para poder verificar la matematica con certeza antes de integrarla con
MoveIt2 (computeCartesianPath + reparametrizacion), que se hace en un
paso posterior.
"""

import numpy as np

# ============================================================
#  PARAMETROS
# ============================================================
PICK_POSITION = np.array([0.229, 0.580, 0.284])
PICK_S = np.array([0.056, 0.098, -0.994])  # vector de aproximacion (columna 3 de la matriz de pick)
D_RETROCESO = 0.1   # distancia de pre-pick a pick (m) -- ajustable

N_INTERMEDIOS = 3   # puntos intermedios (sin contar los extremos)

# Restricciones tramo ROJO (ida: pre-pick -> pick)
VMAX_ROJO = 0.200   # m/s
AMAX_ROJO = 0.300   # m/s^2

# Restricciones tramo AZUL (retorno; se documentan aqui para reutilizar
# en la Parte 4D, pero el analisis principal de este archivo es el ROJO)
VMAX_AZUL = 0.100   # m/s
AMAX_AZUL = 0.020   # m/s^2

N_MUESTRAS = 200    # resolucion temporal para las graficas/tablas
# ============================================================


def calcular_pre_pick(pick_pos, S_vector, d):
    """pre-pick = pick - d * S_normalizado (formula ya validada antes)."""
    S_normalizado = S_vector / np.linalg.norm(S_vector)
    return pick_pos - d * S_normalizado


# pre-pick SIEMPRE se deriva de pick, nunca se escribe un numero fijo
PRE_PICK_POSITION = calcular_pre_pick(PICK_POSITION, PICK_S, D_RETROCESO)


def tiempo_total_cubico(d, vmax, amax):
    """Tiempo minimo T tal que un polinomio cubico (reposo-reposo) que
    recorre distancia d respeta v_max y a_max."""
    T_por_velocidad = 1.5 * d / vmax
    T_por_aceleracion = np.sqrt(6.0 * d / amax)
    return max(T_por_velocidad, T_por_aceleracion)


def tiempo_total_quintico(d, vmax, amax):
    """Tiempo minimo T tal que un polinomio quintico (reposo-reposo,
    aceleracion tambien nula en los extremos) respeta v_max y a_max."""
    T_por_velocidad = 1.875 * d / vmax
    T_por_aceleracion = np.sqrt((10.0 / np.sqrt(3.0)) * d / amax)
    return max(T_por_velocidad, T_por_aceleracion)


def perfil_cubico(d, T, t):
    """Posicion, velocidad y aceleracion escalares (a lo largo del
    camino, 0 a d) para un polinomio cubico reposo-reposo."""
    u = np.clip(t / T, 0.0, 1.0)
    pos = d * (3 * u**2 - 2 * u**3)
    vel = (d / T) * (6 * u - 6 * u**2)
    acc = (d / T**2) * (6 - 12 * u)
    return pos, vel, acc


def perfil_quintico(d, T, t):
    """Posicion, velocidad y aceleracion escalares para un polinomio
    quintico reposo-reposo (con aceleracion tambien nula en los
    extremos, a diferencia del cubico)."""
    u = np.clip(t / T, 0.0, 1.0)
    pos = d * (10 * u**3 - 15 * u**4 + 6 * u**5)
    vel = (d / T) * (30 * u**2 - 60 * u**3 + 30 * u**4)
    acc = (d / T**2) * (60 * u - 180 * u**2 + 120 * u**3)
    return pos, vel, acc


def generar_waypoints_cartesianos(pre_pick, pick, s_values, d):
    """Convierte una lista de posiciones escalares a lo largo del
    camino (0 a d) en puntos cartesianos, interpolando linealmente
    entre pre_pick y pick (linea recta, orientacion constante)."""
    direccion = (pick - pre_pick) / d
    return [pre_pick + s * direccion for s in s_values]


def analizar_perfil(nombre, tiempo_fn, perfil_fn, d, vmax, amax, n_intermedios):
    T = tiempo_fn(d, vmax, amax)
    t_muestras = np.linspace(0, T, N_MUESTRAS)
    pos, vel, acc = perfil_fn(d, T, t_muestras)

    v_pico = np.max(np.abs(vel))
    a_pico = np.max(np.abs(acc))

    # Indice de suavidad: energia del jerk (derivada de la aceleracion)
    jerk = np.gradient(acc, t_muestras)
    suavidad = np.sqrt(np.mean(jerk**2))  # RMS del jerk

    # Discontinuidad de aceleracion en los extremos (idealmente 0)
    salto_inicial = abs(acc[0])
    salto_final = abs(acc[-1])

    # Puntos intermedios solicitados (tiempos equiespaciados)
    t_intermedios = np.linspace(0, T, n_intermedios + 2)[1:-1]
    pos_i, vel_i, acc_i = perfil_fn(d, T, t_intermedios)

    print(f"\n--- Perfil {nombre} (tramo ROJO, ida) ---")
    print(f"Tiempo total T = {T:.4f} s")
    print(f"Velocidad pico = {v_pico:.4f} m/s (limite {vmax} m/s)")
    print(f"Aceleracion pico = {a_pico:.4f} m/s^2 (limite {amax} m/s^2)")
    print(f"Salto de aceleracion en t=0 / t=T: {salto_inicial:.4f} / {salto_final:.4f} m/s^2")
    print(f"Indice de suavidad (RMS del jerk) = {suavidad:.4f} m/s^3  (menor = mas suave)")
    print(f"\nPuntos intermedios ({n_intermedios}):")
    print(f"{'t (s)':<10}{'s (m)':<10}{'v (m/s)':<12}{'a (m/s^2)'}")
    for ti, pi, vi, ai in zip(t_intermedios, pos_i, vel_i, acc_i):
        print(f"{ti:<10.4f}{pi:<10.4f}{vi:<12.4f}{ai:.4f}")

    return {
        "T": T, "v_pico": v_pico, "a_pico": a_pico,
        "suavidad": suavidad,
        "salto_inicial": salto_inicial, "salto_final": salto_final,
        "t_intermedios": t_intermedios,
        "s_intermedios": pos_i,
    }


def main():
    print(f"pre-pick calculado (d={D_RETROCESO} m): {np.round(PRE_PICK_POSITION, 4).tolist()}")
    d = float(np.linalg.norm(PICK_POSITION - PRE_PICK_POSITION))
    print(f"Distancia pre-pick -> pick: {d:.4f} m")

    r_cubico = analizar_perfil(
        "CUBICO", tiempo_total_cubico, perfil_cubico,
        d, VMAX_ROJO, AMAX_ROJO, N_INTERMEDIOS,
    )
    r_quintico = analizar_perfil(
        "QUINTICO", tiempo_total_quintico, perfil_quintico,
        d, VMAX_ROJO, AMAX_ROJO, N_INTERMEDIOS,
    )

    print("\n" + "=" * 70)
    print("COMPARACION CUBICO vs QUINTICO (tramo rojo, ida)")
    print("=" * 70)
    print(f"{'Metrica':<30}{'Cubico':<20}{'Quintico'}")
    print(f"{'Tiempo total (s)':<30}{r_cubico['T']:<20.4f}{r_quintico['T']:.4f}")
    print(f"{'Aceleracion pico (m/s^2)':<30}{r_cubico['a_pico']:<20.4f}{r_quintico['a_pico']:.4f}")
    print(f"{'Salto accel. en extremos':<30}{r_cubico['salto_inicial']:<20.4f}{r_quintico['salto_inicial']:.4f}")
    print(f"{'Suavidad (RMS jerk)':<30}{r_cubico['suavidad']:<20.4f}{r_quintico['suavidad']:.4f}")
    print("=" * 70)
    print("(el polinomio quintico deberia mostrar salto de aceleracion ~0")
    print(" en los extremos, a diferencia del cubico -> trayectoria mas suave)")

    # Waypoints cartesianos en los puntos intermedios, listos para
    # pasar a computeCartesianPath() en el siguiente paso
    print("\nWaypoints cartesianos (perfil quintico, ejemplo):")
    waypoints = generar_waypoints_cartesianos(
        PRE_PICK_POSITION, PICK_POSITION, r_quintico["s_intermedios"], d
    )
    for i, (t, wp) in enumerate(zip(r_quintico["t_intermedios"], waypoints)):
        print(f"  t={t:.4f}s -> {np.round(wp, 4).tolist()}")


if __name__ == "__main__":
    main()
