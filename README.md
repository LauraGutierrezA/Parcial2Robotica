# Parcial 2 - Robótica (KUKA KR6 R700-2)

Repositorio del Taller 2: modelado del manipulador, MoveIt2, cinemática inversa,
planeación de trayectorias con evasión de obstáculos, perfiles de velocidad
cúbico/quíntico, y verificación del Jacobiano analítico contra MoveIt2/KDL.

## Estructura

- **`urdf_xacro/kr6_r700_2/`** — Xacro del robot (adaptado de
  `kroshu/kuka_robot_descriptions`, quitando el driver RSI real) y sus mallas
  de colisión/visuales.
- **`moveit_config/`** — Configuración de MoveIt2: `ompl_planning.yaml`
  (planeadores RRTConnect/RRT*, con `simplify_solutions: true` activado para
  suavizar el camino resultante) y el launch file para scripts con `moveit_py`.
- **`matlab/`** — Trabajo en MATLAB, organizado por taller:
  - **`matlab/Taller1/`** — `taller1_Parcial2.mlx` (live script principal),
    `IK_try.m` (cinemática inversa simbólica por desacople), `animar_robot.m`
    (animación 3D de la trayectoria resuelta), y `T_DH.m` (transformación
    homogénea DH, dependencia común de los dos anteriores).
  - **`matlab/Taller2/`** — `verificar_jacobiano_4b_4d_matlab.m` (Jacobiano
    analítico + tabla de verificación de velocidades en los tramos 4B/4D,
    ya con la indexación y el signo de la junta 4 corregidos), `T_DH.m`,
    y las trayectorias reales `trayectoria_4b.json` / `trayectoria_4d.json`
    (mismos archivos que usa el script de Python, para comparación directa).
- **`python/parte4A/`** — `home → pre-pick`, con evasión de obstáculo.
  - `comparar_planeadores.py`: comparación RRTConnect vs. RRT* (tiempo,
    longitud, suavidad) + ejecución real con el ganador.
  - `demo_4a.py`: versión limpia (sin comparación) para la demo animada
    fluida del ciclo completo.
- **`python/parte4B/`** — Acercamiento fino `pre-pick → pick`, perfil quíntico,
  con `attach` de la pieza y `computeCartesianPath()`.
  - `perfil_4b.py`: análisis puramente analítico (sin ROS2).
  - `4b_v2.py`: versión final funcional (agarra la pieza al llegar).
- **`python/parte4C/`** — `pick → pre-place`, evasión de obstáculo con la
  pieza ya cargada (RRTConnect).
- **`python/parte4D/`** — Acercamiento fino `pre-place → place`, perfil
  quíntico, con búsqueda automática de orientación/semilla de codo para
  evitar colisión brazo-mesa, y depósito de la pieza al llegar.
- **`python/parte5/`** — Verificación del Jacobiano: cálculo numérico vía
  diferencias finitas sobre `/compute_fk` (mismo solver KDL configurado en
  el taller), comparado contra las velocidades teóricas del perfil elegido
  en los tramos 4B y 4D.
- **`python/utilidades/`** — `obtener_angulos.py`: consulta rápida de IK
  para cualquier posición/orientación (reutilizable).
- **`ciclo_completo.sh`** — Corre el ciclo completo (4A→4B→4C→4D) en
  secuencia, para la demostración animada en RViz.

## Flujo de ejecución

```bash
# Terminal 1
ros2 launch kuka_R6_R700_moveit_config demo.launch.py

# Terminal 2
./ciclo_completo.sh
python3 python/parte5/verificar_jacobiano_4b_4d.py
```

## Notas técnicas relevantes

- **Convención de signos DH → real** (encontrada en la Parte 3, refinada en
  la Parte 5): `joint1_real=-θ1_DH`, `joint2_real=θ2_DH`,
  `joint3_real=θ3_DH+90°`, `joint4_real=-θ4_DH`, `joint5_real=θ5_DH`,
  `joint6_real=180°-θ6_DH`. El signo de la junta 4 se corrigió tras una
  verificación cruzada con el Jacobiano numérico — los dos puntos usados
  originalmente en la Parte 3 (`pick`/`place`) no distinguían el signo de
  esa junta por casualidad geométrica (uno con la junta en ~0°, el otro
  en ~180°).
- **Convención del Jacobiano analítico**: la columna de la junta *i* usa el
  frame **después** de aplicar la transformación de esa misma junta
  (`z_i`, `O_i`), no el frame anterior — esta es la convención específica
  usada en la fórmula de clase (ver `AnalisisVeloc.pdf`), distinta de la
  convención "textbook" genérica.
- `4b_prueba_hardcode.py`: script de diagnóstico simplificado que permitió
  aislar un bug de un intento anterior más complejo del script de 4B.
