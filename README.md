# Parcial 2 - Robótica (KUKA KR6 R700-2)

Repositorio del Taller 2: modelado del manipulador, MoveIt2, cinemática inversa
y planeación de trayectorias con interpolación cúbica/quíntica.

## Estructura

- **`urdf_xacro/kr6_r700_2/`** — Xacro del robot (adaptado de
  `kroshu/kuka_robot_descriptions`, quitando el driver RSI real) y sus mallas
  de colisión/visuales.
- **`moveit_config/`** — Archivos de configuración adicionales de MoveIt2:
  `ompl_planning.yaml` (planeadores RRTConnect/RRT*) y el launch file para
  correr scripts con `moveit_py`.
- **`matlab/`** — Scripts de comparación entre el modelo DH propio y la
  solución de IK de MoveIt2 (Parte 3 del taller), incluyendo la conversión de
  convención de signos/offsets encontrada entre ambos modelos.
- **`python/parte4A/`** — Comparación de planeadores OMPL (RRTConnect vs.
  RRT*) para el tramo `home → pre-pick`, con evasión de obstáculo.
- **`python/parte4B/`** — Generación de perfiles de movimiento cúbico/quíntico
  para el acercamiento fino `pre-pick → pick`, con integración real a
  `computeCartesianPath()` y medición de aceleración articular.

## Notas

- `perfil_4b.py`: análisis puramente analítico (sin ROS2), valida la teoría
  de los perfiles cúbico/quíntico.
- `4b_v2.py`: versión final funcional, con `attach` de la pieza y colisiones
  activas contra mesa/obstáculo.
- `4b_prueba_hardcode.py`: script de diagnóstico simplificado que permitió
  aislar un bug en la versión más compleja del script (documentado en el
  historial de la conversación de desarrollo).
