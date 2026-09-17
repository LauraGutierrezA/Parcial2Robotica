#!/usr/bin/env python3
"""
Script inicial: mueve el KR6 R700-2 a la pose 'pick' usando moveit_py.
Sirve como base para toda la Parte 4 (se irá extendiendo con
PlanningScene, comparación de planeadores, tramos pre-pick/pre-place).
"""

import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()

    # Nombre del nodo: cualquiera, solo debe ser único en la sesión ROS2
    kr6 = MoveItPy(node_name="kr6_moveit_py")
    manipulator = kr6.get_planning_component("manipulator")

    # --- Definir la pose 'pick' ---
    pick_pose = PoseStamped()
    pick_pose.header.frame_id = "base_link"
    pick_pose.pose.position.x = 0.229
    pick_pose.pose.position.y = 0.580
    pick_pose.pose.position.z = 0.284
    pick_pose.pose.orientation.x = 0.728
    pick_pose.pose.orientation.y = -0.683
    pick_pose.pose.orientation.z = -0.013
    pick_pose.pose.orientation.w = -0.055

    # --- Fijar el estado inicial como el estado actual del robot ---
    manipulator.set_start_state_to_current_state()

    # --- Fijar la meta: la pose 'pick', respecto al frame 'tool0' ---
    manipulator.set_goal_state(pose_stamped_msg=pick_pose, pose_link="tool0")

    # --- Planear ---
    plan_result = manipulator.plan()

    if plan_result:
        print("Plan encontrado. Ejecutando...")
        robot_trajectory = plan_result.trajectory
        kr6.execute(robot_trajectory, controllers=[])
        print("Movimiento a 'pick' completado.")
    else:
        print("La planeación FALLÓ. Revisa la pose o el estado inicial.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
