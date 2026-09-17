#!/usr/bin/env python3
"""Republica mesa_place con su geometria ORIGINAL (sin desplazamientos
ni agrandamientos de las pruebas anteriores) -- la que SI se valido
como libre de colision junto con la nueva orientacion de 'place'."""

import time
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

PLACE_POSITION = [0.093, -0.588, 0.631]
ESFERA_RADIO = 0.03
MESA_SIZE = [0.5, 0.4, 0.05]       # tamano ORIGINAL
MESA_BORDE_MARGEN = 0.03            # margen ORIGINAL


def calcular_mesa(pos_referencia, mesa_size, esfera_radio, borde_margen):
    superficie_z = pos_referencia[2] - esfera_radio
    centro_z = superficie_z - mesa_size[2] / 2.0
    centro_x = pos_referencia[0] - (mesa_size[0] / 2.0 - borde_margen)
    centro_y = pos_referencia[1]
    return [centro_x, centro_y, centro_z]


def main():
    rclpy.init()
    node = Node("restaurar_mesa_original")
    pub = node.create_publisher(CollisionObject, "/collision_object", 10)

    mesa_pos = calcular_mesa(PLACE_POSITION, MESA_SIZE, ESFERA_RADIO, MESA_BORDE_MARGEN)

    caja = SolidPrimitive()
    caja.type = SolidPrimitive.BOX
    caja.dimensions = MESA_SIZE
    pose_caja = Pose()
    pose_caja.position.x, pose_caja.position.y, pose_caja.position.z = mesa_pos
    pose_caja.orientation.w = 1.0

    obj = CollisionObject()
    obj.id = "mesa_place"
    obj.header.frame_id = "base_link"
    obj.primitives = [caja]
    obj.primitive_poses = [pose_caja]
    obj.operation = CollisionObject.ADD

    for _ in range(2):
        pub.publish(obj)
        time.sleep(0.5)
    time.sleep(0.5)

    node.get_logger().info(f"mesa_place RESTAURADA a geometria original: pos={mesa_pos}, size={MESA_SIZE}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
