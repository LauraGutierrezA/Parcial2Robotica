#!/usr/bin/env python3
"""
Republica mesa_place con la posicion validada (libre de colision),
sin tener que rehacer todo el ciclo 4A. Correr esto una vez antes
de probar 4D de nuevo.
"""

import time
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


PLACE_POSITION = [0.093, -0.588, 0.631]
ESFERA_RADIO = 0.03
MESA_SIZE = [0.5, 0.6, 0.05]  # ancho en Y aumentado de 0.4 a 0.6

# Offset validado (dentro del area de la mesa, con margen): -0.15 en Y
MESA_PLACE_POS = [
    PLACE_POSITION[0],
    PLACE_POSITION[1] - 0.15,
    PLACE_POSITION[2] - ESFERA_RADIO - MESA_SIZE[2] / 2.0,
]


def main():
    rclpy.init()
    node = Node("actualizar_mesa_place")
    pub = node.create_publisher(CollisionObject, "/collision_object", 10)

    caja = SolidPrimitive()
    caja.type = SolidPrimitive.BOX
    caja.dimensions = MESA_SIZE
    pose_caja = Pose()
    pose_caja.position.x, pose_caja.position.y, pose_caja.position.z = MESA_PLACE_POS
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

    node.get_logger().info(f"mesa_place actualizada a {MESA_PLACE_POS}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
