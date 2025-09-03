#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

import moveit_commander
from moveit_commander import PlanningSceneInterface


def normalize_angle(angle):
    """Garante que o ângulo esteja no intervalo [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PointToObjectNode(Node):
    def __init__(self):
        super().__init__('point_to_object_node')

        self.get_logger().info("Nó iniciado. Pronto para apontar para objetos.")

        moveit_commander.roscpp_initialize([])
        self.scene = PlanningSceneInterface()

        action_name = '/arm_controller/follow_joint_trajectory'
        self._action_client = ActionClient(self, FollowJointTrajectory, action_name)

        self.joint_names = [
            'arm_shoulder_pan_joint',
            'arm_shoulder_lift_joint',
            'arm_elbow_flex_joint',
            'arm_wrist_flex_joint'
        ]

    def point_at_target(self, target_name):
        self.get_logger().info(f"Procurando pelo objeto '{target_name}' na cena...")

        time.sleep(1)

        # Procura o objeto na cena do MoveIt
        object_poses = self.scene.get_object_poses([target_name])
        if not object_poses:
            self.get_logger().error(f"Não encontrei o objeto '{target_name}' na cena! Adicione-o no RViz.")
            return

        target_pose = object_poses[target_name]
        target_x = target_pose.position.x
        target_y = target_pose.position.y

        base_angle_raw = math.atan2(target_y, target_x)
        base_angle_normalized = normalize_angle(base_angle_raw)

        self.get_logger().info(
            f"Objeto encontrado em (X={target_x:.2f}, Y={target_y:.2f}). "
            f"Ângulo da base: {base_angle_normalized:.2f} rad."
        )

        # Posições alvo [pan, lift, elbow, wrist]
        pointing_positions = [base_angle_normalized, -0.5, 1.0, 0.0]

        self._action_client.wait_for_server()
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = pointing_positions
        point.time_from_start = Duration(sec=3) 
        trajectory.points.append(point)

        goal_msg.trajectory = trajectory

        self.get_logger().info("Enviando meta de trajetória para o braço...")
        send_goal_future = self._action_client.send_goal_async(goal_msg)

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Meta de trajetória REJEITADA pelo servidor de ação.")
                return
            self.get_logger().info("Meta de trajetória ACEITA. Aguardando resultado...")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)

        send_goal_future.add_done_callback(goal_response_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Trajetória concluída com sucesso.")


def main(args=None):
    rclpy.init(args=args)
    node = PointToObjectNode()

    NOME_DO_OBJETO = "Box_0"

    node.get_logger().info(f"Você tem 5 segundos para adicionar o objeto '{NOME_DO_OBJETO}' no RViz...")
    time.sleep(5)

    node.point_at_target(NOME_DO_OBJETO)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
