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

class PointToObjectNode(Node):
    def __init__(self):
        super().__init__('point_to_object_node')
        self.get_logger().info("Nó iniciado.")
        moveit_commander.roscpp_initialize([])
        self.scene = PlanningSceneInterface()
        action_name = '/arm_controller/follow_joint_trajectory'
        self._action_client = ActionClient(self, FollowJointTrajectory, action_name)
        self.joint_names = [
            'waist_joint', 'shoulder_pitch_joint',
            'elbow_pitch_joint', 'wrist_pitch_joint'
        ]

    def point_at_target(self, target_name):
        self.get_logger().info(f"Procurando por '{target_name}'...")
        time.sleep(1)
        object_poses = self.scene.get_object_poses([target_name])
        if not object_poses:
            self.get_logger().error(f"Não encontrei o objeto '{target_name}'!")
            return False

        target_pose = object_poses[target_name]
        target_x, target_y = target_pose.position.x, target_pose.position.y
        base_angle = math.atan2(target_y, target_x)

        self.get_logger().info(f"Objeto encontrado. Ângulo da base: {base_angle:.2f} rad.")
        
        pointing_positions = [base_angle, -0.5, 1.0, 0.0]

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor de ação não disponível.")
            return False
            
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = pointing_positions
        point.time_from_start = Duration(sec=3)
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        self.get_logger().info("Enviando meta para o braço...")
        self._action_client.send_goal_async(goal_msg)
        return True

def main(args=None):
    rclpy.init(args=args)
    node = PointToObjectNode()
    NOME_DO_OBJETO = "Box_0"

    node.get_logger().info(f"Você tem 5s para adicionar o objeto '{NOME_DO_OBJETO}' no RViz...")
    time.sleep(5) 

    if node.point_at_target(NOME_DO_OBJETO):
        node.get_logger().info("Meta enviada. Aguardando 5s para o movimento.")
        time.sleep(5)
    
    node.get_logger().info("Teste concluído.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()