#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class RobustPointAndGripperTester(Node):

    def __init__(self):
        super().__init__('point_node')

        self.arm_joint_names = ['waist_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint', 'wrist_pitch_joint']
        self.gripper_joint_name = ['gripper_finger_joint']

        self.arm_action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_action_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        self.arm_goal_done = False
        self.gripper_goal_done = False

    def send_goals(self):
        if not self.arm_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor de ação do BRAÇO não disponível!")
            return

        arm_positions = [0, 0.0, 0.1, 1.7]
        arm_goal_msg = self.create_trajectory_goal(self.arm_joint_names, arm_positions, 4)
        
        self.get_logger().info(f"Enviando meta para o braço: {arm_positions}")
        send_goal_future = self.arm_action_client.send_goal_async(arm_goal_msg)
        send_goal_future.add_done_callback(self.arm_goal_response_callback)

        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Servidor de ação da GARRA não disponível!")
            return
            
        gripper_position = [0.508]
        gripper_goal_msg = self.create_trajectory_goal(self.gripper_joint_name, gripper_position, 2)
        
        self.get_logger().info(f"Enviando meta para a garra: {gripper_position}")
        send_goal_future_gripper = self.gripper_action_client.send_goal_async(gripper_goal_msg)
        send_goal_future_gripper.add_done_callback(self.gripper_goal_response_callback)

    def create_trajectory_goal(self, joint_names, positions, duration_sec):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration_sec
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        return goal_msg

    #  Callbacks para o BRAÇO 
    def arm_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Meta do BRAÇO foi rejeitada.')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.arm_get_result_callback)

    def arm_get_result_callback(self, future):
        result = future.result().result
        if result.error_code == result.SUCCESSFUL:
            self.get_logger().info("Braço concluiu o movimento com SUCESSO!")
        else:
            self.get_logger().error(f"Movimento do BRAÇO FALHOU com código: {result.error_code}")
        self.arm_goal_done = True
        self.check_completion()

    # Callbacks para a GARRA 
    def gripper_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Meta da GARRA foi rejeitada.')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_get_result_callback)

    def gripper_get_result_callback(self, future):
        result = future.result().result
        if result.error_code == result.SUCCESSFUL:
            self.get_logger().info("Garra concluiu o movimento com SUCESSO!")
        else:
            self.get_logger().error(f"Movimento da GARRA FALHOU com código: {result.error_code}")
        self.gripper_goal_done = True
        self.check_completion()

    def check_completion(self):
        if self.arm_goal_done and self.gripper_goal_done:
            self.get_logger().info("Teste concluído.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    tester_node = RobustPointAndGripperTester()
    tester_node.send_goals()
    rclpy.spin(tester_node)

if __name__ == '__main__':
    main()

