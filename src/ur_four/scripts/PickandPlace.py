#!/usr/bin/env python3
import time
import math
import subprocess
from rclpy.node import Node
from termcolor import colored
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.planning import MoveItPy, PlanRequestParameters, MultiPipelinePlanRequestParameters
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import UInt8, ColorRGBA
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import DisplayTrajectory, PositionConstraint


class PickAndPlace(Node):

    def __init__(self, pipeline, gazebo, init_node = False):

        # Logic for loading correct URDF
        if gazebo:
            urdf_file = 'gazebo_combined.urdf.xacro'
            
        else:
            urdf_file = 'lbr_iisy3_r760_combined.urdf.xacro'

        # Define MoveIt Configuration
        moveit_config = (
            MoveItConfigsBuilder('kuka_lbr_iisy')
            .robot_description(
                file_path=get_package_share_directory('kuka_irl_project') + f'/urdf/{urdf_file}')
            .robot_description_semantic(
                file_path=get_package_share_directory('kuka_irl_project') + '/urdf/lbr_iisy3_r760.srdf')
            .joint_limits(
                file_path=get_package_share_directory("kuka_lbr_iisy_support")+ "/config/lbr_iisy3_r760_joint_limits.yaml")
            .moveit_cpp(
                file_path=get_package_share_directory('kuka_irl_project') + '/config/planning.yaml')
            .to_moveit_configs()
        )

        # Convert MoveIt Configuration to a dictionary
        moveit = moveit_config.to_dict()
        
        # If using Gazebo, use sim time
        if gazebo:
            moveit.update({"use_sim_time": True})
            moveit.update({'qos_overrides./clock.subscription.durability': 'volatile'})
            moveit.update({'qos_overrides./clock.subscription.history': 'keep_last'})
            moveit.update({'qos_overrides./clock.subscription.depth': 10})
            moveit.update({'qos_overrides./clock.subscription.reliability': 'reliable'})

        robot = MoveItPy(node_name="moveit_py", config_dict=moveit)
        group = robot.get_planning_component("manipulator")  
        robot_model = robot.get_robot_model()
        robot_state = RobotState(robot_model)
        planning_scene = robot.get_planning_scene_monitor()
        
        # See planning.yaml for a complete list
        if pipeline == 'ompl':
            self.single_plan_parameters = PlanRequestParameters(robot, "ompl_rrtc_solo")
            self.multi_plan_parameters = None
            self.single_plan_parameters.max_velocity_scaling_factor = 1.0
            self.single_plan_parameters.max_acceleration_scaling_factor = 1.0

        elif pipeline == 'pilz':
            self.single_plan_parameters = PlanRequestParameters(robot, "pilz_lin_solo")
            self.multi_plan_parameters = None
            self.single_plan_parameters.max_velocity_scaling_factor = 1.0
            self.single_plan_parameters.max_acceleration_scaling_factor = 1.0
        
        elif pipeline == 'stomp':
            self.single_plan_parameters = PlanRequestParameters(robot, "stomp_solo")
            self.multi_plan_parameters = None
            self.single_plan_parameters.max_velocity_scaling_factor = 1.0
            self.single_plan_parameters.max_acceleration_scaling_factor = 1.0
        
        # Cannot directly change multi pipeline params, must be done in planning.yaml
        else:
            self.single_plan_parameters = None
            self.multi_plan_parameters = MultiPipelinePlanRequestParameters(robot, ['pilz_lin_multi', 'stomp_multi'])
            # self.multi_plan_parameters = MultiPipelinePlanRequestParameters(robot, ['pilz_lin_multi','ompl_rrtc_multi', 'stomp_multi'])

        # Misc variables
        self.protective_stop = False
        self.robot = robot
        self.group = group
        self.robot_state = robot_state
        self.target_location_x = -100
        self.target_location_y = -100
        self.target_location_z = -100
        self.onion_color = None
        self.prev_placexs = []
        self.prev_placeys = []
        self.planning_scene = planning_scene
        self.counter = 0
        self.gazebo = gazebo
    
    
    def go_to_joint_goal(self, angles, planning_time=10.0, tol=0.001, attempts=5):

        group = self.group        
        group.set_start_state_to_current_state()
        robot = self.robot
        robot_state = self.robot_state
        robot_state.joint_positions = angles
        
        joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=robot.get_robot_model().get_joint_model_group("manipulator"),
            )
        
        if self.single_plan_parameters:
            self.single_plan_parameters.planning_time = planning_time

        group.set_goal_state(motion_plan_constraints=[joint_constraint])
        plan_result = group.plan(single_plan_parameters=self.single_plan_parameters, multi_plan_parameters=self.multi_plan_parameters)
        execution = self.robot.execute(plan_result.trajectory, controllers=[])

        status = execution.status
        
        if status != 'SUCCEEDED':
            
            print(colored('Failed to reach joint goal, checking protective stop', "red"))
            # print('Failed to reach target joint goal, checking protective stop')
            
            if not self.gazebo:
                subprocess.run(["ros2", "run", "kuka_irl_project", "safety_check.py"])
            
            return False
        
        else:
            return True


    def go_to_pose_goal(self, ox, oy, oz, ow, px, py, pz, planning_time=10.0, pose_tol=0.01, ori_tol=0.1, attempts=5):

        group = self.group
        group.set_start_state_to_current_state()

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = 'world'
        pose_goal.pose.orientation.x = ox
        pose_goal.pose.orientation.y = oy
        pose_goal.pose.orientation.z = oz
        pose_goal.pose.orientation.w = ow
        pose_goal.pose.position.x = px
        pose_goal.pose.position.y = py
        pose_goal.pose.position.z = pz

        group.set_goal_state(pose_stamped_msg=pose_goal, pose_link='gripper_base_link')

        if self.single_plan_parameters:
            self.single_plan_parameters.planning_time = planning_time

        plan_result = group.plan(single_plan_parameters=self.single_plan_parameters, multi_plan_parameters=self.multi_plan_parameters)
        execution = self.robot.execute(plan_result.trajectory, controllers=[])
        
        status = execution.status
        
        if status != 'SUCCEEDED':
            
            print(colored('Failed to reach joint goal, checking protective stop', "red"))
            # print('Failed to reach target joint goal, checking protective stop')
            
            if not self.gazebo:
                subprocess.run(["ros2", "run", "kuka_irl_project", "safety_check.py"])
            
            return False
        
        else:
            return True
    

    def goAndPick(self, planning_time=10.0):

        with self.planning_scene.read_only() as scene:
            current_state = scene.current_state
            current_pose = current_state.get_pose('gripper_base_link')
            
        ori = current_pose.orientation
        
        while self.target_location_x == -100:
            time.sleep(0.05)

        print(colored("Attempting to reach {}, {}, {}".format(self.target_location_x, self.target_location_y, current_pose.position.z),'cyan'))
        # print("Attempting to reach {}, {}, {}".format(self.target_location_x, self.target_location_y, current_pose.position.z))
        
        approach = 0.15
        
        if self.gazebo:
            approach += 0.08
        
        status = self.go_to_pose_goal(ori.x, ori.y, ori.z,ori.w, self.target_location_x, self.target_location_y, self.target_location_z + approach, planning_time)
        
        if status:
            print(colored('Successfully reached approach', 'cyan'))
            # print("Successfully reached approach")
            
        else:
            print(colored('Failed to reach approach', 'red'))
            # print("Failed to reach approach")
        
        return status


    def staticDip(self, gripper_length = 0.084, pose_tol=0.01, ori_tol=0.1, planning_time=10.0):
        
        with self.planning_scene.read_only() as scene:
            current_state = scene.current_state
            current_pose = current_state.get_pose('gripper_base_link')

        while self.target_location_x == -100:
            time.sleep(0.05)
        
        dip = self.go_to_pose_goal(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w,
                                   self.target_location_x, self.target_location_y, self.target_location_z + gripper_length, planning_time, pose_tol, ori_tol)
        if dip:
            print(colored('Successfully dipped gripper', 'cyan'))
            # print("Successfully dipped gripper")
            
        else:
            print(colored('Failed to dip gripper', 'red'))
            # print("Failed to dip gripper")

        return dip


    def liftgripper(self, pose_tol=0.01, ori_tol=0.1, planning_time=10.0):

        with self.planning_scene.read_only() as scene:
            current_state = scene.current_state
            current_pose = current_state.get_pose('gripper_base_link')

        z_pose = current_pose.position.z + 0.12

        lifted = self.go_to_pose_goal(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w, 
                                      current_pose.position.x, current_pose.position.y, z_pose, planning_time, pose_tol, ori_tol)

        if lifted:
            print(colored('Successfully lifted gripper', 'cyan'))
            # print("Successfully lifted gripper")
            
        else:
            print(colored('Failed to lift gripper', 'red'))
            # print("Failed to lift gripper")

        return lifted


    def lift_after_place(self, pose_tol=0.01, ori_tol=0.1, planning_time=10.0):

        with self.planning_scene.read_only() as scene:
            current_state = scene.current_state
            current_pose = current_state.get_pose('gripper_base_link')

        z_pose = current_pose.position.z + 0.1

        lifted = self.go_to_pose_goal(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w, 
                                      current_pose.position.x, current_pose.position.y, z_pose, planning_time, pose_tol, ori_tol)
        
        if lifted:
            print(colored('Successfully lifted gripper', 'cyan'))
            # print("Successfully lifted gripper")
            
        else:
            print(colored('Failed to lift gripper', 'red'))
            # print("Failed to lift gripper")

        return lifted


    # (JK) Untested
    def display_trajectory(self, plan):

        """
        Display a movement plan / trajectory
        @param: plan: Plan to be displayed
        """

        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory.append(plan)

        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory_publisher.publish(display_trajectory)


    def goto_home(self, tol=0.01, planning_time=5.0):

        group = self.group
        group.set_start_state_to_current_state()
        current_joints = group.get_start_state().joint_positions

        home_joint_angles = [0.0, -1.74533, 1.5708, 0.0, 1.74533, 0.0]
        
        joint_angles = {'joint_1': home_joint_angles[0],
                        'joint_2': home_joint_angles[1],
                        'joint_3': home_joint_angles[2],
                        'joint_4': home_joint_angles[3],
                        'joint_5': home_joint_angles[4],
                        'joint_6': home_joint_angles[5]}
        
        diff = abs(joint_angles['joint_1']-current_joints['joint_1']) > tol or \
            abs(joint_angles['joint_2']-current_joints['joint_2']) > tol or \
            abs(joint_angles['joint_3']-current_joints['joint_3']) > tol or \
            abs(joint_angles['joint_4']-current_joints['joint_4']) > tol or \
            abs(joint_angles['joint_5']-current_joints['joint_5']) > tol or \
            abs(joint_angles['joint_6']-current_joints['joint_6']) > tol

        reached = self.go_to_joint_goal(joint_angles, planning_time, tol)

        if reached:
            print(colored('Successfully reached Home', 'cyan'))
            # print("Successfully reached Home")
            
        else:
            print(colored('Failed to reach Home', 'red'))
            # print("Failed to reach Home")

        return reached


    def view(self, tol=0.01, planning_time=10.0):
        
        view_joint_angles = [-0.02949, -0.9858265, 1.1699174, 0.0400781, -1.7028536, 0.06918445]
        
        joint_angles = {'joint_1': view_joint_angles[0],
                        'joint_2': view_joint_angles[1],
                        'joint_3': view_joint_angles[2],
                        'joint_4': view_joint_angles[3],
                        'joint_5': view_joint_angles[4],
                        'joint_6': view_joint_angles[5]}

        reached = self.go_to_joint_goal(joint_angles, planning_time, tol)
        
        if reached:
            print(colored('Successfully reached View', 'cyan'))
            # print("Successfully reached Home")
            
        else:
            print(colored('Failed to reach View', 'red'))
            # print("Failed to reach Home")
        
        return reached


    def goto_bin(self, tol=0.01, planning_time=10.0):
        
        bin_joint_angles = [-1.5708, -1.13446, 1.48353, 0.0, 1.22173, -1.5708]

        joint_angles = {'joint_1': bin_joint_angles[0],
                        'joint_2': bin_joint_angles[1],
                        'joint_3': bin_joint_angles[2],
                        'joint_4': bin_joint_angles[3],
                        'joint_5': bin_joint_angles[4],
                        'joint_6': bin_joint_angles[5]}
            
        reached = self.go_to_joint_goal(joint_angles, planning_time, tol)    

        if reached:
            print(colored('Successfully reached Bin', 'cyan'))
            # print("Successfully reached Bin")
            
        else:
            print(colored('Failed to reach Bin', 'red'))
            # print("Failed to reach Bin")

        return reached


    def goto_placeOnConv(self, tol=0.01, planning_time=10.0):
            
        conv_joint_angles = [-0.837758, -0.523599, 0.523599, 0.0, 1.5708, 0.0]

        joint_angles = {'joint_1': conv_joint_angles[0],
                        'joint_2': conv_joint_angles[1],
                        'joint_3': conv_joint_angles[2],
                        'joint_4': conv_joint_angles[3],
                        'joint_5': conv_joint_angles[4],
                        'joint_6': conv_joint_angles[5]}

        reached = self.go_to_joint_goal(joint_angles, planning_time, tol)

        if reached:
            print(colored('Successfully reached Conveyor Approach', 'cyan'))
            # print("Successfully reached Conveyor Approach")
            
        else:
            print(colored('Failed to reach Conveyor Approach', 'red'))
            # print("Failed to reach Conveyor Approach")

        return reached


    def placeOnConveyor(self, pose_tol=0.01, ori_tol=0.1, planning_time=5.0):

        import random
        import time
        from statistics import mean

        miny = 0.42
        maxy = 0.32
        minx = 0.3
        maxx = 0.5
        not_updated = True

        with self.planning_scene.read_only() as scene:
            current_state = scene.current_state
            current_pose = current_state.get_pose('gripper_base_link')

        ori = current_pose.orientation

        while not_updated:

            random.seed(time.time())
            yval = random.normalvariate(mean([miny, maxy]), abs(maxy - miny) / 3)
            xval = random.normalvariate(mean([minx, maxx]), abs(maxx - minx) / 3)

            if len(self.prev_placexs) and len(self.prev_placeys):
                if [abs(xval - prevx) > 0.055 for prevx in self.prev_placexs] and [abs(yval - prevy) > 0.055 for prevy in self.prev_placeys]:
                    print(colored(f"Got new place coordinates! x = {xval}, y = {yval}", "yellow"))
                    print(colored(f"prev_xs: {self.prev_placexs}, prev_ys {self.prev_placeys}\n", "yellow"))
                    # print(f"Got new place coordinates! x = {xval}, y = {yval}", "yellow")
                    # print(f"prev_xs: {self.prev_placexs}, prev_ys {self.prev_placeys}\n", "yellow")
                    self.prev_placexs.append(xval)
                    self.prev_placeys.append(yval)
                    not_updated = False

                else:
                    not_updated = True

            else: 
                not_updated = False
                print(colored(f"First time place coordinates... x: {xval}, y: {yval}", "blue"))
                # print(f"First time place coordinates... x: {xval}, y: {yval")
                self.prev_placexs.append(xval)
                self.prev_placeys.append(yval)

        onConveyor1 = self.go_to_pose_goal(ori.x,ori.y,ori.z,ori.w, xval, yval, current_pose.position.z, planning_time, pose_tol, ori_tol)

        dip = 0.1

        if self.gazebo:
            dip = 0.01

        onConveyor2 = self.go_to_pose_goal(ori.x,ori.y,ori.z,ori.w, xval, yval, current_pose.position.z - dip, planning_time, pose_tol, ori_tol)

        if onConveyor2:
            print(colored('Successfully reached Conveyor', 'cyan'))
            # print('Successfully reached Conveyor')
            
        else:
            print(colored('Failed to reach Conveyor', 'red'))
            # print('Failed to reach Conveyor')

        return onConveyor2

############################################## END OF CLASS ##################################################


# Main for testing
def main(args=None):

    node = PickAndPlace(pipeline='multi', gazebo=True)
    # group = node.group
    # group.set_start_state_to_current_state()
    # current_joints = group.get_start_state().joint_positions
    # print(current_joints)
    node.goto_home()
    # node.goto_bin()
    # node.view()
    # node.goto_placeOnConv()
    # node.placeOnConveyor()


if __name__ == '__main__':
    main()