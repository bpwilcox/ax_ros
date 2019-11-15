import rclpy
from rclpy.node import Node
from ax_ros_interfaces.srv import RunAxTrial
import numpy as np
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from .gazebo_interface import GazeboInterface

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.srv import ClearEntireCostmap

# import tf2_geometry_msgs

import argparse
import math
import sys
import time
from typing import Optional

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
class NavTester(Node):

    def __init__(
        self,
        initial_pose: Pose,
        goal_pose: Pose,
        namespace: str = ''
    ):
        super().__init__(node_name='nav2_tester', namespace=namespace)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
          history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          depth=1)

        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                       'amcl_pose', self.poseCallback, pose_qos)
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self.goal_pose = goal_pose
        self.action_client = ActionClient(self, NavigateToPose, 'NavigateToPose')
        self.clear_local = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.clear_global = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.time_to_goal = -1.0

    def info_msg(self, msg: str):
        self.get_logger().info('\033[1;37;44m' + msg + '\033[0m')

    def warn_msg(self, msg: str):
        self.get_logger().warn('\033[1;37;43m' + msg + '\033[0m')

    def error_msg(self, msg: str):
        self.get_logger().error('\033[1;37;41m' + msg + '\033[0m')

    def setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        self.info_msg('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        self.currentPose = self.initial_pose

    def getStampedPoseMsg(self, pose: Pose):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose = pose
        return msg

    def publishGoalPose(self, goal_pose: Optional[Pose] = None):
        self.goal_pose = goal_pose if goal_pose is not None else self.goal_pose
        self.goal_pub.publish(self.getStampedPoseMsg(self.goal_pose))

    def runNavigateAction(self, goal_pose: Optional[Pose] = None):
        # Sends a `NavToPose` action request and waits for completion
        self.info_msg("Waiting for 'NavigateToPose' action server")
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.info_msg("'NavigateToPose' action server not available, waiting...")

        self.goal_pose = goal_pose if goal_pose is not None else self.goal_pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.getStampedPoseMsg(self.goal_pose)

        self.info_msg('Sending goal request...')
        send_goal_future = self.action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.error_msg('Goal rejected')
            return False

        self.info_msg('Goal accepted')
        get_result_future = goal_handle.get_result_async()

        self.info_msg("Waiting for 'NavigateToPose' action to complete") 
        start = time.time()
        rclpy.spin_until_future_complete(self, get_result_future)
        status = get_result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.info_msg('Goal failed with status code: {0}'.format(status))
            return False
        end = time.time()
        self.time_to_goal = float(end-start)
        self.info_msg('Goal succeeded!')
        return True

    def poseCallback(self, msg):
        self.info_msg('Received amcl_pose')
        self.current_pose = msg.pose.pose
        self.initial_pose_received = True

    def reachesGoal(self, timeout, distance):
        goalReached = False
        start_time = time.time()

        while not goalReached:
            rclpy.spin_once(self, timeout_sec=1)
            if self.distanceFromGoal() < distance:
                goalReached = True
                self.info_msg('*** GOAL REACHED ***')
                return True
            elif timeout is not None:
                if (time.time() - start_time) > timeout:
                    self.error_msg('Robot timed out reaching its goal!')
                    return False

    def distanceFromGoal(self):
        d_x = self.current_pose.position.x - self.goal_pose.position.x
        d_y = self.current_pose.position.y - self.goal_pose.position.y
        distance = math.sqrt(d_x * d_x + d_y * d_y)
        self.info_msg('Distance from goal is: ' + str(distance))
        return distance

    def wait_for_node_active(self, node_name: str):
        # Waits for the node within the tester namespace to become active
        self.info_msg('Waiting for ' + node_name + ' to become active')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(node_service + ' service not available, waiting...')
        req = GetState.Request()  # empty request
        state = 'UNKNOWN'
        while (state != 'active'):
            self.info_msg('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.info_msg('Result of get_state: %s' % state)
            else:
                self.error_msg('Exception while calling service: %r' % future.exception())
            time.sleep(5)

    def shutdown(self):
        self.info_msg('Shutting down')
        transition_service = 'lifecycle_manager/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(transition_service + ' service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().SHUTDOWN
        future = mgr_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

    def clear_costmap(self):
        self.info_msg('Clearing costmap')
        req = ClearEntireCostmap.Request()
        while not self.clear_global.wait_for_service(timeout_sec=1.0):
            self.tester.info_msg('Clear global costmap service is not available...')
        future = self.clear_global.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        while not self.clear_local.wait_for_service(timeout_sec=1.0):
            self.tester.info_msg('Clear local costmap service is not available...')
        future = self.clear_local.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

    def pause(self):
        self.info_msg('Resetting')
        transition_service = 'lifecycle_manager/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(transition_service + ' service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().PAUSE
        future = mgr_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

    def resume(self):
        self.info_msg('Starting Up')
        transition_service = 'lifecycle_manager/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(transition_service + ' service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().RESUME
        future = mgr_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

class NavAxTrialRunner(Node):

    def __init__(self):
        super().__init__('ax_test_server')
        self.gb_interface = GazeboInterface()
        self.srv = self.create_service(RunAxTrial, self.get_name() + '/run_ax_trial', self.ax_trial_callback)
        self.declare_parameter("initial_pose.x", -2.0)
        self.declare_parameter("initial_pose.y", -0.5)
        self.declare_parameter("initial_pose.theta", 0.0)
        self.declare_parameter("goal_pose.x", 0.5)
        self.declare_parameter("goal_pose.y", 1.5)
        self.declare_parameter("goal_pose.theta", 0.0)
        self.tester = NavTester(
            initial_pose = self.get_initial_pose(),
            goal_pose = self.get_goal_pose())

    def get_initial_pose(self):
        pose = Pose()
        x = self.get_parameter("initial_pose.x").value
        y = self.get_parameter("initial_pose.y").value
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.01
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        return pose

    def get_goal_pose(self):
        pose = Pose()
        x = self.get_parameter("goal_pose.x").value
        y = self.get_parameter("goal_pose.y").value
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.01
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        return pose

    def set_initial_pose(self, timeout, retries):
        self.tester.initial_pose_received = False
        retry_count = 1
        while not self.tester.initial_pose_received and retry_count <= retries:
            retry_count += 1
            self.tester.info_msg('Setting initial pose')
            self.tester.setInitialPose()
            self.tester.info_msg('Waiting for amcl_pose to be received')
            rclpy.spin_once(self.tester, timeout_sec=timeout)  # wait for poseCallback

        if (self.tester.initial_pose_received):
            self.tester.info_msg('set_initial_pose PASSED')
        else:
            self.tester.info_msg('set_initial_pose FAILED')
        return self.tester.initial_pose_received

    # def orientationAroundZAxis(self, angle):
    #     return angle

    def ax_trial_callback(self, request, response):
        # self.tester.pause()
        self.gb_interface.reset_gazebo_world()
        # self.tester.clear_costmap()
        # self.tester.resume()
        # self.tester.startup()
        # result = self.set_initial_pose(timeout = 1, retries = 10)
        # result = self.tester.runNavigateAction()
        result = True
        if (result):
            self.tester.wait_for_node_active('amcl')
            result = self.set_initial_pose(timeout = 1, retries = 10)
            time.sleep(4)
            self.tester.clear_costmap()
        if (result):
            self.tester.wait_for_node_active('bt_navigator')
        if (result):
            time.sleep(1)
            result = self.tester.runNavigateAction()
        if not result:
            response.valid_trial = False
            
        # self.gb_interface.set_entity_state_pose("turtlebot3_waffle", self.get_initial_pose())
        # result = self.set_initial_pose(timeout = 1, retries = 10)
        # print(result)
        response.result = [self.tester.time_to_goal]
        return response


def main(args=None):
    rclpy.init(args=args)

    nav2_trial_runner = NavAxTrialRunner()
    rclpy.spin(nav2_trial_runner)  
    rclpy.shutdown()

if __name__ == '__main__':
    main()
