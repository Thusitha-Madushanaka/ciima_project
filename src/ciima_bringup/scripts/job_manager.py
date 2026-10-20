#!/usr/bin/env python3
# MIT License
# 
# Copyright (c) 2025 Innobotics s.r.l
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import rclpy
from rclpy.node import Node
from ibt_ros2_interfaces.msg import *
from mengozzi_interfaces.srv import GetRobotPosInMap
from geometry_msgs.msg import PoseStamped, Pose
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
from geometry_msgs.msg import TwistStamped
from action_msgs.msg import GoalStatus
from enum import Enum
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener, TransformException, TransformStamped
from controller_manager_msgs.srv import SwitchController
from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import String, Float64
import time

from enum import Enum

class AutomationStatus(Enum):
    UNDEFINED = 0
    EMERGENCY = 1
    MANUAL = 2
    RUNNING = 3

class Job(Enum):
    UNDEFINED = 0
    GO_ABSOLUTE = 1
    GO_RELATIVE = 2
    CANCEL_PENDING = 3

class MessageToPlc(Enum):
    UNDEFINED = 0
    INITIALIZED = 1
    JOBFINISHED = 2

class MovementPolicy(Enum):
    UNDEFINED = 0
    ABSOLUTE = 1
    FOLLOW_PARALLEL = 2
    GO_STRAIGHT = 3

def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def yaw_to_quaternion(yaw):
    half_yaw = yaw / 2.0
    qx = 0.0
    qy = 0.0
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)
    return (qx, qy, qz, qw)

def saturate_symmetric(x, limit):
    return max(-limit, min(x, limit))

class ControllerState:
    def __init__(self):
        self.name = 'diffbot_base_controller'
        self.state = 'active'


class JobManager(Node):
    def __init__(self):
        super().__init__('job_manager')

        self.create_subscription(RosComOut,'/from_plc', self.from_plc_c, 1)
        self.to_plc_pub = self.create_publisher(RosComIn, '/to_plc', 1)
        self.to_plc_msg : RosComIn = RosComIn()
        
        self.create_subscription(ComputedErrors, '/computed_errors', self.computed_errors_cb, 5)

        self.create_subscription(Float64, '/front_min_distance', self.front_min_distance_cb, 5)

        self.clearlocal_client = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.clearglobal_client = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')

        while not self.clearlocal_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('In attesa del servizio /local_costmap/clear_entirely_local_costmap')

        #self.clear_timer = self.create_timer(10.0, self.clear_costmap)

        # switch controller
        self.cli = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('In attesa del servizio /controller_manager/switch_controller...')
        self.controller_state : ControllerState = ControllerState()
        self.future_switch_service_running = False

        self._send_goal_future = None
        self.first_init = True

        # Listener TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._action_client_busy = False
        self.goal_handle = None
        self._pose_pub = self.create_publisher(PoseStamped, 'goal_pose_debug', 10)

        self.from_plc : RosComOut = None
        self.computed_errors : ComputedErrors = None
        self.front_dist : Float64 = None

        self.actual_pose_timer = self.create_timer(0.5, self.updateRobotToMap)
        self.actual_robot_pose : TransformStamped = None

        self.getrobotpos_service = self.create_service(
            GetRobotPosInMap,
            'get_robot_pos_in_map',
            self.get_robot_pos_callback
        )

        # TODO:remove
        # home pose respect to 'constructa_map' frame
        # self.home_pose : Pose = Pose()
        # self.home_pose.position.x = 2.236
        # self.home_pose.position.y = -0.708
        # quat = yaw_to_quaternion( 0.0 )
        # self.home_pose.orientation.x = float(quat[0])
        # self.home_pose.orientation.y = float(quat[1])
        # self.home_pose.orientation.z = float(quat[2])
        # self.home_pose.orientation.w = float(quat[3])
        # self.HALF_FOOTPRINT_LENGTH = 1.0
        self.actual_job : Job = Job.UNDEFINED

        self.homeposx = 2.050
        self.homeposy = -0.550 #-0.760

        self.THETA_MAX_DEG = 3.0 # angolo max di “manovra” consentito (tuning)

    def set_controller_state(self, controller_name: str, state: str):
        req = SwitchController.Request()

        if state == "active":
            req.activate_controllers = [controller_name]
            req.deactivate_controllers = []
        elif state == "inactive":
            req.activate_controllers = []
            req.deactivate_controllers = [controller_name]
        else:
            self.get_logger().error(f"Stato non valido: {state}")
            return

        req.strictness = True
        req.timeout.sec = 5

        future = self.cli.call_async(req)
        self.future_switch_service_running = True
        future.add_done_callback(lambda fut: self._handle_switch_result(fut, controller_name, state))

    def _handle_switch_result(self, future, controller_name: str, state: str):
        self.future_switch_service_running = False
        """Callback chiamata quando il servizio risponde."""
        try:
            result = future.result()
            if result.ok:
                self.controller_state.state = state
                self.get_logger().info(f"✅ Controller '{controller_name}' impostato su '{state}'.")
            else:
                self.get_logger().error(f"❌ Errore nel cambiare stato del controller '{controller_name}'.")
        except Exception as e:
            self.get_logger().error(f"❌ Errore nella chiamata del servizio: {e}")

    def from_plc_c(self, msg: RosComOut):
        self.from_plc = msg

        # emergency button management -> switch state of diff drive controller
        if self.first_init:
            self.controller_state.state = 'active' if msg.automation_status == AutomationStatus.RUNNING.value else 'inactive'
            self.first_init = False

        if self.future_switch_service_running:
            return

        if msg.automation_status != AutomationStatus.RUNNING.value:
            if self.controller_state.state == 'active':
                pass
                #self.cancel_goal()
                #self.set_controller_state('diffbot_base_controller', 'inactive')
            self.controller_state.state = 'inactive'

        elif msg.automation_status == AutomationStatus.RUNNING.value:
            #if self.controller_state.state == 'inactive':
                #self.set_controller_state('diffbot_base_controller', 'active')
            self.controller_state.state = 'active'


        # mission management with acks
        if self.actual_job == Job.UNDEFINED:
            self.to_plc_msg.st_area_dati_input.ib_ready_to_new_job = True

        # new job recevied
        if self.from_plc.st_area_dati_output.ob_new_job:
            self.actual_job = Job(self.from_plc.st_area_dati_output.oi_job_num)
            self.to_plc_msg.st_area_dati_input.ib_ready_to_new_job = False 
            self.to_plc_msg.st_area_dati_input.ib_ack_new_job = True

        # Ack for the new job recieved
        if self.to_plc_msg.st_area_dati_input.ib_ack_new_job == True and self.from_plc.st_area_dati_output.ob_new_job == False:
            self.to_plc_msg.st_area_dati_input.ib_ack_new_job = False

        # Ack for the new message sent
        if self.to_plc_msg.st_area_dati_input.ib_new_message == True and  self.from_plc.st_area_dati_output.ob_ack_new_message == True:
            self.to_plc_msg.st_area_dati_input.ib_new_message = False

        
        if self.actual_job == Job.CANCEL_PENDING:
            self.cancel_goal()

        # TODO: send action goal
        #if self._action_client_busy:
            #return
        
        if self.actual_job == Job.GO_ABSOLUTE and not self._action_client_busy:
            self.get_logger().info("Navigation with 'constructa_map' frame")
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'constructa_map'
            pose.pose.position.x = self.from_plc.st_area_dati_output.or_data1
            pose.pose.position.y = self.from_plc.st_area_dati_output.or_data2
            quat = yaw_to_quaternion( math.radians( self.from_plc.st_area_dati_output.or_data3 ) )
            pose.pose.orientation.x = float(quat[0])
            pose.pose.orientation.y = float(quat[1])
            pose.pose.orientation.z = float(quat[2])
            pose.pose.orientation.w = float(quat[3])
            self.send_goal(pose)
        
        elif self.actual_job == Job.GO_RELATIVE and not self._action_client_busy:
            movementpolicy = MovementPolicy(int(self.from_plc.st_area_dati_output.or_data1))
            if movementpolicy == MovementPolicy.ABSOLUTE:
                self.get_logger().info("Relative Navigation with 'constructa_map' frame")
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'constructa_map'
                if self.actual_robot_pose:
                    
                    # robot pose attuale in map
                    xr = self.actual_robot_pose.transform.translation.x
                    yr = self.actual_robot_pose.transform.translation.y

                    # yaw robot in map
                    yaw = quaternion_to_yaw(self.actual_robot_pose.transform.rotation.x, 
                                        self.actual_robot_pose.transform.rotation.y,
                                        self.actual_robot_pose.transform.rotation.z,
                                        self.actual_robot_pose.transform.rotation.w)

                    # goal "desiderato" in map
                    xg = xr + self.from_plc.st_area_dati_output.or_data2
                    yg = self.homeposy

                    # delta in map
                    dx = xg - xr
                    dy = yg - yr

                    # ruota delta in frame robot (body)
                    cy = math.cos(yaw)
                    sy = math.sin(yaw)
                    xb =  cy * dx + sy * dy
                    yb = -sy * dx + cy * dy

                    # limita yb in base a theta_max
                    k = math.tan(math.radians(self.THETA_MAX_DEG))
                    y_limit = abs(xb) * k
                    yb = saturate_symmetric(yb, y_limit)

                    # ritorna in map
                    dx_limited = cy * xb - sy * yb
                    dy_limited = sy * xb + cy * yb

                    # goal finale limitato (in map)
                    pose.pose.position.x = xr + dx_limited
                    pose.pose.position.y = yr + dy_limited

                    # orientamento (nel tuo snippet era 0 deg)
                    quat = yaw_to_quaternion(math.radians(0.0))
                    pose.pose.orientation.x = float(quat[0])
                    pose.pose.orientation.y = float(quat[1])
                    pose.pose.orientation.z = float(quat[2])
                    pose.pose.orientation.w = float(quat[3])

                    self.send_goal(pose)


                    #pose.pose.position.x = self.actual_robot_pose.transform.translation.x + self.from_plc.st_area_dati_output.or_data2
                    #pose.pose.position.y = self.actual_robot_pose.transform.translation.y
                    #pose.pose.position.y = self.homeposy
                    #pose.pose.orientation = self.actual_robot_pose.transform.rotation
                    #quat = yaw_to_quaternion( math.radians(0.0) )
                    #self.send_goal(pose)
                else:
                    self.get_logger().warn("Cannot perform relative navigation: no valid robot pose in map!")
                    self.to_plc_msg.st_area_dati_input.ii_message_num = MessageToPlc.JOBFINISHED.value
                    self.to_plc_msg.st_area_dati_input.ir_data1 = 1.0
                    self.to_plc_msg.st_area_dati_input.ir_data2 = float(self.actual_job.value)
                    self.to_plc_msg.st_area_dati_input.ib_new_message = True
                    self.actual_job = Job.UNDEFINED
            elif movementpolicy == MovementPolicy.FOLLOW_PARALLEL:
                self.get_logger().info("Relative Navigation with 'tracked_base_link' frame, follow walls")
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'tracked_base_link'
                #pose.pose.position.x = self.from_plc.st_area_dati_output.or_data2
                #pose.pose.position.y = self.computed_errors.error_lateral_m

                error_angle = saturate_symmetric(self.computed_errors.error_angle_deg, 3.0)

                pose.pose.position.x = self.from_plc.st_area_dati_output.or_data2 * math.cos(math.radians(-error_angle)) + \
                                       self.computed_errors.error_lateral_m * math.sin(math.radians(-error_angle))
                pose.pose.position.y = -self.from_plc.st_area_dati_output.or_data2*math.sin(math.radians(-error_angle)) + \
                                        self.computed_errors.error_lateral_m * math.cos(math.radians(-error_angle))
                x = pose.pose.position.x
                y = pose.pose.position.y
                k = math.tan(math.radians(self.THETA_MAX_DEG))
                # limite laterale dipendente da x e dall'angolo massimo
                y_limit = abs(x) * k
                pose.pose.position.y = saturate_symmetric(y, y_limit)

                quat = yaw_to_quaternion(math.radians(error_angle))
                pose.pose.orientation.x = float(quat[0])
                pose.pose.orientation.y = float(quat[1])
                pose.pose.orientation.z = float(quat[2])
                pose.pose.orientation.w = float(quat[3])
                self.send_goal(pose)
            elif movementpolicy == MovementPolicy.GO_STRAIGHT:
                self.get_logger().info("Relative Navigation with 'tracked_base_link' frame, only straight")
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'tracked_base_link'
                pose.pose.position.x = self.from_plc.st_area_dati_output.or_data2
                pose.pose.position.y = 0.0
                quat = yaw_to_quaternion(0.0)
                pose.pose.orientation.x = float(quat[0])
                pose.pose.orientation.y = float(quat[1])
                pose.pose.orientation.z = float(quat[2])
                pose.pose.orientation.w = float(quat[3])
                self.send_goal(pose)

        # update actual position
        if self.actual_robot_pose:
            self.to_plc_msg.st_explicit_signals.i_current_x_pos = self.actual_robot_pose.transform.translation.x
            self.to_plc_msg.st_explicit_signals.i_current_y_pos = self.actual_robot_pose.transform.translation.y
            actual_yaw = quaternion_to_yaw(self.actual_robot_pose.transform.rotation.x, 
                                        self.actual_robot_pose.transform.rotation.y,
                                        self.actual_robot_pose.transform.rotation.z,
                                        self.actual_robot_pose.transform.rotation.w)
            if self.front_dist:
                dist_x = self.front_dist.data
            else:
                dist_x = 20.0

            self.to_plc_msg.st_explicit_signals.i_dist_from_front = dist_x
            self.to_plc_msg.st_explicit_signals.i_current_yaw = math.degrees(actual_yaw)
            self.to_plc_msg.st_explicit_signals.i_valid_map = True
        else:
            self.to_plc_msg.st_explicit_signals.i_valid_map = False
            self.to_plc_msg.st_explicit_signals.i_dist_from_front = 0.0


        # TODO: update valid map

        # update plc output variables
        self.to_plc_pub.publish(self.to_plc_msg) 

    def computed_errors_cb(self, msg: ComputedErrors):
        self.computed_errors = msg

    def front_min_distance_cb(self, msg: Float64):
        self.front_dist = msg

    def updateRobotToMap(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'constructa_map',    
                'tracked_base_link',    
                rclpy.time.Time())
            self.actual_robot_pose = t
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform constructa_map to tracked_base_link: {ex}')
            self.actual_robot_pose = None

    def send_goal(self, pose: PoseStamped):
        self.clear_costmap()
        self.get_logger().info(f'Sending goal: {pose.pose.position.x}, {pose.pose.position.y}')
        self._goal_state = GoalStatus.STATUS_UNKNOWN
        self._action_client_busy = True
        self._pose_pub.publish(pose)

        # Aspettiamo che il server sia pronto
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        # Inviare goal e registrare callback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self._goal_state = GoalStatus.STATUS_ABORTED
            self._action_client_busy = False
            return

        self.get_logger().info('Goal accepted :)')
        self._goal_state = GoalStatus.STATUS_ACCEPTED
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result received: {result}')
        self._goal_state = GoalStatus.STATUS_SUCCEEDED
        self._action_client_busy = False

        self.to_plc_msg.st_area_dati_input.ii_message_num = MessageToPlc.JOBFINISHED.value
        #self.to_plc_msg.st_area_dati_input.ii_message_num = 2
        self.to_plc_msg.st_area_dati_input.ir_data1 = 0.0 if result.error_code == 0 else 1.0
        self.to_plc_msg.st_area_dati_input.ir_data2 = float(self.actual_job.value)
        self.to_plc_msg.st_area_dati_input.ib_new_message = True
        self.actual_job = Job.UNDEFINED

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._goal_state = GoalStatus.STATUS_EXECUTING
        self._action_client_busy = True
        #self.get_logger().info(f'Current pose: {feedback.current_pose.pose.position.x}, {feedback.current_pose.pose.position.y}')

    def cancel_goal(self):
        if self.goal_handle is not None:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().warn('Nessun goal da cancellare.')

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.to_plc_msg.st_area_dati_input.ir_data1 = 0.0
            self.get_logger().info('✅ Goal cancellato con successo.')
        else:
            self.to_plc_msg.st_area_dati_input.ir_data1 = 1.0
            self.get_logger().warn('⚠️ Nessun goal cancellato — potrebbe essere già terminato.')
        self.to_plc_msg.st_area_dati_input.ii_message_num = MessageToPlc.JOBFINISHED.value
        self.to_plc_msg.st_area_dati_input.ir_data2 = float(self.actual_job.value)
        self.to_plc_msg.st_area_dati_input.ib_new_message = True
        self.actual_job = Job.UNDEFINED

    def clear_costmap(self):
        req = ClearEntireCostmap.Request()
        future = self.clearlocal_client.call_async(req)

    def get_robot_pos_callback(self, request, response):

        if self.actual_robot_pose:
            response.valid_map = True
            response.current_x_pos = self.actual_robot_pose.transform.translation.x
            response.current_y_pos = self.actual_robot_pose.transform.translation.y
            actual_yaw = quaternion_to_yaw(self.actual_robot_pose.transform.rotation.x, 
                                        self.actual_robot_pose.transform.rotation.y,
                                        self.actual_robot_pose.transform.rotation.z,
                                        self.actual_robot_pose.transform.rotation.w)
            response.current_yaw = math.degrees(actual_yaw)
        else:
            response.valid_map = False
            response.current_x_pos = 0.0
            response.current_y_pos = 0.0
            response.current_yaw = 0.0
            
        self.get_logger().info("Posizione robot richiesta")
        return response


    
def main(args=None):
    rclpy.init(args=args)
    node = JobManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()