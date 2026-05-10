#  BSD 3-Clause License
#
#  Copyright (c) 2026, National Institute of Advanced Industrial Science
#  and Technology(AIST)
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
#  OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
#  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
#  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#  Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
#
from rclpy.parameter_client       import AsyncParameterClient
from rclpy.callback_groups        import MutuallyExclusiveCallbackGroup
from action_msgs.msg              import GoalStatus
from control_msgs.action          import GripperCommand
from control_msgs.msg             import GripperCommand as GripperCommandMsg
from aist_robotiq_msgs.srv        import SetVelocity
from aist_robotiq_msgs.action     import SetMode
from aist_robotiq_msgs.action     import SuctionCommand
from aist_robotiq_msgs.msg        import SuctionCommand as SuctionCommandMsg
from task_wrappers.service_client import ServiceClient
from task_wrappers.action_client  import SimpleActionClient

#************************************************************************
#  class RobotiqGripper                                                 *
#************************************************************************
class RobotiqGripper(SimpleActionClient):
    """ Action client of the controller for Robotiq grippers.
    """
    def __init__(self, node, name='a_bot_gripper', max_effort=0.0):
        """ Create a RobotiqGripper client.

        :param node: The ROS node to add the suction tool client to.
        :param name: Name of the suction tool
        :param max_effort: Maximum effort to be applied when grasping.
        """
        self._name    = name
        controller_ns = name + '_controller'

        self._callback_group = MutuallyExclusiveCallbackGroup()
        super().__init__(node, GripperCommand, controller_ns + '/gripper_cmd',
                         self._callback_group)

        # Create service client for setting velocity.
        self._set_velocity = ServiceClient(node, SetVelocity,
                                           controller_ns + '/set_velocity',
                                           self._callback_group)

        # Get properties for computing gap values from the controller.
        self._properties = {'max_effort': max_effort}
        self._get_controller_parameters(node, controller_ns)

        # Create action client for switching mode.
        self._mode = SetMode.Goal.BASIC
        self._individual_control_fingers = True
        self._individual_control_scissor = True
        self._set_mode = SimpleActionClient(node, SetMode,
                                            controller_ns + '/set_mode',
                                            self._callback_group)

    @property
    def name(self):
        return self._name

    @property
    def base_link(self):
        return self._name + '_base_link'

    @property
    def tip_link(self):
        return self._name + '_tip_link'

    @property
    def properties(self):
        """ Return a dictionary of gripper properties

        :return: Dictionary of gripper properties with string keys.
        """
        return self._properties

    def pregrasp(self):
        self.release(0.0)

    def grasp(self, timeout_sec=None):
        """ Grasp an object with the gripper.

        Desired finger position and applied effort are specified by properties
        with ``grasp_position`` and ``max_effort`` keys, respectively.

        :param timeout_sec:
          - Seconds to wait until the gripper complets movement, if positive.
          - Wait forever, if ``None``.
          - Return immediately without waiting for competion,
            if zero or negative.
        :return: A tuple of the goal status and the movement result of
            control_msgs.action.GripperCommand.Result type
        """
        return self.move(self.properties['grasp_position'],
                         self.properties['max_effort'], timeout_sec)

    def postgrasp(self):
        self.grasp(0.0)

    def release(self, timeout_sec=None):
        """ Release an object grasped by the gripper.

        Desired finger position is specified by a parameter
        with ``release_positio``' key. No effort is applied.

        :param timeout_sec:
          - Seconds to wait until the gripper complets movement, if positive.
          - Wait forever, if ``None``.
          - Return immediately without waiting for competion,
            if zero or negative.
        :return: A tuple of the goal status and the movement result of
            control_msgs.action.GripperCommand.Result type
        """
        return self.move(self.properties['release_position'], 0.0, timeout_sec)

    def move(self, gap, max_effort=0.0, timeout_sec=None):
        """ Move gripper to the desired position.

        :param gap: Desired gap between the fingers.
         param max_effort: Desired maximum effort to be applied.
        :param timeout_sec:
          - Seconds to wait until the gripper complets movement, if positive.
          - Wait forever, if ``None``.
          - Return immediately without waiting for competion,
            if zero or negative.
        :return: A tuple of the goal status and the movement result of
            control_msgs.action.GripperCommand.Result type
        """
        return self.send_goal(GripperCommand.Goal(
                                  command=GripperCommandMsg(
                                      position=self._position(gap),
                                      max_effort=max_effort)),
                              timeout_sec=timeout_sec)

    def wait(self, timeout_sec=None):
        """ Wait for the result of gripper command or cancel request.

        Wait until the result of the gripper command or a cancel request
        issued by `cancel()` becomes available.

        :param timeout_sec:
          - Seconds to wait, if positive.
          - Wait forever, if ``None``.
          - Return immediately without waiting, if zero or negative.
        :return:
          - A tuple of the goal status and the gripper command/cancel result,
            if the result becomes available within ``timeout_sec``.
          - A tuple of the current (non-terminal) goal state
            and ``None``. otherwise.
        """
        status, result = super().wait(timeout_sec)
        if result is not None:
             # Convert joint angle to gap.
            result.position = self._gap(result.position)
        return status, result

    def set_velocity(self, velocity):
        """ Set finger velocity value to the gripper.
        """
        return self._set_vel_clnt.call(SetVelocity.Request(velocity=velocity))

    def set_mode(self, mode, individual_control_fingers=False,
                 individual_control_scissor=False, timeout_sec=None):
        """ Set operation mode of the gripper.

        This fuction is effective only for Robotiq-3D grippers.

        :param mode:
        :param individual_control_fingers:
        :param individual_control_scissor:
        """
        status, result \
            = self._set_mode_clnt.send_goal(
                  SetMode.Goal(
                      mode=mode,
                      individual_control_fingers=individual_control_fingers,
                      individual_control_scissor=individual_control_scissor),
                  timeout_sec=timeout_sec)
        if status == GoalStatus.STATUS_SUCCEEDED and result.success:
            self._mode = mode
            self._individual_control_fingers = individual_control_fingers
            self._individual_control_scissor = individual_control_scissor
            return True
        else:
            return False

    def _get_controller_parameters(self, node, controller_ns):
        def _get_parameters_cb(future):
            values = future.result().values
            self._min_gap      = values[0].double_array_value
            self._max_gap      = values[1].double_array_value
            self._min_position = values[2].double_array_value
            self._max_position = values[3].double_array_value
            self._properties['grasp_position']   = self._min_gap[0]
            self._properties['release_position'] = self._max_gap[0]

        AsyncParameterClient(node, controller_ns) \
            .get_parameters(['min_gap', 'max_gap',
                             'min_position', 'max_position'],
                            _get_parameters_cb)

    def _position(self, gap):
        idx = self._idx()
        return (gap - self._min_gap[idx]) * self._position_per_gap(idx) \
             + self._min_position[idx]

    def _gap(self, position):
        idx = self._idx()
        return (position - self._min_position[idx]) \
             / self._position_per_gap(idx) + self._min_gap[idx]

    def _position_per_gap(self, idx):
        return (self._max_position[idx] - self._min_position[idx]) \
             / (self._max_gap[idx]      - self._min_gap[idx])

    def _idx(self):
        return 3 if self._mode == SetMode.Goal.SCISSOR else 0

#************************************************************************
#  class RobotiqSuction                                                 *
#************************************************************************
class RobotiqSuction(SimpleActionClient):
    """ Action client of controller for Robotiq EPick grippers.
    """
    def __init__(self, node, name='a_bot_gripper', advanced_mode=True,
                 grasp_pressure=-78.0, detection_pressure=-10.0,
                 release_pressure=0.0, grasp_timeout_sec=0.0):
        """ Create a RobotiqSuction client.

        :param node: The ROS node to add the suction tool client to.
        :param name: Name of the gripper.
        :param advanced_mode: If ``True``, operates in advanced mode.
            Otherwise, operates in test mode
        """
        self._name           = name
        self._callback_group = MutuallyExclusiveCallbackGroup()
        super().__init__(node, SuctionCommand,
                         name + '_controller/gripper_cmd',
                         self._callback_group)

        self._properties = {'advanced_mode':      advanced_mode,
                            'grasp_pressure':     grasp_pressure,
                            'detection_pressure': detection_pressure,
                            'release_pressure':   release_pressure,
                            'grasp_timeout':      grasp_timeout_sec}

    @property
    def name(self):
        return self._name

    @property
    def base_link(self):
        return self._name + '_base_link'

    @property
    def tip_link(self):
        return self._name + '_tip_link'

    @property
    def properties(self):
        """Return a dictionary of gripper properties

        :return: Dictionary of gripper properties with string keys.
        """
        return self._properties

    def pregrasp(self):
        self.suck(self.properties['grasp_pressure'],
                  self.properties['detection_pressure'], 0.0, 0.0)

    def grasp(self, timeout_sec=None):
        """ Grasp an object with the gripper.

        Pressure applied and pressure threshold for object detection are
        specified by properties ``grasp_pressure`` and ``detection_pressure``,
        respectively.

        :param timeout_sec:
          - Seconds to wait, if positive.
          - Wait forever, if ``None``.
          - Return immediately without waiting, if zero or negative.
        :return:
          - A tuple of the goal status and the gripper command/cancel result,
            if the result becomes available within ``timeout_sec``.
          - A tuple of the current (non-terminal) goal state
            and ``None``. otherwise.
        """
        return self.suck(self.properties['grasp_pressure'],
                         self.properties['detection_pressure'],
                         self.properties['grasp_timeout'],
                         timeout_sec)

    def postgrasp(self):
        self.pregrasp()

    def release(self, timeout_sec=None):
        """ Release an object grasped by the gripper.

        Value of applied pressure is specified by a parameter
        ``release_pressure`` which should be non-negative.

        :param timeout_sec:
          - Seconds to wait, if positive.
          - Wait forever, if ``None``.
          - Return immediately without waiting, if zero or negative.
        :return:
          - A tuple of the goal status and the gripper command/cancel result,
            if the result becomes available within ``timeout_sec``.
          - A tuple of the current (non-terminal) goal state
            and ``None``. otherwise.
        """
        return self.suck(self.properties['release_pressure'],
                         self.properties['detection_pressure'],
                         self.properties['grasp_timeout'],
                         timeout_sec)

    def suck(self, max_pressure, min_pressure=None, grasp_timeout_sec=None,
             timeout_sec=None):
        """ Generate pressure.

        :param max_pressure: Maximum pressure value applied
        :param min_pressure: Minimum pressure value for object detection
        :param timeout_sec:
          - Seconds to wait, if positive.
          - Wait forever, if ``None``.
          - Return immediately without waiting, if zero or negative.
        :return:
          - A tuple of the goal status and the gripper command/cancel result,
            if the result becomes available within ``timeout_sec``.
          - A tuple of the current (non-terminal) goal state
            and ``None``. otherwise.
        """
        if not min_pressure:
            min_pressure = self.properties['detection_pressure']
        if not grasp_timeout_sec:
            grasp_timeout_sec = self.properties['grasp_timeout']
        return self.send_goal(
                   SuctionCommand.Goal(
                       command=SuctionCommandMsg(
                           advanced_mode=self.properties['advanced_mode'],
                           max_pressure=max_pressure,
                           min_pressure=min_pressure,
                           timeout=grasp_timeout_sec)),
                   timeout_sec=timeout_sec)
