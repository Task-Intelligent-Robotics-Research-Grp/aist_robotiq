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
import threading, copy
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
from ddynamic_reconfigure2.client import ParameterClient

from rclpy.node                   import Node
from typing                       import Optional

#************************************************************************
#  class RobotiqGripper                                                 *
#************************************************************************
class RobotiqGripper(SimpleActionClient):
    """ Action client of the controller for Robotiq grippers.
    """
    def __init__(self, node: Node, name: str='a_bot_gripper',
                 *, max_effort: float=0.0):
        """ Create a RobotiqGripper client.

        :param node: The ROS node to add the suction tool client to.
        :param name: Name of the suction tool
        :param max_effort: Maximum effort to be applied when grasping.
        """
        self._node    = node
        self._name    = name
        controller_ns = name + '_controller'

        # Create action client for gripper command.
        self._cbg = MutuallyExclusiveCallbackGroup()
        super().__init__(node, GripperCommand, controller_ns + '/gripper_cmd',
                         callback_group=self._cbg)

        # Create service client for setting velocity.
        self._set_velocity = ServiceClient(node, SetVelocity,
                                           controller_ns + '/set_velocity',
                                           callback_group=self._cbg)

        # Create action client for switching mode.
        self._mode = SetMode.Goal.BASIC
        self._individual_control_fingers = True
        self._individual_control_scissor = True
        self._set_mode = SimpleActionClient(node, SetMode,
                                            controller_ns + '/set_mode',
                                            callback_group=self._cbg)

        # These values are required for computing gap(in meters) from gripper's
        # position(in radians), which will be obtained from the controller
        # on demand.
        self._min_gap      = None
        self._max_gap      = None
        self._min_position = None
        self._max_position = None

        # Initialize property dictionary with initial max_effort value.
        # Other parameters, 'grasp_position' and 'release_position',
        # for computing gap values will be obtained from the controller.
        self._parameters = {'max_effort': max_effort}

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        if not self._min_gap:
            self._get_controller_parameters()
        return 'two_finger' if len(self._min_gap) == 1 else 'three_finger'

    @property
    def base_link(self):
        return self._name + '_base_link'

    @property
    def tip_link(self):
        return self._name + '_tip_link'

    @property
    def parameters(self):
        """ Return a dictionary of gripper parameters

        :return: Dictionary of gripper parameters with string keys.
        """
        if 'grasp_position' not in self._parameters:
            self._get_controller_parameters()
            self._parameters['grasp_position']   = self._min_gap[0]
            self._parameters['release_position'] = self._max_gap[0]
        return self._parameters

    def pregrasp(self):
        self.release(timeout_sec=0.0)

    def grasp(self, *, timeout_sec: Optional[float]=None):
        """ Grasp an object with the gripper.

        Desired finger position and applied effort are specified by parameters
        with ``grasp_position`` and ``max_effort`` keys, respectively.

        :param timeout_sec:
          - Seconds to wait until the gripper complets movement, if positive.
          - Wait forever, if ``None``.
          - Return immediately without waiting for competion,
            if zero or negative.
        :return: A tuple of the goal status and the movement result of
            control_msgs.action.GripperCommand.Result type
        """
        return self.move(self.parameters['grasp_position'],
                         max_effort=self.parameters['max_effort'],
                         timeout_sec=timeout_sec)

    def postgrasp(self):
        self.grasp(timeout_sec=0.0)

    def release(self, *, timeout_sec: Optional[float]=None):
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
        return self.move(self.parameters['release_position'],
                         max_effort=0.0, timeout_sec=timeout_sec)

    def move(self, gap: float, *,
             max_effort: float=0.0, timeout_sec: Optional[float]=None):
        """ Move gripper to the desired position.

        :param gap: Desired gap between the fingers.
        :param max_effort: Desired maximum effort to be applied.
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

    def wait(self, *, timeout_sec: Optional[float]=None):
        """ Wait for the result of gripper command or cancel request.

        Wait until the result of the gripper command or a cancel request
        issued by `cancel_goal()` becomes available.

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
        status, result = super().wait(timeout_sec=timeout_sec)
        if result is not None:
             # Convert joint angle to gap.
            result = copy.deepcopy(result)
            result.position = self._gap(result.position)
        return status, result

    def set_velocity(self, velocity: float):
        """ Set finger velocity value to the gripper.
        """
        return self._set_velocity.call(SetVelocity.Request(velocity=velocity))

    def set_max_effort(self, max_effort: float):
        """ Set maximum effort to be applied when grasping.

        :param max_effort: Maximum effort to be applied when grasping.
        """
        self._parameters['max_effort'] = max_effort

    def set_mode(self, mode: str,
                 *,
                 individual_control_fingers: bool=False,
                 individual_control_scissor: bool=False,
                 timeout_sec: Optional[float]=None):
        """ Set operation mode of the gripper.

        This fuction is effective only for Robotiq-3D grippers.

        :param mode:
        :param individual_control_fingers:
        :param individual_control_scissor:
        """
        status, result \
            = self._set_mode.send_goal(
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

    def _get_controller_parameters(self):
        timeout_sec = 10.0
        values = ParameterClient(self._node, self._name + '_controller') \
                .get_parameters_sync(['min_gap', 'max_gap',
                                      'min_position', 'max_position'],
                                     timeout_sec=timeout_sec)
        self._min_gap      = values[0]
        self._max_gap      = values[1]
        self._min_position = values[2]
        self._max_position = values[3]

    def _position(self, gap: float):
        idx = self._idx()
        return (gap - self._min_gap[idx]) * self._position_per_gap(idx) \
             + self._min_position[idx]

    def _gap(self, position: float):
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
    def __init__(self, node: Node, name: str='a_bot_gripper', *,
                 advanced_mode:      bool=True,
                 grasp_pressure:     float=-78.0,
                 detection_pressure: float=-10.0,
                 release_pressure:   float=0.0,
                 grasp_timeout_sec:  float=0.0):
        """ Create a RobotiqSuction client.

        :param node: The ROS node to add the suction tool client to.
        :param name: Name of the gripper.
        :param advanced_mode: If ``True``, operates in advanced mode.
            Otherwise, operates in test mode
        """
        self._name = name
        self._cbg  = MutuallyExclusiveCallbackGroup()
        super().__init__(node, SuctionCommand,
                         name + '_controller/gripper_cmd',
                         callback_group=self._cbg)
        self.wait_for_server()

        self._parameters = {'advanced_mode':      advanced_mode,
                            'grasp_pressure':     grasp_pressure,
                            'detection_pressure': detection_pressure,
                            'release_pressure':   release_pressure,
                            'grasp_timeout':      grasp_timeout_sec}

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        return 'suction'

    @property
    def base_link(self):
        return self._name + '_base_link'

    @property
    def tip_link(self):
        return self._name + '_tip_link'

    @property
    def parameters(self):
        """Return a dictionary of gripper parameters

        :return: Dictionary of gripper parameters with string keys.
        """
        return self._parameters

    def pregrasp(self):
        self.suck(max_pressure=self.parameters['grasp_pressure'],
                  min_pressure=self.parameters['detection_pressure'],
                  grasp_timeout_sec=0.0,
                  timeout_sec=0.0)

    def grasp(self, *, timeout_sec: Optional[float]=None):
        """ Grasp an object with the gripper.

        Pressure applied and pressure threshold for object detection are
        specified by parameters ``grasp_pressure`` and ``detection_pressure``,
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
        return self.suck(max_pressure=self.parameters['grasp_pressure'],
                         min_pressure=self.parameters['detection_pressure'],
                         grasp_timeout_sec=self.parameters['grasp_timeout'],
                         timeout_sec=timeout_sec)

    def postgrasp(self):
        self.pregrasp()

    def release(self, *, timeout_sec: Optional[float]=None):
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
        return self.suck(max_pressure=self.parameters['release_pressure'],
                         min_pressure=self.parameters['detection_pressure'],
                         grasp_timeout_sec=self.parameters['grasp_timeout'],
                         timeout_sec=timeout_sec)

    def suck(self, max_pressure: float, *,
             min_pressure:      Optional[float]=None,
             grasp_timeout_sec: Optional[float]=None,
             timeout_sec:       Optional[float]=None):
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
            min_pressure = self.parameters['detection_pressure']
        if not grasp_timeout_sec:
            grasp_timeout_sec = self.parameters['grasp_timeout']
        return self.send_goal(
                   SuctionCommand.Goal(
                       command=SuctionCommandMsg(
                           advanced_mode=self.parameters['advanced_mode'],
                           max_pressure=max_pressure,
                           min_pressure=min_pressure,
                           timeout=grasp_timeout_sec)),
                   timeout_sec=timeout_sec)
