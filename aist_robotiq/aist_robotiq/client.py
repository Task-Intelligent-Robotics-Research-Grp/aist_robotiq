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
from aist_robotiq_msgs.action     import SuctionCommand
from aist_robotiq_msgs.msg        import SuctionCommand as SuctionCommandMsg
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

    _RemoteParams = (
        'velocity', 'mode',
        'individual_control_fingers', 'individual_control_scissor',
    )

    def __init__(self, node: Node, name: str='a_bot_gripper'):
        """
        Args:
          node: The ROS node to add the suction tool client to.
          name: Name of the suction tool
          max_effort: Maximum effort to be applied when grasping.
        """
        self._node    = node
        self._name    = name
        controller_ns = name + '_controller'

        # Create action client for gripper command.
        super().__init__(node, GripperCommand, controller_ns + '/gripper_cmd',
                         callback_group=MutuallyExclusiveCallbackGroup())

        # Create parameter client for setting/getting controller parameters.
        self._param_clnt = ParameterClient(node, controller_ns)

        # These values are required for computing gap(in meters) from gripper's
        # position(in radians), which will be obtained from the controller
        # on demand.
        self._min_gap      = None
        self._max_gap      = None
        self._min_position = None
        self._max_position = None

        # Initialize parameter dictionary with initial max_effort value.
        # Other parameters, 'grasp_position' and 'release_position',
        # will be obtained from the controller on demand.
        self._local_params = {'max_effort': 0.0}

    @property
    def name(self)-> str:
        """ Name of the gripper.
        """
        return self._name

    @property
    def type(self)-> str:
        """ Name of the gripper's type.
        """
        if self._min_gap is None:
            self._get_controller_parameters()
        return 'two_finger' if len(self._min_gap) == 1 else 'three_finger'

    @property
    def base_link(self)-> str:
        """ Name of the gripper's base link.
        """
        return self._name + '_base_link'

    @property
    def tip_link(self)-> str:
        """ Name of the gripper's tip link.
        """
        return self._name + '_tip_link'

    @property
    def parameters(self)-> dict:
        """ Dictionary of gripper parameters.
        """
        if self._min_gap is None:
            self._get_controller_parameters()
        if 'grasp_position' not in self._local_params:
            self._local_params['grasp_position']   = self._min_gap[0]
            self._local_params['release_position'] = self._max_gap[0]

        timeout_sec = 10.0
        values = self._param_clnt \
                     .get_parameters_sync(RobotiqGripper._RemoteParams,
                                          timeout_sec=timeout_sec)
        if len(values) != len(RobotiqGripper._RemoteParams):
            values = (0.085, 0, False, False)  # fallback to robotiq_85
        remote_params = dict(zip(RobotiqGripper._RemoteParams, values))
        return self._local_params | remote_params

    def set_parameters(self, params: dict):
        """ Set gripper parameters.

        Args:
          params: Dictionary of gripper parameters. Effective keys are
          - 'max_effort': Maximum effort in Newton applied when grasping.
          - 'grasp_position': Gap between fingers in meters when grasping.
          - 'release_position': Gap between fingers in meters when releasing.
          - 'velocity': Finger velocity
          - 'mode': Grasping mode(0: BASIC, 1: PINCH, 2: WIDE, 3: SCISSOR).
            Effective only for Robotiq-3F gripper.
          - 'individual_control_fingers': Control each finger independently,
            if `True`. Effective only for Robotiq-3F gripper.
          - 'individual_control_scissor': Control scissor independently,
            if `True`. Effective only for Robotiq-3F gripper.
        """
        self._local_params |= dict(filter(lambda item: item[0]
                                          not in RobotiqGripper._RemoteParams,
                                          params.items()))

        remote_params = dict(filter(lambda item: item[0]
                                    in RobotiqGripper._RemoteParams,
                                    params.items()))
        timeout_sec = 1.0
        self._param_clnt.set_parameters_sync(remote_params,
                                             timeout_sec=timeout_sec)

    def pregrasp(self)-> None:
        """ Move to release position and return immediatelty.
        """
        self.release(timeout_sec=0.0)

    def grasp(self, *, timeout_sec: Optional[float]=None):
        """ Grasp an object with the gripper.
        Desired finger position and applied effort are specified by parameters
        with 'grasp_position' and 'max_effort' keys, respectively.

        Args:
          timeout_sec: Timeout time waiting for the gripper to complete
            grasping. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          A tuple of the goal status and the movement result of
          `control_msgs.action.GripperCommand.Result` type.
        """
        return self.move(self.parameters['grasp_position'],
                         max_effort=self.parameters['max_effort'],
                         timeout_sec=timeout_sec)

    def postgrasp(self)-> None:
        """ Move to grasp position and return immediatelty.
        """
        self.grasp(timeout_sec=0.0)

    def release(self, *, timeout_sec: Optional[float]=None):
        """ Release an object grasped by the gripper.
        Desired finger position is specified by a parameter
        with 'release_position' key. No effort is applied.

        Args:
          timeout_sec: Timeout time waiting for the gripper to complete
            releasing. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          A tuple of the goal status and the movement result of
          `control_msgs.action.GripperCommand.Result` type.
        """
        return self.move(self.parameters['release_position'],
                         max_effort=0.0, timeout_sec=timeout_sec)

    def move(self, gap: float, *,
             max_effort: float=0.0, timeout_sec: Optional[float]=None):
        """ Move gripper to the desired position.

        Args:
          gap: Desired gap between the fingers.
          max_effort: Desired maximum effort to be applied.
          timeout_sec: Timeout time waiting for the gripper to complete
            movement. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
            A tuple of the goal status and the movement result of
            `control_msgs.action.GripperCommand.Result` type
        """
        if self._min_gap is not None:
            self._get_controller_parameters()
        return self.send_goal(GripperCommand.Goal(
                                  command=GripperCommandMsg(
                                      position=self._position(gap),
                                      max_effort=max_effort)),
                              timeout_sec=timeout_sec)

    def wait(self, *, timeout_sec: Optional[float]=None):
        """ Wait for the result of gripper command or cancel request.
        Blocked until the result of the gripper command or a cancel request
        issued by `cancel_goal()` becomes available.

        Args:
          timeout_sec: Timeout time waiting for the result of grasping or
            releasing. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          * A tuple of the goal status and the gripper command/cancel result,
            if the result becomes available within `timeout_sec`.
          * A tuple of the current (non-terminal) goal state
            and `None`. otherwise.
        """
        status, result = super().wait(timeout_sec=timeout_sec)
        if result is not None:
             # Convert joint angle to gap.
            result = copy.deepcopy(result)
            result.position = self._gap(result.position)
        return status, result

    def grasped(self, *, timeout_sec: Optional[float]=None):
        _, result = self.wait(timeout_sec=timeout_sec)
        return result.stalled

    def _get_controller_parameters(self)-> None:
        timeout_sec = 10.0
        values = self._param_clnt.get_parameters_sync(['min_gap', 'max_gap',
                                                       'min_position',
                                                       'max_position'],
                                                      timeout_sec=timeout_sec)
        if len(values) == 4:
            self._min_gap      = values[0]
            self._max_gap      = values[1]
            self._min_position = values[2]
            self._max_position = values[3]
        else:  # fallback to robotiq_85 parameters
            self._min_gap      = [0.000]
            self._max_gap      = [0.085]
            self._min_position = [0.81]
            self._max_position = [0.00]

    def _position(self, gap: float)-> float:
        idx = self._idx()
        return (gap - self._min_gap[idx]) * self._position_per_gap(idx) \
             + self._min_position[idx]

    def _gap(self, position: float)-> float:
        idx = self._idx()
        return (position - self._min_position[idx]) \
             / self._position_per_gap(idx) + self._min_gap[idx]

    def _position_per_gap(self, idx: int)-> float:
        return (self._max_position[idx] - self._min_position[idx]) \
             / (self._max_gap[idx]      - self._min_gap[idx])

    def _idx(self)-> int:
        return 3 if self.parameters['mode'] == 3 else 0

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
        """
        Args:
          node: The ROS node to add the suction gripper client to.
          name: Name of the gripper.
          advanced_mode: If `True`, operates in advanced mode.
            Otherwise, operates in test mode.
          grasp_pressure: Maximum pressure value applied when grasping.
          detection_pressure: Minimum pressure value for detecting object.
          release_pressure: Maximum pressure value applied when releasing.
          grasp_timeout_sec: Timeout time waiting for success of grasping
            or releasing command issued asynchronously, that is, zero
            `timeout_sec` value is specified.
        """
        self._name = name
        super().__init__(node, SuctionCommand,
                         name + '_controller/gripper_cmd',
                         callback_group=MutuallyExclusiveCallbackGroup())
        self.wait_for_server()

        self._parameters = {'advanced_mode':      advanced_mode,
                            'grasp_pressure':     grasp_pressure,
                            'detection_pressure': detection_pressure,
                            'release_pressure':   release_pressure,
                            'grasp_timeout':      grasp_timeout_sec}

    @property
    def name(self)-> str:
        """ Name of the gripper.
        """
        return self._name

    @property
    def type(self)-> str:
        """ Name of the gripper's type.
        """
        return 'suction'

    @property
    def base_link(self)-> str:
        """ Name of the gripper's base link.
        """
        return self._name + '_base_link'

    @property
    def tip_link(self)-> str:
        """ Name of the gripper's tip link.
        """
        return self._name + '_tip_link'

    @property
    def parameters(self)-> dict:
        """ Dictionary of gripper parameters.
        """
        return self._parameters

    def set_parameters(self, params: dict):
        self._parameters |= params

    def pregrasp(self)-> None:
        """ Suck forever and return immediately.
        """
        self.suck(max_pressure=self.parameters['grasp_pressure'],
                  min_pressure=self.parameters['detection_pressure'],
                  timeout_sec=0.0)

    def grasp(self, *, timeout_sec: Optional[float]=None):
        """ Grasp an object with the gripper.
        Pressure applied and pressure threshold for object detection are
        specified by parameters 'grasp_pressure' and 'detection_pressure',
        respectively.

        Args:
          timeout_sec: Timeout time waiting for the gripper to complete
            grasping. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          * A tuple of the goal status and the command/cancel result for
            grasping, if the result becomes available within `timeout_sec`.
          * A tuple of the current (non-terminal) goal state
            and `None`. otherwise.
        """
        return self.suck(max_pressure=self.parameters['grasp_pressure'],
                         min_pressure=self.parameters['detection_pressure'],
                         timeout_sec=timeout_sec)

    def postgrasp(self)-> None:
        """ Suck forever and return immediately.
        """
        self.pregrasp()

    def release(self, *, timeout_sec: Optional[float]=None):
        """ Release an object grasped by the gripper.
        Value of applied pressure is specified by a parameter
        'release_pressure' which should be non-negative.

        Args:
          timeout_sec: Timeout time waiting for the gripper to complete
            releasing. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          * A tuple of the goal status and the command/cancel result for
            releasing, if the result becomes available within `timeout_sec`.
          * A tuple of the current (non-terminal) goal state
            and `None`, otherwise.
        """
        return self.suck(max_pressure=self.parameters['release_pressure'],
                         min_pressure=self.parameters['detection_pressure'],
                         timeout_sec=timeout_sec)

    def suck(self, max_pressure: float, *,
             min_pressure: Optional[float]=None,
             timeout_sec:  Optional[float]=None):
        """ Generate pressure.

        Args:
          max_pressure: Maximum pressure value applied.
          min_pressure: Minimum pressure value for object detection.
          timeout_sec: Timeout time waiting for the result of grasping or
            releasing. Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, if zero or negative.

        Returns:
          * A tuple of the goal status and the command/cancel result for
            sucking, if the result becomes available within `timeout_sec`.
          * A tuple of the current (non-terminal) goal state
            and `None`, otherwise.
        """
        if not min_pressure:
            min_pressure = self.parameters['detection_pressure']
        if timeout_sec is None:
            grasp_timeout = 0.0          # Wait forever
        elif timeout_sec > 0.0:
            grasp_timeout = timeout_sec  # Wait same duration as action timeout
        else:
            grasp_timeout = self.parameters['grasp_timeout']
        return self.send_goal(
                   SuctionCommand.Goal(
                       command=SuctionCommandMsg(
                           advanced_mode=self.parameters['advanced_mode'],
                           max_pressure=max_pressure,
                           min_pressure=min_pressure,
                           timeout=grasp_timeout)),
                   timeout_sec=timeout_sec)
