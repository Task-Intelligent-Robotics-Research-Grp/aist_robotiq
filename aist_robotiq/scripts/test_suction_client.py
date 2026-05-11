#!/usr/bin/env python3
#
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
import rclpy, threading
from rclpy.node          import Node
from aist_robotiq.client import RobotiqSuction

#########################################################################
#  class TestSuctionClient                                              #
#########################################################################
class TestSuctionClient(Node):
    def __init__(self, name):
        super().__init__(name)

        gripper_name       = self.declare_parameter('gripper_name',
                                                    'a_bot_gripper').value
        advanced_mode      = self.declare_parameter('advanced_mode',
                                                    True).value
        grasp_pressure     = self.declare_parameter('grasp_pressure',
                                                    -78.0).value
        detection_pressure = self.declare_parameter('detection_pressure',
                                                    -10.0).value
        release_pressure   = self.declare_parameter('release_pressure',
                                                    0.0).value
        grasp_timeout_sec  = self.declare_parameter('grasp_timeout', 0.0).value

        self._gripper = RobotiqSuction(self, gripper_name,
                                       advanced_mode=advanced_mode,
                                       grasp_pressure=grasp_pressure,
                                       detection_pressure=detection_pressure,
                                       release_pressure=release_pressure,
                                       grasp_timeout_sec=grasp_timeout_sec)

        threading.Thread(target=self.interactive, daemon=True).start()

    def interactive(self):
        def is_float(s):
            try:
                float(s)
            except ValueError:
                return False
            else:
                return True

        while rclpy.ok():
            print('==== Available commands ====')
            print('  g:         Grasp')
            print('  r:         Release')
            print('  <numeric>: Set specified pressure value to the gripper')
            print('  c:         Cancel sucking')
            print('  w:         Wait until goal completed')
            print('  q:         Quit\n')

            key = input('>> ')
            if key == 'g':
                self._gripper.grasp()
            elif key == 'r':
                self._gripper.release()
            elif is_float(key):
                self._gripper.suck(float(key), timeout_sec=0.0)
            elif key == 'c':
                self._gripper.cancel_goal()
            elif key == 'w':
                status, result = self._gripper.wait(timeout_sec=2.0)
                print(result)
            elif key=='q':
                break
            else:
                print('unknown command: %s' % key)

        self.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()

    test = TestSuctionClient('test_suction_client')
    rclpy.spin(test)
