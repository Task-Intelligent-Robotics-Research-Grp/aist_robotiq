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
import rclpy, sys, threading
from rclpy.node          import Node
from aist_robotiq.client import RobotiqGripper, RobotiqSuction

#************************************************************************
#  class TestDualGripperClient                                          *
#************************************************************************
class TestDualGripperClient(Node):
    def __init__(self, name):
        super().__init__(name)

        gripper_names = self.declare_parameter('gripper_names',
                                               ['robotiq_85', 'right_epick']) \
                            .value
        self._gripper = RobotiqGripper(self, gripper_names[0])
        self._suction = RobotiqSuction(self, gripper_names[1])

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
            print('==== Gripper commands ====')
            print('  gg:        Gripper grasp')
            print('  gr:        Gripper release')
            print('  <numeric>: Open gripper with the specified gap value')
            print('  gc:        Cancel gripper motion')
            print('  gw:        Wait until gripper goal completed')
            print('  gv:        Set gripper velocity')
            print('  ge:        Set maximum effort to be applied')
            print('==== Suction commands ====')
            print('  sg:        Suction grasp')
            print('  sr:        Suction release')
            print('  sp:        Set specified pressure value to suction')
            print('  sc:        Cancel suction')
            print('  sw:        Wait until suction goal completed')
            print('  q:         Quit\n')

            key = input('>> ')
            if key == 'gg':
                self._gripper.grasp()
            elif key == 'gr':
                self._gripper.release()
            elif is_float(key):
                self._gripper.move(float(key), max_effort=0.0, timeout_sec=0.0)
            elif key == 'gc':
                self._gripper.cancel_goal()
            elif key == 'gw':
                status, result = self._gripper.wait()
                print(result)
            elif key == 'gv':
                velocity = float(input('  velocity: '))
                success = self._gripper.set_velocity(velocity)
                print('%s to set velocity'
                      % ('succeeded' if success else 'failed'))
            elif key == 'ge':
                max_effort = float(input('  maximum effort: '))
                self._gripper.parameters['max_effort'] = max_effort

            elif key == 'sg':
                self._suction.grasp()
            elif key == 'sr':
                self._suction.release()
            elif key == 'sp':
                pressure = float(input('  pressure: '))
                self._gripper.suck(pressure, timeout_sec=0.0)
            elif key == 'sc':
                self._suction.cancel_goal()
            elif key == 'sw':
                status, result = self._suction.wait(timeout_sec=2.0)
                print(result)

            elif key=='q':
                break
            else:
                print('unknown command: %s' % key)

        self.destroy_node()
        rclpy.shutdown()

#************************************************************************
#  Entry point                                                          *
#************************************************************************
def main():
    rclpy.init(args=sys.argv)

    test = TestDualGripperClient('test_dual_gripper_client')
    rclpy.spin(test)

if __name__ == '__main__':
    main()
