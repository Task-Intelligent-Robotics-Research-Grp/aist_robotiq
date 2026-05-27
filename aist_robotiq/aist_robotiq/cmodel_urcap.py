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
#  BASED ON: https://dof.robotiq.com/discussion/1962/programming-options-ur16e-2f-85#latest
#  Ported to ROS by felixvd
#  Modified by T.Ueshiba
#
import time, socket, threading
from aist_robotiq.cmodel_base import CModelBase
from aist_robotiq_msgs.msg    import CModelStatus

#************************************************************************
#  class CModelURCap                                                    *
#************************************************************************
class CModelURCap(CModelBase):
    """ Communicates with the gripper directly via socket with string commands,
    leveraging string names for variables.

    Uses port 63352 which is opened by the Robotiq Gripper URCap
    and receives ASCII commands.
    """
    def __init__(self, name: str):
        """ Constructor
        """
        super().__init__(name)
        ip = self.declare_parameter('ip', '10.66.171.40').value
        self._lock   = threading.Lock()
        try:
            self._socket = self.connect(ip)
        except Exception as e:
            self.get_logger().error('failed to connect[ip=%s]: %s' % (ip, e))
            raise
        self.get_logger().info('started[ip=%s]' % ip)

    def connect(self, hostname: str, port: int=63352,
                socket_timeout: float=2.0):
        """ Connects to a gripper at the given address.

        Args:
          hostname: Hostname or ip.
          port: Port.
          socket_timeout: Timeout for blocking socket operations.

        Returns:
          Socket opened.
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((hostname, port))
        s.settimeout(socket_timeout)
        self.get_logger().info("connected to %s:%d" % (hostname, port))
        return s

    def disconnect(self):
        """ Closes the connection with the gripper
        """
        self._socket.close()

    def activate_devices(self):
        pass

    def put_command(self, command):
        command = self._clip_command(command)

        # Specify target device.
        self._set_var('SID', command.r_sid)

        # Do not set variable 'ACT' because setting zero value will cause
        # the device reset.
        vars = [# ('ACT', command.r_act),
                ('MOD', command.r_mod),                 # Byte 0
                ('GTO', command.r_gto),
                ('ATR', command.r_atr),
                ('ARD', command.r_ard),
                ('POS', command.r_pr),                  # Byte 3
                ('SPE', command.r_sp),                  # Byte 4
                ('FOR', command.r_fr)]                  # Byte 5
        if self._device_types[command.r_sid] == 'arg3f':
            vars += [('ICF', command.r_icf),            # Byte 1
                     ('ICS', command.r_ics),
                     ('PRB', command.r_prb),            # Byte 6
                     ('SPB', command.r_spb),            # Byte 7
                     ('FRB', command.r_frb),            # Byte 8
                     ('PRC', command.r_prc),            # Byte 9
                     ('SPC', command.r_spc),            # Byte 10
                     ('FRC', command.r_frc),            # Byte 11
                     ('PRS', command.r_prs),            # Byte 12
                     ('SPS', command.r_sps),            # Byte 13
                     ('FRS', command.r_frs)]            # Byte 14
        self._set_vars(vars)

    def get_status(self, slave_id):
        # Specify target device.
        self._set_var('SID', slave_id)

        # Assign status values to their respective variables
        status = CModelStatus()
        status.g_sid = slave_id
        status.g_act = self._get_var('ACT')             # Byte 0
        status.g_mod = self._get_var('MOD')
        status.g_gto = self._get_var('GTO')
        status.g_sta = self._get_var('STA')
        status.g_obj = self._get_var('OBJ')
        status.g_flt = self._get_var('FLT')             # Byte 2
        status.g_pr  = self._get_var('PRE')             # Byte 3
        status.g_po  = self._get_var('POS')             # Byte 4
        status.g_cou = self._get_var('COU')             # Byte 5
        if self._device_types[slave_id] == 'arg3f':
            status.g_dta = self._get_var('DTA')         # Byte 1
            status.g_dtb = self._get_var('DTB')
            status.g_dtc = self._get_var('DTC')
            status.g_dts = self._get_var('DTS')
            status.g_prb = self._get_var('PRB')         # Byte 6
            status.g_pob = self._get_var('POB')         # Byte 7
            status.g_cub = self._get_var('CUB')         # Byte 8
            status.g_prc = self._get_var('PRC')         # Byte 9
            status.g_poc = self._get_var('POC')         # Byte 10
            status.g_cuc = self._get_var('CUC')         # Byte 11
            status.g_prs = self._get_var('PRS')         # Byte 12
            status.g_pos = self._get_var('POS')         # Byte 13
            status.g_cus = self._get_var('CUS')         # Byte 14
        elif self._device_types[slave_id] == 'epick':
            status.g_dta = self._get_var('VST')         # Byte 1
        return status

    def _set_vars(self, vars):
        """ Set values to variables.
        Sends the appropriate command via socket to set the value
        of n variables, and waits for its 'ack' response.

        Args:
          vars: List of tuples of (variable_name, value).

        Returns:
          `True` on successful reception of ack, `False` if no ack was
          received, indicating the set may not have been effective.
        """
        # construct unique command
        cmd = 'SET'
        for var in vars:
            cmd += ' ' + var[0] + ' ' + str(var[1])
        cmd += '\n'  # new line is required for the command to finish
        # atomic commands send/rcv
        with self._lock:
            self._socket.sendall(cmd.encode('UTF-8'))
            data = self._socket.recv(1024)

        return self._is_ack(data)

    def _set_var(self, variable, value):
        """ Set value to variable.
        Sends the appropriate command via socket to set the value
        of a variable, and waits for its 'ack' response.

        Args:
          variable: Variable to set.
          value:    Value to set for the variable.

        Returns:
          `True` on successful reception of ack, `False` if no ack was
          received, indicating the set may not have been effective.
        """
        return self._set_vars([(variable, value)])

    def _get_var(self, variable):
        """ Get value of a variable.
        Sends the appropriate command to retrieve the value
        of a variable from the gripper, blocking until the response
        is received or the socket times out.

        Args:
          variable: Name of the variable to retrieve.

        Returns:
          Value of the variable as integer.
        """
        # atomic commands send/rcv
        with self._lock:
            cmd = 'GET ' + variable + '\n'
            self._socket.sendall(cmd.encode('UTF-8'))
            data = self._socket.recv(1024)

        # expect data of the form 'VAR x', where VAR is an echo
        # of the variable name, and x the value
        # note some special variables (like FLT) may send 2 bytes,
        # instead of an integer. We assume integer here
        var_name, value_str = data.decode('UTF-8').split(maxsplit=1)
        #self.get_logger().info('### %s=%s' % (var_name, value_str[0:-1]))
        if var_name != variable:
            raise ValueError('Unexpected response ' + str(data)
                             + ' does not match "' + variable + '"')
        try:
            return int(value_str)
        except ValueError:
            return eval(value_str)[0]

    @staticmethod
    def _is_ack(data):
        if data != b'ack':
            self.get_logger().error('no acknowledge from device: %s' % data)
            return False
        return True
