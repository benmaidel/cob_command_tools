#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
from subprocess import Popen, PIPE
import re
import paramiko

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class IwConfig():
    def __init__(self, hostname, user, password):
        self.norm = ""
        self.essid = ""
        self.mode = ""
        self.frequency = 0.0
        self.access_point = ""
        self.bit_rate = 0.0
        self.tx_power = 0.0
        self.retry_short_limit = 0
        self.rts_thr = ""
        self.fragment_thr = ""
        self.power_management = ""
        self.link_quality = ""
        self.link_quality_percent = 0
        self.signal_level = 0.0
        self.rx_invalid_nwid = 0
        self.rx_invalid_crypt = 0
        self.rx_invalid_frag = 0
        self.tx_excessive_retries = 0
        self.invalic_misc = 0
        self.missed_beacon = 0
        self.ssh = paramiko.SSHClient()
        self.ssh.load_system_host_keys()
        self.ssh.connect(str(hostname), username=user, password=password)

    def update(self, interface):
        try:
            (ssh_stdin, ssh_stdout, ssh_stderr) = self.ssh.exec_command("iwconfig %s"%interface)
        except Exception as e:
            rospy.logerr("Update Error: %s" %e)
            return False

        output = ''.join(ssh_stdout.readlines())
        return self._parse_info(output)

    def _parse_info(self, info):
        try:
            split = info.split('IEEE ',1)
            split = split[1].split('ESSID:',1)
            self.norm = split[0].encode('utf8')
            split = split[1].split('\n',1)
            self.essid = split[0].encode('utf8')
            split = split[1].split('Mode:',1)
            split = split[1].split('Frequency:',1)
            self.mode = split[0].encode('utf8')
            split = split[1].split(' GHz',1)
            self.frequency = float(split[0])
            split = split[1].split('Access Point: ',1)
            split = split[1].split('\n',1)
            self.access_point = split[0].encode('utf8')
            split = split[1].split('Bit Rate=',1)
            split = split[1].split(' Mb/s',1)
            self.bit_rate = float(split[0])
            if split[1].find('Tx-Power') != -1:
                split = split[1].split('Tx-Power=',1)
                split = split[1].split(' dBm',1)
                self.tx_power = float(split[0])
            if split[1].find('Retry short limit:') != -1:
                split = split[1].split('Retry short limit:',1)
            if split[1].find('Retry short limit=') != -1:
                split = split[1].split('Retry short limit=',1)
            if split[1].find('RTS thr:') != -1:
                split = split[1].split('RTS thr:',1)
            if split[1].find('RTS thr=') != -1:
                split = split[1].split('RTS thr=',1)
            self.retry_short_limit = split[0].encode('utf8')
            if split[1].find('Fragment thr:') != -1:
                split = split[1].split('Fragment thr:',1)
            if split[1].find('Fragment thr=') != -1:
                split = split[1].split('Fragment thr=',1)
            self.rts_thr = split[0].encode('utf8')
            split = split[1].split('\n',1)
            self.fragment_thr = split[0].encode('utf8')
            split = split[1].split('Power Management:',1)
            split = split[1].split('\n',1)
            self.power_managment = split[0].encode('utf8')
            split = split[1].split('Link Quality=',1)
            split = split[1].split('Signal level=',1)
            self.link_quality = split[0].encode('utf8')
            self.link_quality_percent = split[0].split('/')
            self.link_quality_percent = int(float(self.link_quality_percent[0]) / float(self.link_quality_percent[1])*100.0)
            split = split[1].split(' dBm',1)
            self.signal_level = float(split[0])
            split = split[1].split('Rx invalid nwid:',1)
            split = split[1].split('Rx invalid crypt:',1)
            self.rx_invalid_nwid = int(split[0])
            split = split[1].split('Rx invalid frag:',1)
            self.rx_invalid_crypt = int(split[0])
            split = split[1].split('\n',1)
            self.rx_invalid_frag = int(split[0])
            split = split[1].split('Tx excessive retries:',1)
            split = split[1].split('Invalid misc:',1)
            self.tx_excessive_retries = int(split[0])
            split = split[1].split('Missed beacon:',1)
            self.invalid_misc = int(split[0])
            split = split[1].split('\n',1)
            self.missed_beacon = int(split[0])

        except Exception as e:
            rospy.logerr("Parsing Error: %s" %e)
            return False
        return True

class WlanMonitor():
    def __init__(self):
        rospy.init_node("wlan_monitor")
        self.get_params()
        try:
            self.iwconfig = IwConfig(self.diag_hostname, self.user, self.password)
        except Exception as e:
            rospy.logerr("Error connecting ssh to host: %s",e.message)

        stat = DiagnosticStatus()
        stat.level = 0
        stat.name = '%s WLAN Info' % self.diag_hostname
        stat.message = "OK"
        stat.hardware_id = self.diag_hostname
        stat.values = []
        self.msg = DiagnosticArray()
        self.msg.header.stamp = rospy.get_rostime()
        self.msg.status = [stat]

        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)
        self.diag_timer = rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)
        self.monitor_timer = rospy.Timer(rospy.Duration(1.0), self.update_diagnostics)

    def update_diagnostics(self, event):
        stat = DiagnosticStatus()
        stat.level = 0
        stat.name = '%s WLAN Info' % self.diag_hostname
        stat.message = "OK"
        stat.hardware_id = self.diag_hostname
        stat.values = []

        if (self.iwconfig.update(self.interface)):
            stat.level = DiagnosticStatus.OK
            stat.message = "OK"
            stat.values = [ KeyValue("Interface" , self.interface),
                            KeyValue("IEEE Norm", self.iwconfig.norm),
                            KeyValue("ESSID", self.iwconfig.essid),
                            KeyValue("Mode", self.iwconfig.mode),
                            KeyValue("Frequency", str(self.iwconfig.frequency)),
                            KeyValue("Access Point", self.iwconfig.access_point),
                            KeyValue("Bit Rate [Mb/s]", str(self.iwconfig.bit_rate)),
                            KeyValue("Tx-Power [dBm]", str(self.iwconfig.tx_power)),
                            KeyValue("Retry short limit", str(self.iwconfig.retry_short_limit)),
                            KeyValue("RTS thr", self.iwconfig.rts_thr),
                            KeyValue("Fragment thr", self.iwconfig.fragment_thr),
                            KeyValue("Power Managment", self.iwconfig.power_management),
                            KeyValue("Link Quality", self.iwconfig.link_quality),
                            KeyValue("Link Quality %", str(self.iwconfig.link_quality_percent)),
                            KeyValue("Signal level [dBm]", str(self.iwconfig.signal_level)),
                            KeyValue("Rx invalid nwid", str(self.iwconfig.rx_invalid_nwid)),
                            KeyValue("Rx invalid crypt", str(self.iwconfig.rx_invalid_crypt)),
                            KeyValue("Rx invalid frag", str(self.iwconfig.rx_invalid_frag)),
                            KeyValue("Tx excessive retries", str(self.iwconfig.tx_excessive_retries)),
                            KeyValue("Invalid misc", str(self.iwconfig.invalic_misc)),
                            KeyValue("Missed beacon", str(self.iwconfig.missed_beacon)) ]

            #ToDo: set diagnostic warning/error level accordingly

        else:
            stat.level = DiagnosticStatus.ERROR
            stat.message = "Error Running 'iwconfig'"

        self.msg = DiagnosticArray()
        self.msg.header.stamp = rospy.get_rostime()
        self.msg.status = [stat]

    def publish_diagnostics(self, event):
        self.diag_pub.publish(self.msg)

    def get_params(self):
        self.diag_hostname = rospy.get_param('~diag_hostname', "cob4-2")
        self.interface = rospy.get_param('~interface', "wlan0")
        self.user = rospy.get_param('~user', "root")
        self.password = rospy.get_param('~password', "admin")

if __name__ == "__main__":
    ntp = WlanMonitor()
    rospy.spin()
