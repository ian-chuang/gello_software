#!/usr/bin/env python
"""Small helper script to start the tool communication interface"""

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2019 FZI Forschungszentrum Informatik
# Created on behalf of Universal Robots A/S
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# -- END LICENSE BLOCK ------------------------------------------------

import subprocess


def main():
    """Starts socat"""
    # IP address of the robot
    robot_ip = "192.168.1.102"
    # Port on which the remote pc (robot) publishes the interface
    tcp_port = "54321"

    # By default, socat will create a pty in /dev/pts/N with n being an increasing number.
    # Additionally, a symlink at the given location will be created. Use an absolute path here.
    local_device = '/tmp/ttyUR'

    print("Remote device is available at '" + local_device + "'")

    cfg_params = ["pty"]
    cfg_params.append("link=" + local_device)
    cfg_params.append("raw")
    cfg_params.append("ignoreeof")
    cfg_params.append("waitslave")

    cmd = ["socat"]
    cmd.append(",".join(cfg_params))
    cmd.append(":".join(["tcp", robot_ip, tcp_port]))

    print("Starting socat with following command:\n" + " ".join(cmd))
    subprocess.call(cmd)


if __name__ == '__main__':
    main()