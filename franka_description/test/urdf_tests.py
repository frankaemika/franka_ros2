#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import xacro
from ament_index_python.packages import get_package_share_directory
from os import path

panda_xacro_file_name = path.join(get_package_share_directory("franka_description"), "robots",
                                  "panda_arm.urdf.xacro")


def test_load():
    urdf = xacro.process_file(panda_xacro_file_name).toxml()
    assert urdf.find("panda_finger_joint") == -1


def test_load_with_gripper():
    urdf = xacro.process_file(panda_xacro_file_name,
                              mappings={"hand": 'true'}).toxml()
    assert urdf.find("panda_finger_joint") != -1


def test_load_with_fake_hardware():
    urdf = xacro.process_file(panda_xacro_file_name,
                              mappings={"use_fake_hardware": 'true'}).toxml()
    assert urdf.find("fake_components/GenericSystem") != -1


def test_load_with_robot_ip():
    urdf = xacro.process_file(panda_xacro_file_name,
                              mappings={"robot_ip": 'franka_ip_address'}).toxml()
    assert urdf.find("franka_ip_address") != -1


def test_load_with_arm_id():
    urdf = xacro.process_file(panda_xacro_file_name,
                              mappings={"arm_id": 'totally_different_arm'}).toxml()
    assert urdf.find("totally_different_arm_joint1") != -1


if __name__ == '__main__':
    pass
