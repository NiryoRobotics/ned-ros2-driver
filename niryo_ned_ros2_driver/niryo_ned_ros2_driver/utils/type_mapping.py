# Copyright (c) 2025 Niryo.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# /usr/bin/env python3

import re
from .constants import ROS1_INTERFACE_PACKAGES, ROS2_INTERFACE_PACKAGE


def convert_ros1_to_ros2_type(ros1_type: str, interface_type: str) -> str:
    if interface_type not in ["srv", "msg", "action"]:
        raise ValueError(
            f"Invalid interface type '{interface_type}'. Expected 'srv', 'msg' or 'action'."
        )

    pkg, type = ros1_type.split("/")
    if interface_type == "action":
        type = type.replace("Action", "")

    if pkg in ROS1_INTERFACE_PACKAGES:
        return f"{ROS2_INTERFACE_PACKAGE}/{interface_type}/{type}"
    else:
        return f"{pkg}/{interface_type}/{type}"


def guess_action_type_from_goal_type(goal_type: str) -> str:
    """
    Given a goal type, return the corresponding action type.
    """
    pattern = r"(?P<package>.+)/(?P<basename>.+)ActionGoal"
    match = re.match(pattern, goal_type)
    if not match:
        raise ValueError(f"Provided type {goal_type} is not an ActionGoal type.")

    package = match.group("package")
    basename = match.group("basename")
    return f"{package}/{basename}Action"
