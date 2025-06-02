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

import time
from typing import Callable, Tuple, Any


def execute_and_return_duration(
    label: str, func: Callable, *args, **kwargs
) -> Tuple[Any, float, str]:
    """
    Execute a function and return its result, the time taken for execution, and a label.

    Args:
        label (str): A descriptive label for the executed function.
        func (Callable): The function to execute.
        *args: Variable length argument list to pass to the function.
        **kwargs: Arbitrary keyword arguments to pass to the function.

    Returns:
        Tuple[Any, float, str]: A tuple containing:
            - result (Any): The return value of the executed function.
            - duration (float): The time taken for execution in seconds.
            - label (str): The descriptive label passed as input.
    """
    start = time.time()
    result = func(*args, **kwargs)
    duration = time.time() - start
    return result, duration, label
