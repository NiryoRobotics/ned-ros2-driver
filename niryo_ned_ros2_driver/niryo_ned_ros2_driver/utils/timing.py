# /usr/bin/env python3

import time
from typing import Callable, Tuple, Any


def execute_and_return_duration(
    label: str, func: Callable, *args, **kwargs
) -> Tuple[Any, float, str]:
    start = time.time()
    result = func(*args, **kwargs)
    duration = time.time() - start
    return result, duration, label
