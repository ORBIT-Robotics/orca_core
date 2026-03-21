# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import numpy as np


def linear_interp(t):
    return t


def ease_in_out(t):
    return 0.5 * (1 - np.cos(np.pi * t))


def interpolate_waypoints(start, end, duration, step_time, mode="linear"):
    n_steps = int(duration / step_time)
    interp_func = linear_interp if mode == "linear" else ease_in_out
    for i in range(n_steps + 1):
        t = i / n_steps
        alpha = interp_func(t)
        yield [(1 - alpha) * s + alpha * e for s, e in zip(start, end)]
