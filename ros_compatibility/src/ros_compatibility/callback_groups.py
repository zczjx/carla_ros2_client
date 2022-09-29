#!/usr/bin/env python3

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#


import rclpy.callback_groups

class ReentrantCallbackGroup(rclpy.callback_groups.ReentrantCallbackGroup):
    pass

class MutuallyExclusiveCallbackGroup(rclpy.callback_groups.MutuallyExclusiveCallbackGroup):
    pass
