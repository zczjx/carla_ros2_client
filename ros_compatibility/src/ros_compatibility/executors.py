#!/usr/bin/env python3

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#


import rclpy.executors

class SingleThreadedExecutor(rclpy.executors.SingleThreadedExecutor):
    pass

class MultiThreadedExecutor(rclpy.executors.MultiThreadedExecutor):
    pass

