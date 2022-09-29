#!/usr/bin/env python3

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

import rclpy

def logdebug(msg):
    rclpy.logging.get_logger("default").debug(msg)

def loginfo(msg):
    rclpy.logging.get_logger("default").info(msg)

def logwarn(msg):
    rclpy.logging.get_logger("default").warn(msg)

def logerr(msg):
    rclpy.logging.get_logger("default").error(msg)

def logfatal(msg):
    rclpy.logging.get_logger("default").fatal(msg)
