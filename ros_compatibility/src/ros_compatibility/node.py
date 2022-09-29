#!/usr/bin/env python3

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

import ros_compatibility.qos
from ros_compatibility.exceptions import *

import time
from rclpy import Parameter
from rclpy.node import Node
from rclpy.task import Future
import rclpy.qos

_DURABILITY_POLICY_MAP = {
    ros_compatibility.qos.DurabilityPolicy.TRANSIENT_LOCAL: rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
    ros_compatibility.qos.DurabilityPolicy.VOLATILE: rclpy.qos.DurabilityPolicy.VOLATILE
}


def _get_rclpy_qos_profile(qos_profile):
    return rclpy.qos.QoSProfile(
        depth=qos_profile.depth,
        durability=_DURABILITY_POLICY_MAP[qos_profile.durability]
    )


class CompatibleNode(Node):

    def __init__(self, name, **kwargs):
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        super(CompatibleNode, self).__init__(
            name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
            parameter_overrides=[param],
            **kwargs)

    def get_param(self, name, alternative_value=None):
        return self.get_parameter_or(
            name,
            Parameter(name, value=alternative_value)).value

    def get_time(self):
        t = self.get_clock().now()
        t = t.seconds_nanoseconds()
        return float(t[0] + (t[1] / 10**9))

    def logdebug(self, text):
        self.get_logger().debug(text)

    def loginfo(self, text):
        self.get_logger().info(text)

    def logwarn(self, text):
        self.get_logger().warn(text)

    def logerr(self, text):
        self.get_logger().error(text)

    def logfatal(self, text):
        self.get_logger().fatal(text)

    def new_publisher(self, msg_type, topic, qos_profile, callback_group=None):
        if isinstance(qos_profile, int):
            qos_profile = ros_compatibility.qos.QoSProfile(depth=qos_profile)
        rclpy_qos_profile = _get_rclpy_qos_profile(qos_profile)

        return self.create_publisher(
            msg_type, topic, rclpy_qos_profile,
            callback_group=callback_group)

    def new_subscription(self, msg_type, topic, callback, qos_profile, callback_group=None):
        if isinstance(qos_profile, int):
            qos_profile = ros_compatibility.qos.QoSProfile(depth=qos_profile)
        rclpy_qos_profile = _get_rclpy_qos_profile(qos_profile)

        return self.create_subscription(
            msg_type, topic, callback, rclpy_qos_profile,
            callback_group=callback_group)

    def new_rate(self, frequency):
        return self.create_rate(frequency)

    def new_timer(self, timer_period_sec, callback, callback_group=None):
        return self.create_timer(
            timer_period_sec, callback, callback_group=callback_group)

    def wait_for_message(self, topic, msg_type, timeout=None, qos_profile=1):
        """
        Wait for one message from topic.
        This will create a new subcription to the topic, receive one message, then unsubscribe.

        Do not call this method in a callback or a deadlock may occur.
        """
        s = None
        try:
            future = Future()
            s = self.new_subscription(
                msg_type,
                topic,
                lambda msg: future.set_result(msg),
                qos_profile=qos_profile)
            rclpy.spin_until_future_complete(self, future, self.executor, timeout)
        finally:
            if s is not None:
                self.destroy_subscription(s)

        return future.result()

    def new_service(self, srv_type, srv_name, callback, callback_group=None):
        return self.create_service(
            srv_type, srv_name, callback, callback_group=callback_group)

    def new_client(self, srv_type, srv_name, timeout_sec=None, callback_group=None):
        client = self.create_client(
            srv_type, srv_name, callback_group=callback_group)
        ready = client.wait_for_service(timeout_sec=timeout_sec)
        if not ready:
            raise ROSException("Timeout of {}sec while waiting for service".format(timeout_sec))
        return client

    def call_service(self, client, req, timeout=None, spin_until_response_received=False):
        if not spin_until_response_received:
            response = client.call(req)
            return response
        else:
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, self.executor, timeout)

            if future.done():
                return future.result()
            else:
                if timeout is not None:
                    raise ServiceException(
                        'Service did not return a response before timeout {}'.format(timeout))
                else:
                    raise ServiceException('Service did not return a response')

    def spin(self):
        rclpy.spin(self, self.executor)

    def destroy(self):
        self.destroy_node()
