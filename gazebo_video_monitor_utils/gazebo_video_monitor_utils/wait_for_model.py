#!/usr/bin/env python3

import argparse
import os
import sys

import rclpy
import rclpy.duration
import rclpy.node

from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty


class WaitForModel(rclpy.node.Node):
    def __init__(self, model: str, service:  str, delay: float):
        super().__init__("wait_for_model")

        self._model = model
        self._delay = rclpy.duration.Duration(seconds=delay)
        self._model_exists = False
        self._wait_inited = False
        self._call_time = None

        self.get_logger().info(f"Waiting for model {model}")
        self._init_srv_client = self.create_client(Empty, service)
        self._model_states_sub = self.create_subscription(
            ModelStates, "/gazebo/model_states", self._model_states_cb, 1
        )

    def _model_states_cb(self, msg):
        if self._model not in msg.name:
            return
        if not self._wait_inited:
            self._call_time = self.get_clock().now() + self._delay
            self._wait_inited = True
        if self.get_clock().now() >= self._call_time:
            self.get_logger().info(f"Detected model {self._model}")
            self._model_exists = True

    def run(self):
        while rclpy.ok() and not self._model_exists:
            rclpy.spin_once(self)
            pass

        if self._model_exists:
            if not self._init_srv_client.wait_for_service(1):
                return os.EX_SOFTWARE
            self.get_logger().info(f"Calling service {self._init_srv_client.srv_name}")
            future = self._init_srv_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, future)
        return os.EX_OK if self._model_exists else os.EX_SOFTWARE


def main(args=None):
    args = args or sys.argv
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(
        description="Waits for a model to exist and then calls a service"
    )
    parser.add_argument(
        "--model",
        type=str,
        metavar="MODEL",
        required=True,
        help="Model for which to wait",
    )
    parser.add_argument(
        "--service",
        type=str,
        metavar="SRV",
        required=True,
        help="Service to call when the model appears",
    )
    parser.add_argument(
        "--delay",
        type=float,
        metavar="SEC",
        default=0.0,
        help=(
            "Extra time to wait after the model has appeared "
            "and before calling the service"
        ),
    )

    args_without_ros = rclpy.utilities.remove_ros_args(args)
    own_args = parser.parse_args(args_without_ros[1:])
    return WaitForModel(own_args.model, own_args.service, own_args.delay).run()
