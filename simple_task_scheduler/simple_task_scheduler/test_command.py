#!/usr/bin/env python3

# Copyright 2023 ROS Industrial Consortium Asia Pacific.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import asyncio
import argparse
import sys
import uuid
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import HistoryPolicy as History
from rclpy.qos import DurabilityPolicy as Durability
from rclpy.qos import ReliabilityPolicy as Reliability
from rmf_task_msgs.msg import ApiRequest, ApiResponse
from cmd_api_msgs.models.bag_command_request import BagCommandRequest

class SendPlayTaskCommand(Node):
    def __init__(self, argv=sys.argv):
        super().__init__("send_play_task_command")
        parser = argparse.ArgumentParser()
        parser.add_argument('-c', '--command', required=True,
                            type=str, help='Commands: [start] or [stop]')
        self.args = parser.parse_args(argv[1:])

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        # Publisher
        self.pub = self.create_publisher(
            ApiRequest,
            'cmd_api_requests',
            transient_qos
        )

        # subscriber
        self.sub = self.create_subscription(
            ApiResponse,
            'cmd_api_responses',
            self.receive_response,
            10
        )

        # create message
        self.response = asyncio.Future()
        self.msg = ApiRequest()

        # return if command is not valid
        if not (self.args.command == "start" or self.args.command == "stop"):
            self.get_logger().info(f"No such command {self.args.command}\n"
                f"Only commands: 'stop' and 'start' rea accepted")
            self.response.cancel(f"No such command {self.args.command}")
            return

        self.command = BagCommandRequest(type="play_task_command", command=self.args.command)
        self.msg.request_id = str(uuid.uuid4())
        self.msg.json_msg = json.dumps(self.command.dict())

        self.pub.publish(self.msg)

    # callback for subscription
    def receive_response(self,msg: ApiResponse):
        if msg.request_id == self.msg.request_id:
            self.response.set_result(json.loads(msg.json_msg))

def main(argv=sys.argv):
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)
    send_play_task_command = SendPlayTaskCommand(args_without_ros)
    rclpy.spin_until_future_complete(
        send_play_task_command,
        send_play_task_command.response,
        timeout_sec=5.0)
    if send_play_task_command.response.cancelled():
        print('Cancelled')
    elif send_play_task_command.response.done():
        print(f'Got response:\n{send_play_task_command.response.result()}')
    else:
        print('Did not get a response')
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
