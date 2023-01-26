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


# NOTE: This is based on rmf_demos_panel rmf_msg_observer.py

import asyncio
import json
from rclpy.node import Node
import threading
from rx.subject.subject import Subject
from typing import Dict, List, Optional, Tuple

import websockets
from websockets.exceptions import ConnectionClosedError


class RmfMsgType:
    FleetState = 'fleet_state_update'
    TaskState = 'task_state_update'
    TaskLog = 'task_log_update'
    FleetLog = 'fleet_log_update'


def filter_rmf_msg(
        json_str: str,
        filters: Dict[RmfMsgType, List[str]] = {}
) -> Optional[Tuple[RmfMsgType, Dict]]:

    obj = json.loads(json_str)
    if 'type' not in obj:
        print('ERRORRRR: type is not avail as json key')
        return None

    if obj['type'] not in filters:
        return None

    msg_type = obj['type']
    data = obj['data']
    if not filters[msg_type]:  # empty list
        return msg_type, data

    for filter_ in filters[msg_type]:
        if filter_ not in data:
            print(f' Key ERROR!!, indicated data_filter: '
                  '[{filter_}] is not avail in {data}')
            return None

        data = data[filter_]
    return msg_type, data


class WebSocketMonitor():
    def __init__(self,
                 node: Node,
                 task_state_event: Subject,
                 server_url: str = 'localhost',
                 server_port: str = '7877',
                 output_url: str = 'localhost',
                 output_port: str = '7878',
                 msg_filters: Dict[RmfMsgType, List[str]] = {RmfMsgType.TaskState: [],
                                                             RmfMsgType.FleetState: [],
                                                             RmfMsgType.TaskLog: [],
                                                             RmfMsgType.FleetLog: []
                                                             },
                 printout_data: bool = False
                 ) -> None:

        self.node = node
        self.task_state_event = task_state_event
        self.server_url = server_url
        self.server_port = server_port
        self.output_url = output_url
        self.output_port = output_port
        self.msg_filters = msg_filters
        self.printout_data = printout_data

        self.task_states = {}
        self.task_states_changed = False

        # Finally, start the async spin thread
        self.start_websocket_spin_thread()

    def start_websocket_spin_thread(self):
        self.ws_th = threading.Thread(target=self.websocket_spin, daemon=True)
        self.ws_th.start()

    def websocket_spin(self, future=asyncio.Future()):
        self.future = future
        asyncio.run(self.__internal_spin())

    async def __reproduce(self, message: str, host: str, port: str) -> None:
        # TODO(Hanif): Perhaps maintain connection in init and
        # do reconnection checks. But currently performs ok.
        async with websockets.connect(f'ws://{host}:{port}') as ws:
            await ws.send(message)

    async def __msg_handler(self, websocket, path):
        try:
            async for message in websocket:
                ret_data = filter_rmf_msg(
                    message, self.msg_filters)
                if ret_data:
                    # passthrough
                    try:
                        await self.__reproduce(message, self.output_url, self.output_port)
                    except ConnectionRefusedError as e:
                        self.node.get_logger().error(
                            f'Error! Websocket reproduce connection refused: {e}')

                    # call the provided callback function
                    msg_type, data = ret_data

                    self.callback_fn(msg_type, data)

        except ConnectionClosedError as e:
            self.node.get_logger().error(f'Error! Websocket connection closed: {e}')
            self.node.get_logger().info('Waiting for reconnection')

    async def __check_future(self):
        while not self.future.done():
            await asyncio.sleep(1)  # arbitrary loop freq check
        self.node.get_logger().info('Received exit signal')

    async def __internal_spin(self):
        self.node.get_logger().info('Beginning websocket.serve')
        async with websockets.serve(
                self.__msg_handler, self.server_url, self.server_port):
            await self.__check_future()

    def callback_fn(self, msg_type, data):
        if msg_type == RmfMsgType.TaskState:
            self.task_state_event.on_next(data)

