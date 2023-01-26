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

import json
from typing import List, Dict
from threading import Lock
import uuid
from rx.subject.subject import Subject
from rx.disposable.disposable import Disposable

from rclpy.node import Node, Publisher
from rmf_task_msgs.msg import ApiRequest, ApiResponse

# class to encapsulate the methods for managing a thread of tasks
class TaskList():
    def __init__(self,
        node: Node,
        task_api_responses: Subject,
        task_state_event: Subject,
        task_api_request_event: Subject,
        task_list: List[ApiRequest],
        task_api_pub: Publisher,
        num_loops: int):

        self.node = node
        self.task_api_responses = task_api_responses
        self.task_state_event = task_state_event
        self.task_api_request_event = task_api_request_event
        self.task_list = task_list
        self.num_loops = num_loops
        self.task_api_pub = task_api_pub

        self.execute_task_subject = Subject()
        self.paused = True # start off idle
        self.task_idx = 0

        self.request_id: str = None
        self.current_task_id: str = None
        self.request_id_lock = Lock()
        self.current_task_id_lock = Lock()
        self.__subscriptions: List[Disposable] = []

        self.__subscriptions.append(
            self.task_api_responses.subscribe(
                on_next=lambda msg: self.handle_api_response(msg))
        )
        self.__subscriptions.append(
            self.task_state_event.subscribe(
                on_next=lambda data : self.handle_task_completion_state(data))
        )

    # TODO(KW): Consider rx piping operators that could
    # aid in this function.
    def handle_api_response(self, resp: ApiResponse):
        self.request_id_lock.acquire()

        if resp.request_id == self.request_id:
            # set request_id to None. We are no longer waiting for response
            self.request_id = None

            json_msg = json.loads(resp.json_msg)
            success = json_msg["success"]

            if success:
                self.node.get_logger().info(f'API response [{resp.request_id}] successful')
                self.current_task_id_lock.acquire()
                self.current_task_id = json_msg['state']['booking']['id']
                self.current_task_id_lock.release()

                # if this is last task reduce num_loop by one
                if  self.task_idx == (len(self.task_list) - 1):
                    # if already set to infinite loop
                    # do not reduce num_loop
                    if self.num_loops != -1:
                        self.num_loops -= 1

                # task successfully dispatched
                # increase task index by one
                self.task_idx = (self.task_idx + 1)%len(self.task_list)

            else:
                self.node.get_logger().info(f"API Response Error {json_msg['errors']}")
                self.request_id_lock.acquire()
                self.request_id = None

        self.request_id_lock.release()

    def handle_task_completion_state(self, data):
        self.current_task_id_lock.acquire()

        # if current task completed start next task
        if data['booking']['id'] == self.current_task_id:
            if data['status'] == 'completed':
                self.current_task_id = None
                self.start_next_task()

        self.current_task_id_lock.release()

    def start_next_task(self):
        self.node.get_logger().info(
            f"Starting task list of length {len(self.task_list)}\n"
            f"Currently at index {self.task_idx}\n"
            f"Number of loops left :{self.num_loops if self.num_loops != -1 else 'infinite'}\n")

        # if no more taks left
        if self.num_loops == 0 or self.paused:
            return

        # TODO(KW): Find a better implementation
        # rather than using index
        api_request = self.task_list[self.task_idx]
        # TODO(KW): Set the id to reflect the type of task
        api_request.request_id = str(uuid.uuid4())

        # Set the request_id
        self.request_id_lock.acquire()
        self.request_id = self.task_list[self.task_idx].request_id
        self.request_id_lock.release()
        # Publish next api request using RX Subject
        self.task_api_request_event.on_next(api_request)

    def pause(self):
        self.paused = True

    def start(self):
        self.paused = False
        self.start_next_task()