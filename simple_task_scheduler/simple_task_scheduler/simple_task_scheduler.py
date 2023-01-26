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

import os
import json
import sys
import yaml
from typing import List, Dict
from rx.subject.subject import Subject

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import HistoryPolicy as History
from rclpy.qos import DurabilityPolicy as Durability
from rclpy.qos import ReliabilityPolicy as Reliability
from rmf_task_msgs.msg import ApiRequest, ApiResponse
from cmd_api_msgs.models.play_tasks_command_request import PlayTasksCommandRequest
from cmd_api_msgs.models.play_tasks_command_response import PlayTasksCommandResponse
from ament_index_python.packages import get_package_share_directory

from simple_task_scheduler.task_list import TaskList
from simple_task_scheduler.web_socket_monitor import WebSocketMonitor

class SimpleTaskScheduler(Node):
    def __init__(self, argv=sys.argv):
        super().__init__('simple_task_scheduler')

        # Declare parameters
        self.declare_parameter('input_socket_url', 'localhost')
        self.declare_parameter('input_socket_port', 7879)
        self.declare_parameter('output_socket_url', 'localhost')
        self.declare_parameter('output_socket_port', 7878)
        self.declare_parameter('config_file', 'default_task_list.yaml')

        # Get parameters
        self.input_socket_url = \
            self.get_parameter('input_socket_url').get_parameter_value().string_value
        self.input_socket_port = \
            self.get_parameter('input_socket_port').get_parameter_value().integer_value
        self.output_socket_url = \
            self.get_parameter('output_socket_url').get_parameter_value().string_value
        self.output_socket_port = \
            self.get_parameter('output_socket_port').get_parameter_value().integer_value
        self.config_file = \
            self.get_parameter('config_file').get_parameter_value().string_value

        self.config_file_path = os.path.join(
            get_package_share_directory('simple_task_scheduler'),
            'config',
            self.config_file,
        )

        # Starts off paused
        self.pause = True

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=5,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        # Task API request publisher
        self.task_api_req_pub = self.create_publisher(
            ApiRequest,
            'task_api_requests',
            transient_qos
        )

        # Task API response subscriber
        self.task_api_resp_sub = self.create_subscription(
            ApiResponse,
            'task_api_responses',
            self.api_resp_cb,
            transient_qos
        )

        # Command API response publisher
        self.cmd_api_resp_pub = self.create_publisher(
            ApiResponse,
            'cmd_api_responses',
            transient_qos
        )

        # Command API request subscriber
        self.cmd_api_req_sub = self.create_subscription(
            ApiRequest,
            'cmd_api_requests',
            self.cmd_api_cb,
            transient_qos
        )

        # create rx subjects
        self.task_api_responses = Subject()
        self.task_state_event = Subject()
        self.task_api_request_event = Subject()

        # List to store all task lists
        self.task_lists: List[TaskList] = []

        self.task_api_request_event.subscribe(
            on_next= lambda msg: self.publish_task_api_request(msg)
        )

        self.load_lists()

        # NOTE: ws monitor starts spining once constructed
        self.ws_monitor = WebSocketMonitor(
            node=self,
            task_state_event=self.task_state_event,
            server_url= self.input_socket_url,
            server_port= self.input_socket_port,
            output_url= self.output_socket_url,
            output_port= self.output_socket_port,
        )

    def publish_task_api_request(self, req: ApiRequest):
        self.task_api_req_pub.publish(req)

    # load all task lists from config file
    def load_lists(self):
        task_lists = None
        with open(self.config_file_path,'r') as f:
            task_lists = yaml.safe_load(f)

        if task_lists is None:
            return

        for task_list in task_lists:
            num_loops = task_list['num_loops']
            task_list = self.load_tasks(task_list['list_file'])

            task_list = TaskList(node=self,
                task_api_responses=self.task_api_responses,
                task_state_event=self.task_state_event,
                task_api_request_event=self.task_api_request_event,
                task_list=task_list,
                task_api_pub=self.task_api_req_pub,
                num_loops=num_loops)

            self.task_lists.append(task_list)

    # Load tasks from task list file
    def load_tasks(self, file) -> List[ApiRequest]:
        file_path = os.path.join(
            get_package_share_directory('simple_task_scheduler'),
            'config',
            file)

        yaml_data = None
        with open(file_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
        print(yaml_data)
        task_list: List[ApiRequest] = []
        for task in yaml_data:
            print(f"Loading task {task}")
            # Load task depending on type
            if task['type'] == 'dict':
                task_list.append(self.load_task_from_dict(task))

            elif task['type'] == 'yaml_file':
                task_list.append(self.load_task_from_file(task))
        return task_list

    # TODO(KW): Handle Key errors here
    def load_task_from_dict(self, dict: Dict) -> ApiRequest:
        api_request = ApiRequest()
        request = {}
        payload = {}

        if dict['robot'] and dict['fleet']:
            payload['type'] = 'robot_task_request'
            payload['robot'] = dict['robot']
            payload['fleet'] = dict['fleet']
        else:
            payload['type'] = 'dispatch_task_request'

        request['category'] = dict['category']
        request['description'] = dict['description']
        payload['request'] = request

        api_request.json_msg = json.dumps(payload)
        return api_request

    # this file should only contain a single task
    def load_task_from_file(self, task) -> ApiRequest:
        file_name = task['yaml_file']
        file_path = os.path.join(
            get_package_share_directory('simple_task_scheduler'),
            'config',
            file_name
        )
        task_dict = None
        with open(file_path, 'r') as f:
            task_dict = yaml.safe_load(f)

        return self.load_task_from_dict(task_dict)

    def api_resp_cb(self, msg: ApiRequest):
        self.task_api_responses.on_next(msg)

    def cmd_api_cb(self, msg: ApiRequest):
        json_msg = json.loads(msg.json_msg)
        command = json_msg['command']

        resp = ApiResponse()
        resp.request_id = msg.request_id

        # TODO(KW): Handle stopping and starting
        # of separate task lists
        if command == 'start' and self.pause:
            self.get_logger().info("Starting Task Scheduler")
            self.start_all()
        elif command == 'stop' and not self.pause:
            self.get_logger().info("Pausing Task Scheduler")
            self.pause_all()
        else:
            command_response = PlayTasksCommandResponse(status='failure')
            resp.json_msg = json.dumps(command_response.dict())
            self.cmd_api_resp_pub.publish(resp)
            return
            # Other cases:
            # command is start and pause is False -> pass
            # command is stop and and pause is True -> pass

        command_response = PlayTasksCommandResponse(status='success')
        resp.json_msg = json.dumps(command_response.dict())
        self.cmd_api_resp_pub.publish(resp)

    def start_all(self):
        self.pause = False
        for list in self.task_lists:
            list.start()

    def pause_all(self):
        self.pause = True
        for list in self.task_lists:
            list.pause()


