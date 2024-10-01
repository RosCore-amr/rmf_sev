#!/usr/bin/env python
# -*- coding: utf-8 -*-

import jwt
import uvicorn
import requests
import yaml
import json
from fastapi import FastAPI, Request
from pydantic import BaseModel
import threading

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from robot_interfaces.srv import (
    CreatMission,
    SearchStock,
    ExcuteMission,
    CommandApi,
    Collision,
    GetInformation,
)
from robot_interfaces.msg import MissionTransport, MissionCurrent
from fastapi.middleware.cors import CORSMiddleware


app = FastAPI(
    title="Robot API",
    openapi_url="/openapi.json",
    docs_url="/docs",
    description="controlsystem",
)


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class SumResponse(BaseModel):
    sum: int


class MissionOccupy(BaseModel):
    location_code: str
    map_code: str
    occupe_type: int


class MissionRequestClient(Node):
    def __init__(self):
        super().__init__("mission_process_client_async")
        self.cb = None

        self.timer_cb = MutuallyExclusiveCallbackGroup()

        self.cli_empty_cart = self.create_client(GetInformation, "search_empty_cart")
        self.cli_new_mission = self.create_client(CreatMission, "creation_mission")
        self.cli_data_update_status = self.create_client(
            CommandApi, "update_data_database"
        )
        self.cli_excute_mission = self.create_client(
            ExcuteMission, "processing_excute_mission"
        )

        self.cli_collision = self.create_client(Collision, "collision_process")

        self._stock_req = GetInformation.Request()
        self._mission_creat_req = CreatMission.Request()
        self._excute_mission_req = ExcuteMission.Request()
        self.timer = self.create_timer(
            5.0, self.main_loop, callback_group=self.timer_cb
        )

        self.subscription_mission_current = self.create_subscription(
            String,
            "mission_current_request",
            self.mission_current_callback,
            10,
        )

        self.mission_take_cart_empty = self.create_subscription(
            MissionTransport, "transport_empty_cart", self.cart_empty_callback, 10
        )

        self.query_mission_take_cart_empty = False
        self.mission_transport_goods_current = None
        self.mission_transport_empty_cart_current = None

        @app.post("/occupy_location")
        async def occupy_locations(location_occupy: dict):

            # 22 change
            # 23 supply goods cap
            # 25 take cart empty lay
            # 3 co xe, co hang
            # 5 vi tri trong
            # self.get_logger().info(str(empty_location))

            _occupy_location = self.find_location_infor_client(
                str("query_location/" + "return_locations/" + location_occupy["name"])
            )
            _location = eval(_occupy_location.msg_response)
            if not _location:
                return {"mission_code": 0, "msg": "Call error"}

            if location_occupy["excute_code"] == "transport_empty_cart":
                mission_empty_cart = self.mission_creat_transport_empty_cart(_location)
                self.get_logger().info('mission_empty_cart: "%s"' % mission_empty_cart)
                return mission_empty_cart
            elif location_occupy["excute_code"] == "transport_goods":
                mission_transport_goods = self.mission_creat_transport_goods(_location)
                self.get_logger().info(
                    'mission_transport_goods: "%s"' % mission_transport_goods
                )
                return mission_transport_goods
            else:
                return "Error call excute_code "

        @app.post("/update_robot_status")
        async def update_robot_status(robot_update: dict, request: Request):
            _ip_host = request.client.host
            _ip = {"ip_machine": _ip_host}
            robot_update.update(_ip)

            element_update = robot_update.keys()
            if (
                "collision" not in element_update
                and "robot_status" not in element_update
            ):
                return {"code": 0}
            _return = {}
            collision_process = {
                "ip_machine": _ip_host,
                "collision": robot_update["collision"],
                "robot_status": robot_update["robot_status"],
                "map_code": robot_update["map_code"],
            }
            self.get_logger().info('collision_process: "%s"' % collision_process)
            self.get_logger().info('element_update: "%s"' % element_update)

            url_update_robot = "update_robotStatus"
            data_return = self.processing_update_client(
                str(url_update_robot), robot_update
            )
            _return.update(eval(data_return.msg_response))

            return _return

        @app.post("/robot_request_mission")
        async def robot_request_mission(robot_request: dict):
            # self.get_logger().info('robot_update: "%s"' % robot_update)
            _map_code = robot_request["map_code"]
            process_request = self.robot_processing_take_mission(_map_code, 2)
            return process_request

        @app.post("/robot_comfirm_mission")
        async def robot_comfirm_mission(robot_comfirm: dict):
            process_mission = self.mission_runing_process(robot_comfirm)
            self.get_logger().info('robot_update: "%s"' % process_mission)

            return process_mission

        @app.post("/robot_update_location")
        async def robot_update_location(robot_comfirm: dict, request: Request):
            # process_mission = self.mission_runing_process(robot_comfirm)
            # self.get_logger().info('robot_update: "%s"' % process_mission)
            client_host = request.client.host
            # process_mission = self.mission_runing_process(robot_comfirm)
            self.get_logger().info('client_host: "%s"' % client_host)
            return robot_comfirm

        @app.post("/collision_request")
        async def collision(robot_comfirm: dict):
            # client_host = request.client.host
            # # process_mission = self.mission_runing_process(robot_comfirm)
            sever_response = self.collision_client(robot_comfirm)
            self.get_logger().info('resutl: "%s"' % sever_response.result)

            return eval(sever_response.result)

    def mission_creat_transport_goods(self, _location_return):

        location_goods = self.find_location_infor_client(
            "find_products/" + str(_location_return["name"])
        )
        _location_have_goods = eval(location_goods.msg_response)
        if not _location_have_goods:
            return {"mission_code": 0, "msg": "dont have goods to take"}

        # mx = self.pre_location()
        # self.get_logger().info('location_return : "%s"' % location_return)
        task_performed = "transport_goods"

        body_request = {
            "entry_location": {
                "location_code": _location_have_goods["name"],
                "map_code": _location_have_goods["map_code"],
                "point": _location_have_goods["point"],
            },
            "end_location": {
                "location_code": _location_return["name"],
                "map_code": _location_return["map_code"],
                "point": _location_return["point"],
            },
            "excute_code": task_performed,
        }

        mission_carry_goods = self.creation_mission_client(body_request)
        self.process_excute_mission_client(
            "missions_excute_update",
            task_performed,
            mission_carry_goods.mission_code,
        )
        # self.get_logger().info('body_request: "%s"' % body_request)

        # return True
        return {"mission_code": mission_carry_goods.mission_code, "msg": 1}

    def mission_creat_transport_empty_cart(self, _empty_cart_location):

        available_location = self.find_location_infor_client(
            "available_location/pickup_locations"
        )
        _available_location = eval(available_location.msg_response)
        if not _available_location:
            return {"mission_code": 0, "msg": "not have available location"}

        # _available_location = eval(available_location.msg_response)[0]
        task_performed = "transport_empty_cart"

        body_request = {
            "entry_location": {
                "location_code": _empty_cart_location["name"],
                "map_code": _empty_cart_location["map_code"],
                "point": _empty_cart_location["point"],
            },
            "end_location": {
                "location_code": _available_location["name"],
                "map_code": _available_location["map_code"],
                "point": _available_location["point"],
            },
            "excute_code": task_performed,
        }

        mission_carry_empty_cart = self.creation_mission_client(body_request)
        self.process_excute_mission_client(
            "missions_excute_update",
            task_performed,
            mission_carry_empty_cart.mission_code,
        )
        return mission_carry_empty_cart.mission_code
        self.get_logger().info('body_request: "%s"' % body_request)

        return True

    def collision_client(self, request_body):
        req = Collision.Request()
        while not self.cli_collision.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        # req.url = _url
        req.robot_code = request_body["robot_code"]
        req.position_collision = request_body["position_collision"]
        future = self.cli_collision.call_async(req)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

        return None

    def processing_update_client(self, _url, request_body):
        req = CommandApi.Request()
        while not self.cli_data_update_status.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        req.url = _url
        req.msg_request = str(request_body)
        future = self.cli_data_update_status.call_async(req)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

        return None

    def find_location_infor_client(self, _url):
        while not self.cli_empty_cart.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        self._stock_req.url = _url
        future = self.cli_empty_cart.call_async(self._stock_req)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

        return None

    def creation_mission_client(self, body_request):
        while not self.cli_new_mission.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        # self._mission_creat_req.entry_location.location_code = body_request[
        #     "entry_location"
        # ]["location_code"]

        # self._mission_creat_req.entry_location.map_code = body_request[
        #     "entry_location"
        # ]["map_code"]

        # self._mission_creat_req.end_location.location_code = body_request[
        #     "end_location"
        # ]["location_code"]

        # self._mission_creat_req.end_location.map_code = body_request["end_location"][
        #     "map_code"
        # ]
        self._mission_creat_req.mission_request = str(body_request)
        future = self.cli_new_mission.call_async(self._mission_creat_req)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

        return None

    def process_excute_mission_client(self, _url, excute_code, value):
        while not self.cli_excute_mission.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        self._excute_mission_req.url = _url
        self._excute_mission_req.excute_code = excute_code
        self._excute_mission_req.value = value

        future = self.cli_excute_mission.call_async(self._excute_mission_req)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

        return None

    def main_loop(self) -> None:

        if self.query_mission_take_cart_empty:
            new_mission_code = self.creat_mission_take_empty_cart()
            # self.get_logger().info(str(new_mission_code))

            # self.get_logger().info("loop run")

    def creat_mission_take_empty_cart(self):

        response_empty_cart = self.find_location_infor_client("find_cart_empty/6")

        _response_empty_cart = eval(response_empty_cart.msg_response)
        if not _response_empty_cart:
            return {"mission_code": 0, "msg": "dont have empty cart"}

        # self.get_logger().info("call missiin")
        mission_transport_empty_cart = self.mission_creat_transport_empty_cart(
            _response_empty_cart
        )
        return mission_transport_empty_cart

    def mission_runing_process(self, request):
        url_name = "remove_pending_task"
        data_return = self.processing_update_client(str(url_name), request)
        return eval(data_return.msg_response)

    def robot_processing_take_mission(self, _excute_code, n):

        if n == 0:
            return {"mission_code": 0, "code": "dont have mission for robot"}

        _excute_task_code = ["transport_empty_cart", "transport_goods"]
        if _excute_code == "transport_empty_cart":
            mission_infor = self.mission_transport_empty_cart_current

        elif _excute_code == "transport_goods":

            mission_infor = self.mission_transport_goods_current
        else:
            return {"mission_code": 0, "code": "type work not exist"}

        if not mission_infor["mission_code"]:
            _excute_task_code.remove(_excute_code)
            return self.robot_processing_take_mission(_excute_task_code[0], n - 1)

        _location = {"excute_code": _excute_code}
        url_name = "missions_excute_pop"
        data_return = self.processing_update_client(str(url_name), _location)

        return mission_infor

    def mission_current_callback(self, msg):

        _mission_transport_goods_current = eval(msg.data)

    def cart_empty_callback(self, msg):
        # self.get_logger().info("hello on here")

        if len(msg.mission_excute) < 2:
            self.query_mission_take_cart_empty = True
        else:
            self.query_mission_take_cart_empty = False
        # self.get_logger().info(
        #     'query_mission_take_cart_empty: "%d"' % self.query_mission_take_cart_empty
        # )


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MissionRequestClient()

    # executor = MultiThreadedExecutor()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(minimal_client)
    # executor.spin()
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    minimal_client.get_logger().info("Swagger run http://127.0.0.0:2100/docs#/")

    uvicorn.run(app, host="0.0.0.0", port=2100, log_level="warning")
    minimal_client.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
