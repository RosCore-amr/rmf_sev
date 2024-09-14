#!/usr/bin/env python
# -*- coding: utf-8 -*-

import jwt
import uvicorn
import requests

from fastapi import FastAPI
from pydantic import BaseModel
import threading

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from robot_interfaces.srv import CreatMission, SearchStock, ExcuteMission
from robot_interfaces.msg import MissionTransport


app = FastAPI(
    title="FastAPI JWT",
    openapi_url="/openapi.json",
    docs_url="/docs",
    description="fastapi jwt",
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
        self.get_logger().info("Swagger run http://127.0.0.1:5000/docs#/")

        self.timer_cb = MutuallyExclusiveCallbackGroup()

        self.cli_empty_cart = self.create_client(SearchStock, "search_empty_cart")
        self.cli_new_mission = self.create_client(CreatMission, "creation_mission")
        self.cli_excute_mission = self.create_client(
            ExcuteMission, "processing_excute_mission"
        )

        self._stock_req = SearchStock.Request()
        self._mission_creat_req = CreatMission.Request()
        self._excute_mission_req = ExcuteMission.Request()
        self.timer = self.create_timer(
            5.0, self.main_loop, callback_group=self.timer_cb
        )

        self.mission_take_cart_empty = self.create_subscription(
            MissionTransport, "transport_empty_cart", self.cart_empty_callback, 10
        )
        self.query_mission_take_cart_empty = False

        @app.post("/occupy_location")
        async def occupy_locations(location_occupy: dict):

            # 22 change
            # 23 supply goods cap
            # 25 take cart empty lay
            # 3 co xe, co hang
            # 5 vi tri trong
            # empty_location = self.find_location_infor_client("find_cart_empty/6")
            # self.get_logger().info(str(empty_location))
            if location_occupy["excute_code"] == "transport_empty_cart":
                mission_empty_cart = self.mission_creat_transport_empty_cart(
                    location_occupy
                )
                self.get_logger().info('mission_empty_cart: "%s"' % mission_empty_cart)
                return mission_empty_cart
            elif location_occupy["excute_code"] == "transport_goods":
                mission_transport_goods = self.mission_creat_transport_goods(
                    location_occupy
                )
                self.get_logger().info(
                    'mission_transport_goods: "%s"' % mission_transport_goods
                )
                return mission_transport_goods
            else:
                return "Error call excute_code "

    def mission_creat_transport_goods(self, end_location):

        location_goods_pickup = self.find_location_infor_client(
            "find_products/" + str(end_location["location_code"])
        )
        if not location_goods_pickup.code:
            return {"code": " dont have goods to take "}

        body_request = {
            "entry_location": {
                "location_code": location_goods_pickup.name,
                "map_code": location_goods_pickup.map_code,
            },
            "end_location": {
                "location_code": end_location["location_code"],
                "map_code": end_location["map_code"],
            },
        }

        mission_carry_goods = self.creation_mission_client(body_request)
        self.process_excute_mission_client(
            "missions_excute_update",
            end_location["excute_code"],
            mission_carry_goods.mission_code,
        )
        return mission_carry_goods.mission_code

    def mission_creat_transport_empty_cart(self, entry_location):
        response_available_location = self.find_location_infor_client(
            "available_location/pickup_locations"
        )
        if not response_available_location.code:
            return {"code": "not have available location"}

        body_request = {
            "entry_location": {
                "location_code": entry_location["location_code"],
                "map_code": entry_location["map_code"],
            },
            "end_location": {
                "location_code": response_available_location.name,
                "map_code": response_available_location.map_code,
            },
        }

        mission_carry_empty_cart = self.creation_mission_client(body_request)
        self.process_excute_mission_client(
            "missions_excute_update",
            entry_location["excute_code"],
            mission_carry_empty_cart.mission_code,
        )
        return mission_carry_empty_cart.mission_code

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

        self._mission_creat_req.entry_location.location_code = body_request[
            "entry_location"
        ]["location_code"]

        self._mission_creat_req.entry_location.map_code = body_request[
            "entry_location"
        ]["map_code"]

        self._mission_creat_req.end_location.location_code = body_request[
            "end_location"
        ]["location_code"]

        self._mission_creat_req.end_location.map_code = body_request["end_location"][
            "map_code"
        ]
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
            self.get_logger().info(str(new_mission_code))

        self.get_logger().info("loop run")

    def creat_mission_take_empty_cart(self):
        response_empty_cart = self.find_location_infor_client("find_cart_empty/6")
        if not response_empty_cart.code:
            return {"code": "not have empty cart"}
        response_available_location = self.find_location_infor_client(
            "available_location/pickup_locations"
        )
        if not response_available_location.code:
            return {"code": "not have available location"}

        body_request = {
            "entry_location": {
                "location_code": response_empty_cart.name,
                "map_code": response_empty_cart.map_code,
            },
            "end_location": {
                "location_code": response_available_location.name,
                "map_code": response_available_location.map_code,
            },
        }
        new_mission_code = self.creation_mission_client(body_request)
        self.process_excute_mission_client(
            "missions_excute_update",
            "transport_empty_cart",
            new_mission_code.mission_code,
        )
        return new_mission_code.mission_code

    def cart_empty_callback(self, msg):
        # self.get_logger().info("hello on here")

        if len(msg.mission_excute) < 6:
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

    uvicorn.run(app, port=5000, log_level="warning")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
