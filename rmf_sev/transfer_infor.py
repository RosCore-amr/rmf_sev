#!/usr/bin/env python
# -*- coding: utf-8 -*-

import jwt
import uvicorn
import requests

from fastapi import FastAPI
from pydantic import BaseModel
import threading
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import InforSevt, Token
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from robot_interfaces.srv import CreatMission, SearchStock, ExcuteMission

# from mission_req import MissionRequestClient

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


class SumClient(Node):
    def __init__(self):
        super().__init__("int_sum_service")
        # self._client = AddTwoIntsClientAsync()
        self.get_logger().info("Swagger run http://127.0.0.1:5000/docs#/")
        self.timer_cb = MutuallyExclusiveCallbackGroup()
        self.cli_empty_cart = self.create_client(SearchStock, "search_empty_cart")
        self.cli_new_mission = self.create_client(CreatMission, "creation_mission")

        self._stock_req = SearchStock.Request()

        self.timer = self.create_timer(
            2.0, self.main_loop, callback_group=self.timer_cb
        )

        @app.post("/occupy_location")
        async def occupy_locations(location_occupy: dict):

            # 22 change
            # 23 supply goods cap
            # 25 take cart empty lay
            # 3 co xe, co hang
            # 5 vi tri trong

            self.get_logger().info('code: "%s"' % location_occupy)

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

    def main_loop(self) -> None:
        ms = self.find_location_infor_client("find_cart_empty/6")
        self.get_logger().info(str(ms))
        self.get_logger().info("loop run")


def main(args=None):
    rclpy.init()
    sum_server = SumClient()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(sum_server)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    uvicorn.run(app, port=5000, log_level="warning")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
