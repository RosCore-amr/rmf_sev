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
from tutorial_interfaces.msg import InforSevt, Token
from tutorial_interfaces.srv import LocationAsk


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
        self.cli_location_available = self.create_client(
            LocationAsk, "check_location_available"
        )
        # self.cli_new_mission = self.create_client(
        #     LocationAsk, "check_location_available"
        # )
        # if self.token:
        # self.get_logger().info("Sending request to service ")

        @app.post("/occupy_location")
        async def occupy_locations(location_occupy: dict):

            # 22 change
            # 23 supply cap
            # 25 take lay
            # 3 co xe, co hang
            # 5 vi tri trong
            permission_task = location_occupy["occupy_code"] - 20
            # if location_occupy["occupy_code"] == 22:
            #     permission_task = 3
            #     mission_task = 42
            # elif location_occupy["occupy_code"] == 23:
            #     permission_task = 3
            #     mission_task = 43
            # elif location_occupy["occupy_code"] == 25:
            #     permission_task = 5
            #     mission_task = 45
            # else:
            #     return "creat task error"
            location_execute = self.available_locaiton_client(
                location_occupy, permission_task
            )
            if not location_execute["code"]:
                return location_execute

            # trigger_mission = self.mission_client(
            #     location_occupy, location_execute, mission_task
            # )

            # return response_model

    def mission_client(self, location_occupy, location_execute, mission_task):
        if mission_task == 43:
            mission_transport = {
                "start": {
                    "location_code": location_execute["name"],
                    "map_code": location_execute["mapcode"],
                },
                "end": {
                    "location_code": location_occupy["name"],
                    "map_code": location_occupy["name"],
                },
            }
        elif mission_task == 45:
            mission_transport = {
                "start": {
                    "location_code": location_occupy["name"],
                    "map_code": location_occupy["mapcode"],
                },
                "end": {
                    "location_code": location_execute["name"],
                    "map_code": location_execute["name"],
                },
            }
        elif mission_task == 45:
            pass
        else:
            return "location not exit "

    # def create_tasks(self , )
    def available_locaiton_client(self, _request_body, _permission_task):
        req = LocationAsk.Request()
        while not self.cli_location_available.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        req.location_code = _request_body["location_code"]
        req.map_code = _request_body["map_code"]
        req.occupy_code = _permission_task
        future = self.cli_location_available.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        Response_body = ""
        try:
            response = future.result()
            # self.get_logger().info('code: "%s"' % response.code)
            # self.get_logger().info('msg: "%s"' % response.msg)
            Response_body = {"code": response.code, "msg": response.msg}
            # self.get_logger().info("service not available, waiting again...")
            # self.get_logger().info(f"{a} + {b} = {response.sum}")
        except Exception as e:
            self.get_logger().info(f"Service call failed {e}")

        return Response_body


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
