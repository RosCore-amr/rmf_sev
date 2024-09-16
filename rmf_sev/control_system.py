import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.msg import MissionTransport
from robot_interfaces.srv import SearchStock, CommandApi
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from example_interfaces.srv import AddTwoInts
from functools import partial

# from amd_sevtsv import ServerControl


class ControlsystemRMF(Node):

    def __init__(self):
        super().__init__("rmf_sever_control")
        self.publisher_ = self.create_publisher(String, "demotest", 10)
        timer_period = 1  # seconds
        self.client_command = self.create_client(
            CommandApi, "update_standard_robot_status"
        )
        self.timer_cb = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            2.0, self.main_loop, callback_group=self.timer_cb
        )

        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self._command_req = CommandApi.Request()

    def find_location_infor_client(self):
        while not self.client_command.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        mx = {"map_code": "pickup_locations", "location_status": 3}
        self.get_logger().info('response_api: "%s"' % mx)
        self._command_req.msg_request = str(mx)
        future = self.client_command.call_async(self._command_req)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

        return None

    def main_loop(self):
        msg = String()
        # self.send_request()
        request_body = {
            "location_code": "zone9",
            "map_code": "return_locations",
            "excute_code": "transport_goods",
        }

        test = self.find_location_infor_client()
        # self.find_cart_empty()
        # self.get_logger().info('response_api: "%s"' % test)

        msg.data = str(request_body)
        self.publisher_.publish(msg)
        self.i += 1
        self.get_logger().info(
            'query_mission_take_cart_empty: "%s"' % "minhsieudeptrai"
        )


def main(args=None):
    rclpy.init(args=args)

    rmf_sever_control = ControlsystemRMF()

    rclpy.spin(rmf_sever_control)
    rmf_sever_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
