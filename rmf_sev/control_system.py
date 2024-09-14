import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.msg import MissionTransport
from robot_interfaces.srv import SearchStock
from example_interfaces.srv import AddTwoInts
from functools import partial
from amd_sevtsv import ServerControl


class Controlsystem(Node):

    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.mission_take_cart_empty = self.create_subscription(
            MissionTransport, "transport_empty_cart", self.cart_empty_callback, 10
        )
        self.mission_take_cart_empty  # prevent unused variable warning
        self.cli_empty_cart = self.create_client(SearchStock, "search_empty_cart")
        self.cli_2point = self.create_client(AddTwoInts, "add_two_ints")
        self._stock_req = SearchStock.Request()

    def send_request(self):
        # client = node.create_client(AddTwoInts, "add_two_ints")
        # while not self.cli_2point.wait_for_service(1.0):
        #     self.get_logger().warn("Waiting for Server Add Two Ints...")
        #     return True

        # request = AddTwoInts.Request()
        # request.a = 3
        # request.b = 8
        mx = ServerControl.searching_information_db("find_cart_empty6")
        # self.get_logger().info(mx)

        # future.result()

    def callback_call_add_two_ints(self, future):
        try:
            response = future.result()
            self.get_logger().info(str(response.sum))
            # if response is not None:
            # self.hhhmmm(response)
            # hh = "asdasdasda"
            return response
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
        return response

    def hhhmmm(self, respon):
        self.get_logger().info(str(respon.sum))

    def cart_empty_callback(self, msg):
        self.get_logger().info("service not available, waiting again...")
        # self.send_request()
        if len(msg.mission_excute) < 4:
            # self.find_cart_empty()
            #  request mission
            self.get_logger().info('I heard: "%s"' % msg.mission_type)

    def send_requestssss(self):
        # req = SearchStock.Request()
        while not self.cli_empty_cart.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        self._stock_req.url = "find_cart_empty6"
        self.get_logger().info(", waiting again...")
        return self.cli_empty_cart.call_async(self._stock_req)

    def find_cart_empty(self):
        future = self.send_requestssss()
        self.get_logger().info(" future")
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(", waiting again...")

        try:
            response = future.result()
            self.get_logger().info(str(response))
            # if response is not None:
            return_body = {"name": "Asdasdasd"}
            return return_body
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
        return response

        # req = SearchStock.Request()
        # while not self.cli_empty_cart.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("service not available, waiting again...")
        #     return False

        # req.url = "find_cart_empty6"
        # self.get_logger().info(", waiting again...")
        # future = self.cli_empty_cart.call_async(req)
        # while rclpy.ok():
        #     # self.get_logger().info(future.done() )
        #     if future.done() and future.result():
        #         self.get_logger().info(future.result())
        #         # return self.future.result()

        # # hh = future.add_done_callback(partial(self.find_empty_cart_callback))
        # self.get_logger().info(("axsdas"))

    def find_empty_cart_callback(self, future):
        return_body = {}
        try:
            response = future.result()
            self.get_logger().info(str(response))
            # if response is not None:
            return_body = {"name": "Asdasdasd"}
            return return_body
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
        return response

    def timer_callback(self):
        msg = String()
        self.send_request()
        # self.find_cart_empty()
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Controlsystem()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
