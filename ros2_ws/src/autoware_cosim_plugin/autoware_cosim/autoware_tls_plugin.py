import rclpy

from terasim_cosim.constants import *
from terasim_cosim.redis_msgs import SUMOSignalDict
from terasim_cosim.redis_client_wrapper import create_redis_client

from rclpy.node import Node
from autoware_perception_msgs.msg import (
    TrafficSignal,
    TrafficSignalArray,
    TrafficSignalElement,
)


class AutowareTLSPlugin(Node):
    def __init__(self):
        super().__init__("autoware_tls_plugin")

        # Publisher for TrafficSignalArray
        self.pub_signal_array = self.create_publisher(
            TrafficSignalArray,
            "/perception/traffic_light_recognition/traffic_signals",
            10,
        )

        # Timer setup
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.on_start()

        print("Starting autoware traffic light interface with redis...")

    def on_start(self):
        # temporary solution to accomodate use case 3
        key_value_config = {COSIM_TLS_INFO: SUMOSignalDict}
        self.redis_client = create_redis_client(key_value_config=key_value_config)

    def on_timer(self):
        try:
            cosim_tls_info = self.redis_client.get(COSIM_TLS_INFO)
        except:
            print("cosim_tls_info not available. Exiting...")
            return

        if cosim_tls_info:
            data = cosim_tls_info.data

            signals = self.get_signal_info(data)

            traffic_signal_array = TrafficSignalArray()
            traffic_signal_array.stamp = self.get_clock().now().to_msg()
            traffic_signal_array.signals = signals

            self.pub_signal_array.publish(traffic_signal_array)

    def get_signal_info(self, traffic_signals_tls_info):
        signals = []

        for node_number, info in traffic_signals_tls_info.items():
            node_number = node_number[5:7]

            for i, light_state in enumerate(info.tls):
                traffic_signal_element = TrafficSignalElement()
                traffic_signal_element.shape = TrafficSignalElement.CIRCLE
                traffic_signal_element.status = TrafficSignalElement.SOLID_ON
                traffic_signal_element.confidence = 1.0

                traffic_light_id = f"{node_number}{i}000"
                if light_state == "r":
                    traffic_signal_element.color = TrafficSignalElement.RED
                elif light_state == "y":
                    traffic_signal_element.color = TrafficSignalElement.AMBER
                else:
                    traffic_signal_element.color = TrafficSignalElement.GREEN

                traffic_signal = TrafficSignal()
                traffic_signal.traffic_signal_id = int(traffic_light_id)
                traffic_signal.elements = [traffic_signal_element]
                signals.append(traffic_signal)

        return signals


def main(args=None):
    rclpy.init(args=args)
    autoware_tls_plugin = AutowareTLSPlugin()
    rclpy.spin(autoware_tls_plugin)
    autoware_tls_plugin.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
