import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__("image_subscriber")

        # Subscribing to three different topics
        self.my_subscriptions = []  # Renamed to avoid conflict
        self.window_names = [
            "/camera/image_color/compressed",
        ]

        for topic in self.window_names:
            subscription = self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, topic=topic: self.listener_callback(msg, topic),
                10,
            )
            self.my_subscriptions.append(subscription)  # Use the renamed attribute

    def listener_callback(self, msg, topic):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Display the image in a separate window for each topic
        cv2.imshow(topic, image_np)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
