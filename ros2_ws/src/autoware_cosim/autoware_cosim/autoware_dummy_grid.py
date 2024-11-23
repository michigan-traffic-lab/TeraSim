import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid


class AutowareDummyGrid(Node):
    def __init__(self):
        super().__init__('occ_grid_converter')

        self.saved_odom_msg = Odometry()
        
        self.pub_occ_grid = self.create_publisher(OccupancyGrid, '/perception/occupancy_grid_map/map', 10)
        self.sub_ego_odom = self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        self.timer = self.create_timer(0.2, self.on_timer)
        
        self.init()
        
        print("Starting autoware occupancy grid system...")
        
    def init(self):
        self.map = OccupancyGrid()
        self.map.info.map_load_time = self.get_clock().now().to_msg()
        self.map.info.resolution = 0.5
        self.map.info.origin.position.x = 0.0
        self.map.info.origin.position.y = 0.0
        self.map.info.origin.position.z = 0.0
        self.map.info.origin.orientation.x = 0.0
        self.map.info.origin.orientation.y = 0.0
        self.map.info.origin.orientation.z = 0.0
        self.map.info.origin.orientation.w = 1.0
        
        self.map_height = 300
        self.map_width = 300
        self.map.info.height = self.map_height
        self.map.info.width = self.map_width
        self.map.data = [0] * (self.map.info.width * self.map.info.height)  # Initialize occupancy grid with value 0 (free)

    def on_timer(self):
        self.map.info.origin.position.x = self.saved_odom_msg.pose.pose.position.x - (self.map_width / 2)
        self.map.info.origin.position.y = self.saved_odom_msg.pose.pose.position.y - (self.map_height / 2)
        self.map.info.origin.position.z = 0.0
        
        self.map.header.frame_id = "map"
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.pub_occ_grid.publish(self.map)

    def odom_callback(self, msg):
        self.saved_odom_msg = msg


def main(args=None):
    rclpy.init(args=args)
    autoware_dummy_grid = AutowareDummyGrid()
    rclpy.spin(autoware_dummy_grid)
    autoware_dummy_grid.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
