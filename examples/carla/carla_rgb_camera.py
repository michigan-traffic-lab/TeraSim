import cv2
import time
import carla
import numpy as np
import threading


# CARLA sensor manager class
class SensorManager:
    """Manages CARLA RGB camera sensor and processes data."""

    def __init__(self, world, transform, attached, sensor_options):
        self.world = world
        self.sensor_options = sensor_options
        self.sensor = self.init_sensor(transform, attached, sensor_options)
        self.latest_image = None
        self.lock = threading.Lock()

    def init_sensor(self, transform, attached, sensor_options):
        camera_bp = self.world.get_blueprint_library().find("sensor.camera.rgb")

        for key, value in sensor_options.items():
            camera_bp.set_attribute(key, value)

        camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
        camera.listen(self.camera_callback)

        return camera

    def camera_callback(self, image):
        """Callback function for the RGB camera."""
        # Convert the raw image data to a numpy array
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(
            array, (image.height, image.width, 4)
        )  # BGRA format from CARLA
        img_bgr = array[:, :, :3]  # Remove alpha channel (BGR format)

        # Store the latest image in a thread-safe manner
        with self.lock:
            self.latest_image = img_bgr

    def get_latest_image(self):
        """Get the latest image in a thread-safe manner."""
        with self.lock:
            return self.latest_image

    def destroy(self):
        """Clean up the sensor."""
        if self.sensor is not None:
            self.sensor.destroy()
        cv2.destroyAllWindows()


def main():
    # Connect to the CARLA server
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    # Find the vehicle with the role_name "CAV"
    CAV = None
    while CAV is None:
        actor_list = world.get_actors().filter("vehicle.*")
        for x in actor_list:
            if x.attributes.get("role_name") == "CAV":
                CAV = x
        print("CAV not found. Waiting for CAV to spawn...")
        time.sleep(0.1)

    print("Found CAV, attaching RGB camera...")

    # Create and attach the RGB camera
    sensor_manager = SensorManager(
        world,
        carla.Transform(
            carla.Location(x=-11.0, y=0.0, z=7.0),  # Adjust camera position
            carla.Rotation(roll=0.0, pitch=-25.0, yaw=0.0),  # Adjust camera rotation
        ),
        CAV,
        {
            "image_size_x": "1920",
            "image_size_y": "1080",
        },
    )

    try:
        while True:
            time.sleep(0.01)
            print("Running...")

            # Get the latest image from the sensor manager
            img_bgr = sensor_manager.get_latest_image()
            if img_bgr is not None:
                # Display the image using OpenCV
                cv2.imshow("RGB Camera", img_bgr)
                cv2.waitKey(1)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        sensor_manager.destroy()


if __name__ == "__main__":
    main()
