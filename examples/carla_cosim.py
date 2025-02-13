import time
import redis
import math
import carla
import random
import sys
from collections import defaultdict
from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client
from terasim_cosim.redis_msgs import Vehicle, VehicleDict, ConstructionZone
from utils.actor import *
from utils.geometry import *


class CarlaCosimPlugin(object):
    def __init__(
        self,
        cosim_controlled_vehicle_keys=["terasim_cosim_vehicle_info"],
        # cosim_controlled_pedestrian_keys=["terasim_cosim_pedestrian_info"],
        cosim_controlled_participant_keys=["terasim_cosim_participants_info"],
        control_cav=False,
    ):
        self.client = carla.Client("127.0.0.1", 2000)
        self.client.set_timeout(2.0)

        self.world = self.client.get_world()

        self.control_cav = control_cav
        self.cosim_controlled_vehicle_keys = cosim_controlled_vehicle_keys
        # self.cosim_controlled_pedestrian_keys = cosim_controlled_pedestrian_keys
        self.cosim_controlled_participant_keys = cosim_controlled_participant_keys

        self.vehicle_blueprints = create_vehicle_blueprint(self.world)
        self.motor_blueprints = create_motor_blueprint(self.world)
        self.pedestrian_blueprints = create_pedestrian_blueprint(self.world)
        self.bike_blueprints = create_bike_blueprint(self.world)

        height_path = "/home/zhijie/terasim/TeraSim/examples/utils/test_heights.json"
        self.heights = extract_data_from_json_file(height_path)

        key_value_config = {
            CAV_COSIM_VEHICLE_INFO: VehicleDict,
            TERASIM_COSIM_PARTICIPANTS_INFO: VehicleDict,
            CONSTRUCTION_ZONE_COSIM_INFO: ConstructionZone,
        }
        for key in self.cosim_controlled_vehicle_keys:
            key_value_config[key] = VehicleDict

        self.redis_client = create_redis_client(key_value_config=key_value_config)
        # self.sync_cosim_construction_zone_to_carla()

    def tick(self):
        if self.control_cav:
            self.sync_carla_cav_to_cosim()
        else:
            self.sync_cosim_cav_to_carla()

        self.sync_cosim_participants_to_carla()

        self.world.tick()

    def sync_carla_cav_to_cosim(self):
        vehicle_status, carla_id = get_actor_id_from_attribute(self.world, "CAV")

        if not vehicle_status:
            print("CAV not found in Carla simulation. Exiting...")
            return

        CAV = self.world.get_actor(carla_id)
        transform = CAV.get_transform()
        draw_text(self.world, transform.location + carla.Location(z=2.5), "MCITY-CAV")
        # draw_point(
        #     self.world,
        #     size=0.05,
        #     color=(255, 0, 0),
        #     location=transform.location + carla.Location(z=2.5),
        #     life_time=0,
        # )

        velocity = CAV.get_velocity()
        speed = (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5

        cav_cosim_vehicle_info = VehicleDict()
        cav_cosim_vehicle_info.header.timestamp = time.time()

        x = transform.location.x
        y = transform.location.y

        utm_x, utm_y = carla_to_utm(x, y)
        height = get_carla_height(self.heights, x, y)

        cav_cosim_vehicle_info.data["CAV"] = Vehicle(
            x=utm_x,
            y=utm_y,
            z=height,
            length=5.0,
            width=1.8,
            height=1.8,
            orientation=math.radians(-transform.rotation.yaw),
            speed_long=speed,
        )

        self.redis_client.set(CAV_COSIM_VEHICLE_INFO, cav_cosim_vehicle_info)

    def sync_cosim_cav_to_carla(self):
        try:
            cav_cosim_vehicle_info = self.redis_client.get(CAV_COSIM_VEHICLE_INFO)
        except:
            print("cav_cosim_vehicle_info not available. Exiting...")
            return

        if cav_cosim_vehicle_info:
            cav_info = cav_cosim_vehicle_info.data["CAV"]

            x, y = utm_to_carla(cav_info.x, cav_info.y)
            start_location = carla.Location(x, y, cav_info.z + 5)
            end_location = carla.Location(x, y, cav_info.z - 5)
            # z = get_carla_height(self.heights, x, y)
            yaw = -math.degrees(cav_info.orientation)

            vehicle_status, carla_id = get_actor_id_from_attribute(self.world, "CAV")

            if not vehicle_status:
                blueprint_library = self.world.get_blueprint_library()
                print(blueprint_library.filter("vehicle.lincoln*"))
                blueprint = blueprint_library.find("vehicle.lincoln.mkz_2020")
                blueprint.set_attribute("color", "255, 0, 0")
                blueprint.set_attribute("role_name", "CAV")
                z = self.get_z_offset(start_location, end_location)
                transform = carla.Transform(
                    carla.Location(x=x, y=y, z=z + 0.5), carla.Rotation(yaw=yaw)
                )
                carla_id = spawn_actor(self.client, blueprint, transform)
                print("Spawned CAV in CARLA simulation")
            else:
                CAV = self.world.get_actor(carla_id)
                current_transform = CAV.get_transform()
                z = self.get_z_offset(start_location, end_location, current_transform.location.z)
                transform = carla.Transform(
                    carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw)
                )
                CAV.set_transform(transform)
                draw_text(
                    self.world, transform.location + carla.Location(z=2.5), "MCITY-CAV"
                )
                # draw_point(
                #     self.world,
                #     size=0.05,
                #     color=(255, 0, 0),
                #     location=transform.location + carla.Location(z=2.5),
                #     life_time=0,
                # )
        # self.update_spectator_camera(transform)

    def update_spectator_camera(self, vehicle_transform, follow_distance=10):
        spectator = self.world.get_spectator()
        vehicle_location = vehicle_transform.location
        vehicle_rotation = vehicle_transform.rotation

        # Calculate the position of the camera relative to the vehicle
        camera_location = carla.Location(
            x=vehicle_location.x - follow_distance * math.cos(math.radians(vehicle_rotation.yaw)),
            y=vehicle_location.y - follow_distance * math.sin(math.radians(vehicle_rotation.yaw)),
            z=vehicle_location.z + 5  # Adjust this value to change the height of the camera
        )
        camera_rotation = carla.Rotation(pitch=-30, yaw=vehicle_rotation.yaw)

        # Set the spectator camera transform
        spectator.set_transform(carla.Transform(camera_location, camera_rotation))

    def sync_cosim_construction_zone_to_carla(self):
        def add_interpolated_points(points, offset):
            """
            Interpolates additional points to ensure no two consecutive points 
            after UTM transformation have a distance greater than the specified offset.
            """
            refined_points = []
            print("enter add_interpolated_points")
            for i in range(len(points) - 1):
                p1 = points[i]
                p2 = points[i + 1]
                # p1 = utm_to_carla(points[i][0], points[i][1])
                # p2 = utm_to_carla(points[i + 1][0], points[i + 1][1])
                refined_points.append(p1)  # Add the current transformed point

                # Calculate the 2D distance between transformed points (x, y only)
                distance = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                if distance > offset:
                    # Add intermediate points
                    num_new_points = int(distance // offset)
                    for j in range(1, num_new_points + 1):
                        # Linear interpolation to find new points
                        new_x = p1[0] + j * (p2[0] - p1[0]) / (num_new_points + 1)
                        new_y = p1[1] + j * (p2[1] - p1[1]) / (num_new_points + 1)
                        refined_points.append((new_x, new_y))
            
            refined_points.append(points[-1])  # Add the last transformed point
            return refined_points

        try:
            construction_zone_info = self.redis_client.get(CONSTRUCTION_ZONE_COSIM_INFO)
            if not construction_zone_info:
                print("construction_zone_info is None or empty")
                return
        except Exception as e:
            print(f"Error fetching construction zone info: {e}")
            return
        
        print('entering construction zone')
        if construction_zone_info:
            closed_lane_shapes = construction_zone_info.closed_lane_shapes
            
            for closed_lane_shape in closed_lane_shapes:
                closed_lane_shape = add_interpolated_points(closed_lane_shape, 10)
                for cone_point in closed_lane_shape:
                    construction_cone = create_construction_zone_blueprint(self.world)
                    spawn_point = carla.Transform()
                    spawn_point.location.x, spawn_point.location.y = utm_to_carla(cone_point[0], cone_point[1])
                    spawn_point.location.z = self.get_z_offset(start_location=carla.Location(spawn_point.location.x, spawn_point.location.y, 300),end_location=carla.Location(spawn_point.location.x, spawn_point.location.y, 200))
                    id = spawn_actor(client=self.client, blueprint=construction_cone, transform=spawn_point)
                    print(f"created construction cone: {id}")

    def sync_cosim_participants_to_carla(self):
        """Update all participants in cosim to CARLA."""

        cosim_id_record = set()
        for key in self.cosim_controlled_participant_keys:
            try:
                cosim_controlled_participant_info = self.redis_client.get(key)
            except redis.exceptions.ConnectionError:
                print(key + " not found available. Exiting...")
                continue

        if cosim_controlled_participant_info:
            data = cosim_controlled_participant_info.data

        for id in data:
            if isPedestrian(id):
                self._process_pedestrian(id, data[id], cosim_id_record)
            elif isVehicle(id):
                self._process_vehicle(id, data[id], cosim_id_record)

        self._cleanup_actors("vehicle", "vehicle.*", cosim_id_record)
        self._cleanup_actors("pedestrian", "walker.pedestrian.*", cosim_id_record)


    def get_z_offset(self, start_location, end_location, previous_state=None):
        raycast_result = self.world.cast_ray(start_location, end_location)
        if not raycast_result:
            print("Ray did not hit the ground.")
            return 0

        if previous_state is None:
            # If previous_state is None, just return the minimum `z` value in raycast results.
            height = min((item.location.z for item in raycast_result), default=0)
        else:
            # Find the height closest to previous_state.
            height = min(
                (item.location.z for item in raycast_result if item.label == carla.CityObjectLabel.Roads),
                default=raycast_result[0].location.z,
                key=lambda z: abs(previous_state - z)
            )
        return height

    def _process_vehicle(self, id, veh_info, cosim_id_record):
        """Process a vehicle participant."""
        cosim_id_record.add(id)

        x, y = utm_to_carla(veh_info.x, veh_info.y)
        # z = get_carla_height(self.heights, x, y)
        start_location = carla.Location(x, y, veh_info.z + 5)
        end_location = carla.Location(x, y, veh_info.z - 5)
        yaw = -math.degrees(veh_info.orientation)

        vehicle_status, carla_id = get_actor_id_from_attribute(self.world, id)

        if not vehicle_status:
            blueprint = random.choice(
                self.motor_blueprints
                if veh_info.type == "DEFAULT_BIKETYPE"
                else self.vehicle_blueprints
            )
            blueprint.set_attribute("role_name", id)
            blueprint.set_attribute("color", "0, 102, 204")
            z = self.get_z_offset(start_location, end_location)
            transform = carla.Transform(
                carla.Location(x=x, y=y, z=z + 0.5), carla.Rotation(yaw=yaw)
            )
            carla_id = spawn_actor(self.client, blueprint, transform)
            print(f"Spawned vehicle in CARLA: {id}")
        else:
            vehicle = self.world.get_actor(carla_id)
            current_transform = vehicle.get_transform()
            z = self.get_z_offset(start_location, end_location, current_transform.location.z)
            transform = carla.Transform(
                carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw)
            )
            vehicle.set_transform(transform)

        if carla_id > 0:
            vehicle = self.world.get_actor(carla_id)
            # control = carla.VehicleControl()

            # Simulate movement
            # vehicle.apply_control(control)
            # pass

    def _process_pedestrian(self, id, vru_info, cosim_id_record):
        """Process a pedestrian participant."""
        cosim_id_record.add(id)

        x, y = utm_to_carla(vru_info.x, vru_info.y)
        # z = get_carla_height(self.heights, x, y)
        start_location = carla.Location(x, y, vru_info.z + 5)
        end_location = carla.Location(x, y, vru_info.z - 5)
        yaw = -math.degrees(vru_info.orientation)


        vru_status, carla_id = get_actor_id_from_attribute(self.world, id)

        if not vru_status:
            blueprint = random.choice(
                self.bike_blueprints
                if vru_info.type == "DEFAULT_BIKETYPE"
                else self.pedestrian_blueprints
            )
            blueprint.set_attribute("role_name", id)
            z = self.get_z_offset(start_location, end_location)
            transform = carla.Transform(
                carla.Location(x=x, y=y, z=z+1), carla.Rotation(yaw=yaw)
            )
            carla_id = spawn_actor(self.client, blueprint, transform)
            print(f"Spawned pedestrian in CARLA: {id}, ID: {carla_id}")
        else:
            # if vru_info.type != "DEFAULT_BIKETYPE":
                # transform.location.z += 1.0
            pedestrian = self.world.get_actor(carla_id)
            current_transform = pedestrian.get_transform()
            z = self.get_z_offset(start_location, end_location, current_transform.location.z)
            transform = carla.Transform(
                carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw)
            )

        if carla_id > 0:
            if vru_info.type != "DEFAULT_BIKETYPE":
                walker_control = carla.WalkerControl(
                    direction=carla.Vector3D(
                        vru_info.direction_x, vru_info.direction_y, 0
                    ),
                    speed=vru_info.speed_long,
                )
                self.world.get_actor(carla_id).apply_control(walker_control)
            else:
                control = carla.VehicleControl()
                self.world.get_actor(carla_id).apply_control(control)

    def _cleanup_actors(self, actor_type, pattern, cosim_id_record):
        """Clean up CARLA actors not in the cosim participant record."""
        actors_to_destroy = [
            actor
            for actor in self.world.get_actors().filter(pattern)
            if actor.attributes.get("role_name") not in cosim_id_record
            and actor.attributes.get("role_name") != "CAV"
        ]

        for actor in actors_to_destroy:
            actor.destroy()
            print(f"Destroyed {actor_type}: {actor.attributes.get('role_name')}")


def main():
    carla_cosim_plugin = CarlaCosimPlugin()
    step_length = 0.04

    settings = carla_cosim_plugin.world.get_settings()
    settings.fixed_delta_seconds = step_length
    settings.synchronous_mode = True
    carla_cosim_plugin.world.apply_settings(settings)

    carla_cosim_plugin.world.set_weather(carla.WeatherParameters.WetSunset)

    try:
        while True:
            start = time.time()
            carla_cosim_plugin.tick()
            end = time.time()
            elapsed = end - start
            if elapsed < step_length:
                time.sleep(step_length - elapsed)

    except KeyboardInterrupt:
        print("Cancelled by user.")

    finally:
        print("Cleaning synchronization")
        close(carla_cosim_plugin.world)


if __name__ == "__main__":
    main()
