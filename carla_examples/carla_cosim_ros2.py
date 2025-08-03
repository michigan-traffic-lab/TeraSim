import time
import redis
import math
import carla
import random
from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client
from terasim_cosim.redis_msgs import (
    Actor,
    ActorDict,
    ConstructionZone,
    SUMOSignalDict,
    VehicleState,
)
from utils import *


class CarlaCosimPlugin(object):
    def __init__(
        self,
        cosim_controlled_actor_keys=[TERASIM_ACTOR_INFO],
    ):
        self.client = carla.Client("127.0.0.1", 2000)
        self.client.set_timeout(2.0)

        self.world = self.client.get_world()

        self.traffic_lights = self.world.get_actors().filter("traffic.traffic_light")
        for traffic_light in self.traffic_lights:
            traffic_light.set_state(carla.TrafficLightState.Off)
            traffic_light.freeze(True)

        self.cosim_controlled_actor_keys = cosim_controlled_actor_keys

        self.vehicle_blueprints = create_vehicle_blueprint(self.world)
        self.motor_blueprints = create_motor_blueprint(self.world)
        self.pedestrian_blueprints = create_pedestrian_blueprint(self.world)
        self.bike_blueprints = create_bike_blueprint(self.world)

        key_value_config = {
            CAV_INFO: ActorDict,
            VEHICLE_STATE: VehicleState,
            TERASIM_ACTOR_INFO: ActorDict,
            CONSTRUCTION_ZONE_INFO: ConstructionZone,
            TLS_INFO: SUMOSignalDict,
        }

        self.redis_client = create_redis_client(key_value_config=key_value_config)
        self.sync_cosim_construction_zone_to_carla()

    def tick(self):
        self.sync_cosim_actor_to_carla()
        self.sync_cosim_tls_to_carla()

        self.world.tick()

    def sync_cosim_tls_to_carla(self):
        try:
            cosim_tls_info = self.redis_client.get(TLS_INFO)
        except:
            print("cosim_tls_info not available. Exiting...")
            return

        if cosim_tls_info:
            cosim_tls_data = cosim_tls_info.data

            for node in cosim_tls_data:
                node_tls = cosim_tls_data[node]
                carla_tls_node = TLS_NODES[node]

                for i in range(len(carla_tls_node)):
                    light_ids = carla_tls_node[i]

                    if light_ids:
                        for light_id in light_ids:
                            light_actor = self.world.get_actor(light_id)
                            light_state = node_tls.tls[i]

                            if light_state == "G" or light_state == "g":
                                light_actor.set_state(carla.TrafficLightState.Green)
                            elif light_state == "Y" or light_state == "y":
                                light_actor.set_state(carla.TrafficLightState.Yellow)
                            elif light_state == "R" or light_state == "r":
                                light_actor.set_state(carla.TrafficLightState.Red)

    def sync_cosim_actor_to_carla(self):
        """Update all actors in cosim to CARLA."""

        cosim_id_record = set()
        for key in self.cosim_controlled_actor_keys:
            try:
                cosim_controlled_actor_info = self.redis_client.get(key)
            except redis.exceptions.ConnectionError:
                print(key + " not found available. Exiting...")
                continue

            if cosim_controlled_actor_info:
                data = cosim_controlled_actor_info.data

                for id in data:
                    if isPedestrian(id):
                        self._process_pedestrian(id, data[id], cosim_id_record)
                    elif isVehicle(id):
                        self._process_vehicle(id, data[id], cosim_id_record)

            self._cleanup_actors("vehicle", "vehicle.*", cosim_id_record)
            self._cleanup_actors("pedestrian", "walker.pedestrian.*", cosim_id_record)

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

                refined_points.append(p1)  # Add the current transformed point

                # Calculate the 2D distance between transformed points (x, y only)
                distance = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
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
            construction_zone_info = self.redis_client.get(CONSTRUCTION_ZONE_INFO)
            if not construction_zone_info:
                print("construction_zone_info is None or empty")
                return
        except Exception as e:
            print(f"Error fetching construction zone info: {e}")
            return

        print("entering construction zone")
        if construction_zone_info:
            closed_lane_shapes = construction_zone_info.closed_lane_shapes

            for closed_lane_shape in closed_lane_shapes:
                closed_lane_shape = add_interpolated_points(closed_lane_shape, 10)
                for cone_point in closed_lane_shape:
                    construction_cone = create_construction_zone_blueprint(self.world)
                    spawn_point = carla.Transform()
                    spawn_point.location.x, spawn_point.location.y = utm_to_carla(
                        cone_point[0], cone_point[1]
                    )
                    spawn_point.location.z = get_z_offset(
                        self.world,
                        start_location=carla.Location(
                            spawn_point.location.x, spawn_point.location.y, 300
                        ),
                        end_location=carla.Location(
                            spawn_point.location.x, spawn_point.location.y, 200
                        ),
                    )
                    id = spawn_actor(
                        client=self.client,
                        blueprint=construction_cone,
                        transform=spawn_point,
                    )
                    print(f"created construction cone: {id}")

    def _process_vehicle(self, id, veh_info, cosim_id_record):
        """Process a vehicle actor."""
        cosim_id_record.add(id)

        x, y = utm_to_carla(veh_info.x, veh_info.y)
        start_location = carla.Location(x, y, 300)
        end_location = carla.Location(x, y, 200)
        yaw = -math.degrees(veh_info.orientation)

        vehicle_status, carla_id = get_actor_id_from_attribute(self.world, id)

        if not vehicle_status:
            blueprint = random.choice(
                self.motor_blueprints
                if veh_info.type == "DEFAULT_BIKETYPE"
                else self.vehicle_blueprints
            )
            blueprint.set_attribute("role_name", id)

            if "CAV" in id:
                blueprint.set_attribute("color", "255, 0, 0")
            else:
                blueprint.set_attribute("color", "0, 102, 204")

            z = get_z_offset(self.world, start_location, end_location)
            transform = carla.Transform(
                carla.Location(x=x, y=y, z=z + 0.5), carla.Rotation(yaw=yaw)
            )
            carla_id = spawn_actor(self.client, blueprint, transform)
            print(f"Spawned vehicle in CARLA: {id}")
        else:
            vehicle = self.world.get_actor(carla_id)
            current_transform = vehicle.get_transform()
            z = get_z_offset(
                self.world, start_location, end_location, current_transform.location.z
            )
            transform = carla.Transform(
                carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw)
            )
            vehicle.set_transform(transform)

    def _process_pedestrian(self, id, vru_info, cosim_id_record):
        """Process a pedestrian actor."""
        cosim_id_record.add(id)

        x, y = utm_to_carla(vru_info.x, vru_info.y)
        start_location = carla.Location(x, y, 300)
        end_location = carla.Location(x, y, 200)
        yaw = -math.degrees(vru_info.orientation)

        vru_status, carla_id = get_actor_id_from_attribute(self.world, id)

        if not vru_status:
            blueprint = random.choice(
                self.bike_blueprints
                if vru_info.type == "DEFAULT_BIKETYPE"
                else self.pedestrian_blueprints
            )
            blueprint.set_attribute("role_name", id)
            z = get_z_offset(self.world, start_location, end_location)
            transform = carla.Transform(
                carla.Location(x=x, y=y, z=z + 1), carla.Rotation(yaw=yaw)
            )
            carla_id = spawn_actor(self.client, blueprint, transform)
            print(f"Spawned pedestrian in CARLA: {id}, ID: {carla_id}")
        else:
            pedestrian = self.world.get_actor(carla_id)
            current_transform = pedestrian.get_transform()
            z = get_z_offset(
                self.world, start_location, end_location, current_transform.location.z
            )
            if vru_info.type != "DEFAULT_BIKETYPE":
                z += 1
            transform = carla.Transform(
                carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw)
            )
            pedestrian.set_transform(transform)

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
        """Clean up CARLA actors not in the cosim actor record."""
        actors_to_destroy = [
            actor
            for actor in self.world.get_actors().filter(pattern)
            if actor.attributes.get("role_name") not in cosim_id_record
            and "CAV" not in actor.attributes.get("role_name")
        ]

        for actor in actors_to_destroy:
            actor.destroy()
            print(f"Destroyed {actor_type}: {actor.attributes.get('role_name')}")


def main():
    carla_cosim_plugin = CarlaCosimPlugin()
    step_length = 0.02

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
