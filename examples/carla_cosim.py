import time

import redis
import math
import carla
import random

from terasim_cosim.constants import *
from terasim_cosim.redis_client_wrapper import create_redis_client
from terasim_cosim.redis_msgs import Vehicle, VehicleDict

from utils.actor import *
from utils.geometry import *


class CarlaCosimPlugin(object):
    def __init__(
        self,
        control_cav=False,
    ):
        self.client = carla.Client("127.0.0.1", 2000)
        self.client.set_timeout(2.0)

        self.world = self.client.get_world()

        self.control_cav = control_cav

        self.vehicle_blueprints = create_vehicle_blueprint(self.world)
        self.pedestrian_blueprints = create_pedestrian_blueprint(self.world)

        height_path = "utils/heights_smoothed.json"
        self.heights = extract_data_from_json_file(height_path)

        key_value_config = {
            CAV_COSIM_VEHICLE_INFO: VehicleDict,
            TERASIM_COSIM_VEHICLE_INFO: VehicleDict,
        }

        self.cosim_vehicle_updated = False
        self.prev_cosim_veh_msg_timestamp = 0.0

        self.redis_client = create_redis_client(key_value_config=key_value_config)

    def tick(self):
        self.sync_cosim_vehicle_to_carla()

        if self.control_cav:
            self.sync_carla_cav_to_cosim()
        else:
            self.sync_cosim_cav_to_carla()

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
        if not self.cosim_vehicle_updated:
            return
        else:
            self.cosim_vehicle_updated = False

        try:
            cav_cosim_vehicle_info = self.redis_client.get(CAV_COSIM_VEHICLE_INFO)
        except:
            print("cav_cosim_vehicle_info not available. Exiting...")
            return

        if cav_cosim_vehicle_info:
            cav_info = cav_cosim_vehicle_info.data["CAV"]

            x, y = utm_to_carla(cav_info.x, cav_info.y)
            z = get_carla_height(self.heights, x, y)
            yaw = -math.degrees(cav_info.orientation)

            transform = carla.Transform()
            transform.location.x = x
            transform.location.y = y
            transform.location.z = z
            transform.rotation.yaw = yaw

            vehicle_status, carla_id = get_actor_id_from_attribute(self.world, "CAV")

            if not vehicle_status:
                blueprint_library = self.world.get_blueprint_library()
                print(blueprint_library.filter("vehicle.lincoln*"))
                blueprint = blueprint_library.find("vehicle.lincoln.mkz_2020")
                blueprint.set_attribute("color", "255, 0, 0")
                blueprint.set_attribute("role_name", "CAV")
                transform.location.z += 2.0
                carla_id = spawn_actor(self.client, blueprint, transform)
                print("Spawned CAV in CARLA simulation")
            else:
                CAV = self.world.get_actor(carla_id)
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

    def sync_cosim_vehicle_to_carla(self):
        try:
            cosim_controlled_vehicle_info = self.redis_client.get(
                TERASIM_COSIM_VEHICLE_INFO
            )
        except redis.exceptions.ConnectionError:
            print(TERASIM_COSIM_VEHICLE_INFO + " not found available. Exiting...")
            return

        cosim_vehicle_id_record = []

        if cosim_controlled_vehicle_info:
            cosim_veh_msg_timestamp = cosim_controlled_vehicle_info.header.timestamp
            if (cosim_veh_msg_timestamp - self.prev_cosim_veh_msg_timestamp) < 1e-3:
                return

            self.cosim_vehicle_updated = True
            self.prev_cosim_veh_msg_timestamp = cosim_veh_msg_timestamp

            data = cosim_controlled_vehicle_info.data

            # iterates over sumo actors and updates them in carla.
            for vehID in data:
                cosim_vehicle_id_record.append(vehID)
                veh_info = data[vehID]

                x, y = utm_to_carla(veh_info.x, veh_info.y)
                z = get_carla_height(self.heights, x, y)
                yaw = -math.degrees(veh_info.orientation)

                transform = carla.Transform()
                transform.location.x = x
                transform.location.y = y
                transform.location.z = z
                transform.rotation.yaw = yaw

                actor_status, carla_id = get_actor_id_from_attribute(self.world, vehID)

                # Creating new carla actor or updating existing one.
                if not actor_status:
                    if isVehicle(vehID):
                        blueprint = random.choice(self.vehicle_blueprints)
                        blueprint.set_attribute("role_name", vehID)
                        blueprint.set_attribute("color", "0, 102, 204")
                        transform.location.z += 2.0
                        carla_id = spawn_actor(self.client, blueprint, transform)
                        print("Spawned actor in CARLA: ", vehID)
                    elif isPedestrian(vehID):
                        blueprint = random.choice(self.pedestrian_blueprints)
                        blueprint.set_attribute("role_name", vehID)
                        transform.location.z += 2.0
                        carla_id = spawn_actor(self.client, blueprint, transform)
                        print("Spawned actor in CARLA: ", vehID)
                else:
                    actor = self.world.get_actor(carla_id)

                    if isVehicle(vehID):
                        transform.location.z += 0.0
                    if isPedestrian(vehID):
                        transform.location.z += 1.0

                    actor.set_transform(transform)

        carla_actors = list(self.world.get_actors().filter("vehicle.*")) + list(
            self.world.get_actors().filter("walker.pedestrian.*")
        )

        for actor in carla_actors:
            attribute = actor.attributes.get("role_name")
            if attribute not in cosim_vehicle_id_record and attribute != "CAV":
                actor.destroy()
                print("Destroy actor: ", attribute)


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
