import carla
import logging
import utm
import pyproj
import math


ZONE_NUMBER = 17
ZONE_LETTER = "T"
GNSS_ORIGIN = [42.3005934157, -83.699283188811]

TLS_NODES = {
    "NODE_11": [(83,), (92,), None, (88,), (89,), (86,), None, (84,)],
    "NODE_12": [(78,), (82,), (81,), (80,)],
    "NODE_17": [(61,), (62,), (67,), (68,), (65,), (66,), None, (63,), (64,)],
    "NODE_18": [(69, 70), None, (75,), None, (73, 74), None, (71,), None],
    "NODE_23": [(110,), None, (112,), (107,)],
    "NODE_24": [(47,), None, (48,), (52,)],
}


def create_vehicle_blueprint(world):
    blueprint_library = world.get_blueprint_library()

    car_keywords = [
        "vehicle.lincoln.mkz_2020",
        # "vehicle.audi",
        # "vehicle.bmw",
        # "vehicle.chevrolet",
        # "vehicle.citroen",
        # "vehicle.dodge",
        # "vehicle.mercedes",
        # "vehicle.nissan",
        # "vehicle.seat",
        # "vehicle.toyota",
        # "vehicle.tesla",
        # "vehicle.volkswagen",
    ]

    vehicle_blueprints = [
        bp
        for bp in blueprint_library.filter("vehicle.*")
        if any(keyword in bp.id for keyword in car_keywords)
    ]
    return vehicle_blueprints


def create_bike_blueprint(world):
    blueprint_library = world.get_blueprint_library()

    bike_keywords = [
        "vehicle.gazelle.omafiets",
        "vehicle.diamondback.century",
        "vehicle.bh.crossbike",
    ]

    bike_blueprints = [
        bp
        for bp in blueprint_library.filter("vehicle.*")
        if any(keyword in bp.id for keyword in bike_keywords)
    ]

    return bike_blueprints


def create_pedestrian_blueprint(world):
    blueprint_library = world.get_blueprint_library()

    pedestrian_blueprints = blueprint_library.filter("walker.pedestrian.*")

    return pedestrian_blueprints


def create_motor_blueprint(world):
    blueprint_library = world.get_blueprint_library()
    motor_keywords = [
        "vehicle.yamaha.yzf",
        "vehicle.vespa.zx125",
        "vehicle.kawasaki.ninja",
        "vehicle.harley-davidson.low_rider",
    ]
    motor_blueprints = [
        bp
        for bp in blueprint_library.filter("vehicle.*")
        if any(keyword in bp.id for keyword in motor_keywords)
    ]
    return motor_blueprints


def create_construction_zone_blueprint(world):
    blueprint_library = world.get_blueprint_library()
    construction_zone_blueprint = blueprint_library.find("static.prop.trafficcone01")
    return construction_zone_blueprint


def isVehicle(actorID):
    return "BV" in actorID or "CAV" in actorID


def isPedestrian(actorID):
    return "VRU" in actorID or "PED" in actorID


def spawn_actor(client, blueprint, transform):
    """
    Spawns a new actor.

        :param blueprint: blueprint of the actor to be spawned.
        :param transform: transform where the actor will be spawned.
        :return: actor id if the actor is successfully spawned. Otherwise, INVALID_carla_id.
    """

    batch = [
        carla.command.SpawnActor(blueprint, transform).then(
            carla.command.SetSimulatePhysics(carla.command.FutureActor, False)
        )
    ]
    response = client.apply_batch_sync(batch, True)[0]
    if response.error:
        logging.error("Spawn carla actor failed. %s", response.error)
        return -1

    return response.actor_id


def destroy_all_actors(world):
    carla_actors = (
        list(world.get_actors().filter("vehicle.*"))
        + list(world.get_actors().filter("walker.pedestrian.*"))
        + list(world.get_actors().filter("static.prop.constructioncone"))
        + list(world.get_actors().filter("static.prop.trafficcone01"))
    )

    for actor in carla_actors:
        actor.destroy()
        print("Destroy actor: ", actor.id)


def get_actor_id_from_attribute(world, attribute):
    actor_list = world.get_actors()
    for actor in actor_list:
        if actor.attributes.get("role_name") == attribute:
            return True, actor.id

    return False, -1


def get_z_offset(world, start_location, end_location, previous_state=None):
    raycast_result = world.cast_ray(start_location, end_location)
    if not raycast_result:
        print("Ray did not hit the ground.")
        return 0

    if previous_state is None:
        # If previous_state is None, just return the minimum `z` value in raycast results.
        height = min((item.location.z for item in raycast_result), default=0)
    else:
        roads_sidewalks = [
            item.location.z
            for item in raycast_result
            if item.label
            in (carla.CityObjectLabel.Roads, carla.CityObjectLabel.Sidewalks)
        ]
        ground_hits = [
            item.location.z
            for item in raycast_result
            if item.label == carla.CityObjectLabel.Ground
        ]

        # First priority: roads/sidewalks if they exist
        if roads_sidewalks:
            height = min(roads_sidewalks, key=lambda z: abs(previous_state - z))
        # If no roads/sidewalks found, check ground
        elif ground_hits:
            height = min(ground_hits, key=lambda z: abs(previous_state - z))
        # If neither roads/sidewalks nor ground labels are present, use whatever we got
        else:
            # You might want to log or print out the labels if this case happens frequently
            height = min(
                (item.location.z for item in raycast_result),
                default=raycast_result[0].location.z,
                key=lambda z: abs(previous_state - z),
            )
    return height


# Define the function to draw text in the simulation
def draw_text(world, location, text, color=(255, 0, 0), life_time=0.05):
    debug = world.debug
    debug.draw_string(
        location,
        text,
        draw_shadow=False,
        color=carla.Color(r=color[0], g=color[1], b=color[2]),
        life_time=life_time,
        persistent_lines=True,
    )


# Define the function to draw text in the simulation
def draw_point(world, location, size=0.1, color=(255, 0, 0), life_time=0.05):
    debug = world.debug
    debug.draw_point(
        location,
        size,
        color=carla.Color(r=color[0], g=color[1], b=color[2]),
        life_time=life_time,
    )


def update_spectator_camera(self, vehicle_transform, follow_distance=10):
    spectator = self.world.get_spectator()
    vehicle_location = vehicle_transform.location
    vehicle_rotation = vehicle_transform.rotation

    # Calculate the position of the camera relative to the vehicle
    camera_location = carla.Location(
        x=vehicle_location.x
        - follow_distance * math.cos(math.radians(vehicle_rotation.yaw)),
        y=vehicle_location.y
        - follow_distance * math.sin(math.radians(vehicle_rotation.yaw)),
        z=vehicle_location.z
        + 5,  # Adjust this value to change the height of the camera
    )
    camera_rotation = carla.Rotation(pitch=-30, yaw=vehicle_rotation.yaw)

    # Set the spectator camera transform
    spectator.set_transform(carla.Transform(camera_location, camera_rotation))


def close(world):
    """
    Cleans synchronization and resets the simulation settings.
    """
    # Configuring carla simulation in async mode.
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)

    destroy_all_actors(world)


def latlon_to_xy(lat_center, lon_center, lat_point, lon_point):
    # Define a Transverse Mercator projection with the center point as the origin
    projection = pyproj.Proj(
        proj="tmerc", lat_0=lat_center, lon_0=lon_center, ellps="WGS84"
    )

    # Convert the point lat/lon to x, y coordinates relative to the center point
    x, y = projection(lon_point, lat_point)

    return x, y


def xy_to_latlon(lat_center, lon_center, x, y):
    # Define the same Transverse Mercator projection with the center point as the origin
    projection = pyproj.Proj(
        proj="tmerc", lat_0=lat_center, lon_0=lon_center, ellps="WGS84"
    )

    # Convert x, y coordinates back to lat/lon using the inverse of the projection
    lon_point, lat_point = projection(x, y, inverse=True)

    return lat_point, lon_point


def utm_to_carla(utm_x, utm_y):
    lat, lon = utm.to_latlon(utm_x, utm_y, ZONE_NUMBER, ZONE_LETTER)
    local_x, local_y = latlon_to_xy(GNSS_ORIGIN[0], GNSS_ORIGIN[1], lat, lon)

    return local_x, -local_y


def carla_to_utm(x, y):
    lat, lon = xy_to_latlon(GNSS_ORIGIN[0], GNSS_ORIGIN[1], x, -y)
    utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)

    return utm_x, utm_y
