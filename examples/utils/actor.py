import carla
import logging


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


def create_pedestrian_blueprint(world):
    blueprint_library = world.get_blueprint_library()

    pedestrian_keywords = ["walker.pedestrian.0002"]

    pedestrian_blueprints = [
        bp
        for bp in blueprint_library.filter("walker.*")
        if any(keyword in bp.id for keyword in pedestrian_keywords)
    ]

    return pedestrian_blueprints


def isVehicle(actorID):
    return "BV" in actorID or "CAV" in actorID or "POV" in actorID or "VUT" in actorID


def isPedestrian(actorID):
    return "VRU" in actorID


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
    response = client.apply_batch_sync(batch, False)[0]
    if response.error:
        logging.error("Spawn carla actor failed. %s", response.error)
        return -1

    return response.actor_id


def destroy_all_actors(world):
    carla_actors = list(world.get_actors().filter("vehicle.*")) + list(
        world.get_actors().filter("walker.pedestrian.*")
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
