from terasim_cosim.terasim_plugin.utils import (
    send_user_av_control_wrapper,
    send_user_av_planning_wrapper,
)


def user_step(traci):
    """
    Description: executes user defined sumo commands.

    Args:
        traci (object): The traci object.

    Returns:
        bool: Should continue simulation flag.
    """

    # get the vehicle information
    vehicle_info = get_vehicle_info(traci)
    # print(vehicle_info)

    # get the traffic light information
    tls_info = get_tls_info(traci)
    # print(tls_info)

    # send control or planning commands (only one should be active)
    # send_av_control(vehicle_info)
    # send_av_planning(vehicle_info)

    # IMPORANT
    # update BVs through traci, only use this function for plain terasim simulation
    # trying to customize the BVs in NDE simulation may lead to simulation crash
    update_bv_action(traci, vehicle_info)

    continue_flag = True
    return continue_flag


def get_vehicle_info(traci):
    """
    Description: Get the information of all vehicles in the simulation.
    """

    vehicle_info = {}
    vehicle_ids = traci.vehicle.getIDList()

    for vehID in vehicle_ids:
        vehicle = {}

        vehicle["heading"] = traci.vehicle.getAngle(vehID)
        vehicle["location"] = traci.vehicle.getPosition3D(vehID)
        vehicle["speed"] = traci.vehicle.getSpeed(vehID)
        vehicle["acc"] = traci.vehicle.getAcceleration(vehID)

        vehicle["edgeID"] = traci.vehicle.getRoadID(vehID)
        vehicle["route"] = traci.vehicle.getRoute(vehID)

        vehicle["length"] = traci.vehicle.getLength(vehID)
        vehicle["width"] = traci.vehicle.getWidth(vehID)
        vehicle["height"] = traci.vehicle.getHeight(vehID)

        vehicle_info[vehID] = vehicle

    return vehicle_info


def get_tls_info(traci):
    # Get all traffic light IDs
    tls_ids = traci.trafficlight.getIDList()

    # Create a dictionary to store the status of each traffic light
    tls_info = {}

    # Iterate through each traffic light and get its status
    for tl_id in tls_ids:
        # Get the current state of the traffic light (e.g., "rrGGgrr")
        tls_state = traci.trafficlight.getRedYellowGreenState(tl_id)
        tls_info[tl_id] = tls_state

    return tls_info


def update_bv_action(traci, vehicle_info):
    """
    Some example traci functions to control the simulation.
    For more information, please refer to the traci documentation.
    https://sumo.dlr.de/docs/TraCI.html
    """

    for vehID, veh_info in vehicle_info.items():
        if vehID != "CAV":
            traci.vehicle.setColor(vehID, (255, 255, 0, 255))


def send_av_control(vehicle_info):
    """
    Description: Send low-level control commands to control the AV.
    """

    brake_cmd = 0.2  # [0.0, 1.0]0
    throttle_cmd = 0.0  # [0.0, 1.0]
    steering_cmd = 0.0  # [-1.0, 1.0] for CARLA, [-2.5pi, 2.5pi] for real AV

    # PARK       =1
    # REVERSE    =2
    # NEUTRAL    =3
    # DRIVE      =4
    gear_cmd = 1

    user_msg = "your message"

    send_user_av_control_wrapper(
        brake_cmd,
        throttle_cmd,
        steering_cmd,
        gear_cmd,
        user_msg=user_msg,
    )


def send_av_planning(vehicle_info):
    """
    Description: Send high-level planning commands to control the AV.
    All coordinates follow the mcity sumo map.

    For best control performance, the resolution between each waypoint should be 0.04 seconds.
    Users should send a minimum of 50 waypoints (2 seconds) and a maximum of 125 waypoints (5 seconds).
    """

    x_list = [0.0] * 20  # [m]
    y_list = [0.0] * 20  # [m]
    speed_list = [0.0] * 20  # [m/s]
    orientation_list = [0.0] * 20  # [degrees]

    user_msg = "your message"

    send_user_av_planning_wrapper(
        x_list=x_list,
        y_list=y_list,
        speed_list=speed_list,
        orientation_list=orientation_list,
        user_msg=user_msg,
    )
