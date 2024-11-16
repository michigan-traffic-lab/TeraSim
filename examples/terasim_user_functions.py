from terasim_cosim.terasim_plugin.utils import send_user_av_control_wrapper


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
    print(vehicle_info)

    # update the BV action through traci
    update_bv_action(vehicle_info, traci)

    # send raw control commands to the AV
    send_av_control(vehicle_info)

    continue_flag = True
    return continue_flag


def get_vehicle_info(traci, vehicle_info):
    """
    Some example traci functions to control the simulation.
    For more information, please refer to the traci documentation.
    https://sumo.dlr.de/docs/TraCI.html
    """

    vehicle_list = traci.vehicle.getIDList()
    print("Vehicle ID list: ", vehicle_list)

    vehicles = {}

    for vehID in vehicle_list:
        vehicle = {}

        vehicle["vehID"] = vehID

        vehicle["heading"] = traci.vehicle.getAngle(vehID)
        vehicle["location"] = traci.vehicle.getPosition3D(vehID)
        vehicle["speed"] = traci.vehicle.getSpeed(vehID)
        vehicle["acc"] = traci.vehicle.getAcceleration(vehID)

        vehicle["edgeID"] = traci.vehicle.getRoadID(vehID)
        vehicle["route"] = traci.vehicle.getRoute(vehID)

        vehicle["length"] = traci.vehicle.getLength(vehID)
        vehicle["width"] = traci.vehicle.getWidth(vehID)
        vehicle["height"] = traci.vehicle.getHeight(vehID)

        vehicles[vehID] = vehicle

    return vehicles


def update_bv_action(traci, vehicle_info):
    """
    Some example traci functions to control the simulation.
    For more information, please refer to the traci documentation.
    https://sumo.dlr.de/docs/TraCI.html
    """

    traci.vehicle.setColor("CAV", (255, 0, 0, 255))

    for vehicle in vehicle_info:
        vehID = vehicle["vehID"]
        traci.vehicle.setColor(vehID, (0, 255, 255, 255))


def send_av_control(traci):
    """
    Description: Send the control commands to control the AV.
    """

    brake_cmd = 0.2  # [0.0, 1.0]
    throttle_cmd = 0.0  # [0.0, 1.0]
    steering_cmd = 0.0  # [-2.5pi, 2.5pi]

    # NONE   	 =0
    # PARK       =1
    # REVERSE    =2
    # NEUTRAL    =3
    # DRIVE      =4
    # LOW        =5
    gear_cmd = 1

    send_user_av_control_wrapper(
        brake_cmd,
        throttle_cmd,
        steering_cmd,
        gear_cmd,
        user_msg="your message",
    )
