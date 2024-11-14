def user_step(traci):
    """
    Description: executes user defined sumo commands.

    Args:
        traci (object): The traci object.

    Returns:
        bool: Should continue simulation flag.
    """

    # replace with your own implementation
    return user_step_example(traci)


def user_step_example(traci):
    """
    Some example traci functions to control the simulation.
    For more information, please refer to the traci documentation.
    https://sumo.dlr.de/docs/TraCI.html
    """

    vehicle_list = traci.vehicle.getIDList()
    print("Vehicle ID list: ", vehicle_list)

    for vehID in vehicle_list:
        heading = traci.vehicle.getAngle(vehID)
        location = traci.vehicle.getPosition3D(vehID)
        speed = traci.vehicle.getSpeed(vehID)
        acc = traci.vehicle.getAcceleration(vehID)

        edgeID = traci.vehicle.getRoadID(vehID)
        route = traci.vehicle.getRoute(vehID)

        length = traci.vehicle.getLength(vehID)
        width = traci.vehicle.getWidth(vehID)
        height = traci.vehicle.getHeight(vehID)

        print(
            f"Vehicle ID: {vehID}, "
            f"Heading: {heading:.2f} degrees, "
            f"Location: {location}, "
            f"EdgeID: {edgeID}, "
            f"Route: {route}, "
            f"Speed: {speed:.2f} m/s, "
            f"Acc: {acc:.2f} m/s², "
            f"Length: {length:.2f} m, "
            f"Width: {width:.2f} m, "
            f"Height: {height:.2f} m\n"
        )

    # Set the vehicle color to red
    traci.vehicle.setColor("CAV", (255, 0, 0, 255))

    # # Set the vehicle speed mode to 0 to force it to follow the desired speed exactly
    # traci.vehicle.setSpeedMode("CAV", 0)

    # traci.vehicle.setSpeed("CAV", 6.0)  # set the speed to 6 m/s
    # traci.vehicle.setSpeed("CAV", -1)  # reset the speed to the default value

    # # Command the vehicle to change lanes (if applicable)
    # traci.vehicle.changeLane("CAV", 1, duration=1.0)  # change to the right lane
    # traci.vehicle.changeLane("CAV", -1, duration=1.0)  # change to the left lane

    continue_flag = True
    return continue_flag
