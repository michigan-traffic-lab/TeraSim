import math


UTM_OFFSET = [-277600 + 102.89, -4686800 + 281.25, 0.0]


def utm_to_sumo_coordinate(utm_coordinate, offset):
    """
    Convert the UTM coordinate to the SUMO coordinate. the input will be a list of [x, y].
    """
    return [utm_coordinate[0] + offset[0], utm_coordinate[1] + offset[1]]


def utm_to_sumo_coordinate_withprojection(utm_coordinate, projection):
    """
    Convert the UTM coordinate to the SUMO coordinate. the input will be a list of [x, y].
    """
    raw_pose = projection["transformer_U2S"].transform(
        utm_coordinate[0],
        utm_coordinate[1],
    )
    return [
        raw_pose[0] + projection["SUMO_net_offset"][0],
        raw_pose[1] + projection["SUMO_net_offset"][1],
    ]


def sumo_to_utm_coordinate(sumo_coordinate, offset):
    """
    Convert the SUMO coordinate to the UTM coordinate. the input will be a list of [x, y].
    """
    return [sumo_coordinate[0] - offset[0], sumo_coordinate[1] - offset[1]]


def sumo_to_utm_coordinate_withprojection(sumo_coordinate, projection):
    """
    Convert the SUMO coordinate to the UTM coordinate. the input will be a list of [x, y].
    """
    x, y = sumo_coordinate
    raw_x = x - projection["SUMO_net_offset"][0]
    raw_y = y - projection["SUMO_net_offset"][1]
    utm_x, utm_y = projection["transformer_S2U"].transform(raw_x, raw_y)
    return [utm_x, utm_y]


def center_coordinate_to_sumo_coordinate(x, y, heading, length=5.0):
    """
    Convert the center coordinate to the SUMO coordinate. the input will be a list of {x, y, heading, vx}.
    """
    x = x + math.cos(heading) * 0.5 * length
    y = y + math.sin(heading) * 0.5 * length
    return x, y


def sumo_coordinate_to_center_coordinate(x, y, heading, length=5.0):
    """
    Convert the SUMO coordinate to the center coordinate. the input will be a list of {x, y, heading, vx}.
    """
    x = x - math.cos(heading) * 0.5 * length
    y = y - math.sin(heading) * 0.5 * length
    return x, y


def sumo_heading_to_orientation(sumo_heading):
    """
    Convert the SUMO heading to orientation.
    """
    radians = math.radians(90 - sumo_heading)
    return math.atan2(math.sin(radians), math.cos(radians))


def orientation_to_sumo_heading(orientation):
    """
    Convert the orientation to SUMO heading.
    """
    degrees = math.degrees(orientation)
    degrees = (degrees + 360) % 360
    degrees = (90 - degrees) % 360
    return degrees


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)

    return [qx, qy, qz, qw]


def cal_dist(p1, p2):
    x1, y1 = p1[0], p1[1]
    x2, y2 = p2[0], p2[1]
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def get_circle_center_list(traj_point):
    veh_length = 5.0
    veh_width = 2.0
    circle_r = 1.3
    tem_len = math.sqrt(circle_r**2 - (veh_width / 2) ** 2)

    center1 = (traj_point["x_lon"], traj_point["x_lat"])
    heading = traj_point["heading"]
    center0 = (
        center1[0] + (veh_length / 2 - tem_len) * math.cos(heading),
        center1[1] + (veh_length / 2 - tem_len) * math.sin(heading),
    )
    center2 = (
        center1[0] - (veh_length / 2 - tem_len) * math.cos(heading),
        center1[1] - (veh_length / 2 - tem_len) * math.sin(heading),
    )
    center_list = [center0, center1, center2]
    return center_list


def get_min_distance_between_two_veh_original(veh1, veh2):
    center_list_1 = get_circle_center_list(veh1)
    center_list_2 = get_circle_center_list(veh2)
    distance_list = []
    for p1 in center_list_1:
        for p2 in center_list_2:
            dist = cal_dist(p1, p2)
            distance_list.append(dist)
    return min(distance_list)


def get_min_dist(x1, y1, theta1, x2, y2, theta2):
    veh1 = {"x_lon": x1, "x_lat": y1, "heading": theta1}
    veh2 = {"x_lon": x2, "x_lat": y2, "heading": theta2}

    return get_min_distance_between_two_veh_original(veh1, veh2)
