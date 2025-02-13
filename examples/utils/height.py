import json
import carla
import numpy as np

def extract_data_from_json_file(file_path):
    """
    Read and extract data from a JSON file.

    :param file_path: Path to the JSON file
    :return: Extracted data (assuming the file contains a list of dictionaries)
    """
    with open(file_path, "r") as file:
        data = json.load(file)
    return data


def measure_data(x_start, x_end, y_start, y_end):
    # Connect to the CARLA server
    client = carla.Client("localhost", 2000)
    client.set_timeout(2.0)

    # Get the CARLA world
    world = client.get_world()

    height_data = {}
    num_total = (x_end - x_start) * (y_end - y_start)
    count = 0

    for x in range(x_start, x_end):
        for y in range(y_start, y_end):
            count += 1
            if count % 100 == 0:
                print("progress", round(count / num_total * 100, 2), "%")

            # Define the start location (above the point you want to measure)
            start_location = carla.Location(
                x, y, 300
            )  # Adjust Z value to start above the expected ground

            # Define the end location (below the expected ground)
            end_location = carla.Location(x, y, 0)  # Ensure this is below the ground

            # Perform ray casting
            raycast_result = world.cast_ray(start_location, end_location)

            # Check if the ray hit something (i.e., the ground)
            if raycast_result:
                height = raycast_result[0].location.z
                height_data[str((x, y))] = height
            else:
                print(f"Ray did not hit the ground at ({x}, {y}).")

    # Save the dictionary to a JSON file
    with open("heights.json", "w") as json_file:
        json.dump(height_data, json_file, indent=4)

    print(f"results saved to 'heights.json'")


def smooth_data(max_gap=5):
    height_data = extract_data_from_json_file("heights.json")
    height_data_smoothed = extract_data_from_json_file("heights.json")

    num_changed = 0

    for iter in range(10):
        for key, height in height_data_smoothed.items():
            point = key[1:-1]
            x, y = point.split(",")
            x, y = int(x), int(y)

            neighbor_1 = str((x + 1, y))
            neighbor_2 = str((x - 1, y))
            neighbor_3 = str((x, y + 1))
            neighbor_4 = str((x, y - 1))

            neighbors = [
                neighbor_1,
                neighbor_2,
                neighbor_3,
                neighbor_4,
            ]

            for neighbor in neighbors:
                updated = False
                if neighbor in height_data:
                    height_neighbor = height_data[neighbor]
                    if height - height_neighbor > 1.0 and not updated:
                        height_data_smoothed[key] = height_neighbor
                        num_changed += 1
                        updated = True

        print("num_changed", num_changed)
        num_changed = 0

        height_data = height_data_smoothed.copy()

    # Save the dictionary to a JSON file
    with open("heights_smoothed.json", "w") as json_file:
        json.dump(height_data_smoothed, json_file, indent=4)

    print(f"results saved to 'heights_smoothed.json'")


def new_measure_data(x_start, x_end, y_start, y_end):
    # Connect to the CARLA server
    client = carla.Client("localhost", 2000)
    client.set_timeout(2.0)

    # Get the CARLA world
    world = client.get_world()

    height_data = {}
    
    # Generate points with a precision of 0.5
    x_points = np.arange(x_start, x_end, 0.5)
    y_points = np.arange(y_start, y_end, 0.5)
    num_total = len(x_points) * len(y_points)
    count = 0

    sum_heights = 0
    valid_points = 0

    for x in x_points:
        for y in y_points:
            count += 1
            if count % 100 == 0:
                print("progress", round(count / num_total * 100, 2), "%")

            # Define the start location (above the point you want to measure)
            start_location = carla.Location(x, y, 300)  # Adjust Z value to start above the expected ground

            # Define the end location (below the expected ground)
            end_location = carla.Location(x, y, 0)  # Ensure this is below the ground

            # Perform ray casting
            raycast_result = world.cast_ray(start_location, end_location)

            # Check if the ray hit something (i.e., the ground)
            if raycast_result:
                height = raycast_result[0].location.z
                for item in raycast_result:
                    if item.label == carla.CityObjectLabel.Roads:
                        height = item.location.z
                        break
                location = carla.Location(x=x, y=y, z=height)
                waypoint = world.get_map().get_waypoint(
                    location, project_to_road=True, lane_type=carla.LaneType.Any
                )
                height_data[f"({x:.1f}, {y:.1f})"] = height
            else:
                print(f"Ray did not hit the ground at ({x:.1f}, {y:.1f}).")

    # Save the dictionary to a JSON file
    with open("test_heights.json", "w") as json_file:
        json.dump(height_data, json_file, indent=4)

    print(f"results saved to 'test_height.json'")

def compare_reversed_data(x_start, x_end, y_start, y_end):
    # Connect to the CARLA server
    client = carla.Client("localhost", 2000)
    client.set_timeout(2.0)

    # Get the CARLA world
    world = client.get_world()

    comparison_data = {}

    # Generate points with a precision of 0.5
    x_points = np.arange(x_start, x_end, 0.5)
    y_points = np.arange(y_start, y_end, 0.5)
    num_total = len(x_points) * len(y_points)
    count = 0

    for x in x_points:
        for y in y_points:
            count += 1
            if count % 100 == 0:
                print("progress", round(count / num_total * 100, 2), "%")

            # Define the original start and end locations
            start_location = carla.Location(x, y, 300)
            end_location = carla.Location(x, y, 0)

            # Perform ray casting with original order
            original_result = world.cast_ray(start_location, end_location)

            # Perform ray casting with reversed order
            reversed_result = world.cast_ray(end_location, start_location)

            original_height = -1
            reversed_height = -1

            # Extract height from original ray cast
            if original_result:
                original_height = round(original_result[0].location.z, 2)

            # Extract height from reversed ray cast
            if reversed_result:
                reversed_height = round(reversed_result[0].location.z, 2)

            # Compare and store results
            comparison_data[f"({x:.1f}, {y:.1f})"] = {
                "original_height": original_height,
                "reversed_height": reversed_height,
                "difference": original_height - reversed_height
            }

    # Save the comparison data to a JSON file
    with open("comparison_heights.json", "w") as json_file: 
        json.dump(comparison_data, json_file, indent=4)

    print(f"Comparison results saved to 'comparison_heights.json'")

def test():
    # Connect to the CARLA server
    client = carla.Client("localhost", 2000)
    client.set_timeout(2.0)
    x = 68.0
    y = -117.5
    # Get the CARLA world
    world = client.get_world()
    start_location = carla.Location(x, y, 300)
    end_location = carla.Location(x, y, 0)
    raycast_result = world.cast_ray(start_location, end_location)
    for item in raycast_result:
        print(item.label == carla.CityObjectLabel.Roads)


def main():
    # test()
    new_measure_data(x_start=-10, x_end=220, y_start=-240, y_end=170)
    # compare_reversed_data(x_start=-10, x_end=220, y_start=-240, y_end=170)
    # measure_data(x_start=-10, x_end=220, y_start=-240, y_end=170)
    # smooth_data(max_gap=5)


if __name__ == "__main__":
    main()
