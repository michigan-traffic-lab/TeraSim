import json
import carla


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


def main():
    # measure_data(x_start=-10, x_end=220, y_start=-240, y_end=170)
    smooth_data(max_gap=5)


if __name__ == "__main__":
    main()
