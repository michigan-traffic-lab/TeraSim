import carla
import json
import numpy as np

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

            original_height = None
            reversed_height = None

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