import utm
import json
import pyproj


zone_number = 17
zone_letter = "T"
GNSS_origin = [42.3005934157, -83.699283188811]


def extract_data_from_json_file(file_path):
    """
    Read and extract data from a JSON file.
    
    :param file_path: Path to the JSON file
    :return: Extracted data (assuming the file contains a list of dictionaries)
    """
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

def get_carla_height(heights, x, y):
    x_int = int(x)
    y_int = int(y)

    key = str((x_int, y_int))

    if key in heights:
        return heights[key]
    else:
        print((x_int, y_int), "height not available, using default height")
        return 280.0

def latlon_to_xy(lat_center, lon_center, lat_point, lon_point):
    # Define a Transverse Mercator projection with the center point as the origin
    projection = pyproj.Proj(proj='tmerc', lat_0=lat_center, lon_0=lon_center, ellps='WGS84')
    
    # Convert the point lat/lon to x, y coordinates relative to the center point
    x, y = projection(lon_point, lat_point)
    
    return x, y

def xy_to_latlon(lat_center, lon_center, x, y):
    # Define the same Transverse Mercator projection with the center point as the origin
    projection = pyproj.Proj(proj='tmerc', lat_0=lat_center, lon_0=lon_center, ellps='WGS84')
    
    # Convert x, y coordinates back to lat/lon using the inverse of the projection
    lon_point, lat_point = projection(x, y, inverse=True)
    
    return lat_point, lon_point

def utm_to_carla(utm_x, utm_y):
    lat, lon = utm.to_latlon(utm_x, utm_y, zone_number, zone_letter)
    local_x, local_y = latlon_to_xy(GNSS_origin[0], GNSS_origin[1], lat, lon)

    return local_x, -local_y

def carla_to_utm(x, y):
    lat, lon = xy_to_latlon(GNSS_origin[0], GNSS_origin[1], x, -y)
    utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)

    return utm_x, utm_y
