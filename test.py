import csv
import math

def haversine_bearing(lat1, lon1, lat2, lon2):
    """Calculate initial bearing between two points"""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    
    bearing = math.atan2(x, y)
    return bearing

def offset_point(lat, lon, bearing, distance_m):
    """
    Offset a point by a distance in meters at a given bearing
    bearing in radians, distance in meters
    """
    R = 6371000  # Earth's radius in meters
    
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    
    lat2 = math.asin(math.sin(lat1) * math.cos(distance_m / R) +
                     math.cos(lat1) * math.sin(distance_m / R) * math.cos(bearing))
    
    lon2 = lon1 + math.atan2(math.sin(bearing) * math.sin(distance_m / R) * math.cos(lat1),
                             math.cos(distance_m / R) - math.sin(lat1) * math.sin(lat2))
    
    return math.degrees(lat2), math.degrees(lon2)

def create_track_bounds(input_csv, output_csv, road_width_m=10):
    """
    Create inner and outer bounds of a track from centerline coordinates
    
    Args:
        input_csv: Path to CSV with lat,lon columns
        output_csv: Path for output CSV
        road_width_m: Total road width in meters (default 10m)
    """
    # Read centerline points
    points = []
    with open(input_csv, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            lat = float(row['latitude'])
            lon = float(row['longitude'])
            points.append((lat, lon))
    
    if len(points) < 2:
        raise ValueError("Need at least 2 points to create bounds")
    
    offset_distance = road_width_m / 2
    
    # Calculate bounds for each point
    inner_bounds = []
    outer_bounds = []
    node_0 = []
    node_1 = []
    node_2 = []
    node_3 = []
    node_4 = []
    node_5 = []
    node_6 = []
    node_7 = []
    node_8 = []
    node_9 = []
    
    
    for i in range(len(points)):
        lat, lon = points[i]
        
        # Calculate bearing (tangent direction)
        if i == 0:
            # First point: use forward bearing
            bearing = haversine_bearing(lat, lon, points[i+1][0], points[i+1][1])
        elif i == len(points) - 1:
            # Last point: use backward bearing
            bearing = haversine_bearing(points[i-1][0], points[i-1][1], lat, lon)
        else:
            # Middle points: average of incoming and outgoing bearings
            bearing_in = haversine_bearing(points[i-1][0], points[i-1][1], lat, lon)
            bearing_out = haversine_bearing(lat, lon, points[i+1][0], points[i+1][1])
            bearing = (bearing_in + bearing_out) / 2
        
        # Calculate perpendicular bearings (left and right)
        left_bearing = bearing + math.pi / 2  # 90 degrees left
        right_bearing = bearing - math.pi / 2  # 90 degrees right
        
        # Create offset points
        left_lat, left_lon = offset_point(lat, lon, left_bearing, offset_distance)
        right_lat, right_lon = offset_point(lat, lon, right_bearing, offset_distance)
        node_0_lat, node_0_lon = offset_point(lat, lon, left_bearing, 5)
        node_1_lat, node_1_lon = offset_point(lat, lon, left_bearing, 4)
        node_2_lat, node_2_lon = offset_point(lat, lon, left_bearing, 3)
        node_3_lat, node_3_lon = offset_point(lat, lon, left_bearing, 2)
        node_4_lat, node_4_lon = offset_point(lat, lon, left_bearing, 1)
        node_5_lat, node_5_lon = offset_point(lat, lon, right_bearing, 1)
        node_6_lat, node_6_lon = offset_point(lat, lon, right_bearing, 2)
        node_7_lat, node_7_lon = offset_point(lat, lon, right_bearing, 3)
        node_8_lat, node_8_lon = offset_point(lat, lon, right_bearing, 4)
        node_9_lat, node_9_lon = offset_point(lat, lon, right_bearing, 5)
        
        inner_bounds.append((left_lat, left_lon))
        outer_bounds.append((right_lat, right_lon))
        node_0.append((node_0_lat, node_0_lon))
        node_1.append((node_1_lat, node_1_lon))
        node_2.append((node_2_lat, node_2_lon))
        node_3.append((node_3_lat, node_3_lon))
        node_4.append((node_4_lat, node_4_lon))
        node_5.append((node_5_lat, node_5_lon))
        node_6.append((node_6_lat, node_6_lon))
        node_7.append((node_7_lat, node_7_lon))
        node_8.append((node_8_lat, node_8_lon))
        node_9.append((node_9_lat, node_9_lon))


    # Write output CSV
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['type', 'latitude', 'longitude', 'left_latitude', 'left_longitude', 'right_latitude', 'right_longitude', 'node_0_latitude', 'node_0_longitude', 'node_1_latitude', 'node_1_longitude', 'node_2_latitude', 'node_2_longitude', 'node_3_latitude', 'node_3_longitude', 'node_4_latitude', 'node_4_longitude', 'node_5_latitude', 'node_5_longitude', 'node_6_latitude', 'node_6_longitude', 'node_7_latitude', 'node_7_longitude', 'node_8_latitude', 'node_8_longitude', 'node_9_latitude', 'node_9_longitude'])
        
        # Write centerline
        for i in range(len(points)):
            center_lat, center_lon = points[i]
            inner_lat, inner_lon = inner_bounds[i]
            outer_lat, outer_lon = outer_bounds[i]
            node_0_lat, node_0_lon = node_0[i]
            node_1_lat, node_1_lon = node_1[i]
            node_2_lat, node_2_lon = node_2[i]
            node_3_lat, node_3_lon = node_3[i]
            node_4_lat, node_4_lon = node_4[i]
            node_5_lat, node_5_lon = node_5[i]
            node_6_lat, node_6_lon = node_6[i]
            node_7_lat, node_7_lon = node_7[i]
            node_8_lat, node_8_lon = node_8[i]
            node_9_lat, node_9_lon = node_9[i]
            writer.writerow([i, center_lat, center_lon, inner_lat, inner_lon, outer_lat, outer_lon, node_0_lat, node_0_lon, node_1_lat, node_1_lon, node_2_lat, node_2_lon, node_3_lat, node_3_lon, node_4_lat, node_4_lon, node_5_lat, node_5_lon, node_6_lat, node_6_lon, node_7_lat, node_7_lon, node_8_lat, node_8_lon, node_9_lat, node_9_lon])
        # Write inner bound
        #for lat, lon in inner_bounds:
         #   writer.writerow(['inner', lat, lon])
        
        # Write outer bound
        #for lat, lon in outer_bounds:        #   writer.writerow(['outer', lat, lon])
    
    print(f"Created bounds with {len(points)} points per boundary")
    print(f"Output saved to: {output_csv}")

# Example usage
if __name__ == "__main__":
    # Adjust these parameters
    INPUT_FILE = "sem_apme_2025-track_coordinates.csv"  # Your input CSV
    OUTPUT_FILE = "track_with_bounds.csv"  # Output CSV
    ROAD_WIDTH = 12  # Total road width in meters
    
    create_track_bounds(INPUT_FILE, OUTPUT_FILE, ROAD_WIDTH)