import pandas as pd
import numpy as np

def lat_lon_to_xy(csv_file , origin_lat, origin_lon, output_file="carsim_coordinates.csv"):
    """
    Convert latitude/longitude coordinates from CSV to local x,y coordinates.
    
    Args:
        csv_file: Path to CSV file with 'lat' and 'lon' columns
        origin_lat: Reference latitude for origin (0, 0)
        origin_lon: Reference longitude for origin (0, 0)
        output_file: Optional path to save converted coordinates
    
    Returns:
        DataFrame with x, y columns
    """
    # Read CSV
    df = pd.read_csv(csv_file)
    
    # Earth's radius in meters
    R = 6371000
    
    # Convert to radians
    lat_rad = np.radians(df['latitude'])
    lon_rad = np.radians(df['longitude'])
    origin_lat_rad = np.radians(origin_lat)
    origin_lon_rad = np.radians(origin_lon)
    
    # Calculate local x, y using equirectangular approximation
    x = R * (lon_rad - origin_lon_rad) * np.cos(origin_lat_rad)
    y = R * (lat_rad - origin_lat_rad)
    
    # Create result DataFrame
    result = pd.DataFrame({
        'x': x,
        'y': y
    })
    
    # Save if output file specified
    if output_file:
        result.to_csv(output_file, index=False)
    
    return result

# Usage example:
df = lat_lon_to_xy('sem_apme_2025-track_coordinates.csv', origin_lat=51.5, origin_lon=-0.1, output_file='sem_apme_2025-track_local_coords.csv')