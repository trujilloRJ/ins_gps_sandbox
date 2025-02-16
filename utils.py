import numpy as np
from datetime import datetime

def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts Euler angles (roll, pitch, yaw) into a quaternion.
    
    Args:
        roll (float): Rotation around X-axis in radians (Roll).
        pitch (float): Rotation around Y-axis in radians (Pitch).
        yaw (float): Rotation around Z-axis in radians (Yaw).
        
    Returns:
        np.array: Quaternion (q_w, q_x, q_y, q_z).
    """
    # Half angles
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    # Quaternion components
    q_w = cr * cp * cy + sr * sp * sy
    q_x = sr * cp * cy - cr * sp * sy
    q_y = cr * sp * cy + sr * cp * sy
    q_z = cr * cp * sy - sr * sp * cy
    
    return np.array([q_w, q_x, q_y, q_z])


def quaternion_to_euler(q):
    """
    Converts a quaternion (q_w, q_x, q_y, q_z) to Euler angles (roll, pitch, yaw).
    
    Args:
        q (np.array): Quaternion [q_w, q_x, q_y, q_z].
        
    Returns:
        tuple: Euler angles (roll, pitch, yaw) in radians.
    """
    q_w, q_x, q_y, q_z = q

    roll = np.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))
    pitch = np.arcsin(2 * (q_w * q_y - q_z * q_x))
    yaw = np.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))

    return roll, pitch, yaw

def lat_lon_to_ecef_with_height(lat, lon, height):
    """
    Convert latitude, longitude, and height (above the geoid) to ECEF (Earth-Centered, Earth-Fixed) coordinates.
    
    Args:
        lat (float): Latitude in degrees.
        lon (float): Longitude in degrees.
        height (float): Height above the geoid in meters.
        
    Returns:
        tuple: ECEF coordinates (X, Y, Z) in meters.
    """
    # WGS84 parameters
    a = 6378137.0  # Semi-major axis in meters
    f = 1 / 298.257223563  # Flattening
    e2 = 2 * f - f**2  # Eccentricity squared
    
    # Convert latitude and longitude from degrees to radians
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    
    # Compute prime vertical radius of curvature
    N = a / np.sqrt(1 - e2 * np.sin(lat_rad)**2)
    
    # Compute ECEF coordinates using height as altitude
    X = (N + height) * np.cos(lat_rad) * np.cos(lon_rad)
    Y = (N + height) * np.cos(lat_rad) * np.sin(lon_rad)
    Z = ((1 - e2) * N + height) * np.sin(lat_rad)
    
    return X, Y, Z

def time_difference(timestamp1, timestamp2):
    """
    Computes the time difference in seconds between two timestamps.

    Args:
        timestamp1 (str): First timestamp in the format "YYYY-MM-DD HH:MM:SS.sssssssss".
        timestamp2 (str): Second timestamp in the format "YYYY-MM-DD HH:MM:SS.sssssssss".

    Returns:
        float: Time difference in seconds.
    """
    # Define the datetime format
    time_format = "%Y-%m-%d %H:%M:%S.%f"

    # Convert strings to datetime objects
    dt1 = datetime.strptime(timestamp1[:26], time_format)  # Trim nanoseconds to microseconds
    dt2 = datetime.strptime(timestamp2[:26], time_format)  # Trim nanoseconds to microseconds

    # Compute the time difference in seconds
    return (dt2 - dt1).total_seconds()