import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.animation import FuncAnimation
import csv
from scipy.interpolate import interp1d

class BicycleModel:
    """
    Bicycle model for Shell Eco Marathon vehicle dynamics
    Track width: 101 cm (1.01 m)
    """
    
    def __init__(self, L=1.5, track_width=1.01, dt=0.01):
        """
        Parameters:
        L: wheelbase (m) - distance between front and rear axles
        track_width: track width (m) - 101 cm for SEM vehicle
        dt: time step (s)
        """
        self.L = L  # wheelbase
        self.track_width = track_width
        self.dt = dt
        
        # State variables: [x, y, theta, v]
        # x, y: position (m)
        # theta: heading angle (rad)
        # v: velocity (m/s)
        self.state = np.array([0.0, 0.0, 0.0, 0.0])
        
        # History for plotting
        self.history = {'x': [], 'y': [], 'theta': [], 'v': [], 't': []}
        self.time = 0.0
        
    def update(self, a, delta):
        """
        Update vehicle state using bicycle model
        
        Parameters:
        a: acceleration (m/s^2)
        delta: steering angle at front wheel (rad)
        """
        x, y, theta, v = self.state
        
        # Bicycle model kinematics
        x_dot = v * np.cos(theta)
        y_dot = v * np.sin(theta)
        theta_dot = (v / self.L) * np.tan(delta)
        v_dot = a
        
        # Update state (Euler integration)
        self.state[0] += x_dot * self.dt  # x
        self.state[1] += y_dot * self.dt  # y
        self.state[2] += theta_dot * self.dt  # theta
        self.state[3] += v_dot * self.dt  # v
        
        # Keep heading angle in [-pi, pi]
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))
        
        # Store history
        self.history['x'].append(self.state[0])
        self.history['y'].append(self.state[1])
        self.history['theta'].append(self.state[2])
        self.history['v'].append(self.state[3])
        self.history['t'].append(self.time)
        
        self.time += self.dt
        
        return self.state.copy()
    
    def get_wheel_positions(self):
        """Calculate positions of all four wheels"""
        x, y, theta, v = self.state
        
        # Rear axle center (reference point)
        rear_x, rear_y = x, y
        
        # Front axle center
        front_x = x + self.L * np.cos(theta)
        front_y = y + self.L * np.sin(theta)
        
        # Half track width
        half_track = self.track_width / 2
        
        # Perpendicular direction (left side of vehicle)
        perp_x = -np.sin(theta)
        perp_y = np.cos(theta)
        
        # Four wheel positions
        wheels = {
            'rear_left': (rear_x + half_track * perp_x, rear_y + half_track * perp_y),
            'rear_right': (rear_x - half_track * perp_x, rear_y - half_track * perp_y),
            'front_left': (front_x + half_track * perp_x, front_y + half_track * perp_y),
            'front_right': (front_x - half_track * perp_x, front_y - half_track * perp_y)
        }
        
        return wheels
    
    def calculate_turning_radius(self, delta):
        """Calculate turning radius for given steering angle"""
        if abs(delta) < 1e-6:
            return np.inf
        return self.L / np.tan(delta)
    
    def reset(self, x=0, y=0, theta=0, v=0):
        """Reset vehicle to initial state"""
        self.state = np.array([x, y, theta, v], dtype=float)
        self.history = {'x': [], 'y': [], 'theta': [], 'v': [], 't': []}
        self.time = 0.0


def load_gps_waypoints(csv_file):
    """
    Load GPS coordinates from CSV file
    Expected format: latitude, longitude (in degrees)
    Returns: numpy array of [lat, lon] coordinates
    """
    waypoints = []
    try:
        with open(csv_file, 'r') as f:
            reader = csv.reader(f)
            # Skip header if present
            first_row = next(reader)
            try:
                lat, lon = float(first_row[0]), float(first_row[1])
                waypoints.append([lat, lon])
            except ValueError:
                pass  # Was a header row
            
            for row in reader:
                if len(row) >= 2:
                    lat, lon = float(row[0]), float(row[1])
                    waypoints.append([lat, lon])
        
        return np.array(waypoints)
    except FileNotFoundError:
        print(f"Error: File '{csv_file}' not found")
        return None
    except Exception as e:
        print(f"Error loading CSV: {e}")
        return None


def gps_to_local(waypoints):
    """
    Convert GPS coordinates (lat, lon in degrees) to local Cartesian coordinates (meters)
    Uses equirectangular projection (suitable for small areas)
    
    Parameters:
    waypoints: Nx2 array of [latitude, longitude] in degrees
    
    Returns:
    Nx2 array of [x, y] in meters
    """
    # Use first point as origin
    lat0, lon0 = waypoints[0]
    
    # Earth radius in meters
    R = 6371000
    
    # Convert to radians
    lat_rad = np.deg2rad(waypoints[:, 0])
    lon_rad = np.deg2rad(waypoints[:, 1])
    lat0_rad = np.deg2rad(lat0)
    lon0_rad = np.deg2rad(lon0)
    
    # Equirectangular projection
    x = R * (lon_rad - lon0_rad) * np.cos(lat0_rad)
    y = R * (lat_rad - lat0_rad)
    
    return np.column_stack([x, y])


def calculate_path_headings(waypoints):
    """
    Calculate desired heading angles between consecutive waypoints
    
    Parameters:
    waypoints: Nx2 array of [x, y] coordinates
    
    Returns:
    Array of heading angles (radians) for each segment
    """
    dx = np.diff(waypoints[:, 0])
    dy = np.diff(waypoints[:, 1])
    headings = np.arctan2(dy, dx)
    
    # Add final heading (same as second-to-last)
    headings = np.append(headings, headings[-1])
    
    return headings


class PathFollowingController:
    """
    Controller to follow a predefined path using pure pursuit algorithm
    """
    
    def __init__(self, waypoints, lookahead_distance=3.0, max_steering=np.deg2rad(30)):
        """
        Parameters:
        waypoints: Nx2 array of [x, y] coordinates in meters
        lookahead_distance: distance ahead to aim for (m)
        max_steering: maximum steering angle (rad)
        """
        self.waypoints = waypoints
        self.lookahead_distance = lookahead_distance
        self.max_steering = max_steering
        self.current_waypoint_idx = 0
        self.path_complete = False
        
    def find_lookahead_point(self, vehicle_state):
        """
        Find the lookahead point on the path
        """
        x, y, theta, v = vehicle_state
        
        # Find closest waypoint ahead
        min_dist = float('inf')
        target_idx = self.current_waypoint_idx
        
        for i in range(self.current_waypoint_idx, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            dist = np.sqrt((wx - x)**2 + (wy - y)**2)
            
            if dist >= self.lookahead_distance:
                target_idx = i
                break
            
            if dist < min_dist and i > self.current_waypoint_idx:
                self.current_waypoint_idx = i
                min_dist = dist
        
        # Check if path is complete
        if target_idx >= len(self.waypoints) - 1:
            self.path_complete = True
            target_idx = len(self.waypoints) - 1
        
        return self.waypoints[target_idx], target_idx
    
    def calculate_steering(self, vehicle_state, wheelbase):
        """
        Calculate steering angle using pure pursuit
        """
        if self.path_complete:
            return 0.0
        
        x, y, theta, v = vehicle_state
        
        # Get lookahead point
        lookahead_point, _ = self.find_lookahead_point(vehicle_state)
        lx, ly = lookahead_point
        
        # Transform lookahead point to vehicle frame
        dx = lx - x
        dy = ly - y
        
        # Rotate to vehicle frame
        local_x = dx * np.cos(theta) + dy * np.sin(theta)
        local_y = -dx * np.sin(theta) + dy * np.cos(theta)
        
        # Calculate curvature
        ld = np.sqrt(local_x**2 + local_y**2)
        if ld < 0.1:
            return 0.0
        
        curvature = 2 * local_y / (ld ** 2)
        
        # Calculate steering angle
        steering = np.arctan(curvature * wheelbase)
        
        # Limit steering angle
        steering = np.clip(steering, -self.max_steering, self.max_steering)
        
        return steering


def simulate_path_following(csv_file, speed_profile='constant', target_speed=5.0, 
                           lookahead_distance=3.0, duration=120):
    """
    Simulate vehicle following GPS waypoints from CSV file
    
    Parameters:
    csv_file: path to CSV file with latitude, longitude coordinates
    speed_profile: 'constant', 'accelerate', or 'variable'
    target_speed: desired speed in m/s
    lookahead_distance: pure pursuit lookahead distance (m)
    duration: maximum simulation duration (s)
    """
    # Load GPS waypoints
    gps_waypoints = load_gps_waypoints(csv_file)
    if gps_waypoints is None:
        print("Creating sample waypoints for demonstration...")
        # Create sample circular track
        angles = np.linspace(0, 2*np.pi, 20)
        radius = 50
        gps_waypoints = np.column_stack([
            33.6405 + (radius/111320) * np.sin(angles),  # Sample latitude
            73.0680 + (radius/(111320*np.cos(np.deg2rad(33.6405)))) * np.cos(angles)  # Sample longitude
        ])
    
    print(f"Loaded {len(gps_waypoints)} waypoints")
    
    # Convert to local coordinates
    local_waypoints = gps_to_local(gps_waypoints)
    
    print(f"Track length: {np.sum(np.sqrt(np.sum(np.diff(local_waypoints, axis=0)**2, axis=1))):.2f} m")
    
    # Initialize vehicle at first waypoint
    x0, y0 = local_waypoints[0]
    heading0 = calculate_path_headings(local_waypoints)[0]
    
    vehicle = BicycleModel(L=1.5, track_width=1.01, dt=0.1)
    vehicle.reset(x=x0, y=y0, theta=heading0, v=0.0)
    
    # Initialize controller
    controller = PathFollowingController(local_waypoints, lookahead_distance=lookahead_distance)
    
    steps = int(duration / vehicle.dt)
    
    for i in range(steps):
        t = i * vehicle.dt
        
        # Calculate steering angle
        steering = controller.calculate_steering(vehicle.state, vehicle.L)
        
        # Speed control
        current_speed = vehicle.state[3]
        
        if speed_profile == 'constant':
            if current_speed < target_speed:
                acceleration = 1.0
            elif current_speed > target_speed:
                acceleration = -0.5
            else:
                acceleration = 0.0
        
        elif speed_profile == 'accelerate':
            if t < 10:
                acceleration = 1.0
            elif current_speed > target_speed:
                acceleration = -0.5
            else:
                acceleration = 0.0
        
        elif speed_profile == 'variable':
            # Slow down for tight turns
            if abs(steering) > np.deg2rad(10):
                desired_speed = target_speed * 0.6
            else:
                desired_speed = target_speed
            
            if current_speed < desired_speed:
                acceleration = 0.8
            elif current_speed > desired_speed:
                acceleration = -0.5
            else:
                acceleration = 0.0
        else:
            acceleration = 0.0
        
        # Update vehicle
        vehicle.update(acceleration, steering)
        
        # Check if path is complete
        if controller.path_complete:
            # Check if close to final waypoint
            final_wp = local_waypoints[-1]
            dist_to_final = np.sqrt((vehicle.state[0] - final_wp[0])**2 + 
                                   (vehicle.state[1] - final_wp[1])**2)
            if dist_to_final < 2.0:
                print(f"Path completed at t={t:.1f}s")
                break
    
    return vehicle, local_waypoints, gps_waypoints


def plot_path_following(vehicle, waypoints, gps_waypoints):
    """Plot vehicle trajectory following waypoints"""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # Trajectory plot with waypoints
    ax = axes[0, 0]
    ax.plot(waypoints[:, 0], waypoints[:, 1], 'r--', linewidth=2, 
            label='Desired Path', marker='o', markersize=6)
    ax.plot(vehicle.history['x'], vehicle.history['y'], 'b-', linewidth=2, 
            label='Actual Path')
    ax.plot(vehicle.history['x'][0], vehicle.history['y'][0], 'go', 
            markersize=12, label='Start')
    ax.plot(vehicle.history['x'][-1], vehicle.history['y'][-1], 'ro', 
            markersize=12, label='End')
    
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title('Path Following - Local Coordinates', fontsize=14, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # Calculate tracking error
    tracking_errors = []
    for i, (x, y) in enumerate(zip(vehicle.history['x'], vehicle.history['y'])):
        distances = np.sqrt((waypoints[:, 0] - x)**2 + (waypoints[:, 1] - y)**2)
        tracking_errors.append(np.min(distances))
    
    # Tracking error plot
    ax = axes[0, 1]
    ax.plot(vehicle.history['t'], tracking_errors, 'r-', linewidth=2)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Tracking Error (m)', fontsize=12)
    ax.set_title(f'Tracking Error (Mean: {np.mean(tracking_errors):.3f} m)', 
                 fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Velocity plot
    ax = axes[1, 0]
    ax.plot(vehicle.history['t'], vehicle.history['v'], 'b-', linewidth=2)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Velocity (m/s)', fontsize=12)
    ax.set_title('Velocity Profile', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Heading angle plot
    ax = axes[1, 1]
    ax.plot(vehicle.history['t'], np.rad2deg(vehicle.history['theta']), 'g-', linewidth=2)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Heading Angle (deg)', fontsize=12)
    ax.set_title('Heading Angle', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Print statistics
    print("\n" + "="*50)
    print("Path Following Statistics:")
    print("="*50)
    print(f"Mean Tracking Error: {np.mean(tracking_errors):.3f} m")
    print(f"Max Tracking Error: {np.max(tracking_errors):.3f} m")
    print(f"Average Speed: {np.mean(vehicle.history['v']):.2f} m/s")
    print(f"Total Distance: {np.sum(np.sqrt(np.diff(vehicle.history['x'])**2 + np.diff(vehicle.history['y'])**2)):.2f} m")
    print(f"Total Time: {vehicle.history['t'][-1]:.2f} s")
    
    return fig


def simulate_maneuver(maneuver_type='circle', duration=30):
    """
    Simulate different driving maneuvers
    
    maneuver_type: 'circle', 'slalom', 'straight', 'lane_change'
    """
    vehicle = BicycleModel(L=1.5, track_width=1.01, dt=0.05)
    
    steps = int(duration / vehicle.dt)
    
    for i in range(steps):
        t = i * vehicle.dt
        
        if maneuver_type == 'circle':
            # Drive in circle
            a = 0.5 if t < 5 else 0  # Accelerate then maintain
            delta = np.deg2rad(10)  # Constant steering angle
            
        elif maneuver_type == 'slalom':
            # Slalom maneuver
            a = 0.3 if t < 3 else 0
            delta = np.deg2rad(15 * np.sin(0.5 * t))  # Sinusoidal steering
            
        elif maneuver_type == 'straight':
            # Straight line acceleration
            a = 0.5 if t < 10 else 0
            delta = 0
            
        elif maneuver_type == 'lane_change':
            # Lane change maneuver
            a = 0.3 if t < 3 else 0
            if 5 < t < 8:
                delta = np.deg2rad(8)
            elif 8 < t < 11:
                delta = np.deg2rad(-8)
            else:
                delta = 0
        else:
            a, delta = 0, 0
        
        vehicle.update(a, delta)
    
    return vehicle


def plot_trajectory(vehicle):
    """Plot vehicle trajectory and state variables"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Trajectory plot
    ax = axes[0, 0]
    ax.plot(vehicle.history['x'], vehicle.history['y'], 'b-', linewidth=2, label='Path')
    ax.plot(vehicle.history['x'][0], vehicle.history['y'][0], 'go', markersize=10, label='Start')
    ax.plot(vehicle.history['x'][-1], vehicle.history['y'][-1], 'ro', markersize=10, label='End')
    
    # Draw vehicle at final position
    x, y, theta, v = vehicle.state
    wheels = vehicle.get_wheel_positions()
    
    # Draw chassis
    half_track = vehicle.track_width / 2
    corners_local = np.array([
        [-0.3, -half_track],
        [vehicle.L + 0.3, -half_track],
        [vehicle.L + 0.3, half_track],
        [-0.3, half_track]
    ])
    
    rot_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                           [np.sin(theta), np.cos(theta)]])
    corners_global = corners_local @ rot_matrix.T + np.array([x, y])
    
    chassis = plt.Polygon(corners_global, alpha=0.3, color='blue')
    ax.add_patch(chassis)
    
    # Draw wheels
    for wheel_name, (wx, wy) in wheels.items():
        ax.plot(wx, wy, 'ko', markersize=8)
    
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title('Vehicle Trajectory', fontsize=14, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # Velocity plot
    ax = axes[0, 1]
    ax.plot(vehicle.history['t'], vehicle.history['v'], 'r-', linewidth=2)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Velocity (m/s)', fontsize=12)
    ax.set_title('Velocity Profile', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Heading angle plot
    ax = axes[1, 0]
    ax.plot(vehicle.history['t'], np.rad2deg(vehicle.history['theta']), 'g-', linewidth=2)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Heading Angle (deg)', fontsize=12)
    ax.set_title('Heading Angle', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Speed vs position
    ax = axes[1, 1]
    distances = np.sqrt(np.diff(vehicle.history['x'])**2 + np.diff(vehicle.history['y'])**2)
    total_distance = np.cumsum(np.concatenate([[0], distances]))
    ax.plot(total_distance, vehicle.history['v'], 'm-', linewidth=2)
    ax.set_xlabel('Distance (m)', fontsize=12)
    ax.set_ylabel('Velocity (m/s)', fontsize=12)
    ax.set_title('Velocity vs Distance', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


# Example usage and demonstration
if __name__ == "__main__":
    print("Shell Eco Marathon - GPS Path Following Simulation")
    print("=" * 50)
    print(f"Track Width: 101 cm (1.01 m)")
    print(f"Wheelbase: 1.5 m")
    print()
    
    # Example 1: Load from CSV file
    print("Example: Path Following from CSV")
    print("-" * 50)
    csv_filename = "waypoints.csv"  # Change this to your CSV file
    
    vehicle, waypoints, gps_waypoints = simulate_path_following(
        csv_filename,
        speed_profile='variable',  # 'constant', 'accelerate', or 'variable'
        target_speed=8.0,  # m/s (about 29 km/h)
        lookahead_distance=4.0,
        duration=120
    )
    
    fig = plot_path_following(vehicle, waypoints, gps_waypoints)
    fig.suptitle('Shell Eco Marathon - GPS Path Following', 
                 fontsize=16, fontweight='bold', y=0.995)
    plt.show()
    
    # Example 2: Create sample CSV file
    print("\n" + "="*50)
    print("Creating sample CSV file: 'sample_waypoints.csv'")
    print("="*50)
    
    # Create a sample track (figure-8 pattern)
    t = np.linspace(0, 2*np.pi, 30)
    radius = 30
    center_lat = 33.6405  # Sample latitude (Islamabad)
    center_lon = 73.0680  # Sample longitude (Islamabad)
    
    # Figure-8 pattern
    lat = center_lat + (radius/111320) * np.sin(t)
    lon = center_lon + (radius/(111320*np.cos(np.deg2rad(center_lat)))) * np.sin(2*t)
    
    with open('sample_waypoints.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['latitude', 'longitude'])  # Header
        for lat_val, lon_val in zip(lat, lon):
            writer.writerow([f'{lat_val:.8f}', f'{lon_val:.8f}'])
    
    print("Sample CSV created successfully!")
    print("\nCSV Format:")
    print("  latitude,longitude")
    print("  33.64050000,73.06800000")
    print("  33.64052345,73.06801234")
    print("  ...")
    print("\nYou can replace 'sample_waypoints.csv' with your own GPS coordinates.")
    
    # Simulate with sample file
    print("\nSimulating with sample waypoints...")
    vehicle2, waypoints2, gps_waypoints2 = simulate_path_following(
        'smoothed_lusail.csv',
        speed_profile='constant',
        target_speed=6.0,
        lookahead_distance=3.0,
        duration=120
    )
    
    fig2 = plot_path_following(vehicle2, waypoints2, gps_waypoints2)
    fig2.suptitle('Shell Eco Marathon - Sample Track', 
                  fontsize=16, fontweight='bold', y=0.995)
    plt.show()