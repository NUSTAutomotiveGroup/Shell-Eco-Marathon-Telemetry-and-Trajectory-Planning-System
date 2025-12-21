import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import math
from math import radians, sin, cos, sqrt, atan2, degrees
import pandas as pd
import csv

# Import physics from the prototype
# Physics constants (from physics_prototype.py)
drivetrain_efficiency = .85
tire_coeff = 0.01
g = 9.81
prim_red = 1
sec_red = 1
diff_red = 2.769
gear_1 = 37.06
gear_2 = 21.86
gear_3 = 15.30 
gear_4 = 9.889
gears = [float(gear_1), float(gear_2), float(gear_3), float(gear_4)]
idle_rpm = 1500
frontal_area = 1.298
drag_coeff = 0.327335
mass = 220
wheel_radius = 0.29
time_step = 0.1
pi = math.pi

# Load power curve
df = pd.read_csv('70_power_curve.csv')

# Physics functions from prototype
def haversine(lat1, lon1, lat2, lon2):
    R = 6371.0
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):     
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    bearing = atan2(x, y)
    bearing = degrees(bearing)
    return (bearing + 360) % 360

def rpm_round(rpm):
    closest_rpm = df['rpm'].iloc[(df['rpm'] - rpm).abs().argsort()[:1]]
    if not closest_rpm.empty:
        if rpm >= 2000:
            if rpm < 9500:
                return closest_rpm.values[0]
            elif rpm >= 9500:
                return 9500
        elif rpm < 2000:
            return 2000
    return 2000

def get_hp(rounded_rpm):
    hp = df.loc[df['rpm'] == rounded_rpm, 'hp']
    return int(hp.values[0]) if not hp.empty else 0

def get_torque(rounded_rpm):
    torque = df.loc[df['rpm'] == rounded_rpm, 'torque']
    return float(torque.values[0]) if not torque.empty else 0

def drag_effect(init_speed_ms):
    drag_force = 0.5 * drag_coeff * frontal_area * (init_speed_ms * init_speed_ms) * 1.225
    drag_accel = drag_force / mass
    return drag_force, drag_accel

def speed_calc(rpm, gear):
    gear_ratio = gears[gear - 1]
    combined_ratio = gear_ratio 
    init_speed_ms = ((rpm * 2 * pi * wheel_radius) / (60 * combined_ratio))
    speed_kmh = init_speed_ms * 3.6
    return init_speed_ms, speed_kmh

def deceleration_calculation(drag_accel):
    neg_acceleration = -(tire_coeff * g) - drag_accel
    return neg_acceleration

def torque_gear_ratio_calculation(torque, gear, throttle):
    gear_ratio = gears[gear - 1]
    throttle_torque = torque * prim_red * sec_red * gear_ratio * drivetrain_efficiency
    corrected_torque = throttle_torque * (throttle / 100)
    return corrected_torque, throttle_torque

def acceleration_calculation(corrected_torque):
    accel = (corrected_torque * wheel_radius) / (mass * wheel_radius**2)
    return accel

def accel_total(accel, neg_acceleration):
    return accel + neg_acceleration

def speed_update(init_speed_ms, total_accel):
    final_speed_ms = init_speed_ms + (total_accel * time_step)
    if final_speed_ms < 0:
        final_speed_ms = 0
    return final_speed_ms

def Kinetic_Energy(final_speed_ms):
    KE_final = 0.5 * mass * (final_speed_ms ** 2)
    return KE_final

def distance_per_time(init_speed_ms, final_speed_ms):
    distance = (init_speed_ms + final_speed_ms) / 2 * time_step
    return distance

def final_speed_to_rpm(final_speed_ms, gear):
    gear_ratio = gears[gear - 1]
    combined_ratio = gear_ratio 
    rpm = (final_speed_ms * combined_ratio * 60) / (2 * pi * wheel_radius)
    return rpm

def gear_change(rpm, gear):
    if rpm < 2500 and gear > 1:
        rpm = (gears[gear - 2] * rpm) / gears[gear - 1]
        gear -= 1
        return gear, rpm
    elif rpm > 8000 and gear < 4:
        rpm = (gears[gear] * rpm) / gears[gear - 1]
        gear += 1
        return gear, rpm
    return gear, rpm


class RacingEnvironment:
    def __init__(self, track_coords, target_lap_time=30.0):
        self.track_coords = track_coords
        self.target_lap_time = target_lap_time
        self.n_waypoints = len(track_coords)
        
        # Calculate total track distance
        self.track_distance = 0
        for i in range(1, len(track_coords)):
            self.track_distance += haversine(
                track_coords[i-1][0], track_coords[i-1][1],
                track_coords[i][0], track_coords[i][1]
            ) * 1000  # Convert to meters
        
        print(f"Track distance: {self.track_distance:.2f} meters")
        
        # State variables
        self.current_waypoint = 0
        self.lat = track_coords[0][0]
        self.lon = track_coords[0][1]
        self.current_bearing = 0
        self.rpm = 4500
        self.gear = 1
        self.speed_ms = 0.0
        self.distance_covered = 0.0
        self.lap_time = 0.0
        self.total_ke = 0.0
        self.step_count = 0
        self.max_steps = 3000
        
        self.reset()
    
    def reset(self):
        """Reset environment to start position"""
        self.current_waypoint = 0
        self.lat = self.track_coords[0][0]
        self.lon = self.track_coords[0][1]
        
        # Calculate initial bearing
        if len(self.track_coords) > 1:
            self.current_bearing = calculate_bearing(
                self.track_coords[0][0], self.track_coords[0][1],
                self.track_coords[1][0], self.track_coords[1][1]
            )
        else:
            self.current_bearing = 0
        
        self.rpm = 4500
        self.gear = 1
        self.speed_ms = 0.0
        self.distance_covered = 0.0
        self.lap_time = 0.0
        self.total_ke = 0.0
        self.step_count = 0
        
        return self._get_state()
    
    def _get_state(self):
        """Get discretized state representation"""
        # Get target bearing to next waypoint
        next_wp = min(self.current_waypoint + 1, self.n_waypoints - 1)
        target_bearing = calculate_bearing(
            self.lat, self.lon,
            self.track_coords[next_wp][0], self.track_coords[next_wp][1]
        )
        
        # Calculate bearing error
        bearing_error = target_bearing - self.current_bearing
        if bearing_error > 180:
            bearing_error -= 360
        elif bearing_error < -180:
            bearing_error += 360
        
        # Discretize state
        bearing_discrete = int(np.clip(bearing_error / 10, -18, 18)) + 18  # 0-36
        speed_discrete = int(np.clip(self.speed_ms, 0, 50))  # 0-50
        rpm_discrete = int(np.clip(self.rpm / 500, 0, 20))  # 0-20
        gear_discrete = self.gear - 1  # 0-3
        progress_discrete = int((self.distance_covered / self.track_distance) * 10)  # 0-10
        
        return (bearing_discrete, speed_discrete, rpm_discrete, gear_discrete, progress_discrete)
    
    def step(self, action):
        """Execute action and return next state, reward, done"""
        # Action: [throttle_action, steering_action]
        # throttle_action: 0=0%, 1=25%, 2=50%, 3=75%, 4=100%
        # steering_action: 0=left, 1=straight, 2=right
        
        throttle_action = action // 3  # 0-4
        steering_action = action % 3  # 0-2
        
        throttle = throttle_action * 25  # 0, 25, 50, 75, 100
        
        # Calculate target bearing
        next_wp = min(self.current_waypoint + 1, self.n_waypoints - 1)
        target_bearing = calculate_bearing(
            self.lat, self.lon,
            self.track_coords[next_wp][0], self.track_coords[next_wp][1]
        )
        
        # Calculate bearing error
        bearing_error = target_bearing - self.current_bearing
        if bearing_error > 180:
            bearing_error -= 360
        elif bearing_error < -180:
            bearing_error += 360
        
        # Apply steering (simple model: steering adjusts bearing)
        steering_adjustment = 0
        if steering_action == 0:  # Left
            steering_adjustment = -5 if bearing_error < 0 else -2
        elif steering_action == 2:  # Right
            steering_adjustment = 5 if bearing_error > 0 else 2
        
        # Update bearing
        self.current_bearing = (self.current_bearing + steering_adjustment) % 360
        
        # Physics simulation (from prototype)
        rounded_rpm = rpm_round(self.rpm)
        torque = get_torque(rounded_rpm)
        
        init_speed_ms, speed_kmh = speed_calc(self.rpm, self.gear)
        drag_force, drag_accel = drag_effect(init_speed_ms)
        neg_acceleration = deceleration_calculation(drag_accel)
        corrected_torque, throttle_torque = torque_gear_ratio_calculation(torque, self.gear, throttle)
        accel = acceleration_calculation(corrected_torque)
        total_accel = accel_total(accel, neg_acceleration)
        final_speed_ms = speed_update(init_speed_ms, total_accel)
        
        # Update kinetic energy
        ke = Kinetic_Energy(final_speed_ms)
        self.total_ke += ke
        
        # Update position
        distance = distance_per_time(init_speed_ms, final_speed_ms)
        self.distance_covered += distance
        
        # Update RPM and gear
        self.rpm = final_speed_to_rpm(final_speed_ms, self.gear)
        self.gear, self.rpm = gear_change(self.rpm, self.gear)
        self.rpm = rpm_round(self.rpm)
        self.speed_ms = final_speed_ms
        
        # Update position on track
        self._update_position(distance)
        
        # Update time
        self.lap_time += time_step
        self.step_count += 1
        
        # Calculate reward
        reward = self._calculate_reward(bearing_error, ke, throttle)
        
        # Check if done
        lap_complete = self.distance_covered >= self.track_distance
        too_long = self.step_count >= self.max_steps
        off_track = abs(bearing_error) > 90  # Too far off course
        
        done = lap_complete or too_long or off_track
        
        info = {
            'lap_time': self.lap_time if lap_complete else None,
            'distance': self.distance_covered,
            'total_ke': self.total_ke,
            'bearing_error': bearing_error
        }
        
        return self._get_state(), reward, done, info
    
    def _update_position(self, distance):
        """Update lat/lon position based on bearing and distance"""
        # Simple approximation: move along current bearing
        R = 6371000  # Earth radius in meters
        bearing_rad = radians(self.current_bearing)
        
        lat_rad = radians(self.lat)
        lon_rad = radians(self.lon)
        
        new_lat_rad = math.asin(sin(lat_rad) * cos(distance/R) +
                                cos(lat_rad) * sin(distance/R) * cos(bearing_rad))
        new_lon_rad = lon_rad + atan2(sin(bearing_rad) * sin(distance/R) * cos(lat_rad),
                                      cos(distance/R) - sin(lat_rad) * sin(new_lat_rad))
        
        self.lat = degrees(new_lat_rad)
        self.lon = degrees(new_lon_rad)
        
        # Check if reached next waypoint
        if self.current_waypoint < self.n_waypoints - 1:
            dist_to_next = haversine(
                self.lat, self.lon,
                self.track_coords[self.current_waypoint + 1][0],
                self.track_coords[self.current_waypoint + 1][1]
            ) * 1000
            
            if dist_to_next < 10:  # Within 10 meters
                self.current_waypoint += 1
    
    def _calculate_reward(self, bearing_error, ke, throttle):
        """Calculate reward based on multiple factors"""
        reward = 0
        
        # Reward for staying on track (low bearing error)
        if abs(bearing_error) < 5:
            reward += 5
        elif abs(bearing_error) < 15:
            reward += 2
        elif abs(bearing_error) < 30:
            reward += 0.5
        else:
            reward -= abs(bearing_error) * 0.5
        
        # Reward for making progress
        reward += 1
        
        # Penalty for excessive kinetic energy
        reward -= ke * 0.0001
        
        # Check if lap completed
        if self.distance_covered >= self.track_distance:
            time_diff = abs(self.lap_time - self.target_lap_time)
            
            if time_diff < 2:
                reward += 1000
            else:
                reward -= time_diff * 50
            
            # Penalty for total kinetic energy used
            reward -= self.total_ke * 0.001
        
        return reward


class QLearningAgent:
    def __init__(self, n_actions=15, learning_rate=0.1, discount_factor=0.99,
                 epsilon_start=1.0, epsilon_min=0.01, epsilon_decay=0.995):
        self.n_actions = n_actions  # 5 throttle * 3 steering = 15 actions
        self.lr = learning_rate
        self.gamma = discount_factor
        self.epsilon = epsilon_start
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay
        
        self.q_table = defaultdict(lambda: np.zeros(n_actions))
    
    def select_action(self, state):
        if np.random.random() < self.epsilon:
            return np.random.randint(self.n_actions)
        else:
            return np.argmax(self.q_table[state])
    
    def update(self, state, action, reward, next_state, done):
        current_q = self.q_table[state][action]
        
        if done:
            target_q = reward
        else:
            max_next_q = np.max(self.q_table[next_state])
            target_q = reward + self.gamma * max_next_q
        
        self.q_table[state][action] = current_q + self.lr * (target_q - current_q)
    
    def decay_epsilon(self):
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)


def read_coords(filename):
    coords = []
    with open(filename, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            lat = float(row['latitude'])
            lon = float(row['longitude'])
            coords.append((lat, lon))
    return coords


def train_agent(coords, n_episodes=200, target_lap_time=30.0):
    env = RacingEnvironment(coords, target_lap_time=target_lap_time)
    agent = QLearningAgent()
    
    episode_rewards = []
    episode_lap_times = []
    episode_ke = []
    
    print(f"Training for {n_episodes} episodes...")
    print(f"Target lap time: {target_lap_time}s")
    print(f"Track distance: {env.track_distance:.2f}m")
    print("-" * 60)
    
    for episode in range(n_episodes):
        state = env.reset()
        total_reward = 0
        done = False
        
        while not done:
            action = agent.select_action(state)
            next_state, reward, done, info = env.step(action)
            agent.update(state, action, reward, next_state, done)
            
            state = next_state
            total_reward += reward
        
        agent.decay_epsilon()
        episode_rewards.append(total_reward)
        
        if info['lap_time'] is not None:
            episode_lap_times.append(info['lap_time'])
            episode_ke.append(info['total_ke'])
        
        if (episode + 1) % 20 == 0:
            avg_reward = np.mean(episode_rewards[-20:])
            avg_lap_time = np.mean(episode_lap_times[-10:]) if episode_lap_times else 0
            avg_ke = np.mean(episode_ke[-10:]) if episode_ke else 0
            print(f"Episode {episode + 1:3d} | Reward: {avg_reward:7.1f} | "
                  f"Lap: {avg_lap_time:5.1f}s | KE: {avg_ke:10.1f} | "
                  f"ε: {agent.epsilon:.3f} | Distance: {info['distance']:.1f}m")
    
    print("\nTraining completed!")
    return agent, env, episode_rewards, episode_lap_times, episode_ke


def plot_results(rewards, lap_times, ke_values, target_lap_time):
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Rewards
    axes[0, 0].plot(rewards, alpha=0.4)
    if len(rewards) >= 20:
        window = 20
        moving_avg = np.convolve(rewards, np.ones(window)/window, mode='valid')
        axes[0, 0].plot(range(window-1, len(rewards)), moving_avg, linewidth=2)
    axes[0, 0].set_xlabel('Episode')
    axes[0, 0].set_ylabel('Total Reward')
    axes[0, 0].set_title('Training Rewards')
    axes[0, 0].grid(True, alpha=0.3)
    
    # Lap times
    if lap_times:
        axes[0, 1].scatter(range(len(lap_times)), lap_times, alpha=0.6, s=20)
        axes[0, 1].axhline(y=target_lap_time, color='r', linestyle='--', linewidth=2)
        axes[0, 1].set_xlabel('Completed Lap')
        axes[0, 1].set_ylabel('Lap Time (s)')
        axes[0, 1].set_title('Lap Times')
        axes[0, 1].grid(True, alpha=0.3)
    
    # Kinetic Energy
    if ke_values:
        axes[1, 0].plot(ke_values, marker='o', alpha=0.6)
        axes[1, 0].set_xlabel('Completed Lap')
        axes[1, 0].set_ylabel('Total KE (J)')
        axes[1, 0].set_title('Kinetic Energy per Lap')
        axes[1, 0].grid(True, alpha=0.3)
    
    # Efficiency (KE vs Lap Time)
    if lap_times and ke_values:
        axes[1, 1].scatter(lap_times, ke_values, alpha=0.6, s=50)
        axes[1, 1].set_xlabel('Lap Time (s)')
        axes[1, 1].set_ylabel('Total KE (J)')
        axes[1, 1].set_title('Efficiency: KE vs Lap Time')
        axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('rl_training_results.png', dpi=150)
    print("Results saved to 'rl_training_results.png'")
    plt.show()


if __name__ == "__main__":
    # Load track coordinates
    coords = read_coords('track_with_bounds.csv')
    
    # Train agent
    TARGET_LAP_TIME = 525
    N_EPISODES = 200
    
    agent, env, rewards, lap_times, ke_values = train_agent(
        coords,
        n_episodes=N_EPISODES,
        target_lap_time=TARGET_LAP_TIME
    )
    
    # Plot results
    plot_results(rewards, lap_times, ke_values, TARGET_LAP_TIME)
    
    print(f"\nFinal Statistics:")
    print(f"Q-table size: {len(agent.q_table)} states")
    if lap_times:
        print(f"Best lap time: {min(lap_times):.2f}s")
        print(f"Best KE: {min(ke_values):.2f}J")
        print(f"Laps completed: {len(lap_times)}/{N_EPISODES}")